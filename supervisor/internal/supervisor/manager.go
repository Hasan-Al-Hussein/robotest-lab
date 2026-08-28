// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

// Package supervisor owns bounded child process groups and restart policy.
package supervisor

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"os"
	"os/exec"
	"sort"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/Hasan-Al-Hussein/robotest-lab/supervisor/internal/config"
)

const (
	defaultCleanupKillGrace    = 2 * time.Second
	defaultCleanupPollInterval = 20 * time.Millisecond
	cleanupFailureKind         = "process_group_cleanup_incomplete"
)

// errProcessGroupCleanupIncomplete is returned after shutdown when a child
// leader could not be reaped or its captured process group could not be proven
// empty, or when a cleanup operation failed. The manager remains alive but
// unready until that shutdown request.
var errProcessGroupCleanupIncomplete = errors.New(cleanupFailureKind)

type cleanupPolicy struct {
	terminationGrace time.Duration
	killGrace        time.Duration
	pollInterval     time.Duration
}

type startedProcess struct {
	pid  int
	wait <-chan error
}

type processSystem interface {
	Start(config.Child) (startedProcess, error)
	SignalProcessGroup(int, syscall.Signal) error
	ProcessGroupExists(int) (bool, error)
}

type unixProcessSystem struct{}

func (unixProcessSystem) Start(spec config.Child) (startedProcess, error) {
	cmd := exec.Command(spec.Argv[0], spec.Argv[1:]...)
	cmd.Dir = spec.WorkingDirectory
	cmd.Env = mergedEnvironment(spec.Environment)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true, Pdeathsig: syscall.SIGTERM}
	if err := cmd.Start(); err != nil {
		return startedProcess{}, err
	}

	wait := make(chan error, 1)
	go func() {
		wait <- cmd.Wait()
		close(wait)
	}()
	return startedProcess{pid: cmd.Process.Pid, wait: wait}, nil
}

func (unixProcessSystem) SignalProcessGroup(pgid int, signal syscall.Signal) error {
	return syscall.Kill(-pgid, signal)
}

func (unixProcessSystem) ProcessGroupExists(pgid int) (bool, error) {
	if pgid <= 0 {
		return false, nil
	}
	err := syscall.Kill(-pgid, 0)
	switch {
	case err == nil:
		return true, nil
	case errors.Is(err, syscall.ESRCH):
		return false, nil
	default:
		return true, err
	}
}

type cleanupResult struct {
	leaderReaped bool
	waitErr      error
	groupEmpty   bool
	err          error
}

func (result cleanupResult) complete() bool {
	return result.leaderReaped && result.groupEmpty && result.err == nil
}

type cleanupMode int

const (
	cleanupRequested cleanupMode = iota
	cleanupResidual
)

type childMonitorResult struct {
	failureKind     string
	continuousReady time.Duration
	stop            bool
	cleanupErr      error
}

type restartWaitResult struct {
	committed bool
	restart   bool
}

type childRuntime struct {
	spec                       config.Child
	state                      ChildSnapshot
	tracker                    *RestartTracker
	heartbeatObservedThisStart bool
	readySince                 time.Time
}

// Manager supervises one fixed, validated set of child process groups.
type Manager struct {
	cfg     config.Config
	clock   Clock
	logger  *slog.Logger
	store   *eventStore
	lock    *stateLock
	system  processSystem
	cleanup cleanupPolicy

	transitionMu       sync.Mutex
	mu                 sync.RWMutex
	children           map[string]*childRuntime
	shuttingDown       bool
	persistenceHealthy bool
	lastReady          bool
	startedAt          time.Time
	runStarted         bool
	runActive          bool
	closed             bool
	terminalErr        error
	wg                 sync.WaitGroup
}

// New builds a manager without starting any process.
func New(cfg config.Config, logger *slog.Logger) (*Manager, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	if logger == nil {
		logger = slog.Default()
	}
	lock, err := acquireStateLock(cfg.StateDirectory)
	if err != nil {
		return nil, err
	}
	store, err := newEventStore(cfg.StateDirectory, cfg.MaximumEventEntries, cfg.MaximumEventBytes)
	if err != nil {
		return nil, errors.Join(err, lock.close())
	}
	children := make(map[string]*childRuntime, len(cfg.Children))
	for _, spec := range cfg.Children {
		children[spec.Name] = &childRuntime{
			spec: spec,
			state: ChildSnapshot{
				Name:           spec.Name,
				Required:       spec.Required,
				HeartbeatFresh: spec.HeartbeatFile == "",
			},
			tracker: newRestartTracker(cfg.Restart),
		}
	}
	return &Manager{
		cfg:    cfg,
		clock:  realClock{},
		logger: logger,
		store:  store,
		lock:   lock,
		system: unixProcessSystem{},
		cleanup: cleanupPolicy{
			terminationGrace: cfg.TerminationGrace(),
			killGrace:        defaultCleanupKillGrace,
			pollInterval:     defaultCleanupPollInterval,
		},
		children:           children,
		persistenceHealthy: !store.isSaturated(),
	}, nil
}

// Close releases the singleton lock after Run has stopped. It is safe to call more than once.
func (manager *Manager) Close() error {
	manager.mu.Lock()
	if manager.closed {
		manager.mu.Unlock()
		return nil
	}
	if manager.runActive {
		manager.mu.Unlock()
		return errors.New("cannot close supervisor manager while Run is active")
	}
	manager.closed = true
	lock := manager.lock
	manager.lock = nil
	manager.mu.Unlock()
	return lock.close()
}

// SetClock is intended for deterministic package tests before Run starts.
func (manager *Manager) SetClock(clock Clock) {
	manager.mu.Lock()
	defer manager.mu.Unlock()
	manager.clock = clock
}

// Run starts all configured children and blocks until cancellation and cleanup complete.
func (manager *Manager) Run(ctx context.Context) error {
	manager.mu.Lock()
	if manager.closed {
		manager.mu.Unlock()
		return errors.New("supervisor manager is closed")
	}
	if manager.runStarted {
		manager.mu.Unlock()
		return errors.New("supervisor manager Run may be called only once")
	}
	manager.runStarted = true
	manager.runActive = true
	manager.startedAt = manager.clock.Now()
	manager.mu.Unlock()
	manager.record(Event{Kind: "supervisor_started"})

	for _, spec := range manager.cfg.Children {
		runtime := manager.children[spec.Name]
		manager.wg.Add(1)
		go func() {
			defer manager.wg.Done()
			manager.manageChild(ctx, runtime)
		}()
	}

	<-ctx.Done()
	manager.beginShutdown()

	done := make(chan struct{})
	go func() {
		manager.wg.Wait()
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(manager.cfg.ShutdownTimeout()):
		manager.record(Event{Kind: "shutdown_timeout"})
		go func() {
			<-done
			manager.mu.Lock()
			manager.runActive = false
			manager.mu.Unlock()
		}()
		return errors.Join(
			errors.New("supervisor shutdown exceeded configured timeout"),
			manager.terminalError(),
		)
	}
	manager.record(Event{Kind: "supervisor_stopped"})
	manager.mu.Lock()
	manager.runActive = false
	manager.mu.Unlock()
	return manager.terminalError()
}

func (manager *Manager) beginShutdown() {
	manager.transitionMu.Lock()
	defer manager.transitionMu.Unlock()
	manager.mu.Lock()
	manager.shuttingDown = true
	manager.mu.Unlock()
	manager.record(Event{Kind: "shutdown_requested"})
}

// transitionPermittedLocked requires manager.mu to be held.
func (manager *Manager) transitionPermittedLocked(ctx context.Context) bool {
	return ctx.Err() == nil && !manager.shuttingDown
}

func (manager *Manager) transitionPermitted(ctx context.Context) bool {
	if ctx.Err() != nil {
		return false
	}
	manager.mu.RLock()
	defer manager.mu.RUnlock()
	return !manager.shuttingDown
}

func (manager *Manager) manageChild(ctx context.Context, runtime *childRuntime) {
	firstStart := true
	for {
		if ctx.Err() != nil {
			return
		}
		startedAt, pgid, wait, committed, startErr := manager.startChild(ctx, runtime, !firstStart)
		firstStart = false
		if startErr != nil {
			if !manager.markFailure(ctx, runtime, "start_failed", nil, startErr, true) {
				return
			}
			restart := manager.waitForRestart(ctx, runtime, 0)
			if !restart.committed || !restart.restart {
				return
			}
			continue
		}
		if !committed {
			manager.cleanupUncommittedStartedChild(runtime, pgid, wait)
			return
		}

		result := manager.monitorStartedChild(ctx, runtime, startedAt, pgid, wait)
		if result.stop {
			return
		}
		if result.failureKind == "" {
			return
		}
		restart := manager.waitForRestart(ctx, runtime, result.continuousReady)
		if !restart.committed || !restart.restart {
			return
		}
	}
}

func (manager *Manager) monitorStartedChild(
	ctx context.Context,
	runtime *childRuntime,
	startedAt time.Time,
	pgid int,
	wait <-chan error,
) childMonitorResult {
	heartbeatTimer := manager.clock.After(manager.cfg.HeartbeatPoll())
	for {
		select {
		case waitErr, ok := <-wait:
			continuousReady := manager.continuousReadyDuration(runtime, manager.clock.Now())
			leaderReaped := ok
			if !ok {
				waitErr = errors.New("child wait channel closed without an exit status")
			}
			stopping := ctx.Err() != nil
			failureCommitted := false
			if !stopping {
				code := exitCode(waitErr)
				failureCommitted = manager.markFailure(
					ctx, runtime, "unexpected_exit", &code, waitErr, false,
				)
			}
			cleanup := manager.cleanupProcessGroup(
				context.Background(), runtime.spec.Name, pgid, nil, leaderReaped, waitErr, cleanupResidual,
			)
			if !cleanup.complete() {
				manager.markCleanupIncomplete(runtime, pgid, cleanup)
				return childMonitorResult{
					continuousReady: continuousReady,
					stop:            true,
					cleanupErr:      cleanup.err,
				}
			}
			manager.markStopped(runtime, exitCode(cleanup.waitErr))
			if !failureCommitted || stopping || ctx.Err() != nil {
				return childMonitorResult{continuousReady: continuousReady, stop: true}
			}
			return childMonitorResult{
				failureKind:     "unexpected_exit",
				continuousReady: continuousReady,
			}
		case <-heartbeatTimer:
			now := manager.clock.Now()
			continuousReady := manager.continuousReadyDuration(runtime, now)
			if ctx.Err() != nil {
				return manager.stopStartedChild(runtime, pgid, wait, continuousReady)
			}
			fresh, age := heartbeatState(runtime.spec.HeartbeatFile, startedAt, now, manager.cfg.HeartbeatStale())
			manager.updateHeartbeat(runtime, fresh, age)
			manager.mu.RLock()
			observed := runtime.heartbeatObservedThisStart
			manager.mu.RUnlock()
			failureAfter := manager.cfg.HeartbeatStale()
			heartbeatFailureKind := "heartbeat_stale"
			if !observed {
				failureAfter = manager.cfg.HeartbeatStartupTimeout()
				heartbeatFailureKind = "heartbeat_startup_timeout"
			}
			if !fresh && now.Sub(startedAt) >= failureAfter {
				if ctx.Err() != nil {
					return manager.stopStartedChild(runtime, pgid, wait, continuousReady)
				}
				failureCommitted := manager.markFailure(
					ctx, runtime, heartbeatFailureKind, nil, nil, false,
				)
				cleanup := manager.cleanupProcessGroup(
					context.Background(), runtime.spec.Name, pgid, wait, false, nil, cleanupRequested,
				)
				if !cleanup.complete() {
					manager.markCleanupIncomplete(runtime, pgid, cleanup)
					return childMonitorResult{
						continuousReady: continuousReady,
						stop:            true,
						cleanupErr:      cleanup.err,
					}
				}
				manager.markStopped(runtime, exitCode(cleanup.waitErr))
				if !failureCommitted || ctx.Err() != nil {
					return childMonitorResult{continuousReady: continuousReady, stop: true}
				}
				return childMonitorResult{
					failureKind:     heartbeatFailureKind,
					continuousReady: continuousReady,
				}
			}
			heartbeatTimer = manager.clock.After(manager.cfg.HeartbeatPoll())
		case <-ctx.Done():
			continuousReady := manager.continuousReadyDuration(runtime, manager.clock.Now())
			return manager.stopStartedChild(runtime, pgid, wait, continuousReady)
		}
	}
}

func (manager *Manager) stopStartedChild(
	runtime *childRuntime,
	pgid int,
	wait <-chan error,
	continuousReady time.Duration,
) childMonitorResult {
	cleanup := manager.cleanupProcessGroup(
		context.Background(), runtime.spec.Name, pgid, wait, false, nil, cleanupRequested,
	)
	if !cleanup.complete() {
		manager.markCleanupIncomplete(runtime, pgid, cleanup)
		return childMonitorResult{
			continuousReady: continuousReady,
			stop:            true,
			cleanupErr:      cleanup.err,
		}
	}
	manager.markStopped(runtime, exitCode(cleanup.waitErr))
	return childMonitorResult{continuousReady: continuousReady, stop: true}
}

func (manager *Manager) cleanupUncommittedStartedChild(
	runtime *childRuntime,
	pgid int,
	wait <-chan error,
) {
	cleanup := manager.cleanupProcessGroup(
		context.Background(), runtime.spec.Name, pgid, wait, false, nil, cleanupRequested,
	)
	if !cleanup.complete() {
		manager.markCleanupIncomplete(runtime, pgid, cleanup)
	}
}

func (manager *Manager) startChild(
	ctx context.Context,
	runtime *childRuntime,
	restarted bool,
) (time.Time, int, <-chan error, bool, error) {
	startedAt := manager.clock.Now()
	process, err := manager.system.Start(runtime.spec)
	if err != nil {
		return startedAt, 0, nil, false, err
	}
	pid := process.pid

	manager.transitionMu.Lock()
	manager.mu.Lock()
	if !manager.transitionPermittedLocked(ctx) {
		manager.mu.Unlock()
		manager.transitionMu.Unlock()
		return startedAt, pid, process.wait, false, nil
	}
	runtime.state.Running = true
	runtime.state.PID = pid
	runtime.state.PGID = pid
	runtime.state.StartedUTC = startedAt.UTC().Format(time.RFC3339Nano)
	runtime.state.HeartbeatFresh = runtime.spec.HeartbeatFile == ""
	runtime.state.HeartbeatAgeMS = nil
	runtime.state.CircuitOpen = false
	runtime.heartbeatObservedThisStart = runtime.spec.HeartbeatFile == ""
	if runtime.spec.HeartbeatFile == "" {
		runtime.state.LastReadyUTC = runtime.state.StartedUTC
		runtime.readySince = startedAt
	} else {
		runtime.readySince = time.Time{}
	}
	if restarted {
		runtime.state.RestartCount++
	}
	manager.mu.Unlock()
	manager.record(Event{Kind: "child_started", Child: runtime.spec.Name, PID: pid, PGID: pid})
	manager.refreshReadiness()
	manager.transitionMu.Unlock()
	return startedAt, pid, process.wait, true, nil
}

func (manager *Manager) cleanupProcessGroup(
	ctx context.Context,
	child string,
	pgid int,
	wait <-chan error,
	leaderReaped bool,
	waitErr error,
	mode cleanupMode,
) cleanupResult {
	result := cleanupResult{leaderReaped: leaderReaped, waitErr: waitErr}
	result.groupEmpty, result.err = manager.processGroupEmpty(pgid)
	if mode == cleanupResidual && result.complete() {
		return result
	}

	if mode == cleanupResidual {
		manager.record(Event{Kind: "residual_process_group_detected", Child: child, PGID: pgid})
	} else {
		manager.record(Event{Kind: "child_stop_requested", Child: child, PGID: pgid})
	}
	result.err = errors.Join(result.err, manager.signalProcessGroup(pgid, syscall.SIGTERM))
	result = manager.waitEmpty(ctx, pgid, wait, result, manager.cleanup.terminationGrace)
	if result.complete() {
		if mode == cleanupResidual {
			manager.record(Event{Kind: "residual_process_group_stopped", Child: child, PGID: pgid})
		}
		return result
	}

	killEvent := "child_kill_escalated"
	if mode == cleanupResidual {
		killEvent = "residual_process_group_kill_escalated"
	}
	manager.record(Event{Kind: killEvent, Child: child, PGID: pgid})
	result.err = errors.Join(result.err, manager.signalProcessGroup(pgid, syscall.SIGKILL))
	result = manager.waitEmpty(ctx, pgid, wait, result, manager.cleanup.killGrace)
	if !result.leaderReaped {
		result.err = errors.Join(result.err, errors.New("child leader was not reaped within the cleanup bound"))
	}
	if !result.groupEmpty {
		result.err = errors.Join(result.err, errors.New("captured process group is not empty after SIGKILL"))
	}
	return result
}

func (manager *Manager) signalProcessGroup(pgid int, signal syscall.Signal) error {
	if pgid <= 0 {
		return nil
	}
	err := manager.system.SignalProcessGroup(pgid, signal)
	if err == nil || errors.Is(err, syscall.ESRCH) {
		return nil
	}
	return fmt.Errorf("signal process group %d with %s: %w", pgid, signal, err)
}

func (manager *Manager) processGroupEmpty(pgid int) (bool, error) {
	exists, err := manager.system.ProcessGroupExists(pgid)
	if err != nil {
		return false, fmt.Errorf("probe process group %d: %w", pgid, err)
	}
	return !exists, nil
}

// waitEmpty uses real time because process termination is a kernel operation,
// not restart policy. The final non-blocking wait and group probe close races at
// both cancellation and deadline boundaries.
func (manager *Manager) waitEmpty(
	ctx context.Context,
	pgid int,
	wait <-chan error,
	result cleanupResult,
	timeout time.Duration,
) cleanupResult {
	if result.leaderReaped {
		wait = nil
	}
	probe := func(final bool) {
		if final && !result.leaderReaped && wait != nil {
			select {
			case observedErr, ok := <-wait:
				wait = nil
				if ok {
					result.leaderReaped = true
					result.waitErr = observedErr
				} else {
					result.err = errors.Join(
						result.err,
						errors.New("child wait channel closed without an exit status"),
					)
				}
			default:
			}
		}
		groupEmpty, err := manager.processGroupEmpty(pgid)
		result.groupEmpty = groupEmpty
		result.err = errors.Join(result.err, err)
	}
	probe(false)
	if result.leaderReaped && result.groupEmpty {
		return result
	}

	timer := time.NewTimer(timeout)
	defer timer.Stop()
	ticker := time.NewTicker(manager.cleanup.pollInterval)
	defer ticker.Stop()
	for {
		select {
		case observedErr, ok := <-wait:
			wait = nil
			if ok {
				result.leaderReaped = true
				result.waitErr = observedErr
			} else {
				result.err = errors.Join(
					result.err,
					errors.New("child wait channel closed without an exit status"),
				)
			}
			probe(false)
			if result.leaderReaped && result.groupEmpty {
				return result
			}
		case <-ticker.C:
			probe(false)
			if result.leaderReaped && result.groupEmpty {
				return result
			}
		case <-timer.C:
			probe(true)
			return result
		case <-ctx.Done():
			probe(true)
			result.err = errors.Join(result.err, ctx.Err())
			return result
		}
	}
}

func processGroupExists(pgid int) bool {
	exists, _ := (unixProcessSystem{}).ProcessGroupExists(pgid)
	return exists
}

func (manager *Manager) waitForRestart(
	ctx context.Context,
	runtime *childRuntime,
	continuousReady time.Duration,
) restartWaitResult {
	if ctx.Err() != nil {
		return restartWaitResult{}
	}
	now := manager.clock.Now()
	manager.transitionMu.Lock()
	manager.mu.Lock()
	if !manager.transitionPermittedLocked(ctx) {
		manager.mu.Unlock()
		manager.transitionMu.Unlock()
		return restartWaitResult{}
	}
	delay, attempt, exhausted := runtime.tracker.RecordFailure(now, continuousReady)
	if exhausted {
		runtime.state.CircuitOpen = true
	}
	manager.mu.Unlock()
	if exhausted {
		manager.record(Event{Kind: "restart_exhausted", Child: runtime.spec.Name, RestartAttempt: attempt})
		manager.refreshReadiness()
		manager.transitionMu.Unlock()
		return restartWaitResult{committed: true}
	}
	manager.record(Event{
		Kind:           "restart_scheduled",
		Child:          runtime.spec.Name,
		RestartAttempt: attempt,
		BackoffMS:      delay.Milliseconds(),
	})
	manager.transitionMu.Unlock()
	if !manager.transitionPermitted(ctx) {
		return restartWaitResult{committed: true}
	}
	select {
	case <-manager.clock.After(delay):
		if !manager.transitionPermitted(ctx) {
			return restartWaitResult{committed: true}
		}
		return restartWaitResult{committed: true, restart: true}
	case <-ctx.Done():
		return restartWaitResult{committed: true}
	}
}

func (manager *Manager) markFailure(
	ctx context.Context,
	runtime *childRuntime,
	kind string,
	code *int,
	err error,
	processExited bool,
) bool {
	now := manager.clock.Now()
	manager.transitionMu.Lock()
	manager.mu.Lock()
	if !manager.transitionPermittedLocked(ctx) {
		manager.mu.Unlock()
		manager.transitionMu.Unlock()
		return false
	}
	runtime.state.Running = false
	runtime.state.HeartbeatFresh = false
	runtime.state.LastFailureKind = kind
	runtime.state.LastFailureUTC = now.UTC().Format(time.RFC3339Nano)
	runtime.state.LastExitCode = code
	runtime.readySince = time.Time{}
	pid := runtime.state.PID
	pgid := runtime.state.PGID
	if processExited {
		runtime.state.PID = 0
		runtime.state.PGID = 0
	}
	manager.mu.Unlock()
	details := map[string]any{}
	if err != nil {
		details["error"] = err.Error()
	}
	manager.record(Event{
		Kind:        "failure_detected",
		Child:       runtime.spec.Name,
		PID:         pid,
		PGID:        pgid,
		ExitCode:    code,
		FailureKind: kind,
		Details:     details,
	})
	manager.refreshReadiness()
	manager.transitionMu.Unlock()
	return true
}

func (manager *Manager) markCleanupIncomplete(runtime *childRuntime, pgid int, cleanup cleanupResult) {
	now := manager.clock.Now()
	manager.mu.Lock()
	publishedPID := runtime.state.PID
	publishedPGID := runtime.state.PGID
	if publishedPID == 0 {
		publishedPID = pgid
	}
	if publishedPGID == 0 {
		publishedPGID = pgid
	}
	runtime.state.Running = false
	runtime.state.HeartbeatFresh = false
	runtime.state.PID = publishedPID
	runtime.state.PGID = publishedPGID
	runtime.state.CircuitOpen = true
	runtime.state.LastFailureKind = cleanupFailureKind
	runtime.state.LastFailureUTC = now.UTC().Format(time.RFC3339Nano)
	runtime.readySince = time.Time{}
	if cleanup.leaderReaped {
		code := exitCode(cleanup.waitErr)
		runtime.state.PID = 0
		runtime.state.LastExitCode = &code
	}
	// Keep the captured PGID on every incomplete cleanup as durable diagnostic
	// identity. PID is cleared only when the leader's wait status was collected.
	cause := cleanup.err
	if cause == nil {
		cause = errors.New("process group cleanup did not reach a complete state")
	}
	manager.terminalErr = errors.Join(
		manager.terminalErr,
		fmt.Errorf("%w for child %q: %v", errProcessGroupCleanupIncomplete, runtime.spec.Name, cause),
	)
	manager.mu.Unlock()

	manager.record(Event{
		Kind:        "child_group_cleanup_incomplete",
		Child:       runtime.spec.Name,
		PID:         publishedPID,
		PGID:        publishedPGID,
		FailureKind: cleanupFailureKind,
		Details: map[string]any{
			"error":         cause.Error(),
			"leader_reaped": cleanup.leaderReaped,
			"group_empty":   cleanup.groupEmpty,
			"captured_pgid": pgid,
		},
	})
	manager.refreshReadiness()
}

func (manager *Manager) terminalError() error {
	manager.mu.RLock()
	defer manager.mu.RUnlock()
	return manager.terminalErr
}

func (manager *Manager) markStopped(runtime *childRuntime, code int) {
	manager.mu.Lock()
	runtime.state.Running = false
	runtime.state.HeartbeatFresh = false
	runtime.state.PID = 0
	runtime.state.PGID = 0
	runtime.state.LastExitCode = &code
	runtime.readySince = time.Time{}
	manager.mu.Unlock()
	manager.record(Event{Kind: "child_stopped", Child: runtime.spec.Name, ExitCode: &code})
	manager.refreshReadiness()
}

func (manager *Manager) updateHeartbeat(runtime *childRuntime, fresh bool, age *time.Duration) {
	manager.mu.Lock()
	changed := runtime.state.HeartbeatFresh != fresh
	runtime.state.HeartbeatFresh = fresh
	if fresh {
		runtime.heartbeatObservedThisStart = true
		if runtime.readySince.IsZero() {
			runtime.readySince = manager.clock.Now()
		}
	} else {
		runtime.readySince = time.Time{}
	}
	if age == nil {
		runtime.state.HeartbeatAgeMS = nil
	} else {
		milliseconds := age.Milliseconds()
		runtime.state.HeartbeatAgeMS = &milliseconds
	}
	if fresh && changed {
		runtime.state.LastReadyUTC = manager.clock.Now().UTC().Format(time.RFC3339Nano)
	}
	manager.mu.Unlock()
	if changed {
		kind := "heartbeat_fresh"
		if !fresh {
			kind = "heartbeat_stale"
		}
		manager.record(Event{Kind: kind, Child: runtime.spec.Name})
		manager.refreshReadiness()
	}
}

func (manager *Manager) continuousReadyDuration(runtime *childRuntime, now time.Time) time.Duration {
	manager.mu.RLock()
	defer manager.mu.RUnlock()
	if runtime.readySince.IsZero() || now.Before(runtime.readySince) {
		return 0
	}
	return now.Sub(runtime.readySince)
}

func heartbeatState(path string, startedAt, now time.Time, stale time.Duration) (bool, *time.Duration) {
	if path == "" {
		return true, nil
	}
	info, err := os.Stat(path)
	if err != nil || info.ModTime().Before(startedAt) || info.ModTime().After(now) {
		return false, nil
	}
	age := now.Sub(info.ModTime())
	return age <= stale, &age
}

func mergedEnvironment(overrides map[string]string) []string {
	values := make(map[string]string)
	for _, item := range os.Environ() {
		key, value, found := strings.Cut(item, "=")
		if found {
			values[key] = value
		}
	}
	for key, value := range overrides {
		values[key] = value
	}
	keys := make([]string, 0, len(values))
	for key := range values {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	result := make([]string, 0, len(keys))
	for _, key := range keys {
		result = append(result, key+"="+values[key])
	}
	return result
}

func exitCode(err error) int {
	if err == nil {
		return 0
	}
	var exitError *exec.ExitError
	if errors.As(err, &exitError) {
		return exitError.ExitCode()
	}
	return -1
}

func (manager *Manager) record(event Event) {
	now := manager.clock.Now()
	manager.mu.RLock()
	startedAt := manager.startedAt
	manager.mu.RUnlock()
	if !startedAt.IsZero() {
		elapsed := now.Sub(startedAt)
		if elapsed < 0 {
			manager.mu.Lock()
			manager.persistenceHealthy = false
			manager.mu.Unlock()
			manager.logger.Error("monotonic clock regressed", "event", event.Kind)
			elapsed = 0
		}
		event.SteadyWallNS = elapsed.Nanoseconds()
	}
	recorded, err := manager.store.append(now, event)
	if err != nil {
		manager.mu.Lock()
		manager.persistenceHealthy = false
		manager.mu.Unlock()
		manager.logger.Error("event persistence failed", "event", event.Kind, "error", err)
	} else if manager.store.isSaturated() {
		manager.mu.Lock()
		manager.persistenceHealthy = false
		manager.mu.Unlock()
		manager.logger.Error("event persistence saturated", "event", recorded.Kind, "sequence", recorded.Sequence)
	} else {
		manager.logger.Info("supervisor transition",
			"event", recorded.Kind,
			"child", recorded.Child,
			"pid", recorded.PID,
			"pgid", recorded.PGID,
			"restart_attempt", recorded.RestartAttempt,
		)
	}
	if err := writeSnapshotAtomic(manager.cfg.StateDirectory, manager.Snapshot()); err != nil {
		manager.mu.Lock()
		manager.persistenceHealthy = false
		manager.mu.Unlock()
		manager.logger.Error("status persistence failed", "error", err)
	}
}

func (manager *Manager) refreshReadiness() {
	ready := manager.isReady()
	manager.mu.Lock()
	changed := ready != manager.lastReady
	manager.lastReady = ready
	manager.mu.Unlock()
	if changed {
		manager.record(Event{Kind: "readiness_changed", Ready: &ready})
	}
}

func (manager *Manager) isReady() bool {
	manager.mu.RLock()
	ready := !manager.shuttingDown && manager.persistenceHealthy
	for _, runtime := range manager.children {
		if runtime.spec.Required && (!runtime.state.Running || !runtime.state.HeartbeatFresh || runtime.state.CircuitOpen) {
			ready = false
			break
		}
	}
	manager.mu.RUnlock()
	_, dropped := manager.store.counts()
	return ready && dropped == 0
}

// Snapshot returns a deterministic deep value for HTTP and persistence.
func (manager *Manager) Snapshot() Snapshot {
	manager.mu.RLock()
	children := make([]ChildSnapshot, 0, len(manager.children))
	for _, runtime := range manager.children {
		children = append(children, runtime.state)
	}
	shuttingDown := manager.shuttingDown
	persistenceHealthy := manager.persistenceHealthy
	manager.mu.RUnlock()
	sort.Slice(children, func(left, right int) bool { return children[left].Name < children[right].Name })
	entries, dropped := manager.store.counts()
	ready := persistenceHealthy && dropped == 0 && !shuttingDown
	for _, child := range children {
		if child.Required && (!child.Running || !child.HeartbeatFresh || child.CircuitOpen) {
			ready = false
		}
	}
	return Snapshot{
		SchemaVersion:      1,
		SupervisorUTC:      manager.clock.Now().UTC().Format(time.RFC3339Nano),
		Healthy:            true,
		Ready:              ready,
		PersistenceHealthy: persistenceHealthy,
		ShuttingDown:       shuttingDown,
		EventCount:         entries,
		DroppedEvents:      dropped,
		Children:           children,
	}
}

// Metrics returns fixed-cardinality Prometheus-compatible text.
func (manager *Manager) Metrics() string {
	snapshot := manager.Snapshot()
	ready := 0
	if snapshot.Ready {
		ready = 1
	}
	var builder strings.Builder
	fmt.Fprintf(&builder, "# HELP robotest_supervisor_ready Supervisor readiness.\n")
	fmt.Fprintf(&builder, "# TYPE robotest_supervisor_ready gauge\n")
	fmt.Fprintf(&builder, "robotest_supervisor_ready %d\n", ready)
	persistenceHealthy := 0
	if snapshot.PersistenceHealthy {
		persistenceHealthy = 1
	}
	fmt.Fprintf(&builder, "# HELP robotest_supervisor_persistence_healthy Event and status persistence health.\n")
	fmt.Fprintf(&builder, "# TYPE robotest_supervisor_persistence_healthy gauge\n")
	fmt.Fprintf(&builder, "robotest_supervisor_persistence_healthy %d\n", persistenceHealthy)
	fmt.Fprintf(&builder, "# HELP robotest_supervisor_events_dropped_total Events omitted after a configured cap.\n")
	fmt.Fprintf(&builder, "# TYPE robotest_supervisor_events_dropped_total counter\n")
	fmt.Fprintf(&builder, "robotest_supervisor_events_dropped_total %d\n", snapshot.DroppedEvents)
	fmt.Fprintf(&builder, "# HELP robotest_supervisor_child_running Child process running state.\n")
	fmt.Fprintf(&builder, "# TYPE robotest_supervisor_child_running gauge\n")
	fmt.Fprintf(&builder, "# HELP robotest_supervisor_child_heartbeat_fresh Child heartbeat freshness.\n")
	fmt.Fprintf(&builder, "# TYPE robotest_supervisor_child_heartbeat_fresh gauge\n")
	fmt.Fprintf(&builder, "# HELP robotest_supervisor_child_restart_total Completed child restarts.\n")
	fmt.Fprintf(&builder, "# TYPE robotest_supervisor_child_restart_total counter\n")
	fmt.Fprintf(&builder, "# HELP robotest_supervisor_child_circuit_open Child restart circuit state.\n")
	fmt.Fprintf(&builder, "# TYPE robotest_supervisor_child_circuit_open gauge\n")
	for _, child := range snapshot.Children {
		running := 0
		fresh := 0
		circuit := 0
		if child.Running {
			running = 1
		}
		if child.HeartbeatFresh {
			fresh = 1
		}
		if child.CircuitOpen {
			circuit = 1
		}
		fmt.Fprintf(&builder, "robotest_supervisor_child_running{name=%q} %d\n", child.Name, running)
		fmt.Fprintf(&builder, "robotest_supervisor_child_heartbeat_fresh{name=%q} %d\n", child.Name, fresh)
		fmt.Fprintf(&builder, "robotest_supervisor_child_restart_total{name=%q} %d\n", child.Name, child.RestartCount)
		fmt.Fprintf(&builder, "robotest_supervisor_child_circuit_open{name=%q} %d\n", child.Name, circuit)
	}
	return builder.String()
}
