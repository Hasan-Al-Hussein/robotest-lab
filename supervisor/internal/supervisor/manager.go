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

	"github.com/hasanahmed/robotest-lab/supervisor/internal/config"
)

type childRuntime struct {
	spec                       config.Child
	state                      ChildSnapshot
	tracker                    *RestartTracker
	heartbeatObservedThisStart bool
	readySince                 time.Time
}

// Manager supervises one fixed, validated set of child process groups.
type Manager struct {
	cfg    config.Config
	clock  Clock
	logger *slog.Logger
	store  *eventStore
	lock   *stateLock

	mu                 sync.RWMutex
	children           map[string]*childRuntime
	shuttingDown       bool
	persistenceHealthy bool
	lastReady          bool
	startedAt          time.Time
	runStarted         bool
	runActive          bool
	closed             bool
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
		cfg:                cfg,
		clock:              realClock{},
		logger:             logger,
		store:              store,
		lock:               lock,
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
	manager.mu.Lock()
	manager.shuttingDown = true
	manager.mu.Unlock()
	manager.record(Event{Kind: "shutdown_requested"})

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
		return errors.New("supervisor shutdown exceeded configured timeout")
	}
	manager.record(Event{Kind: "supervisor_stopped"})
	manager.mu.Lock()
	manager.runActive = false
	manager.mu.Unlock()
	return nil
}

func (manager *Manager) manageChild(ctx context.Context, runtime *childRuntime) {
	firstStart := true
	for {
		if ctx.Err() != nil {
			return
		}
		startedAt, pgid, wait, startErr := manager.startChild(runtime, !firstStart)
		firstStart = false
		if startErr != nil {
			manager.markFailure(runtime, "start_failed", nil, startErr, true)
			if !manager.waitForRestart(ctx, runtime, 0) {
				return
			}
			continue
		}

		failureKind, continuousReady, stop := manager.monitorStartedChild(ctx, runtime, startedAt, pgid, wait)
		if stop {
			return
		}
		if failureKind == "" {
			return
		}
		if !manager.waitForRestart(ctx, runtime, continuousReady) {
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
) (string, time.Duration, bool) {
	// PGID is captured at start and cleaned on every return path, including a tie
	// between leader exit and cancellation where either select arm may win.
	defer manager.cleanupResidualProcessGroup(runtime.spec.Name, pgid)
	for {
		select {
		case err := <-wait:
			continuousReady := manager.continuousReadyDuration(runtime, manager.clock.Now())
			if ctx.Err() != nil {
				manager.markStopped(runtime, exitCode(err))
				return "", continuousReady, true
			}
			code := exitCode(err)
			manager.markFailure(runtime, "unexpected_exit", &code, err, true)
			return "unexpected_exit", continuousReady, false
		case <-manager.clock.After(manager.cfg.HeartbeatPoll()):
			now := manager.clock.Now()
			continuousReady := manager.continuousReadyDuration(runtime, now)
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
				manager.markFailure(runtime, heartbeatFailureKind, nil, nil, false)
				manager.stopProcessGroup(runtime, pgid, wait)
				return heartbeatFailureKind, continuousReady, false
			}
		case <-ctx.Done():
			manager.stopProcessGroup(runtime, pgid, wait)
			return "", manager.continuousReadyDuration(runtime, manager.clock.Now()), true
		}
	}
}

func (manager *Manager) startChild(runtime *childRuntime, restarted bool) (time.Time, int, <-chan error, error) {
	startedAt := manager.clock.Now()
	cmd := exec.Command(runtime.spec.Argv[0], runtime.spec.Argv[1:]...)
	cmd.Dir = runtime.spec.WorkingDirectory
	cmd.Env = mergedEnvironment(runtime.spec.Environment)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true, Pdeathsig: syscall.SIGTERM}
	if err := cmd.Start(); err != nil {
		return startedAt, 0, nil, err
	}
	pid := cmd.Process.Pid
	wait := make(chan error, 1)
	go func() {
		wait <- cmd.Wait()
		close(wait)
	}()

	manager.mu.Lock()
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
	return startedAt, pid, wait, nil
}

func (manager *Manager) stopProcessGroup(runtime *childRuntime, pgid int, wait <-chan error) {
	if pgid <= 0 {
		return
	}
	manager.record(Event{Kind: "child_stop_requested", Child: runtime.spec.Name, PGID: pgid})
	_ = syscall.Kill(-pgid, syscall.SIGTERM)
	deadline := time.Now().Add(manager.cfg.TerminationGrace())
	var waitErr error
	waitComplete := false
	for time.Now().Before(deadline) {
		if !waitComplete {
			select {
			case waitErr = <-wait:
				waitComplete = true
			default:
			}
		}
		if waitComplete && !processGroupExists(pgid) {
			manager.markStopped(runtime, exitCode(waitErr))
			return
		}
		time.Sleep(20 * time.Millisecond)
	}

	manager.record(Event{Kind: "child_kill_escalated", Child: runtime.spec.Name, PGID: pgid})
	_ = syscall.Kill(-pgid, syscall.SIGKILL)
	if !waitComplete {
		waitErr = <-wait
	}
	killDeadline := time.Now().Add(2 * time.Second)
	for processGroupExists(pgid) && time.Now().Before(killDeadline) {
		time.Sleep(20 * time.Millisecond)
	}
	if processGroupExists(pgid) {
		manager.record(Event{Kind: "child_group_cleanup_incomplete", Child: runtime.spec.Name, PGID: pgid})
	}
	manager.markStopped(runtime, exitCode(waitErr))
}

func processGroupExists(pgid int) bool {
	if pgid <= 0 {
		return false
	}
	err := syscall.Kill(-pgid, 0)
	return err == nil || errors.Is(err, syscall.EPERM)
}

func (manager *Manager) cleanupResidualProcessGroup(child string, pgid int) {
	if !processGroupExists(pgid) {
		return
	}
	manager.record(Event{Kind: "residual_process_group_detected", Child: child, PGID: pgid})
	_ = syscall.Kill(-pgid, syscall.SIGTERM)
	if waitForProcessGroupExit(pgid, manager.cfg.TerminationGrace()) {
		manager.record(Event{Kind: "residual_process_group_stopped", Child: child, PGID: pgid})
		return
	}
	manager.record(Event{Kind: "residual_process_group_kill_escalated", Child: child, PGID: pgid})
	_ = syscall.Kill(-pgid, syscall.SIGKILL)
	if !waitForProcessGroupExit(pgid, 2*time.Second) {
		manager.record(Event{Kind: "child_group_cleanup_incomplete", Child: child, PGID: pgid})
	}
}

func waitForProcessGroupExit(pgid int, timeout time.Duration) bool {
	deadline := time.Now().Add(timeout)
	for processGroupExists(pgid) && time.Now().Before(deadline) {
		time.Sleep(20 * time.Millisecond)
	}
	return !processGroupExists(pgid)
}

func (manager *Manager) waitForRestart(ctx context.Context, runtime *childRuntime, continuousReady time.Duration) bool {
	now := manager.clock.Now()
	delay, attempt, exhausted := runtime.tracker.RecordFailure(now, continuousReady)
	if exhausted {
		manager.mu.Lock()
		runtime.state.CircuitOpen = true
		manager.mu.Unlock()
		manager.record(Event{Kind: "restart_exhausted", Child: runtime.spec.Name, RestartAttempt: attempt})
		manager.refreshReadiness()
		return false
	}
	manager.record(Event{
		Kind:           "restart_scheduled",
		Child:          runtime.spec.Name,
		RestartAttempt: attempt,
		BackoffMS:      delay.Milliseconds(),
	})
	select {
	case <-manager.clock.After(delay):
		return true
	case <-ctx.Done():
		return false
	}
}

func (manager *Manager) markFailure(runtime *childRuntime, kind string, code *int, err error, processExited bool) {
	now := manager.clock.Now()
	manager.mu.Lock()
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
	_, dropped := manager.store.counts()
	manager.mu.RLock()
	defer manager.mu.RUnlock()
	if manager.shuttingDown || !manager.persistenceHealthy || dropped != 0 {
		return false
	}
	for _, runtime := range manager.children {
		if runtime.spec.Required && (!runtime.state.Running || !runtime.state.HeartbeatFresh || runtime.state.CircuitOpen) {
			return false
		}
	}
	return true
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
