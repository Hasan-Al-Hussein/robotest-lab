// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package supervisor

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	"github.com/hasanahmed/robotest-lab/supervisor/internal/config"
)

func managerConfig(root string, argv []string, heartbeat string) config.Config {
	return config.Config{
		SchemaVersion:             1,
		ListenAddress:             "127.0.0.1:9080",
		StateDirectory:            filepath.Join(root, "state"),
		HeartbeatPollMS:           500,
		HeartbeatStaleMS:          2000,
		HeartbeatStartupTimeoutMS: 110000,
		TerminationGraceMS:        5000,
		ShutdownTimeoutMS:         15000,
		MaximumEventEntries:       4096,
		MaximumEventBytes:         8 << 20,
		Restart: config.RestartPolicy{
			InitialBackoffMS: 1000,
			MaximumBackoffMS: 8000,
			MaximumAttempts:  4,
			WindowMS:         60000,
			StableResetMS:    60000,
		},
		Children: []config.Child{{
			Name:             "stack",
			Argv:             argv,
			WorkingDirectory: root,
			Required:         true,
			HeartbeatFile:    heartbeat,
		}},
	}
}

func quietLogger() *slog.Logger { return slog.New(slog.NewJSONHandler(io.Discard, nil)) }

type fakeProcess struct {
	wait        chan error
	waitSent    bool
	groupExists bool
}

type fakeProcessSignal struct {
	pgid   int
	signal syscall.Signal
}

type fakeProcessSystem struct {
	mu              sync.Mutex
	nextPID         int
	processes       map[int]*fakeProcess
	starts          []int
	signals         []fakeProcessSignal
	keepGroupOnTERM bool
	keepGroupOnKILL bool
	reapOnTERM      bool
	reapOnKILL      bool
	termErr         error
	killErr         error
	probeErr        error
	probeStates     []bool
	startHook       func()
	startHooks      map[string]func()
	startErr        error
}

func newFakeProcessSystem() *fakeProcessSystem {
	return &fakeProcessSystem{
		nextPID:    4100,
		processes:  make(map[int]*fakeProcess),
		reapOnTERM: true,
		reapOnKILL: true,
	}
}

func (system *fakeProcessSystem) Start(spec config.Child) (startedProcess, error) {
	if system.startHook != nil {
		system.startHook()
	}
	if hook := system.startHooks[spec.Name]; hook != nil {
		hook()
	}
	if system.startErr != nil {
		return startedProcess{}, system.startErr
	}
	system.mu.Lock()
	defer system.mu.Unlock()
	system.nextPID++
	pid := system.nextPID
	process := &fakeProcess{wait: make(chan error, 1), groupExists: true}
	system.processes[pid] = process
	system.starts = append(system.starts, pid)
	return startedProcess{pid: pid, wait: process.wait}, nil
}

func (system *fakeProcessSystem) SignalProcessGroup(pgid int, signal syscall.Signal) error {
	system.mu.Lock()
	defer system.mu.Unlock()
	system.signals = append(system.signals, fakeProcessSignal{pgid: pgid, signal: signal})
	process, found := system.processes[pgid]
	if !found || !process.groupExists {
		return syscall.ESRCH
	}

	var signalErr error
	var keepGroup, reap bool
	switch signal {
	case syscall.SIGTERM:
		signalErr = system.termErr
		keepGroup = system.keepGroupOnTERM
		reap = system.reapOnTERM
	case syscall.SIGKILL:
		signalErr = system.killErr
		keepGroup = system.keepGroupOnKILL
		reap = system.reapOnKILL
	}
	if signalErr != nil {
		return signalErr
	}
	if !keepGroup {
		process.groupExists = false
	}
	if reap && !process.waitSent {
		process.waitSent = true
		process.wait <- errors.New("fake child terminated")
		close(process.wait)
	}
	return nil
}

func (system *fakeProcessSystem) ProcessGroupExists(pgid int) (bool, error) {
	system.mu.Lock()
	defer system.mu.Unlock()
	if system.probeErr != nil {
		return true, system.probeErr
	}
	if len(system.probeStates) > 0 {
		exists := system.probeStates[0]
		system.probeStates = system.probeStates[1:]
		return exists, nil
	}
	process, found := system.processes[pgid]
	return found && process.groupExists, nil
}

func (system *fakeProcessSystem) exitLeader(pgid int, waitErr error) {
	system.mu.Lock()
	defer system.mu.Unlock()
	process := system.processes[pgid]
	if process == nil || process.waitSent {
		return
	}
	process.waitSent = true
	process.wait <- waitErr
	close(process.wait)
}

func (system *fakeProcessSystem) startPIDs() []int {
	system.mu.Lock()
	defer system.mu.Unlock()
	return append([]int(nil), system.starts...)
}

func (system *fakeProcessSystem) observedSignals() []fakeProcessSignal {
	system.mu.Lock()
	defer system.mu.Unlock()
	return append([]fakeProcessSignal(nil), system.signals...)
}

type armedNowHookClock struct {
	Clock
	mu        sync.Mutex
	remaining int
	hook      func()
}

func (clock *armedNowHookClock) Now() time.Time {
	now := clock.Clock.Now()
	clock.mu.Lock()
	trigger := false
	if clock.remaining > 0 {
		clock.remaining--
		trigger = clock.remaining == 0
	}
	hook := clock.hook
	clock.mu.Unlock()
	if trigger && hook != nil {
		hook()
	}
	return now
}

func (clock *armedNowHookClock) arm() {
	clock.armNth(1)
}

func (clock *armedNowHookClock) armNth(n int) {
	clock.mu.Lock()
	clock.remaining = n
	clock.mu.Unlock()
}

func loadManagerEvents(t *testing.T, root string) []Event {
	t.Helper()
	payload, err := os.ReadFile(filepath.Join(root, "state", "events.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSpace(string(payload)), "\n")
	events := make([]Event, 0, len(lines))
	for _, line := range lines {
		var event Event
		if err := json.Unmarshal([]byte(line), &event); err != nil {
			t.Fatal(err)
		}
		events = append(events, event)
	}
	return events
}

func countManagerEvents(events []Event, kind string) int {
	count := 0
	for _, event := range events {
		if event.Kind == kind {
			count++
		}
	}
	return count
}

func managerEventSequence(root, kind string) (uint64, bool) {
	payload, err := os.ReadFile(filepath.Join(root, "state", "events.jsonl"))
	if err != nil {
		return 0, false
	}
	for _, line := range strings.Split(strings.TrimSpace(string(payload)), "\n") {
		var event Event
		if json.Unmarshal([]byte(line), &event) != nil {
			return 0, false
		}
		if event.Kind == kind {
			return event.Sequence, true
		}
	}
	return 0, false
}

func assertNoRecoveryTransitionAfterShutdown(t *testing.T, events []Event) {
	t.Helper()
	var shutdownSequence uint64
	for _, event := range events {
		if event.Kind == "shutdown_requested" {
			shutdownSequence = event.Sequence
			break
		}
	}
	if shutdownSequence == 0 {
		t.Fatal("shutdown_requested event is missing")
	}
	protected := map[string]bool{
		"child_started":     true,
		"failure_detected":  true,
		"restart_scheduled": true,
		"restart_exhausted": true,
	}
	for _, event := range events {
		if event.Sequence > shutdownSequence && protected[event.Kind] {
			t.Fatalf("%s event sequence %d followed shutdown sequence %d", event.Kind, event.Sequence, shutdownSequence)
		}
	}
}

func waitUntil(t *testing.T, timeout time.Duration, predicate func() bool) {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if predicate() {
			return
		}
		time.Sleep(20 * time.Millisecond)
	}
	t.Fatal("condition did not become true before timeout")
}

func TestManagerOwnsAndStopsWholeProcessGroup(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/sh", "-c", "sleep 60 & wait"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	waitUntil(t, 3*time.Second, func() bool { return manager.Snapshot().Ready })
	snapshot := manager.Snapshot()
	pgid := snapshot.Children[0].PGID
	if pgid <= 0 || snapshot.Children[0].PID != pgid {
		t.Fatalf("invalid process identity: %#v", snapshot.Children[0])
	}
	if err := manager.Close(); err == nil {
		t.Fatal("Close released the singleton lock while Run was active")
	}
	cancel()
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(8 * time.Second):
		t.Fatal("manager did not stop within the grace bound")
	}
	if err := syscall.Kill(-pgid, 0); !errors.Is(err, syscall.ESRCH) {
		t.Fatalf("owned process group %d survived: %v", pgid, err)
	}
	if manager.Snapshot().Ready {
		t.Fatal("manager remained ready after shutdown")
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestManagerSingletonLockRejectsSecondOwnerAndReacquiresAfterClose(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	cfg := managerConfig(root, []string{"/bin/true"}, "")
	first, err := New(cfg, quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := New(cfg, quietLogger()); !errors.Is(err, errStateDirectoryLocked) {
		t.Fatalf("second manager error = %v, want state-directory lock rejection", err)
	}
	if err := first.Close(); err != nil {
		t.Fatal(err)
	}
	if err := first.Close(); err != nil {
		t.Fatalf("idempotent Close failed: %v", err)
	}
	cancelled, cancel := context.WithCancel(context.Background())
	cancel()
	if err := first.Run(cancelled); err == nil || !strings.Contains(err.Error(), "closed") {
		t.Fatalf("Run after Close error = %v", err)
	}
	reacquired, err := New(cfg, quietLogger())
	if err != nil {
		t.Fatalf("lock was not reacquired after Close: %v", err)
	}
	if err := reacquired.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestManagerReloadsEventOverflowFailClosed(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	cfg := managerConfig(root, []string{"/bin/true"}, "")
	cfg.MaximumEventEntries = 1
	first, err := New(cfg, quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	first.record(Event{Kind: "retained"})
	first.record(Event{Kind: "dropped"})
	beforeRestart := first.Snapshot()
	if beforeRestart.PersistenceHealthy || beforeRestart.Ready || beforeRestart.DroppedEvents != 1 {
		t.Fatalf("overflow state before restart = %#v", beforeRestart)
	}
	if err := first.Close(); err != nil {
		t.Fatal(err)
	}
	reloaded, err := New(cfg, quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	afterRestart := reloaded.Snapshot()
	if afterRestart.PersistenceHealthy || afterRestart.Ready || afterRestart.DroppedEvents != 1 {
		t.Fatalf("overflow was hidden by restart: %#v", afterRestart)
	}
	if err := reloaded.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestHeartbeatFreshnessUsesPostStartTimestamp(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	path := filepath.Join(root, "heartbeat")
	old := time.Now().Add(-time.Hour)
	if err := os.WriteFile(path, []byte("stale"), 0o600); err != nil {
		t.Fatal(err)
	}
	if err := os.Chtimes(path, old, old); err != nil {
		t.Fatal(err)
	}
	fresh, age := heartbeatState(path, time.Now(), time.Now(), 2*time.Second)
	if fresh || age != nil {
		t.Fatal("pre-start heartbeat was accepted")
	}
	now := time.Now()
	if err := os.Chtimes(path, now, now); err != nil {
		t.Fatal(err)
	}
	fresh, age = heartbeatState(path, now.Add(-time.Second), now.Add(time.Second), 2*time.Second)
	if !fresh || age == nil || *age != time.Second {
		t.Fatalf("fresh heartbeat = %t, age=%v", fresh, age)
	}
	future := now.Add(time.Millisecond)
	if err := os.Chtimes(path, future, future); err != nil {
		t.Fatal(err)
	}
	if fresh, _ := heartbeatState(path, now.Add(-time.Second), now, 2*time.Second); fresh {
		t.Fatal("future heartbeat timestamp was accepted")
	}
}

func TestInitialHeartbeatUsesBoundedStartupTimeout(t *testing.T) {
	root := t.TempDir()
	missingHeartbeat := filepath.Join(root, "state", "never-created.heartbeat")
	manager, err := New(
		managerConfig(root, []string{"/bin/sleep", "60"}, missingHeartbeat),
		quietLogger(),
	)
	if err != nil {
		t.Fatal(err)
	}
	clock := newFakeClock(time.Now())
	manager.SetClock(clock)
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	waitUntil(t, time.Second, func() bool { return manager.Snapshot().Children[0].Running })
	waitForFakeTimer(t, clock)

	clock.Advance(109 * time.Second)
	waitForFakeTimer(t, clock)
	child := manager.Snapshot().Children[0]
	if !child.Running || child.HeartbeatFresh || manager.Snapshot().Ready {
		t.Fatalf("initializing child state = %#v", child)
	}

	clock.Advance(2 * time.Second)
	waitUntil(t, 3*time.Second, func() bool {
		return manager.Snapshot().Children[0].LastFailureKind == "heartbeat_startup_timeout"
	})
	cancel()
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(3 * time.Second):
		t.Fatal("manager did not stop after startup timeout")
	}
}

func TestStartupTimeoutReturnsZeroContinuousReadiness(t *testing.T) {
	root := t.TempDir()
	heartbeat := filepath.Join(root, "state", "never-created.heartbeat")
	manager, err := New(managerConfig(root, []string{"/bin/sleep", "60"}, heartbeat), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	clock := newFakeClock(time.Now())
	manager.SetClock(clock)
	runtime := manager.children["stack"]
	startedAt, pgid, wait, committed, err := manager.startChild(context.Background(), runtime, false)
	if err != nil || !committed {
		t.Fatalf("startChild() = committed %t, error %v", committed, err)
	}
	type monitorResult struct {
		kind            string
		continuousReady time.Duration
		stop            bool
	}
	result := make(chan monitorResult, 1)
	go func() {
		observed := manager.monitorStartedChild(
			context.Background(), runtime, startedAt, pgid, wait,
		)
		result <- monitorResult{
			kind:            observed.failureKind,
			continuousReady: observed.continuousReady,
			stop:            observed.stop,
		}
	}()
	waitForFakeTimer(t, clock)
	clock.Advance(111 * time.Second)
	select {
	case observed := <-result:
		if observed.kind != "heartbeat_startup_timeout" || observed.continuousReady != 0 || observed.stop {
			t.Fatalf("startup-timeout result = %#v", observed)
		}
	case <-time.After(3 * time.Second):
		t.Fatal("startup-timeout monitor did not return")
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestUnexpectedExitReturnsSixtySecondsContinuousReadiness(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/sleep", "60"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	clock := newFakeClock(time.Now())
	manager.SetClock(clock)
	runtime := manager.children["stack"]
	startedAt, pgid, wait, committed, err := manager.startChild(context.Background(), runtime, false)
	if err != nil || !committed {
		t.Fatalf("startChild() = committed %t, error %v", committed, err)
	}
	type monitorResult struct {
		kind            string
		continuousReady time.Duration
	}
	result := make(chan monitorResult, 1)
	go func() {
		observed := manager.monitorStartedChild(
			context.Background(), runtime, startedAt, pgid, wait,
		)
		result <- monitorResult{
			kind:            observed.failureKind,
			continuousReady: observed.continuousReady,
		}
	}()
	waitForFakeTimer(t, clock)
	clock.Advance(60 * time.Second)
	if err := syscall.Kill(-pgid, syscall.SIGKILL); err != nil {
		t.Fatal(err)
	}
	select {
	case observed := <-result:
		if observed.kind != "unexpected_exit" || observed.continuousReady != 60*time.Second {
			t.Fatalf("ready exit result = %#v", observed)
		}
	case <-time.After(3 * time.Second):
		t.Fatal("ready child monitor did not return")
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestUnexpectedExitClearsPublishedProcessIdentity(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/sh", "-c", "exit 7"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	waitUntil(t, time.Second, func() bool {
		child := manager.Snapshot().Children[0]
		return child.LastFailureKind == "unexpected_exit" && child.PID == 0 && child.PGID == 0
	})
	child := manager.Snapshot().Children[0]
	if child.Running || child.PID != 0 || child.PGID != 0 || child.LastExitCode == nil || *child.LastExitCode != 7 {
		t.Fatalf("unexpected-exit state retained stale identity: %#v", child)
	}
	cancel()
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("manager did not leave restart backoff after cancellation")
	}
	payload, err := os.ReadFile(filepath.Join(root, "state", "events.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	var foundFailure, foundReadyFalse bool
	for _, line := range strings.Split(strings.TrimSpace(string(payload)), "\n") {
		var event Event
		if err := json.Unmarshal([]byte(line), &event); err != nil {
			t.Fatal(err)
		}
		if event.Kind == "failure_detected" && event.FailureKind == "unexpected_exit" {
			foundFailure = true
		}
		if event.Kind == "readiness_changed" && event.Ready != nil && !*event.Ready {
			foundReadyFalse = true
		}
	}
	if !foundFailure || !foundReadyFalse {
		t.Fatalf("missing canonical recovery events: failure=%t ready_false=%t", foundFailure, foundReadyFalse)
	}
}

func TestUnexpectedLeaderExitCleansResidualProcessGroup(t *testing.T) {
	root := t.TempDir()
	manager, err := New(
		managerConfig(root, []string{"/bin/sh", "-c", "sleep 60 & exit 7"}, ""),
		quietLogger(),
	)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()

	var originalPGID int
	waitUntil(t, time.Second, func() bool {
		payload, readErr := os.ReadFile(filepath.Join(root, "state", "events.jsonl"))
		if readErr != nil {
			return false
		}
		for _, line := range strings.Split(strings.TrimSpace(string(payload)), "\n") {
			var event Event
			if json.Unmarshal([]byte(line), &event) == nil && event.Kind == "residual_process_group_detected" {
				originalPGID = event.PGID
				return true
			}
		}
		return false
	})
	waitUntil(t, 3*time.Second, func() bool { return !processGroupExists(originalPGID) })
	cancel()
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(3 * time.Second):
		t.Fatal("manager did not stop after residual-group cleanup")
	}
}

func TestCancellationTiedWithLeaderExitStillCleansCapturedProcessGroup(t *testing.T) {
	root := t.TempDir()
	manager, err := New(
		managerConfig(root, []string{"/bin/sh", "-c", "sleep 60 & exit 7"}, ""),
		quietLogger(),
	)
	if err != nil {
		t.Fatal(err)
	}
	runtime := manager.children["stack"]
	startedAt, pgid, wait, committed, err := manager.startChild(context.Background(), runtime, false)
	if err != nil || !committed {
		t.Fatalf("startChild() = committed %t, error %v", committed, err)
	}
	waitUntil(t, 2*time.Second, func() bool {
		return len(wait) == 1 && processGroupExists(pgid)
	})
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	result := manager.monitorStartedChild(ctx, runtime, startedAt, pgid, wait)
	if !result.stop {
		t.Fatal("canceled monitor requested a restart")
	}
	if processGroupExists(pgid) {
		t.Fatalf("captured process group %d survived the Wait/cancel tie", pgid)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestCancellationDuringStartFailureDoesNotPublishFailureOrRestart(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	system := newFakeProcessSystem()
	system.startHook = cancel
	system.startErr = errors.New("fake start failed after cancellation")
	manager.system = system

	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(time.Second):
		t.Fatal("Run did not honor cancellation from the start hook")
	}
	child := manager.Snapshot().Children[0]
	if child.LastFailureKind != "" || child.CircuitOpen || child.Running || child.PID != 0 || child.PGID != 0 {
		t.Fatalf("canceled start failure mutated child state: %#v", child)
	}
	if failures := manager.children["stack"].tracker.failures; len(failures) != 0 {
		t.Fatalf("canceled start failure mutated restart tracker: %v", failures)
	}
	events := loadManagerEvents(t, root)
	if countManagerEvents(events, "failure_detected") != 0 ||
		countManagerEvents(events, "restart_scheduled") != 0 ||
		countManagerEvents(events, "restart_exhausted") != 0 {
		t.Fatalf("canceled start failure published recovery events: %#v", events)
	}
	if starts := system.startPIDs(); len(starts) != 0 {
		t.Fatalf("failed start unexpectedly created a process: %v", starts)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestBlockedStartReturningAfterShutdownIsReapedWithoutLifecycleCommit(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	startEntered := make(chan struct{})
	releaseStart := make(chan struct{})
	defer func() {
		select {
		case <-releaseStart:
		default:
			close(releaseStart)
		}
	}()
	system := newFakeProcessSystem()
	system.startHook = func() {
		close(startEntered)
		<-releaseStart
	}
	manager.system = system
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	select {
	case <-startEntered:
	case <-time.After(time.Second):
		t.Fatal("fake Start did not reach its blocking hook")
	}
	cancel()
	waitUntil(t, time.Second, func() bool {
		_, found := managerEventSequence(root, "shutdown_requested")
		return found
	})
	close(releaseStart)
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(time.Second):
		t.Fatal("Run did not reap the uncommitted process returned by Start")
	}

	events := loadManagerEvents(t, root)
	if countManagerEvents(events, "child_started") != 0 ||
		countManagerEvents(events, "failure_detected") != 0 ||
		countManagerEvents(events, "restart_scheduled") != 0 ||
		countManagerEvents(events, "restart_exhausted") != 0 {
		t.Fatalf("blocked post-shutdown Start committed a lifecycle transition: %#v", events)
	}
	assertNoRecoveryTransitionAfterShutdown(t, events)
	if starts := system.startPIDs(); len(starts) != 1 {
		t.Fatalf("physical starts = %v, want one uncommitted process", starts)
	}
	signals := system.observedSignals()
	if len(signals) != 1 || signals[0].signal != syscall.SIGTERM {
		t.Fatalf("uncommitted process cleanup signals = %#v", signals)
	}
	child := manager.Snapshot().Children[0]
	if child.Running || child.PID != 0 || child.PGID != 0 || child.LastFailureKind != "" {
		t.Fatalf("uncommitted process leaked published state: %#v", child)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestCancellationFromHeartbeatClockStopsWithoutFailureOrRestart(t *testing.T) {
	root := t.TempDir()
	heartbeat := filepath.Join(root, "state", "never-created.heartbeat")
	manager, err := New(managerConfig(root, []string{"/bin/true"}, heartbeat), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	policyClock := newFakeClock(time.Now())
	ctx, cancel := context.WithCancel(context.Background())
	nowEntered := make(chan struct{})
	releaseNow := make(chan struct{})
	defer func() {
		select {
		case <-releaseNow:
		default:
			close(releaseNow)
		}
	}()
	hookedClock := &armedNowHookClock{
		Clock: policyClock,
		hook: func() {
			cancel()
			close(nowEntered)
			<-releaseNow
		},
	}
	system := newFakeProcessSystem()
	manager.SetClock(hookedClock)
	manager.system = system

	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	waitUntil(t, time.Second, func() bool { return len(system.startPIDs()) == 1 })
	waitForFakeTimer(t, policyClock)
	hookedClock.arm()
	policyClock.Advance(111 * time.Second)
	select {
	case <-nowEntered:
	case <-time.After(time.Second):
		t.Fatal("heartbeat branch did not reach the armed clock hook")
	}
	waitUntil(t, time.Second, func() bool {
		_, found := managerEventSequence(root, "shutdown_requested")
		return found
	})
	close(releaseNow)
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(time.Second):
		t.Fatal("heartbeat-triggered cancellation did not stop the manager")
	}
	child := manager.Snapshot().Children[0]
	if child.LastFailureKind != "" || child.CircuitOpen || child.Running || child.PID != 0 || child.PGID != 0 {
		t.Fatalf("heartbeat cancellation was misclassified as failure: %#v", child)
	}
	if failures := manager.children["stack"].tracker.failures; len(failures) != 0 {
		t.Fatalf("heartbeat cancellation mutated restart tracker: %v", failures)
	}
	events := loadManagerEvents(t, root)
	if countManagerEvents(events, "failure_detected") != 0 ||
		countManagerEvents(events, "restart_scheduled") != 0 ||
		countManagerEvents(events, "restart_exhausted") != 0 {
		t.Fatalf("heartbeat cancellation published recovery events: %#v", events)
	}
	if countManagerEvents(events, "child_stop_requested") != 1 ||
		countManagerEvents(events, "child_stopped") != 1 {
		t.Fatalf("heartbeat cancellation did not perform exactly one cleanup: %#v", events)
	}
	assertNoRecoveryTransitionAfterShutdown(t, events)
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestAlreadyCanceledRestartWaitDoesNotMutateTrackerOrEvents(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	runtime := manager.children["stack"]
	entriesBefore, droppedBefore := manager.store.counts()
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	restart := manager.waitForRestart(ctx, runtime, time.Minute)
	if restart.committed || restart.restart {
		t.Fatal("already-canceled restart wait requested a replacement")
	}
	if len(runtime.tracker.failures) != 0 {
		t.Fatalf("already-canceled restart wait mutated tracker: %v", runtime.tracker.failures)
	}
	entriesAfter, droppedAfter := manager.store.counts()
	if entriesAfter != entriesBefore || droppedAfter != droppedBefore {
		t.Fatalf(
			"already-canceled restart wait mutated event store: before=(%d,%d) after=(%d,%d)",
			entriesBefore,
			droppedBefore,
			entriesAfter,
			droppedAfter,
		)
	}
	if runtime.state.CircuitOpen {
		t.Fatal("already-canceled restart wait opened the restart circuit")
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestNthNowCancellationLinearizesFailureAndRestartTransitions(t *testing.T) {
	tests := []struct {
		name          string
		transition    string
		nthNow        int
		wantCommitted bool
		wantEvent     string
	}{
		{name: "failure-before-gate", transition: "failure", nthNow: 1, wantEvent: "failure_detected"},
		{
			name:          "failure-after-gate",
			transition:    "failure",
			nthNow:        2,
			wantCommitted: true,
			wantEvent:     "failure_detected",
		},
		{name: "restart-before-gate", transition: "restart", nthNow: 1, wantEvent: "restart_scheduled"},
		{
			name:          "restart-after-gate",
			transition:    "restart",
			nthNow:        2,
			wantCommitted: true,
			wantEvent:     "restart_scheduled",
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			root := t.TempDir()
			manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			clock := &armedNowHookClock{Clock: newFakeClock(time.Now()), hook: cancel}
			manager.SetClock(clock)
			shutdownDone := make(chan struct{})
			go func() {
				<-ctx.Done()
				manager.beginShutdown()
				close(shutdownDone)
			}()
			clock.armNth(test.nthNow)

			committed := false
			runtime := manager.children["stack"]
			switch test.transition {
			case "failure":
				committed = manager.markFailure(
					ctx,
					runtime,
					"test_failure",
					nil,
					errors.New("test failure"),
					true,
				)
			case "restart":
				result := manager.waitForRestart(ctx, runtime, 0)
				committed = result.committed
				if result.restart {
					t.Fatal("canceled restart transition requested a replacement")
				}
			default:
				t.Fatalf("unknown transition %q", test.transition)
			}
			select {
			case <-shutdownDone:
			case <-time.After(time.Second):
				t.Fatal("shutdown transition did not finish")
			}
			if committed != test.wantCommitted {
				t.Fatalf("committed = %t, want %t", committed, test.wantCommitted)
			}
			events := loadManagerEvents(t, root)
			count := countManagerEvents(events, test.wantEvent)
			if test.wantCommitted && count != 1 {
				t.Fatalf("committed transition event count = %d, want 1: %#v", count, events)
			}
			if !test.wantCommitted && count != 0 {
				t.Fatalf("rejected transition event count = %d, want 0: %#v", count, events)
			}
			if test.wantCommitted {
				transitionSequence, _ := managerEventSequence(root, test.wantEvent)
				shutdownSequence, _ := managerEventSequence(root, "shutdown_requested")
				if transitionSequence == 0 || transitionSequence >= shutdownSequence {
					t.Fatalf(
						"linearized sequence order = transition %d, shutdown %d",
						transitionSequence,
						shutdownSequence,
					)
				}
			}
			if test.transition == "failure" {
				wantKind := ""
				if test.wantCommitted {
					wantKind = "test_failure"
				}
				if runtime.state.LastFailureKind != wantKind {
					t.Fatalf("LastFailureKind = %q, want %q", runtime.state.LastFailureKind, wantKind)
				}
			} else {
				wantFailures := 0
				if test.wantCommitted {
					wantFailures = 1
				}
				if len(runtime.tracker.failures) != wantFailures {
					t.Fatalf("restart tracker failures = %v, want length %d", runtime.tracker.failures, wantFailures)
				}
			}
			assertNoRecoveryTransitionAfterShutdown(t, events)
			if err := manager.Close(); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestTwoChildTransitionContentionOrdersCommitBeforeShutdown(t *testing.T) {
	root := t.TempDir()
	cfg := managerConfig(root, []string{"/bin/true"}, "")
	alpha := cfg.Children[0]
	alpha.Name = "alpha"
	beta := alpha
	beta.Name = "beta"
	cfg.Children = []config.Child{alpha, beta}
	manager, err := New(cfg, quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	transitionEntered := make(chan struct{})
	releaseTransition := make(chan struct{})
	defer func() {
		select {
		case <-releaseTransition:
		default:
			close(releaseTransition)
		}
	}()
	clock := &armedNowHookClock{
		Clock: newFakeClock(time.Now()),
		hook: func() {
			close(transitionEntered)
			<-releaseTransition
		},
	}
	system := newFakeProcessSystem()
	system.startHooks = map[string]func(){"alpha": clock.arm}
	manager.SetClock(clock)
	manager.system = system
	ctx, cancel := context.WithCancel(context.Background())

	type startOutcome struct {
		pgid      int
		wait      <-chan error
		committed bool
		err       error
	}
	start := func(runtime *childRuntime) <-chan startOutcome {
		result := make(chan startOutcome, 1)
		go func() {
			_, pgid, wait, committed, err := manager.startChild(ctx, runtime, false)
			result <- startOutcome{pgid: pgid, wait: wait, committed: committed, err: err}
		}()
		return result
	}
	alphaResult := start(manager.children["alpha"])
	select {
	case <-transitionEntered:
	case <-time.After(time.Second):
		t.Fatal("alpha transition did not block inside child_started persistence")
	}
	betaResult := start(manager.children["beta"])
	waitUntil(t, time.Second, func() bool { return len(system.startPIDs()) == 2 })
	shutdownDone := make(chan struct{})
	go func() {
		<-ctx.Done()
		manager.beginShutdown()
		close(shutdownDone)
	}()
	cancel()
	close(releaseTransition)

	var alphaOutcome, betaOutcome startOutcome
	select {
	case alphaOutcome = <-alphaResult:
	case <-time.After(time.Second):
		t.Fatal("alpha transition did not finish")
	}
	select {
	case betaOutcome = <-betaResult:
	case <-time.After(time.Second):
		t.Fatal("beta transition did not finish")
	}
	select {
	case <-shutdownDone:
	case <-time.After(time.Second):
		t.Fatal("shutdown transition did not finish after child contention")
	}
	if alphaOutcome.err != nil || !alphaOutcome.committed {
		t.Fatalf("alpha start outcome = %#v, want committed", alphaOutcome)
	}
	if betaOutcome.err != nil || betaOutcome.committed {
		t.Fatalf("beta start outcome = %#v, want rejected", betaOutcome)
	}

	manager.stopStartedChild(manager.children["alpha"], alphaOutcome.pgid, alphaOutcome.wait, 0)
	manager.cleanupUncommittedStartedChild(manager.children["beta"], betaOutcome.pgid, betaOutcome.wait)
	events := loadManagerEvents(t, root)
	if countManagerEvents(events, "child_started") != 1 {
		t.Fatalf("child_started count = %d, want 1: %#v", countManagerEvents(events, "child_started"), events)
	}
	var alphaSequence, shutdownSequence uint64
	for _, event := range events {
		switch {
		case event.Kind == "child_started" && event.Child == "alpha":
			alphaSequence = event.Sequence
		case event.Kind == "child_started" && event.Child == "beta":
			t.Fatalf("beta committed after cancellation: %#v", event)
		case event.Kind == "shutdown_requested":
			shutdownSequence = event.Sequence
		}
	}
	if alphaSequence == 0 || shutdownSequence == 0 || alphaSequence >= shutdownSequence {
		t.Fatalf("contention order = alpha %d, shutdown %d", alphaSequence, shutdownSequence)
	}
	assertNoRecoveryTransitionAfterShutdown(t, events)
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestStaleHeartbeatStopsOldGroupThenRestartsOnceAfterFrozenBackoff(t *testing.T) {
	root := t.TempDir()
	heartbeat := filepath.Join(root, "state", "stack.heartbeat")
	manager, err := New(managerConfig(root, []string{"/bin/true"}, heartbeat), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	clock := newFakeClock(time.Now().Add(-time.Minute).Truncate(time.Millisecond))
	system := newFakeProcessSystem()
	manager.SetClock(clock)
	manager.system = system

	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	waitUntil(t, time.Second, func() bool { return len(system.startPIDs()) == 1 })
	firstPID := system.startPIDs()[0]
	if err := os.WriteFile(heartbeat, []byte("fresh\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	firstHeartbeat := clock.Now()
	if err := os.Chtimes(heartbeat, firstHeartbeat, firstHeartbeat); err != nil {
		t.Fatal(err)
	}
	waitForFakeTimer(t, clock)
	clock.Advance(500 * time.Millisecond)
	waitUntil(t, time.Second, func() bool { return manager.Snapshot().Ready })

	waitForFakeTimer(t, clock)
	clock.Advance(2500 * time.Millisecond)
	waitUntil(t, time.Second, func() bool {
		child := manager.Snapshot().Children[0]
		return child.LastFailureKind == "heartbeat_stale" && child.PID == 0 && child.PGID == 0
	})
	waitForFakeTimer(t, clock)
	if got := system.startPIDs(); len(got) != 1 || got[0] != firstPID {
		t.Fatalf("replacement started before backoff: %v", got)
	}
	clock.Advance(999 * time.Millisecond)
	if got := system.startPIDs(); len(got) != 1 {
		t.Fatalf("replacement started before 1000ms elapsed: %v", got)
	}
	clock.Advance(time.Millisecond)
	waitUntil(t, time.Second, func() bool { return len(system.startPIDs()) == 2 })
	secondPID := system.startPIDs()[1]
	if secondPID == firstPID {
		t.Fatal("replacement reused the old fake process identity")
	}
	secondHeartbeat := clock.Now()
	if err := os.Chtimes(heartbeat, secondHeartbeat, secondHeartbeat); err != nil {
		t.Fatal(err)
	}
	waitForFakeTimer(t, clock)
	clock.Advance(500 * time.Millisecond)
	waitUntil(t, time.Second, func() bool {
		snapshot := manager.Snapshot()
		return snapshot.Ready && snapshot.Children[0].RestartCount == 1
	})

	cancel()
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(time.Second):
		t.Fatal("manager did not stop after recovered replacement")
	}
	events := loadManagerEvents(t, root)
	if countManagerEvents(events, "failure_detected") != 1 ||
		countManagerEvents(events, "restart_scheduled") != 1 ||
		countManagerEvents(events, "child_started") != 2 {
		t.Fatalf("unexpected recovery event counts: %#v", events)
	}
	var restart Event
	for _, event := range events {
		if event.Kind == "restart_scheduled" {
			restart = event
		}
	}
	if restart.RestartAttempt != 1 || restart.BackoffMS != 1000 {
		t.Fatalf("restart schedule = %#v, want attempt 1 after 1000ms", restart)
	}
	signals := system.observedSignals()
	if len(signals) != 2 || signals[0] != (fakeProcessSignal{pgid: firstPID, signal: syscall.SIGTERM}) ||
		signals[1] != (fakeProcessSignal{pgid: secondPID, signal: syscall.SIGTERM}) {
		t.Fatalf("process-group stop signals = %#v", signals)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestCleanupIncompleteOpensStickyCircuitAndNeverRestarts(t *testing.T) {
	root := t.TempDir()
	heartbeat := filepath.Join(root, "state", "never-created.heartbeat")
	manager, err := New(managerConfig(root, []string{"/bin/true"}, heartbeat), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	clock := newFakeClock(time.Now())
	system := newFakeProcessSystem()
	system.keepGroupOnTERM = true
	system.keepGroupOnKILL = true
	manager.SetClock(clock)
	manager.system = system
	manager.cleanup = cleanupPolicy{
		terminationGrace: 5 * time.Millisecond,
		killGrace:        5 * time.Millisecond,
		pollInterval:     time.Millisecond,
	}

	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- manager.Run(ctx) }()
	waitUntil(t, time.Second, func() bool { return len(system.startPIDs()) == 1 })
	originalPGID := system.startPIDs()[0]
	waitForFakeTimer(t, clock)
	clock.Advance(111 * time.Second)
	waitUntil(t, time.Second, func() bool {
		child := manager.Snapshot().Children[0]
		return child.CircuitOpen && child.LastFailureKind == cleanupFailureKind
	})
	child := manager.Snapshot().Children[0]
	if child.Running || child.PID != 0 || child.PGID != originalPGID || manager.Snapshot().Ready {
		t.Fatalf("cleanup-incomplete snapshot = %#v", child)
	}
	clock.Advance(time.Hour)
	if starts := system.startPIDs(); len(starts) != 1 {
		t.Fatalf("cleanup-incomplete child was replaced: %v", starts)
	}
	select {
	case err := <-done:
		t.Fatalf("Run returned before external cancellation: %v", err)
	default:
	}
	events := loadManagerEvents(t, root)
	if countManagerEvents(events, "failure_detected") != 1 ||
		countManagerEvents(events, "restart_scheduled") != 0 ||
		countManagerEvents(events, "child_group_cleanup_incomplete") != 1 {
		t.Fatalf("cleanup failure event counts are not fail-closed: %#v", events)
	}

	cancel()
	select {
	case err := <-done:
		if !errors.Is(err, errProcessGroupCleanupIncomplete) {
			t.Fatalf("Run error = %v, want cleanup sentinel", err)
		}
	case <-time.After(time.Second):
		t.Fatal("Run did not return the cleanup sentinel after cancellation")
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestCleanupBoundsMissingLeaderWaitAfterGroupDisappears(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	system := newFakeProcessSystem()
	system.reapOnTERM = false
	system.reapOnKILL = false
	manager.system = system
	manager.cleanup = cleanupPolicy{
		terminationGrace: 8 * time.Millisecond,
		killGrace:        8 * time.Millisecond,
		pollInterval:     time.Millisecond,
	}
	runtime := manager.children["stack"]
	_, pgid, wait, committed, err := manager.startChild(context.Background(), runtime, false)
	if err != nil || !committed {
		t.Fatalf("startChild() = committed %t, error %v", committed, err)
	}

	started := time.Now()
	cleanup := manager.cleanupProcessGroup(
		context.Background(), runtime.spec.Name, pgid, wait, false, nil, cleanupRequested,
	)
	elapsed := time.Since(started)
	if cleanup.complete() || cleanup.leaderReaped || !cleanup.groupEmpty || cleanup.err == nil {
		t.Fatalf("missing-wait cleanup result = %#v", cleanup)
	}
	if elapsed > 250*time.Millisecond {
		t.Fatalf("missing leader wait exceeded its real-time bound: %s", elapsed)
	}
	signals := system.observedSignals()
	if len(signals) != 2 || signals[0].signal != syscall.SIGTERM || signals[1].signal != syscall.SIGKILL {
		t.Fatalf("bounded cleanup signals = %#v", signals)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestCleanupEscalatesToKillThenProvesReapAndEmpty(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	system := newFakeProcessSystem()
	system.keepGroupOnTERM = true
	system.reapOnTERM = false
	manager.system = system
	manager.cleanup = cleanupPolicy{
		terminationGrace: 5 * time.Millisecond,
		killGrace:        5 * time.Millisecond,
		pollInterval:     time.Millisecond,
	}
	runtime := manager.children["stack"]
	_, pgid, wait, committed, err := manager.startChild(context.Background(), runtime, false)
	if err != nil || !committed {
		t.Fatalf("startChild() = committed %t, error %v", committed, err)
	}

	cleanup := manager.cleanupProcessGroup(
		context.Background(), runtime.spec.Name, pgid, wait, false, nil, cleanupRequested,
	)
	if !cleanup.complete() {
		t.Fatalf("SIGKILL cleanup did not prove leader and group exit: %#v", cleanup)
	}
	signals := system.observedSignals()
	if len(signals) != 2 || signals[0].signal != syscall.SIGTERM || signals[1].signal != syscall.SIGKILL {
		t.Fatalf("escalated cleanup signals = %#v", signals)
	}
	manager.markStopped(runtime, exitCode(cleanup.waitErr))
	child := manager.Snapshot().Children[0]
	if child.Running || child.PID != 0 || child.PGID != 0 || child.CircuitOpen {
		t.Fatalf("successfully escalated cleanup state = %#v", child)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestCleanupSignalFailureKeepsCircuitOpenAfterResourcesDisappear(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	system := newFakeProcessSystem()
	system.termErr = syscall.EPERM
	manager.system = system
	manager.cleanup = cleanupPolicy{
		terminationGrace: 5 * time.Millisecond,
		killGrace:        5 * time.Millisecond,
		pollInterval:     time.Millisecond,
	}
	runtime := manager.children["stack"]
	_, pgid, wait, committed, err := manager.startChild(context.Background(), runtime, false)
	if err != nil || !committed {
		t.Fatalf("startChild() = committed %t, error %v", committed, err)
	}

	cleanup := manager.cleanupProcessGroup(
		context.Background(), runtime.spec.Name, pgid, wait, false, nil, cleanupRequested,
	)
	if cleanup.complete() || !cleanup.leaderReaped || !cleanup.groupEmpty || !errors.Is(cleanup.err, syscall.EPERM) {
		t.Fatalf("signal-failure cleanup result = %#v", cleanup)
	}
	manager.markCleanupIncomplete(runtime, pgid, cleanup)
	child := manager.Snapshot().Children[0]
	if !child.CircuitOpen || child.LastFailureKind != cleanupFailureKind || child.PID != 0 || child.PGID != pgid {
		t.Fatalf("signal-failure circuit state = %#v", child)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestWaitEmptyCancellationPerformsFinalKernelProbe(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	system := newFakeProcessSystem()
	system.probeStates = []bool{true, false}
	manager.system = system
	manager.cleanup.pollInterval = time.Second
	ctx, cancel := context.WithCancel(context.Background())
	cancel()

	started := time.Now()
	result := manager.waitEmpty(
		ctx,
		9001,
		nil,
		cleanupResult{leaderReaped: true},
		time.Second,
	)
	if !result.leaderReaped || !result.groupEmpty || !errors.Is(result.err, context.Canceled) {
		t.Fatalf("canceled wait result = %#v", result)
	}
	if elapsed := time.Since(started); elapsed > 100*time.Millisecond {
		t.Fatalf("canceled wait ignored cancellation for %s", elapsed)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestResidualCleanupFailureRetainsGroupAndOpensCircuit(t *testing.T) {
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	system := newFakeProcessSystem()
	system.keepGroupOnTERM = true
	system.keepGroupOnKILL = true
	manager.system = system
	manager.cleanup = cleanupPolicy{
		terminationGrace: 5 * time.Millisecond,
		killGrace:        5 * time.Millisecond,
		pollInterval:     time.Millisecond,
	}
	runtime := manager.children["stack"]
	startedAt, pgid, wait, committed, err := manager.startChild(context.Background(), runtime, false)
	if err != nil || !committed {
		t.Fatalf("startChild() = committed %t, error %v", committed, err)
	}
	system.exitLeader(pgid, errors.New("fake leader exited"))
	result := manager.monitorStartedChild(context.Background(), runtime, startedAt, pgid, wait)
	if !result.stop || result.failureKind != "" || result.cleanupErr == nil {
		t.Fatalf("residual cleanup monitor result = %#v", result)
	}
	child := manager.Snapshot().Children[0]
	if !child.CircuitOpen || child.LastFailureKind != cleanupFailureKind ||
		child.PID != 0 || child.PGID != pgid || child.Running {
		t.Fatalf("residual cleanup state = %#v", child)
	}
	if !errors.Is(manager.terminalError(), errProcessGroupCleanupIncomplete) {
		t.Fatalf("terminal error = %v, want cleanup sentinel", manager.terminalError())
	}
	events := loadManagerEvents(t, root)
	if countManagerEvents(events, "failure_detected") != 1 ||
		countManagerEvents(events, "residual_process_group_detected") != 1 ||
		countManagerEvents(events, "residual_process_group_kill_escalated") != 1 ||
		countManagerEvents(events, "child_group_cleanup_incomplete") != 1 ||
		countManagerEvents(events, "restart_scheduled") != 0 {
		t.Fatalf("residual cleanup events = %#v", events)
	}
	if err := manager.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestEnvironmentMergeIsSortedAndOverrides(t *testing.T) {
	t.Setenv("ROBOTEST_TEST_ENV", "old")
	environment := mergedEnvironment(map[string]string{"ROBOTEST_TEST_ENV": "new", "AAA_ROBOTEST": "first"})
	joined := strings.Join(environment, "\n")
	if !strings.Contains(joined, "ROBOTEST_TEST_ENV=new") || strings.Contains(joined, "ROBOTEST_TEST_ENV=old") {
		t.Fatalf("override failed: %s", joined)
	}
	for index := 1; index < len(environment); index++ {
		if environment[index-1] > environment[index] {
			t.Fatal("environment is not sorted")
		}
	}
}

func TestMetricsUseOnlyFixedFamiliesAndConfiguredChildLabels(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	manager, err := New(managerConfig(root, []string{"/bin/true"}, ""), quietLogger())
	if err != nil {
		t.Fatal(err)
	}
	metrics := manager.Metrics()
	for _, family := range []string{
		"robotest_supervisor_ready",
		"robotest_supervisor_persistence_healthy",
		"robotest_supervisor_events_dropped_total",
		"robotest_supervisor_child_running",
		"robotest_supervisor_child_heartbeat_fresh",
		"robotest_supervisor_child_restart_total",
		"robotest_supervisor_child_circuit_open",
	} {
		if !strings.Contains(metrics, family) {
			t.Fatalf("missing metric family %q", family)
		}
	}
	if strings.Contains(metrics, "pid=") || strings.Contains(metrics, "failure=") {
		t.Fatalf("metrics expose an unbounded label: %s", metrics)
	}
	if count := strings.Count(metrics, `{name="stack"}`); count != 4 {
		t.Fatalf("child metric label count = %d, want 4", count)
	}
}
