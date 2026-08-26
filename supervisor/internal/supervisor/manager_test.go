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
	startedAt, pgid, wait, err := manager.startChild(runtime, false)
	if err != nil {
		t.Fatal(err)
	}
	type monitorResult struct {
		kind            string
		continuousReady time.Duration
		stop            bool
	}
	result := make(chan monitorResult, 1)
	go func() {
		kind, continuousReady, stop := manager.monitorStartedChild(
			context.Background(), runtime, startedAt, pgid, wait,
		)
		result <- monitorResult{kind: kind, continuousReady: continuousReady, stop: stop}
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
	startedAt, pgid, wait, err := manager.startChild(runtime, false)
	if err != nil {
		t.Fatal(err)
	}
	type monitorResult struct {
		kind            string
		continuousReady time.Duration
	}
	result := make(chan monitorResult, 1)
	go func() {
		kind, continuousReady, _ := manager.monitorStartedChild(
			context.Background(), runtime, startedAt, pgid, wait,
		)
		result <- monitorResult{kind: kind, continuousReady: continuousReady}
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
		return child.LastFailureKind == "unexpected_exit"
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
	startedAt, pgid, wait, err := manager.startChild(runtime, false)
	if err != nil {
		t.Fatal(err)
	}
	waitUntil(t, 2*time.Second, func() bool {
		return len(wait) == 1 && processGroupExists(pgid)
	})
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	_, _, stop := manager.monitorStartedChild(ctx, runtime, startedAt, pgid, wait)
	if !stop {
		t.Fatal("canceled monitor requested a restart")
	}
	if processGroupExists(pgid) {
		t.Fatalf("captured process group %d survived the Wait/cancel tie", pgid)
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
