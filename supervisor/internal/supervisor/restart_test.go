// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package supervisor

import (
	"testing"
	"time"

	"github.com/hasanahmed/robotest-lab/supervisor/internal/config"
)

func restartPolicy() config.RestartPolicy {
	return config.RestartPolicy{
		InitialBackoffMS: 1000,
		MaximumBackoffMS: 8000,
		MaximumAttempts:  4,
		WindowMS:         60000,
		StableResetMS:    60000,
	}
}

func TestRestartBackoffAndExhaustion(t *testing.T) {
	t.Parallel()
	tracker := newRestartTracker(restartPolicy())
	clock := newFakeClock(time.Unix(100, 0))
	for index, want := range []time.Duration{time.Second, 2 * time.Second, 4 * time.Second, 8 * time.Second} {
		if index != 0 {
			clock.Advance(time.Second)
		}
		delay, attempt, exhausted := tracker.RecordFailure(clock.Now(), time.Second)
		if delay != want || attempt != index+1 || exhausted {
			t.Fatalf("failure %d = (%s,%d,%t), want (%s,%d,false)", index, delay, attempt, exhausted, want, index+1)
		}
	}
	clock.Advance(2 * time.Second)
	if _, attempt, exhausted := tracker.RecordFailure(clock.Now(), time.Second); !exhausted || attempt != 5 {
		t.Fatalf("fifth failure = attempt %d exhausted %t", attempt, exhausted)
	}
}

func TestRestartRollingWindowExpiresOldFailures(t *testing.T) {
	t.Parallel()
	tracker := newRestartTracker(restartPolicy())
	now := time.Unix(100, 0)
	tracker.RecordFailure(now, time.Second)
	delay, attempt, exhausted := tracker.RecordFailure(now.Add(61*time.Second), time.Second)
	if delay != time.Second || attempt != 1 || exhausted {
		t.Fatalf("rolling-window reset = (%s,%d,%t)", delay, attempt, exhausted)
	}

}

func TestRestartStableResetRequiresContinuousReadiness(t *testing.T) {
	t.Parallel()
	policy := restartPolicy()
	// Isolate the stable-reset rule from the independent rolling-window rule.
	policy.WindowMS = 10 * policy.StableResetMS
	tracker := newRestartTracker(policy)
	now := time.Unix(100, 0)
	tracker.RecordFailure(now, 0)
	delay, attempt, exhausted := tracker.RecordFailure(now.Add(time.Second), 59*time.Second)
	if delay != 2*time.Second || attempt != 2 || exhausted {
		t.Fatalf("unready/short-ready run reset history: (%s,%d,%t)", delay, attempt, exhausted)
	}
	delay, attempt, exhausted = tracker.RecordFailure(now.Add(2*time.Second), 60*time.Second)
	if delay != time.Second || attempt != 1 || exhausted {
		t.Fatalf("continuous-ready reset = (%s,%d,%t)", delay, attempt, exhausted)
	}
}
