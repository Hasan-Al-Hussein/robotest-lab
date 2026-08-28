// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package supervisor

import (
	"time"

	"github.com/Hasan-Al-Hussein/robotest-lab/supervisor/internal/config"
)

// RestartTracker owns the bounded rolling failure window for one child.
type RestartTracker struct {
	policy   config.RestartPolicy
	failures []time.Time
}

func newRestartTracker(policy config.RestartPolicy) *RestartTracker {
	return &RestartTracker{policy: policy}
}

// RecordFailure returns the next delay, attempt number, and whether the circuit is exhausted.
// Only continuous readiness resets the failure history; process uptime is intentionally irrelevant.
func (tracker *RestartTracker) RecordFailure(now time.Time, continuousReady time.Duration) (time.Duration, int, bool) {
	if continuousReady >= time.Duration(tracker.policy.StableResetMS)*time.Millisecond {
		tracker.failures = tracker.failures[:0]
	}
	window := time.Duration(tracker.policy.WindowMS) * time.Millisecond
	cutoff := now.Add(-window)
	kept := tracker.failures[:0]
	for _, stamp := range tracker.failures {
		if !stamp.Before(cutoff) {
			kept = append(kept, stamp)
		}
	}
	tracker.failures = append(kept, now)
	attempt := len(tracker.failures)
	if attempt > tracker.policy.MaximumAttempts {
		return 0, attempt, true
	}
	delay := time.Duration(tracker.policy.InitialBackoffMS) * time.Millisecond
	for index := 1; index < attempt; index++ {
		delay *= 2
	}
	maximum := time.Duration(tracker.policy.MaximumBackoffMS) * time.Millisecond
	if delay > maximum {
		delay = maximum
	}
	return delay, attempt, false
}
