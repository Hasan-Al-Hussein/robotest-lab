// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package supervisor

import (
	"sync"
	"testing"
	"time"
)

type scheduledFakeTimer struct {
	deadline time.Time
	channel  chan time.Time
}

type fakeClock struct {
	mu     sync.Mutex
	now    time.Time
	timers []scheduledFakeTimer
}

func newFakeClock(now time.Time) *fakeClock { return &fakeClock{now: now} }

func (clock *fakeClock) Now() time.Time {
	clock.mu.Lock()
	defer clock.mu.Unlock()
	return clock.now
}

func (clock *fakeClock) After(delay time.Duration) <-chan time.Time {
	clock.mu.Lock()
	defer clock.mu.Unlock()
	channel := make(chan time.Time, 1)
	clock.timers = append(clock.timers, scheduledFakeTimer{
		deadline: clock.now.Add(delay),
		channel:  channel,
	})
	return channel
}

func (clock *fakeClock) Advance(delta time.Duration) {
	clock.mu.Lock()
	clock.now = clock.now.Add(delta)
	now := clock.now
	remaining := clock.timers[:0]
	var due []chan time.Time
	for _, timer := range clock.timers {
		if timer.deadline.After(now) {
			remaining = append(remaining, timer)
		} else {
			due = append(due, timer.channel)
		}
	}
	clock.timers = remaining
	clock.mu.Unlock()
	for _, channel := range due {
		channel <- now
	}
}

func (clock *fakeClock) pendingTimers() int {
	clock.mu.Lock()
	defer clock.mu.Unlock()
	return len(clock.timers)
}

func waitForFakeTimer(t *testing.T, clock *fakeClock) {
	t.Helper()
	waitUntil(t, time.Second, func() bool { return clock.pendingTimers() > 0 })
}
