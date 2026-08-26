// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package supervisor

import "time"

// Clock isolates monotonic policy from wall-clock tests.
type Clock interface {
	Now() time.Time
	After(time.Duration) <-chan time.Time
}

type realClock struct{}

func (realClock) Now() time.Time                             { return time.Now() }
func (realClock) After(delay time.Duration) <-chan time.Time { return time.After(delay) }
