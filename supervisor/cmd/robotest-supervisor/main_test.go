// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package main

import (
	"io"
	"log/slog"
	"testing"
)

func TestHelpIsSuccessfulAndUnknownFlagsFail(t *testing.T) {
	t.Parallel()
	logger := slog.New(slog.NewJSONHandler(io.Discard, nil))
	if err := run([]string{"-h"}, logger); err != nil {
		t.Fatalf("help failed: %v", err)
	}
	if err := run([]string{"--not-a-real-flag"}, logger); err == nil {
		t.Fatal("unknown flag was accepted")
	}
}
