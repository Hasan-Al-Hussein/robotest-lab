// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package config

import (
	"bytes"
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func validConfig(root string) Config {
	return Config{
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
		Restart: RestartPolicy{
			InitialBackoffMS: 1000,
			MaximumBackoffMS: 8000,
			MaximumAttempts:  4,
			WindowMS:         60000,
			StableResetMS:    60000,
		},
		Children: []Child{{
			Name:             "stack",
			Argv:             []string{"/bin/true"},
			WorkingDirectory: root,
			Required:         true,
			HeartbeatFile:    filepath.Join(root, "state", "stack.heartbeat"),
		}},
	}
}

func TestValidConfig(t *testing.T) {
	t.Parallel()
	if err := validConfig(t.TempDir()).Validate(); err != nil {
		t.Fatalf("valid config rejected: %v", err)
	}
}

func TestConfigRejectsUnsafeAndUnboundedValues(t *testing.T) {
	t.Parallel()
	tests := map[string]func(*Config){
		"bad-schema":        func(cfg *Config) { cfg.SchemaVersion = 2 },
		"non-loopback":      func(cfg *Config) { cfg.ListenAddress = "0.0.0.0:9080" },
		"invalid-port":      func(cfg *Config) { cfg.ListenAddress = "127.0.0.1:http" },
		"relative-state":    func(cfg *Config) { cfg.StateDirectory = "state" },
		"bad-poll":          func(cfg *Config) { cfg.HeartbeatPollMS = 100 },
		"bad-stale":         func(cfg *Config) { cfg.HeartbeatStaleMS = 5000 },
		"bad-startup":       func(cfg *Config) { cfg.HeartbeatStartupTimeoutMS = 2000 },
		"bad-grace":         func(cfg *Config) { cfg.TerminationGraceMS = 100 },
		"too-many-attempts": func(cfg *Config) { cfg.Restart.MaximumAttempts = 5 },
		"duplicate-name":    func(cfg *Config) { cfg.Children = append(cfg.Children, cfg.Children[0]) },
		"relative-cwd":      func(cfg *Config) { cfg.Children[0].WorkingDirectory = "." },
		"relative-program":  func(cfg *Config) { cfg.Children[0].Argv[0] = "program" },
		"noncanonical-path": func(cfg *Config) { cfg.Children[0].Argv[0] = "/bin/../bin/true" },
		"root-state":        func(cfg *Config) { cfg.StateDirectory = "/" },
		"bad-env-name":      func(cfg *Config) { cfg.Children[0].Environment = map[string]string{"BAD NAME": "x"} },
		"invalid-name":      func(cfg *Config) { cfg.Children[0].Name = "Bad Name" },
		"nul-argument":      func(cfg *Config) { cfg.Children[0].Argv = []string{"/bin/echo", "bad\x00arg"} },
		"external-heartbeat": func(cfg *Config) {
			cfg.Children[0].HeartbeatFile = filepath.Join(filepath.Dir(cfg.StateDirectory), "outside.heartbeat")
		},
	}
	for name, mutate := range tests {
		name, mutate := name, mutate
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			cfg := validConfig(t.TempDir())
			mutate(&cfg)
			if err := cfg.Validate(); err == nil {
				t.Fatal("invalid config was accepted")
			}
		})
	}
}

func TestLoadRejectsUnknownAndTrailingJSON(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	cfg := validConfig(root)
	payload, err := json.Marshal(cfg)
	if err != nil {
		t.Fatal(err)
	}
	tests := map[string][]byte{
		"unknown":  []byte(strings.Replace(string(payload), `"children":`, `"unknown":true,"children":`, 1)),
		"trailing": append(append([]byte{}, payload...), []byte(` {}`)...),
		"duplicate": []byte(strings.Replace(
			string(payload),
			`"listen_address":"127.0.0.1:9080"`,
			`"listen_address":"127.0.0.1:9080","listen_address":"127.0.0.1:9081"`,
			1,
		)),
		"oversized": bytes.Repeat([]byte{' '}, maxConfigFileBytes+1),
	}
	for name, document := range tests {
		path := filepath.Join(root, name+".json")
		if err := os.WriteFile(path, document, 0o600); err != nil {
			t.Fatal(err)
		}
		if _, err := Load(path); err == nil {
			t.Fatalf("%s config was accepted", name)
		}
	}
}
