// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

// Package config loads and validates the bounded supervisor configuration.
package config

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"time"
)

const (
	maxChildren                    = 32
	maxArguments                   = 64
	maxEnvironment                 = 64
	maxStringBytes                 = 4096
	maxEventEntries                = 16384
	maxEventFileBytes              = 8 << 20
	maxConfigFileBytes             = 1 << 20
	packagedStackExecutable        = "/usr/libexec/robotest-supervisor/start-robotest-stack"
	runtimeStateDirectoryEnv       = "ROBOTEST_RUNTIME_STATE_DIRECTORY"
	packagedStackHeartbeatFileName = "robotest-stack.heartbeat"
)

var childNamePattern = regexp.MustCompile(`^[a-z][a-z0-9_-]{0,63}$`)
var environmentNamePattern = regexp.MustCompile(`^[A-Za-z_][A-Za-z0-9_]*$`)

// RestartPolicy controls bounded restart and circuit-breaker behavior.
type RestartPolicy struct {
	InitialBackoffMS int `json:"initial_backoff_ms"`
	MaximumBackoffMS int `json:"maximum_backoff_ms"`
	MaximumAttempts  int `json:"maximum_attempts"`
	WindowMS         int `json:"window_ms"`
	StableResetMS    int `json:"stable_reset_ms"`
}

// Child describes one directly owned process group.
type Child struct {
	Name             string            `json:"name"`
	Argv             []string          `json:"argv"`
	WorkingDirectory string            `json:"working_directory"`
	Environment      map[string]string `json:"environment,omitempty"`
	Required         bool              `json:"required"`
	HeartbeatFile    string            `json:"heartbeat_file,omitempty"`
}

// Config is the complete supervisor configuration.
type Config struct {
	SchemaVersion             int           `json:"schema_version"`
	ListenAddress             string        `json:"listen_address"`
	StateDirectory            string        `json:"state_directory"`
	HeartbeatPollMS           int           `json:"heartbeat_poll_ms"`
	HeartbeatStaleMS          int           `json:"heartbeat_stale_ms"`
	HeartbeatStartupTimeoutMS int           `json:"heartbeat_startup_timeout_ms"`
	TerminationGraceMS        int           `json:"termination_grace_ms"`
	ShutdownTimeoutMS         int           `json:"shutdown_timeout_ms"`
	MaximumEventEntries       int           `json:"maximum_event_entries"`
	MaximumEventBytes         int64         `json:"maximum_event_bytes"`
	Restart                   RestartPolicy `json:"restart"`
	Children                  []Child       `json:"children"`
}

// Load reads exactly one strict JSON document and validates all policy bounds.
func Load(path string) (Config, error) {
	file, err := os.Open(path)
	if err != nil {
		return Config{}, fmt.Errorf("open config: %w", err)
	}
	defer file.Close()

	payload, err := io.ReadAll(io.LimitReader(file, maxConfigFileBytes+1))
	if err != nil {
		return Config{}, fmt.Errorf("read config: %w", err)
	}
	if len(payload) > maxConfigFileBytes {
		return Config{}, fmt.Errorf("config exceeds %d bytes", maxConfigFileBytes)
	}
	if err := rejectDuplicateKeys(payload); err != nil {
		return Config{}, err
	}
	decoder := json.NewDecoder(bytes.NewReader(payload))
	decoder.DisallowUnknownFields()
	var cfg Config
	if err := decoder.Decode(&cfg); err != nil {
		return Config{}, fmt.Errorf("decode config: %w", err)
	}
	if err := requireEOF(decoder); err != nil {
		return Config{}, err
	}
	if err := cfg.Validate(); err != nil {
		return Config{}, err
	}
	return cfg, nil
}

func rejectDuplicateKeys(payload []byte) error {
	decoder := json.NewDecoder(bytes.NewReader(payload))
	if err := walkJSONValue(decoder); err != nil {
		return fmt.Errorf("validate config keys: %w", err)
	}
	if _, err := decoder.Token(); !errors.Is(err, io.EOF) {
		if err == nil {
			return errors.New("config contains more than one JSON value")
		}
		return fmt.Errorf("validate trailing config data: %w", err)
	}
	return nil
}

func walkJSONValue(decoder *json.Decoder) error {
	token, err := decoder.Token()
	if err != nil {
		return err
	}
	delimiter, isDelimiter := token.(json.Delim)
	if !isDelimiter {
		return nil
	}
	switch delimiter {
	case '{':
		keys := make(map[string]struct{})
		for decoder.More() {
			keyToken, err := decoder.Token()
			if err != nil {
				return err
			}
			key, ok := keyToken.(string)
			if !ok {
				return errors.New("object key is not a string")
			}
			if _, exists := keys[key]; exists {
				return fmt.Errorf("duplicate object key %q", key)
			}
			keys[key] = struct{}{}
			if err := walkJSONValue(decoder); err != nil {
				return err
			}
		}
		closing, err := decoder.Token()
		if err != nil || closing != json.Delim('}') {
			return errors.New("object is not terminated")
		}
	case '[':
		for decoder.More() {
			if err := walkJSONValue(decoder); err != nil {
				return err
			}
		}
		closing, err := decoder.Token()
		if err != nil || closing != json.Delim(']') {
			return errors.New("array is not terminated")
		}
	default:
		return fmt.Errorf("unexpected JSON delimiter %q", delimiter)
	}
	return nil
}

func requireEOF(decoder *json.Decoder) error {
	var trailing any
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		if err == nil {
			return errors.New("config contains more than one JSON value")
		}
		return fmt.Errorf("decode trailing config data: %w", err)
	}
	return nil
}

// Validate rejects unsafe, ambiguous, or unbounded settings.
func (cfg Config) Validate() error {
	if cfg.SchemaVersion != 1 {
		return errors.New("schema_version must be exactly 1")
	}
	if err := validateListen(cfg.ListenAddress); err != nil {
		return err
	}
	if err := requireAbsoluteDirectory("state_directory", cfg.StateDirectory); err != nil {
		return err
	}
	if cfg.HeartbeatPollMS != 500 {
		return errors.New("heartbeat_poll_ms must be exactly 500")
	}
	if cfg.HeartbeatStaleMS != 2000 {
		return errors.New("heartbeat_stale_ms must be exactly 2000")
	}
	if cfg.HeartbeatStartupTimeoutMS != 110000 {
		return errors.New("heartbeat_startup_timeout_ms must be exactly 110000")
	}
	if cfg.TerminationGraceMS != 5000 {
		return errors.New("termination_grace_ms must be exactly 5000")
	}
	if cfg.ShutdownTimeoutMS < 5000 || cfg.ShutdownTimeoutMS > 30000 {
		return errors.New("shutdown_timeout_ms must be in [5000,30000]")
	}
	if cfg.MaximumEventEntries < 1 || cfg.MaximumEventEntries > maxEventEntries {
		return fmt.Errorf("maximum_event_entries must be in [1,%d]", maxEventEntries)
	}
	if cfg.MaximumEventBytes < 1024 || cfg.MaximumEventBytes > maxEventFileBytes {
		return fmt.Errorf("maximum_event_bytes must be in [1024,%d]", maxEventFileBytes)
	}
	if err := cfg.Restart.validate(); err != nil {
		return err
	}
	if len(cfg.Children) == 0 || len(cfg.Children) > maxChildren {
		return fmt.Errorf("children must contain between 1 and %d entries", maxChildren)
	}
	names := make(map[string]struct{}, len(cfg.Children))
	heartbeats := make(map[string]string)
	for index, child := range cfg.Children {
		if err := child.validate(); err != nil {
			return fmt.Errorf("children[%d]: %w", index, err)
		}
		if child.Argv[0] == packagedStackExecutable {
			if err := child.validatePackagedStackContract(cfg.StateDirectory); err != nil {
				return fmt.Errorf("children[%d]: %w", index, err)
			}
		}
		if _, exists := names[child.Name]; exists {
			return fmt.Errorf("duplicate child name %q", child.Name)
		}
		names[child.Name] = struct{}{}
		if child.HeartbeatFile != "" {
			canonicalHeartbeat := filepath.Clean(child.HeartbeatFile)
			relativeHeartbeat, err := filepath.Rel(cfg.StateDirectory, canonicalHeartbeat)
			if err != nil || relativeHeartbeat == "." || relativeHeartbeat == ".." ||
				strings.HasPrefix(relativeHeartbeat, ".."+string(os.PathSeparator)) {
				return fmt.Errorf(
					"heartbeat file %q must be a child of state_directory",
					canonicalHeartbeat,
				)
			}
			if owner, exists := heartbeats[canonicalHeartbeat]; exists {
				return fmt.Errorf("heartbeat file %q is shared by %q and %q", canonicalHeartbeat, owner, child.Name)
			}
			heartbeats[canonicalHeartbeat] = child.Name
		}
	}
	return nil
}

func validateListen(address string) error {
	if len(address) == 0 || len(address) > maxStringBytes || strings.ContainsRune(address, '\x00') {
		return errors.New("listen_address is invalid")
	}
	host, port, err := net.SplitHostPort(address)
	if err != nil {
		return fmt.Errorf("listen_address: %w", err)
	}
	if host != "127.0.0.1" {
		return errors.New("listen_address must bind exactly to 127.0.0.1")
	}
	if port == "" {
		return errors.New("listen_address port is empty")
	}
	number, err := strconv.Atoi(port)
	if err != nil || number < 1 || number > 65535 {
		return errors.New("listen_address port must be an integer in [1,65535]")
	}
	return nil
}

func (policy RestartPolicy) validate() error {
	if policy.InitialBackoffMS != 1000 || policy.MaximumBackoffMS != 8000 {
		return errors.New("restart backoff must be exactly 1000ms initial and 8000ms maximum")
	}
	if policy.MaximumAttempts != 4 {
		return errors.New("restart maximum_attempts must be exactly 4")
	}
	if policy.WindowMS != 60000 || policy.StableResetMS != 60000 {
		return errors.New("restart window_ms and stable_reset_ms must be exactly 60000")
	}
	return nil
}

func (child Child) validate() error {
	if !childNamePattern.MatchString(child.Name) {
		return errors.New("name must match ^[a-z][a-z0-9_-]{0,63}$")
	}
	if len(child.Argv) == 0 || len(child.Argv) > maxArguments {
		return fmt.Errorf("argv must contain between 1 and %d entries", maxArguments)
	}
	for index, value := range child.Argv {
		if err := validateString(value); err != nil {
			return fmt.Errorf("argv[%d]: %w", index, err)
		}
	}
	if child.Argv[0] == "" {
		return errors.New("argv[0] must not be empty")
	}
	if !filepath.IsAbs(child.Argv[0]) {
		return errors.New("argv[0] must be an absolute executable path")
	}
	if filepath.Clean(child.Argv[0]) != child.Argv[0] {
		return errors.New("argv[0] must be a canonical executable path")
	}
	if err := requireAbsoluteDirectory("working_directory", child.WorkingDirectory); err != nil {
		return err
	}
	if len(child.Environment) > maxEnvironment {
		return fmt.Errorf("environment must contain at most %d entries", maxEnvironment)
	}
	for key, value := range child.Environment {
		if len(key) > maxStringBytes || !environmentNamePattern.MatchString(key) {
			return fmt.Errorf("environment key %q is invalid", key)
		}
		if err := validateString(value); err != nil {
			return fmt.Errorf("environment %q: %w", key, err)
		}
	}
	if child.HeartbeatFile != "" {
		if !filepath.IsAbs(child.HeartbeatFile) || filepath.Clean(child.HeartbeatFile) != child.HeartbeatFile || len(child.HeartbeatFile) > maxStringBytes || strings.ContainsRune(child.HeartbeatFile, '\x00') {
			return errors.New("heartbeat_file must be a bounded canonical absolute path")
		}
	}
	return nil
}

func (child Child) validatePackagedStackContract(stateDirectory string) error {
	runtimeStateDirectory, exists := child.Environment[runtimeStateDirectoryEnv]
	if !exists || runtimeStateDirectory != stateDirectory {
		return fmt.Errorf(
			"environment %s must be present and exactly match state_directory",
			runtimeStateDirectoryEnv,
		)
	}
	expectedHeartbeat := filepath.Join(stateDirectory, packagedStackHeartbeatFileName)
	if child.HeartbeatFile != expectedHeartbeat {
		return fmt.Errorf(
			"heartbeat_file for %s must be exactly %q",
			packagedStackExecutable,
			expectedHeartbeat,
		)
	}
	return nil
}

func requireAbsoluteDirectory(name, value string) error {
	if !filepath.IsAbs(value) || filepath.Clean(value) != value || value == string(os.PathSeparator) || len(value) > maxStringBytes || strings.ContainsRune(value, '\x00') {
		return fmt.Errorf("%s must be a bounded canonical absolute non-root path", name)
	}
	return nil
}

func validateString(value string) error {
	if len(value) > maxStringBytes || strings.ContainsRune(value, '\x00') {
		return errors.New("value exceeds the string bound or contains NUL")
	}
	return nil
}

// Duration helpers keep conversions centralized and overflow-free after validation.
func (cfg Config) HeartbeatPoll() time.Duration {
	return time.Duration(cfg.HeartbeatPollMS) * time.Millisecond
}
func (cfg Config) HeartbeatStale() time.Duration {
	return time.Duration(cfg.HeartbeatStaleMS) * time.Millisecond
}
func (cfg Config) HeartbeatStartupTimeout() time.Duration {
	return time.Duration(cfg.HeartbeatStartupTimeoutMS) * time.Millisecond
}
func (cfg Config) TerminationGrace() time.Duration {
	return time.Duration(cfg.TerminationGraceMS) * time.Millisecond
}
func (cfg Config) ShutdownTimeout() time.Duration {
	return time.Duration(cfg.ShutdownTimeoutMS) * time.Millisecond
}
