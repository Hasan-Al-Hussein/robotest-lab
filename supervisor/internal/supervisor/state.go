// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package supervisor

import (
	"bufio"
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"sync"
	"time"
)

// Event is one bounded structured lifecycle transition.
type Event struct {
	SchemaVersion  int            `json:"schema_version"`
	Sequence       uint64         `json:"sequence"`
	TimestampUTC   string         `json:"timestamp_utc"`
	SteadyWallNS   int64          `json:"steady_wall_ns"`
	Kind           string         `json:"kind"`
	Ready          *bool          `json:"ready,omitempty"`
	Child          string         `json:"child,omitempty"`
	PID            int            `json:"pid,omitempty"`
	PGID           int            `json:"pgid,omitempty"`
	ExitCode       *int           `json:"exit_code,omitempty"`
	RestartAttempt int            `json:"restart_attempt,omitempty"`
	BackoffMS      int64          `json:"backoff_ms,omitempty"`
	FailureKind    string         `json:"failure_kind,omitempty"`
	Details        map[string]any `json:"details,omitempty"`
}

// ChildSnapshot is the externally visible state of one configured child.
type ChildSnapshot struct {
	Name            string `json:"name"`
	Required        bool   `json:"required"`
	Running         bool   `json:"running"`
	HeartbeatFresh  bool   `json:"heartbeat_fresh"`
	HeartbeatAgeMS  *int64 `json:"heartbeat_age_ms"`
	PID             int    `json:"pid"`
	PGID            int    `json:"pgid"`
	RestartCount    int    `json:"restart_count"`
	CircuitOpen     bool   `json:"circuit_open"`
	LastFailureKind string `json:"last_failure_kind,omitempty"`
	LastFailureUTC  string `json:"last_failure_utc,omitempty"`
	LastReadyUTC    string `json:"last_ready_utc,omitempty"`
	StartedUTC      string `json:"started_utc,omitempty"`
	LastExitCode    *int   `json:"last_exit_code,omitempty"`
}

// Snapshot is returned by /v1/status and persisted atomically.
type Snapshot struct {
	SchemaVersion      int             `json:"schema_version"`
	SupervisorUTC      string          `json:"supervisor_utc"`
	Healthy            bool            `json:"healthy"`
	Ready              bool            `json:"ready"`
	PersistenceHealthy bool            `json:"persistence_healthy"`
	ShuttingDown       bool            `json:"shutting_down"`
	EventCount         int             `json:"event_count"`
	DroppedEvents      uint64          `json:"dropped_events"`
	Children           []ChildSnapshot `json:"children"`
}

type eventStore struct {
	mu             sync.Mutex
	stateDirectory string
	path           string
	metadataPath   string
	maximumEntries int
	maximumBytes   int64
	entries        int
	bytes          int64
	dropped        uint64
	sequence       uint64
	saturated      bool
}

type eventStoreMetadata struct {
	SchemaVersion         int    `json:"schema_version"`
	Saturated             bool   `json:"saturated"`
	DroppedEvents         uint64 `json:"dropped_events"`
	LastAttemptedSequence uint64 `json:"last_attempted_sequence"`
}

func newEventStore(stateDirectory string, maximumEntries int, maximumBytes int64) (*eventStore, error) {
	if err := os.MkdirAll(stateDirectory, 0o750); err != nil {
		return nil, fmt.Errorf("create state directory: %w", err)
	}
	store := &eventStore{
		stateDirectory: stateDirectory,
		path:           filepath.Join(stateDirectory, "events.jsonl"),
		metadataPath:   filepath.Join(stateDirectory, "events.meta.json"),
		maximumEntries: maximumEntries,
		maximumBytes:   maximumBytes,
	}
	payload, err := os.ReadFile(store.path)
	if err != nil && !errors.Is(err, os.ErrNotExist) {
		return nil, fmt.Errorf("read existing event log: %w", err)
	}
	if int64(len(payload)) > maximumBytes {
		return nil, errors.New("existing event log exceeds maximum_event_bytes")
	}
	store.bytes = int64(len(payload))
	if len(payload) > 0 && payload[len(payload)-1] != '\n' {
		return nil, errors.New("existing event log has an incomplete final record")
	}
	store.entries = bytes.Count(payload, []byte{'\n'})
	if store.entries > maximumEntries {
		return nil, errors.New("existing event log exceeds maximum_event_entries")
	}
	scanner := bufio.NewScanner(bytes.NewReader(payload))
	scanner.Buffer(make([]byte, 4096), 1<<20)
	var previousSequence uint64
	for scanner.Scan() {
		var event Event
		if err := json.Unmarshal(scanner.Bytes(), &event); err != nil {
			return nil, fmt.Errorf("decode existing event record %d: %w", previousSequence+1, err)
		}
		if event.SchemaVersion != 1 {
			return nil, errors.New("existing event log has an unsupported schema_version")
		}
		if event.Sequence != previousSequence+1 {
			return nil, errors.New("existing event log sequences are not contiguous")
		}
		if event.Kind == "" || event.SteadyWallNS < 0 {
			return nil, errors.New("existing event log has an invalid kind or steady_wall_ns")
		}
		if _, err := time.Parse(time.RFC3339Nano, event.TimestampUTC); err != nil {
			return nil, fmt.Errorf("existing event record %d has invalid timestamp: %w", event.Sequence, err)
		}
		previousSequence = event.Sequence
	}
	if err := scanner.Err(); err != nil {
		return nil, fmt.Errorf("scan existing event log: %w", err)
	}
	metadata, metadataExists, err := loadEventStoreMetadata(store.metadataPath)
	if err != nil {
		return nil, err
	}
	if !metadataExists {
		metadata = eventStoreMetadata{
			SchemaVersion:         1,
			LastAttemptedSequence: previousSequence,
		}
	}
	if metadata.SchemaVersion != 1 {
		return nil, errors.New("event metadata has an unsupported schema_version")
	}
	if metadata.LastAttemptedSequence < previousSequence {
		return nil, errors.New("event metadata sequence precedes the persisted event log")
	}
	// A reserved sequence without a complete log record means the prior process
	// stopped mid-append. Preserve the gap as a durable drop and stay saturated.
	missingAttempts := metadata.LastAttemptedSequence - previousSequence
	if metadata.DroppedEvents > missingAttempts {
		return nil, errors.New("event metadata dropped count exceeds missing attempted sequences")
	}
	metadataChanged := !metadataExists
	if metadata.DroppedEvents < missingAttempts {
		metadata.DroppedEvents = missingAttempts
		metadataChanged = true
	}
	if metadata.DroppedEvents != 0 && !metadata.Saturated {
		metadata.Saturated = true
		metadataChanged = true
	}
	if metadata.Saturated && metadata.DroppedEvents == 0 {
		return nil, errors.New("event metadata is saturated without a dropped event")
	}
	store.sequence = metadata.LastAttemptedSequence
	store.dropped = metadata.DroppedEvents
	store.saturated = metadata.Saturated
	if metadataChanged {
		if err := store.persistMetadata(metadata); err != nil {
			return nil, err
		}
	}
	return store, nil
}

func loadEventStoreMetadata(path string) (eventStoreMetadata, bool, error) {
	payload, err := os.ReadFile(path)
	if errors.Is(err, os.ErrNotExist) {
		return eventStoreMetadata{}, false, nil
	}
	if err != nil {
		return eventStoreMetadata{}, false, fmt.Errorf("read event metadata: %w", err)
	}
	decoder := json.NewDecoder(bytes.NewReader(payload))
	decoder.DisallowUnknownFields()
	var metadata eventStoreMetadata
	if err := decoder.Decode(&metadata); err != nil {
		return eventStoreMetadata{}, false, fmt.Errorf("decode event metadata: %w", err)
	}
	var trailing any
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		if err == nil {
			return eventStoreMetadata{}, false, errors.New("event metadata contains more than one JSON value")
		}
		return eventStoreMetadata{}, false, fmt.Errorf("decode trailing event metadata: %w", err)
	}
	return metadata, true, nil
}

func (store *eventStore) append(now time.Time, event Event) (Event, error) {
	store.mu.Lock()
	defer store.mu.Unlock()
	if store.sequence == ^uint64(0) {
		return event, errors.New("event sequence space is exhausted")
	}
	nextSequence := store.sequence + 1
	event.SchemaVersion = 1
	event.Sequence = nextSequence
	event.TimestampUTC = now.UTC().Format(time.RFC3339Nano)
	payload, err := json.Marshal(event)
	if err != nil {
		return event, fmt.Errorf("marshal event: %w", err)
	}
	payload = append(payload, '\n')
	capacityDrop := store.saturated || store.entries >= store.maximumEntries ||
		store.bytes+int64(len(payload)) > store.maximumBytes
	if capacityDrop {
		if store.dropped == ^uint64(0) {
			return event, errors.New("dropped event counter is exhausted")
		}
		store.sequence = nextSequence
		store.saturated = true
		store.dropped++
		metadata := eventStoreMetadata{
			SchemaVersion:         1,
			Saturated:             true,
			DroppedEvents:         store.dropped,
			LastAttemptedSequence: nextSequence,
		}
		if err := store.persistMetadata(metadata); err != nil {
			return event, err
		}
		return event, nil
	}
	// Persist the reservation before appending so a crash can never make a
	// previously attempted sequence available for reuse.
	store.sequence = nextSequence
	reservation := eventStoreMetadata{
		SchemaVersion:         1,
		LastAttemptedSequence: nextSequence,
	}
	if err := store.persistMetadata(reservation); err != nil {
		return event, store.failReservedAppend(err)
	}
	file, err := os.OpenFile(store.path, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0o640)
	if err != nil {
		return event, store.failReservedAppend(fmt.Errorf("open event log: %w", err))
	}
	written, writeErr := file.Write(payload)
	if writeErr == nil && written != len(payload) {
		writeErr = io.ErrShortWrite
	}
	syncErr := file.Sync()
	closeErr := file.Close()
	if writeErr != nil || syncErr != nil {
		return event, store.failReservedAppend(errors.Join(writeErr, syncErr, closeErr))
	}
	store.entries++
	store.bytes += int64(len(payload))
	if closeErr != nil {
		return event, closeErr
	}
	return event, nil
}

func (store *eventStore) failReservedAppend(cause error) error {
	metadata := eventStoreMetadata{
		SchemaVersion:         1,
		Saturated:             true,
		DroppedEvents:         store.dropped + 1,
		LastAttemptedSequence: store.sequence,
	}
	store.saturated = true
	store.dropped = metadata.DroppedEvents
	return errors.Join(cause, store.persistMetadata(metadata))
}

func (store *eventStore) persistMetadata(metadata eventStoreMetadata) error {
	payload, err := json.Marshal(metadata)
	if err != nil {
		return fmt.Errorf("marshal event metadata: %w", err)
	}
	payload = append(payload, '\n')
	if err := writeFileAtomic(store.stateDirectory, ".events-meta-*.json", store.metadataPath, payload, 0o640); err != nil {
		return fmt.Errorf("persist event metadata: %w", err)
	}
	return nil
}

func (store *eventStore) counts() (int, uint64) {
	store.mu.Lock()
	defer store.mu.Unlock()
	return store.entries, store.dropped
}

func (store *eventStore) isSaturated() bool {
	store.mu.Lock()
	defer store.mu.Unlock()
	return store.saturated
}

func writeSnapshotAtomic(stateDirectory string, snapshot Snapshot) error {
	payload, err := json.Marshal(snapshot)
	if err != nil {
		return fmt.Errorf("marshal status snapshot: %w", err)
	}
	payload = append(payload, '\n')
	if err := writeFileAtomic(
		stateDirectory,
		".status-*.json",
		filepath.Join(stateDirectory, "status.json"),
		payload,
		0o640,
	); err != nil {
		return fmt.Errorf("replace status snapshot: %w", err)
	}
	return nil
}

func writeFileAtomic(
	directory string,
	temporaryPattern string,
	target string,
	payload []byte,
	mode os.FileMode,
) error {
	temporary, err := os.CreateTemp(directory, temporaryPattern)
	if err != nil {
		return fmt.Errorf("create temporary file: %w", err)
	}
	temporaryName := temporary.Name()
	defer os.Remove(temporaryName)
	if err := temporary.Chmod(mode); err != nil {
		temporary.Close()
		return fmt.Errorf("chmod temporary file: %w", err)
	}
	if _, err := temporary.Write(payload); err != nil {
		temporary.Close()
		return fmt.Errorf("write temporary file: %w", err)
	}
	if err := temporary.Sync(); err != nil {
		temporary.Close()
		return fmt.Errorf("sync temporary file: %w", err)
	}
	if err := temporary.Close(); err != nil {
		return fmt.Errorf("close temporary file: %w", err)
	}
	if err := os.Rename(temporaryName, target); err != nil {
		return fmt.Errorf("rename temporary file: %w", err)
	}
	directoryHandle, err := os.Open(directory)
	if err != nil {
		return fmt.Errorf("open target directory for sync: %w", err)
	}
	syncErr := directoryHandle.Sync()
	closeErr := directoryHandle.Close()
	if syncErr != nil || closeErr != nil {
		return fmt.Errorf("sync target directory: %w", errors.Join(syncErr, closeErr))
	}
	return nil
}
