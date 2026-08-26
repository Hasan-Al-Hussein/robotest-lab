// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package supervisor

import (
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

func TestEventStoreResumesSequenceAndPreservesPrefix(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	store, err := newEventStore(root, 3, 4096)
	if err != nil {
		t.Fatal(err)
	}
	for _, kind := range []string{"one", "two"} {
		if _, err := store.append(time.Unix(100, 0), Event{Kind: kind}); err != nil {
			t.Fatal(err)
		}
	}
	reloaded, err := newEventStore(root, 3, 4096)
	if err != nil {
		t.Fatal(err)
	}
	event, err := reloaded.append(time.Unix(101, 0), Event{Kind: "three"})
	if err != nil || event.Sequence != 3 {
		t.Fatalf("resumed event = %#v, error = %v", event, err)
	}
	if _, err := reloaded.append(time.Unix(102, 0), Event{Kind: "omitted"}); err != nil {
		t.Fatal(err)
	}
	entries, dropped := reloaded.counts()
	if entries != 3 || dropped != 1 {
		t.Fatalf("counts = (%d,%d), want (3,1)", entries, dropped)
	}
	payload, err := os.ReadFile(filepath.Join(root, "events.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	lines := strings.Split(strings.TrimSuffix(string(payload), "\n"), "\n")
	if len(lines) != 3 {
		t.Fatalf("retained lines = %d", len(lines))
	}
	for index, line := range lines {
		var retained Event
		if err := json.Unmarshal([]byte(line), &retained); err != nil {
			t.Fatal(err)
		}
		if retained.Sequence != uint64(index+1) {
			t.Fatalf("sequence[%d] = %d", index, retained.Sequence)
		}
		if retained.SchemaVersion != 1 {
			t.Fatalf("schema_version[%d] = %d", index, retained.SchemaVersion)
		}
	}
}

func TestEventStoreRejectsCorruptExistingLog(t *testing.T) {
	t.Parallel()
	timestamp := time.Unix(100, 0).UTC().Format(time.RFC3339Nano)
	tests := map[string]string{
		"incomplete": `{"sequence":1}`,
		"malformed":  "not-json\n",
		"duplicate":  `{"sequence":1,"timestamp_utc":"` + timestamp + `","kind":"one"}` + "\n" + `{"sequence":1,"timestamp_utc":"` + timestamp + `","kind":"two"}` + "\n",
	}
	for name, payload := range tests {
		name, payload := name, payload
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			root := t.TempDir()
			if err := os.WriteFile(filepath.Join(root, "events.jsonl"), []byte(payload), 0o600); err != nil {
				t.Fatal(err)
			}
			if _, err := newEventStore(root, 10, 4096); err == nil {
				t.Fatal("corrupt event log was accepted")
			}
		})
	}
}

func TestEventStoreNeverWritesAfterFirstCapacityDrop(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	store, err := newEventStore(root, 10, 300)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := store.append(time.Unix(100, 0), Event{Kind: "prefix"}); err != nil {
		t.Fatal(err)
	}
	if _, err := store.append(time.Unix(101, 0), Event{
		Kind: "oversized",
		Details: map[string]any{
			"payload": strings.Repeat("x", 500),
		},
	}); err != nil {
		t.Fatal(err)
	}
	if _, err := store.append(time.Unix(102, 0), Event{Kind: "would-fit"}); err != nil {
		t.Fatal(err)
	}
	entries, dropped := store.counts()
	if entries != 1 || dropped != 2 {
		t.Fatalf("counts = (%d,%d), want (1,2)", entries, dropped)
	}
	reloaded, err := newEventStore(root, 10, 300)
	if err != nil {
		t.Fatalf("prefix was not reloadable: %v", err)
	}
	entries, dropped = reloaded.counts()
	if entries != 1 || dropped != 2 || !reloaded.isSaturated() {
		t.Fatalf("reloaded counts = (%d,%d), saturated=%t; want (1,2,true)", entries, dropped, reloaded.isSaturated())
	}
	continued, err := reloaded.append(time.Unix(103, 0), Event{Kind: "still-omitted"})
	if err != nil {
		t.Fatal(err)
	}
	if continued.Sequence != 4 {
		t.Fatalf("continued attempted sequence = %d, want 4", continued.Sequence)
	}
	entries, dropped = reloaded.counts()
	if entries != 1 || dropped != 3 {
		t.Fatalf("continued counts = (%d,%d), want (1,3)", entries, dropped)
	}
	metadataPayload, err := os.ReadFile(filepath.Join(root, "events.meta.json"))
	if err != nil {
		t.Fatal(err)
	}
	var metadata eventStoreMetadata
	if err := json.Unmarshal(metadataPayload, &metadata); err != nil {
		t.Fatal(err)
	}
	if !metadata.Saturated || metadata.DroppedEvents != 3 || metadata.LastAttemptedSequence != 4 {
		t.Fatalf("durable overflow metadata = %#v", metadata)
	}
	reloadedAgain, err := newEventStore(root, 10, 300)
	if err != nil {
		t.Fatal(err)
	}
	finalAttempt, err := reloadedAgain.append(time.Unix(104, 0), Event{Kind: "next-attempt"})
	if err != nil {
		t.Fatal(err)
	}
	if finalAttempt.Sequence != 5 {
		t.Fatalf("sequence reused after second reload: %d", finalAttempt.Sequence)
	}
}

func TestEventStoreRecoversReservedButUnwrittenSequenceFailClosed(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	store, err := newEventStore(root, 10, 4096)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := store.append(time.Unix(100, 0), Event{Kind: "prefix"}); err != nil {
		t.Fatal(err)
	}
	metadata := eventStoreMetadata{
		SchemaVersion:         1,
		LastAttemptedSequence: 2,
	}
	if err := store.persistMetadata(metadata); err != nil {
		t.Fatal(err)
	}
	reloaded, err := newEventStore(root, 10, 4096)
	if err != nil {
		t.Fatal(err)
	}
	entries, dropped := reloaded.counts()
	if entries != 1 || dropped != 1 || !reloaded.isSaturated() {
		t.Fatalf("reserved-gap recovery = (%d,%d,%t)", entries, dropped, reloaded.isSaturated())
	}
	event, err := reloaded.append(time.Unix(101, 0), Event{Kind: "after-gap"})
	if err != nil {
		t.Fatal(err)
	}
	if event.Sequence != 3 {
		t.Fatalf("reserved sequence was reused: %d", event.Sequence)
	}
}

func TestEventStoreDoesNotReuseSequenceAfterMetadataWriteFailure(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	store, err := newEventStore(root, 10, 4096)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.Chmod(root, 0o500); err != nil {
		t.Fatal(err)
	}
	first, appendErr := store.append(time.Unix(100, 0), Event{Kind: "unpersistable"})
	if err := os.Chmod(root, 0o700); err != nil {
		t.Fatal(err)
	}
	if appendErr == nil {
		t.Fatal("metadata write unexpectedly succeeded in a read-only directory")
	}
	if first.Sequence != 1 {
		t.Fatalf("first attempted sequence = %d", first.Sequence)
	}
	second, err := store.append(time.Unix(101, 0), Event{Kind: "after-recovery"})
	if err != nil {
		t.Fatal(err)
	}
	if second.Sequence != 2 {
		t.Fatalf("sequence after metadata failure = %d, want 2", second.Sequence)
	}
	reloaded, err := newEventStore(root, 10, 4096)
	if err != nil {
		t.Fatal(err)
	}
	entries, dropped := reloaded.counts()
	if entries != 0 || dropped != 2 || !reloaded.isSaturated() {
		t.Fatalf("reloaded failed-write state = (%d,%d,%t)", entries, dropped, reloaded.isSaturated())
	}
}

func TestStatusSnapshotIsAtomicallyReplaced(t *testing.T) {
	t.Parallel()
	root := t.TempDir()
	for _, ready := range []bool{false, true} {
		snapshot := Snapshot{SchemaVersion: 1, SupervisorUTC: time.Unix(100, 0).UTC().Format(time.RFC3339Nano), Healthy: true, Ready: ready}
		if err := writeSnapshotAtomic(root, snapshot); err != nil {
			t.Fatal(err)
		}
	}
	payload, err := os.ReadFile(filepath.Join(root, "status.json"))
	if err != nil {
		t.Fatal(err)
	}
	var retained Snapshot
	if err := json.Unmarshal(payload, &retained); err != nil {
		t.Fatal(err)
	}
	if !retained.Ready {
		t.Fatalf("latest snapshot was not retained: %#v", retained)
	}
	info, err := os.Stat(filepath.Join(root, "status.json"))
	if err != nil {
		t.Fatal(err)
	}
	if info.Mode().Perm() != 0o640 {
		t.Fatalf("status mode = %o", info.Mode().Perm())
	}
	temporary, err := filepath.Glob(filepath.Join(root, ".status-*.json"))
	if err != nil || len(temporary) != 0 {
		t.Fatalf("temporary status files remain: %v, error=%v", temporary, err)
	}
}
