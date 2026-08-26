// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package httpapi

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/hasanahmed/robotest-lab/supervisor/internal/supervisor"
)

type fakeSource struct{ ready bool }

func (source fakeSource) Snapshot() supervisor.Snapshot {
	return supervisor.Snapshot{SchemaVersion: 1, Healthy: true, Ready: source.ready}
}

func TestLoopbackListenerLifecycle(t *testing.T) {
	t.Parallel()
	listener, err := net.Listen("tcp4", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	if address, ok := listener.Addr().(*net.TCPAddr); !ok || !address.IP.IsLoopback() {
		t.Fatalf("listener is not loopback: %v", listener.Addr())
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- New(fakeSource{ready: true}).runListener(ctx, listener) }()
	client := &http.Client{Timeout: time.Second}
	url := "http://" + listener.Addr().String() + "/readyz"
	deadline := time.Now().Add(time.Second)
	for {
		response, requestErr := client.Get(url)
		if requestErr == nil {
			response.Body.Close()
			if response.StatusCode != http.StatusOK {
				t.Fatalf("ready status = %d", response.StatusCode)
			}
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("HTTP listener did not become ready: %v", requestErr)
		}
		time.Sleep(10 * time.Millisecond)
	}
	cancel()
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("HTTP listener did not shut down")
	}
}
func (fakeSource) Metrics() string { return "robotest_supervisor_ready 1\n" }

func request(t *testing.T, handler http.Handler, method, path string) *httptest.ResponseRecorder {
	t.Helper()
	recorder := httptest.NewRecorder()
	handler.ServeHTTP(recorder, httptest.NewRequest(method, path, nil))
	return recorder
}

func TestReadOnlyEndpointSemantics(t *testing.T) {
	t.Parallel()
	handler := New(fakeSource{ready: true}).Handler()
	tests := []struct {
		path        string
		contentType string
		body        string
	}{
		{"/healthz", "text/plain", "ok\n"},
		{"/readyz", "text/plain", "ready\n"},
		{"/metrics", "text/plain", "robotest_supervisor_ready 1\n"},
	}
	for _, test := range tests {
		response := request(t, handler, http.MethodGet, test.path)
		if response.Code != http.StatusOK || response.Body.String() != test.body {
			t.Fatalf("%s returned %d %q", test.path, response.Code, response.Body.String())
		}
		if !strings.HasPrefix(response.Header().Get("Content-Type"), test.contentType) {
			t.Fatalf("%s content type = %q", test.path, response.Header().Get("Content-Type"))
		}
	}
	status := request(t, handler, http.MethodGet, "/v1/status")
	var snapshot supervisor.Snapshot
	if err := json.Unmarshal(status.Body.Bytes(), &snapshot); err != nil || !snapshot.Ready {
		t.Fatalf("invalid status response: %v %#v", err, snapshot)
	}
}

func TestReadyFailureAndMutationRejection(t *testing.T) {
	t.Parallel()
	handler := New(fakeSource{ready: false}).Handler()
	if response := request(t, handler, http.MethodGet, "/readyz"); response.Code != http.StatusServiceUnavailable {
		t.Fatalf("not-ready status = %d", response.Code)
	}
	for _, path := range []string{"/healthz", "/readyz", "/v1/status", "/metrics"} {
		response := request(t, handler, http.MethodPost, path)
		if response.Code != http.StatusMethodNotAllowed || response.Header().Get("Allow") != http.MethodGet {
			t.Fatalf("POST %s returned %d", path, response.Code)
		}
	}
	for _, path := range []string{"/restart", "/v1/restart", "/v1/children/stack/stop"} {
		if response := request(t, handler, http.MethodPost, path); response.Code != http.StatusNotFound {
			t.Fatalf("mutation path %s exists with status %d", path, response.Code)
		}
	}
}
