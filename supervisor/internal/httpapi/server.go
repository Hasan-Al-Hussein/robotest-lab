// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

// Package httpapi exposes the read-only loopback supervisor API.
package httpapi

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"time"

	"github.com/hasanahmed/robotest-lab/supervisor/internal/supervisor"
)

// Source supplies immutable values to the HTTP transport.
type Source interface {
	Snapshot() supervisor.Snapshot
	Metrics() string
}

// Server is a bounded, read-only HTTP server.
type Server struct {
	source Source
}

// New constructs the handler without binding a socket.
func New(source Source) *Server { return &Server{source: source} }

// Handler returns a fresh mux with no mutation endpoint.
func (server *Server) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/healthz", server.health)
	mux.HandleFunc("/readyz", server.ready)
	mux.HandleFunc("/v1/status", server.status)
	mux.HandleFunc("/metrics", server.metrics)
	return mux
}

// Run binds the already-validated loopback address and shuts down on context cancellation.
func (server *Server) Run(ctx context.Context, address string) error {
	listener, err := net.Listen("tcp", address)
	if err != nil {
		return fmt.Errorf("listen on %s: %w", address, err)
	}
	return server.runListener(ctx, listener)
}

func (server *Server) runListener(ctx context.Context, listener net.Listener) error {
	httpServer := &http.Server{
		Addr:              listener.Addr().String(),
		Handler:           server.Handler(),
		ReadHeaderTimeout: 2 * time.Second,
		ReadTimeout:       5 * time.Second,
		WriteTimeout:      5 * time.Second,
		IdleTimeout:       10 * time.Second,
		MaxHeaderBytes:    8192,
	}
	done := make(chan error, 1)
	go func() { done <- httpServer.Serve(listener) }()
	select {
	case err := <-done:
		if errors.Is(err, http.ErrServerClosed) {
			return nil
		}
		return err
	case <-ctx.Done():
		shutdownContext, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		shutdownErr := httpServer.Shutdown(shutdownContext)
		serveErr := <-done
		if errors.Is(serveErr, http.ErrServerClosed) {
			serveErr = nil
		}
		return errors.Join(shutdownErr, serveErr)
	}
}

func requireGET(writer http.ResponseWriter, request *http.Request) bool {
	if request.Method == http.MethodGet {
		return true
	}
	writer.Header().Set("Allow", http.MethodGet)
	http.Error(writer, "method not allowed", http.StatusMethodNotAllowed)
	return false
}

func (server *Server) health(writer http.ResponseWriter, request *http.Request) {
	if !requireGET(writer, request) {
		return
	}
	writer.Header().Set("Content-Type", "text/plain; charset=utf-8")
	writer.WriteHeader(http.StatusOK)
	_, _ = writer.Write([]byte("ok\n"))
}

func (server *Server) ready(writer http.ResponseWriter, request *http.Request) {
	if !requireGET(writer, request) {
		return
	}
	writer.Header().Set("Content-Type", "text/plain; charset=utf-8")
	if server.source.Snapshot().Ready {
		writer.WriteHeader(http.StatusOK)
		_, _ = writer.Write([]byte("ready\n"))
		return
	}
	writer.WriteHeader(http.StatusServiceUnavailable)
	_, _ = writer.Write([]byte("not ready\n"))
}

func (server *Server) status(writer http.ResponseWriter, request *http.Request) {
	if !requireGET(writer, request) {
		return
	}
	writer.Header().Set("Content-Type", "application/json")
	writer.WriteHeader(http.StatusOK)
	_ = json.NewEncoder(writer).Encode(server.source.Snapshot())
}

func (server *Server) metrics(writer http.ResponseWriter, request *http.Request) {
	if !requireGET(writer, request) {
		return
	}
	writer.Header().Set("Content-Type", "text/plain; version=0.0.4; charset=utf-8")
	writer.WriteHeader(http.StatusOK)
	_, _ = writer.Write([]byte(server.source.Metrics()))
}
