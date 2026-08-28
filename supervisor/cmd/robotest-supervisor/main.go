// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/Hasan-Al-Hussein/robotest-lab/supervisor/internal/config"
	"github.com/Hasan-Al-Hussein/robotest-lab/supervisor/internal/httpapi"
	"github.com/Hasan-Al-Hussein/robotest-lab/supervisor/internal/supervisor"
)

func main() {
	logger := slog.New(slog.NewJSONHandler(os.Stderr, nil))
	if err := run(os.Args[1:], logger); err != nil {
		logger.Error("supervisor exited with error", "error", err)
		os.Exit(1)
	}
}

func run(arguments []string, logger *slog.Logger) (resultErr error) {
	flags := flag.NewFlagSet("robotest-supervisor", flag.ContinueOnError)
	flags.SetOutput(os.Stderr)
	configPath := flags.String("config", "/etc/robotest-supervisor/config.json", "strict JSON configuration path")
	checkConfig := flags.Bool("check-config", false, "validate configuration and exit without starting children")
	if err := flags.Parse(arguments); err != nil {
		if errors.Is(err, flag.ErrHelp) {
			return nil
		}
		return err
	}
	if flags.NArg() != 0 {
		return fmt.Errorf("unexpected positional arguments: %v", flags.Args())
	}
	cfg, err := config.Load(*configPath)
	if err != nil {
		return err
	}
	if *checkConfig {
		_, err := fmt.Fprintln(os.Stdout, "configuration valid")
		return err
	}
	manager, err := supervisor.New(cfg, logger)
	if err != nil {
		return err
	}
	defer func() {
		resultErr = errors.Join(resultErr, manager.Close())
	}()

	signalContext, stopSignals := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer stopSignals()
	ctx, cancel := context.WithCancel(signalContext)
	defer cancel()
	go func() {
		<-signalContext.Done()
		logger.Info("shutdown signal received; a second signal forces termination")
		stopSignals()
	}()

	type componentResult struct {
		name string
		err  error
	}
	results := make(chan componentResult, 2)
	go func() { results <- componentResult{name: "manager", err: manager.Run(ctx)} }()
	go func() {
		results <- componentResult{name: "http", err: httpapi.New(manager).Run(ctx, cfg.ListenAddress)}
	}()

	first := <-results
	cancel()
	second := <-results
	var failures []error
	for _, result := range []componentResult{first, second} {
		if result.err != nil && !errors.Is(result.err, context.Canceled) {
			failures = append(failures, fmt.Errorf("%s: %w", result.name, result.err))
		}
	}
	return errors.Join(failures...)
}
