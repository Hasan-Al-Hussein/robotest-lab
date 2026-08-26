# RoboTest Supervisor

`robotest-supervisor` is the bounded Go process supervisor for RoboTest Lab.
It owns configured Linux process groups, watches timestamped heartbeat files,
applies a fixed `1, 2, 4, 8` second restart policy with a four-attempt rolling
circuit breaker, and exposes read-only loopback health endpoints. Failure
history is reset only after 60 continuous seconds of child readiness, never
from process uptime alone.

Initial readiness is fail-closed for at most 110 wall seconds while the ROS
stack activates. Once the first post-start heartbeat is observed, a heartbeat
older than 2 wall seconds is a failure; `/readyz` is never true during either
initialization or a stale interval.

The supervisor does not interpret ROS messages or mission outcomes. systemd
owns only this process; this process alone owns its configured child groups.

## Build and test

```bash
go build ./cmd/robotest-supervisor
go test -race ./...
go vet ./...
```

## Run interactively

```bash
./robotest-supervisor --config ./config.example.json
```

The versioned, strict JSON configuration rejects duplicate or unknown keys, non-loopback listeners,
duplicate children or heartbeat files, relative runtime paths, unbounded
strings and collections, and policy values that differ from the frozen Phase
4 targets.

Read-only endpoints are:

- `GET /healthz`: the supervisor HTTP process is alive;
- `GET /readyz`: every required child is running with a fresh heartbeat;
- `GET /v1/status`: bounded structured child and failure state; and
- `GET /metrics`: fixed-cardinality Prometheus-compatible text.

There is intentionally no remote start, stop, restart, signal, or mutation
endpoint. Structured lifecycle transitions are prefix-retained in
`events.jsonl`; durable `events.meta.json` metadata keeps saturation, dropped
counts, and the last attempted sequence fail-closed across restarts. The
latest status snapshot is atomically replaced in `status.json`. An advisory
`supervisor.lock` is retained for the manager lifetime so only one supervisor
can use a state directory at a time.

Copyright 2026 Hasan Ahmed. Licensed under Apache-2.0.
