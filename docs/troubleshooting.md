# RoboTest Lab troubleshooting

This guide diagnoses the first failing layer without erasing evidence or
turning an incomplete gate into a pass. The canonical workspace is
`/home/hasan/robotest-lab` in the WSL2 distribution named `Ubuntu`.

## Non-negotiable recovery rules

- Preserve every failed candidate and its checksum files. Do not rename a
  failed run, replace a failed trial, or edit generated JSON/CSV to make it
  pass.
- Stop at the first source, binding, resource, ownership, or integrity error.
  A later green unit test does not override an earlier failed gate.
- Do not loosen a frozen threshold after observing a result. Record a prior
  decision and create a new candidate if a product change is required.
- Clean only process groups, temporary directories, and evidence paths owned
  by the command that created them. Never use a broad process kill or recursive
  delete as a shortcut.
- `Ubuntu-20.04` is outside project scope. Every Windows-side WSL command must
  name `Ubuntu` explicitly.

## First response

From Windows PowerShell, confirm the target before entering the guest:

```powershell
wsl -l -v
wsl -d Ubuntu -- bash -lc 'cat /etc/os-release; uname -a; ps -p 1 -o comm='
```

Then collect read-only repository state inside Ubuntu:

```bash
cd /home/hasan/robotest-lab
git branch --show-current
git status --short
git rev-parse HEAD
findmnt -T /home/hasan/robotest-lab
```

A fresh public clone normally starts on the default `main` branch. Work on a
different branch or detached candidate is valid only when it is deliberately
selected and its exact `git rev-parse HEAD` value is recorded; the immutable
SHA, not a mutable branch label, identifies qualification evidence. The
repository must remain on the WSL-native filesystem. A candidate, package, or
release gate that requires a clean commit must stop on any tracked, untracked,
`assume-unchanged`, or `skip-worktree` source delta.

## Understand the exit before changing anything

| Observation | Meaning | Next action |
| --- | --- | --- |
| A phase verifier exits nonzero | Its own log and summary identify the first failed check | Preserve the run directory; repair that layer before rerunning |
| Bare `scripts/verify_all.sh` exits `3` with `INCOMPLETE` | Expected local aggregate behavior; it is not a release pass | Retain its exact checksummed aggregate for the later final gate |
| `--release-evidence` is incomplete | At least one caller-selected evidence join is missing or invalid | Correct the named evidence lane; do not substitute a newer or “latest” path |
| A candidate ID already has evidence | That ID is immutable, including failures | Use a new ID only after the fix is committed and rebound |
| A checksum validation fails | The artifact set changed or is incomplete | Preserve both bytes and validation output; regenerate only from authoritative inputs |

## Setup and dependency failures

1. Run the read-only repository-source check before any apply operation.
2. Run `scripts/verify_phase0_preinstall.sh` before installing packages. It
   enforces both the 18 GiB project/dependency projection and the 20 GiB host
   reserve.
3. Use `--apply` only for the documented mutating setup commands. Do not add
   packages manually to bypass the audited apt manifest.
4. If `rosdep`, ROS, Gazebo, Nav2, Python, or Go versions differ, preserve the
   inventory and repair the exact dependency declaration before rerunning the
   post-install verifier.

See the [dependency plan](dependencies.md) and
[environment audit](environment-audit.md) for the supported boundary.

## Build or source-binding failures

- Read the verifier-emitted `build-binding.json`; do not reuse a binding after
  any source/configuration change or commit.
- Rebuild only from the exact current source, then rerun the corresponding
  static verifier to produce a new binding.
- Do not clean the whole workspace to conceal unrelated changes. Build and test
  outputs are evidence only when the recorded source SHA and source hashes
  match the running installation.
- A dirty development build can support diagnosis, but it cannot become an
  authoritative Phase 3 candidate or Phase 4 release result.

## ROS 2 or Gazebo runtime failures

Before starting another stack, inspect rather than broadly killing processes:

```bash
ps -eo pid,ppid,pgid,stat,cmd --sort=pgid
ros2 daemon status
```

Check the run's recorded ROS domain, Gazebo partition, process-group IDs, CPU
affinity, launch log, and cleanup report. Use only the verifier's bounded,
identity-checked cleanup path for its owned group. A surviving unrelated ROS or
Gazebo process is not authority to stop it.

For graph, QoS, clock, or TF failures, compare live observations with the
[topic and TF contract](architecture/topic-and-tf-contract.md). Treat Fast DDS
`history=UNKNOWN` and `depth=0` as unknown; do not report them as proof of an
unbounded or bounded queue. A silent contact stream is not collision-free
evidence.

## Resource or real-time-factor failures

- Preserve the complete failed run and profiler output. Calculate the failure
  from the canonical resource samples; do not discard low windows as “noise”
  unless the frozen contract already excludes them.
- Confirm descendants remained on CPUs `0-5`, peak process-tree RSS stayed
  within the frozen limit, and the host was not paging heavily or running a
  competing benchmark.
- After a long host uptime or sustained memory pressure, a user-authorized
  Windows reboot and a settled preflight can remove external contention. A
  reboot does not convert an old failed candidate into a pass; the next attempt
  uses a new clean, SHA-bound candidate.
- Optimize measured product work without lowering sensor, physics, evidence,
  or acceptance rates. Follow
  [ADR 0008](decisions/0008-phase3-contact-aggregation-performance.md) for the
  Phase 3 performance keep/revert gate.

## Phase 4 package or service failures

- The unprivileged `scripts/verify_phase4.sh` command is preflight only.
  Authoritative lifecycle evidence requires the explicit root `--apply` route
  in the [verification matrix](testing/verification-matrix.md).
- Preserve `/run` evidence and package logs until the service is proven
  inactive and publication replay is complete.
- A failed install, upgrade, remove, purge, conffile, affinity, readiness, or
  process-group check blocks the Phase 4 verdict. Do not call a staged package
  install a lifecycle pass.
- The service must remain disabled by default and bound to loopback for its
  health/readiness API.

## Phase 5, documentation replay, and media

- `scripts/verify_phase5.sh --local` proves only the documented local/static CI
  surface. It does not prove public CI, a live campaign, packaging, or release
  eligibility.
- Remote proof requires a public `origin`, the native Ubuntu `gh` client, an
  authenticated user session, and a completed successful workflow for the
  exact requested SHA. Missing any one leaves the criterion incomplete.
- P5-04 compares every executable README command fence with
  `config/phase5-readme-replay.json`. It validates the first-line canonical
  root assertion without executing that `cd`, then runs the second line with
  cwd mapped to the detached clean candidate checkout, recording both roots.
  Non-Phase 0 units require a clean end state; each Phase 0 unit permits only
  its exact matrix-declared generated-evidence subset and records observed
  sizes/hashes. Any other delta or README/matrix drift requires recovery from
  the documented boundary.
- P5-05 accepts only the deterministic Phase 3 Scenario 5 `runs/12` chart, and
  only when it is byte-identical to that accepted run's
  `localization-error.png` and all result, suite, aggregate, manifest, hash,
  dimension, and candidate joins pass. The Phase 1 RViz screenshot is separate
  manual development evidence and cannot fill that slot.
- Portfolio prepare emits exact Mermaid sources and a render request; it does
  not render them. Render outside the attempt, inspect those exact SVG bytes,
  then create the hash-bound review consumed by finalize. A pre-supplied or
  stale review fails, and a synthetic robot scene is not runtime evidence.
- Each capture, failure, rerender, and recapture gets a new immutable attempt
  child below `artifacts/evidence/phase5/portfolio/<C>/`. Never overwrite a
  failed or superseded attempt; final validation selects one exact PASS child.

Follow the [Phase 5 CI gate](testing/phase5-ci.md) for the only valid
capture/projection/final-gate order.

## What to include when escalating

Provide the exact command, exit code, Git SHA and dirty state, run/candidate ID,
first failing check, relevant summary path, and checksum-validation result.
Include host/runtime contention observations when performance is involved. Do
not paste secrets, authentication tokens, or an entire unrelated system log.
