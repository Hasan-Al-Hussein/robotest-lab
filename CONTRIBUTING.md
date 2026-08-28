# Contributing to RoboTest Lab

RoboTest Lab uses evidence-gated development. A feature is not complete when
it merely looks correct in source; it is complete only at the verification
level claimed by its phase.

## Development environment

Use Ubuntu 24.04 (Noble), ROS 2 Jazzy, and the repository on the WSL-native
filesystem. Do not build the workspace from OneDrive or `/mnt/c`, and never run
project setup against the preserved `Ubuntu-20.04` distribution.

Start with the Phase 0 sequence in [README.md](README.md). Do not add an apt
dependency directly in a script: update `config/apt-packages.txt`, explain the
need in the relevant decision record, rerun the pre-install projection, and
record the resolved version.

## Change workflow

1. Work on a focused branch with a short descriptive name, such as
   `feature/<topic>` or `fix/<topic>`.
2. Keep each commit coherent and use an imperative commit subject.
3. Add or update tests with behavior changes.
4. Run the narrowest relevant verifier while iterating.
5. Run the phase verifier before claiming the phase passes.
6. Review generated evidence for secrets, host-specific paths, and truthful
   status before committing it.
7. Do not edit measured values, timestamps, hashes, or command results to make
   a failure appear successful.

## Code standards

- C++: C++17, `clang-format`, `clang-tidy`, explicit bounded queues, and GTest.
- Python: Python 3.12, type hints, Ruff, pytest, and validated YAML schemas.
- Go: standard library where practical, `gofmt`, `go vet`, unit tests, and
  race-detector coverage for concurrent process supervision.
- ROS 2: explicit QoS, declared parameters, bounded waits, clean shutdown,
  package dependencies in `package.xml`, and one documented owner per TF edge.
- Shell: Bash, `set -Eeuo pipefail`, ShellCheck, quoted expansions, and argv
  arrays for package or command lists. Never use `eval`.

Install the hooks after Phase 0 succeeds:

```bash
pre-commit install
pre-commit run --all-files
```

## Verification language

Report exactly what ran:

- static review does not prove that a package builds;
- a build does not prove that a ROS graph is healthy;
- a simulated run does not prove physical robot safety;
- a target threshold is not a measured result;
- a local pass is not a public CI pass.

Every benchmark table must link to or name the machine-readable evidence from
which it was derived. Failed and incomplete runs remain visible as failures or
incomplete work.

## Pull requests

A pull request should state the affected phase, design trade-offs, commands
run, evidence paths, and known limitations. Keep generated bags, build trees,
and large transient logs out of Git. Security-sensitive findings should follow
[SECURITY.md](SECURITY.md) instead of a public issue.
