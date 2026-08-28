# RoboTest Lab

RoboTest Lab is a CPU-only ROS 2 software-in-the-loop platform for testing,
measuring, fault-injecting, observing, and recovering an autonomous
differential-drive robot.

> **Current status:** the Phase 1 and Phase 2 development runs remain bounded,
> non-release evidence. The Phase 3–5 implementation and static verification
> surfaces are complete; their authoritative campaign, privileged acceptance,
> public-CI, and final release-evidence gates remain separate.

<!-- ROBOTEST_RELEASE_STATUS_START -->
## Final acceptance status

Implementation and static verification are complete. Authoritative Phase 3
campaign evidence, Phase 4 privileged acceptance evidence, and Phase 5 public-CI
and release-evidence validation remain pending.
<!-- ROBOTEST_RELEASE_STATUS_END -->

## Start here

- [Set up and verify the Ubuntu environment](#phase-0-workflow).
- [Run the bounded Phase 1 and Phase 2 development gates](#phase-1-verification).
- [Run the Phase 3–5 static gates](#phase-3-5-static-gates) before any
  separately authorized benchmark, privileged package, or remote-CI step.
- Read the [acceptance criteria](docs/testing/acceptance-criteria.md) and
  [verification matrix](docs/testing/verification-matrix.md) before producing
  evidence.
- Use the [troubleshooting guide](docs/troubleshooting.md) when a gate stops or
  returns incomplete.
- See the [case study](docs/case-study.md) for the engineering narrative and
  the [portfolio notes](docs/portfolio.md) for evidence-bounded project and CV
  language.

## Why this project exists

The goal is not another “robot moves in Gazebo” example. The system
connects a small original Gazebo Harmonic robot and world to ROS 2 Jazzy,
Nav2, deterministic sensor-fault proxies, mission validation, measured result
artifacts, and a bounded Go process supervisor. Every result shown here must be
traceable to a command, configuration hash, and recorded run.

## Verified platform boundary

- Target environment: the WSL2 distribution named `Ubuntu`, verified as
  Ubuntu 24.04.1 LTS (Noble).
- Preserved environment: `Ubuntu-20.04` is out of scope and must never be
  changed by project scripts.
- Project root: `/home/hasan/robotest-lab` on the WSL-native filesystem.
- Runtime target: ROS 2 Jazzy, Gazebo Harmonic, Nav2, C++17, Python 3.12, and
  Go 1.22 or newer.
- Resource policy: headless by default, no GPU requirement, no Docker in the
  core workflow, at most 18 GiB projected project/dependency storage, and at
  least 20 GiB projected free space retained on the Windows host drive.

The [environment audit](docs/environment-audit.md),
[dependency plan](docs/dependencies.md),
[Phase 1 result](docs/results/phase-1/20260825T200725Z-1333.md), and
[Phase 2 result](docs/results/phase-2/20260826T010218Z-466.md) distinguish
measured evidence from planned work.

## Architecture

<!-- ROBOTEST_PORTFOLIO_DIAGRAM: architecture -->
```mermaid
flowchart LR
    Gazebo["Gazebo Harmonic\nheadless by default"]
    Bridge["ros_gz_bridge"]
    Proxy["C++ sensor fault proxy"]
    Nav2["Nav2 + localization"]
    Mission["Python mission runner"]
    Metrics["Metrics + validation"]
    Results["JSON / CSV / report"]
    Supervisor["Go supervisor\nbounded recovery"]

    Gazebo --> Bridge --> Proxy --> Nav2 --> Mission --> Metrics --> Results
    Supervisor -. monitors .-> Gazebo
    Supervisor -. monitors .-> Nav2
    Supervisor -. monitors .-> Mission
```

This diagram describes the implemented design. The Phase 1 and Phase 2 results
remain limited to their documented development scopes; they do not imply that
the later campaign, supervisor, packaging, or public-CI acceptance gates ran.

The release path keeps runtime evidence, tracked projections, public-CI proof,
and the final decision separate:

<!-- ROBOTEST_PORTFOLIO_DIAGRAM: release-flow -->
```mermaid
flowchart TD
    C["Clean candidate commit C"]
    CandidateCI["Push C and complete successful public CI\nfor exact C"]
    Runtime["Phase 3 campaign and Phase 4 --apply\nexact caller-selected evidence"]
    Portfolio["Capture and finalize P5-04 replay and P5-05 media\nfrom C and accepted run evidence"]
    Local["Bare verify_all\nexpected INCOMPLETE local aggregate"]
    CandidateProof["Capture the completed exact-C\npublic-CI proof"]
    Projection["Generate Phase 3 and Phase 4 documents\nand project the six portfolio files"]
    E["Evidence-only child commit E"]
    EvidenceCI["Capture successful public CI\nfor exact E in ignored evidence"]
    Final["Read-only --release-evidence gate\nrelease_eligible only if every join passes"]

    C --> CandidateCI --> Runtime --> Portfolio --> Local
    Local --> CandidateProof --> Projection --> E --> EvidenceCI --> Final
```

P5-05 treats a run-derived chart and a screenshot as different evidence. The
release portfolio chart must be a byte-identical projection of an accepted
Phase 3 Scenario 5 `runs/12` `localization-error.png`, joined to its run ID,
manifest, candidate SHA, and canonical PASS result. The existing
[Phase 1 RViz image](docs/results/phase-1/rviz-phase1.png) is a genuine manual
screenshot from a separate development run; it is not a benchmark chart and
cannot substitute for the Phase 3 artifact. The exact capture and projection
order is documented in the
[Phase 5 CI gate](docs/testing/phase5-ci.md#documentation-replay-and-portfolio-media).

## Phase 0 workflow

Run these commands only inside the verified Ubuntu 24.04 distribution. The
repository and dependency scripts reject Ubuntu 20.04.

The repository-source check is read-only. It is expected to fail until the
pinned official ROS source package has been applied.

```bash
cd /home/hasan/robotest-lab
scripts/setup_ros2_repository.sh --check
```

The first mutating setup step requires explicit authorization and the literal
`--apply` flag:

```bash
cd /home/hasan/robotest-lab
scripts/setup_ros2_repository.sh --apply
```

Simulate the exact manifest and enforce storage/reserve gates before installing
anything:

```bash
cd /home/hasan/robotest-lab
scripts/verify_phase0_preinstall.sh
```

Install the validated manifest, initialize rosdep, create the local tooling
environment, and run the post-install verifier only with explicit
authorization:

```bash
cd /home/hasan/robotest-lab
scripts/install_dependencies.sh --apply
```

The post-install Phase 0 verifier is repeatable and read-only:

```bash
cd /home/hasan/robotest-lab
scripts/verify_phase0.sh
```

The bare aggregate is also read-only. It intentionally exits `3`
(`INCOMPLETE`) until exact caller-selected release evidence is validated
separately:

```bash
cd /home/hasan/robotest-lab
scripts/verify_all.sh
```

Machine-readable Phase 0 results are written below
`artifacts/evidence/phase0/`. Generated evidence records what actually ran;
it must not be edited to manufacture a passing result.

## Phase 1 verification

Run the bounded automated gate inside the verified Ubuntu distribution:

```bash
cd /home/hasan/robotest-lab
taskset -c 0-5 scripts/verify_phase1.sh
```

The seeded development run `20260825T200725Z-1333` passed its build, model,
headless runtime, topic/QoS, stamp, trace, motion, resource, and artifact-hash
checks with `simulator_seed=42` and seed status
`fixed_simulator_seed_recorded`. Headline observations were 103 tests with zero
errors or failures, calculated RTF median `0.9999`, calculated RTF p5 `0.9162`,
peak launch-group RSS `795,476 KiB`, and bounded displacement `0.2962 m`. Ten
checks were skipped because `ament_cppcheck` declines cppcheck 2.13 for its
known performance issue.

Evidence:

- [full Phase 1 report](docs/results/phase-1/20260825T200725Z-1333.md)
- [compact canonical JSON](docs/results/phase-1/20260825T200725Z-1333.json)
- [matching one-row CSV](docs/results/phase-1/20260825T200725Z-1333.csv)
- [separate P1-07 RViz screenshot](docs/results/phase-1/rviz-phase1.png)

The 45-entry run checksum manifest validated completely. The headless run was
made from a dirty worktree and the RViz evidence came from a separate bounded
run, so neither is presented as a release benchmark. Fast DDS exposed live
reliability and durability but reported endpoint history/depth as
`UNKNOWN`/`0`; bounded depths retain static/source-and-test support. The contact
topic was silent, so collision absence is not established. Live TF endpoint and
edge sets were recorded, but per-edge publisher-GID attribution was unavailable.

## Phase 2 verification

Run the bounded, headless mission/action gate inside the verified Ubuntu
distribution:

```bash
cd /home/hasan/robotest-lab
ROBOTEST_SIM_SEED=42 ROBOTEST_CPUSET=0-5 scripts/verify_phase2.sh
```

The development run `20260826T010218Z-466` passed the scoped Phase 2 gate with
seed `42`, six logical CPUs (`0-5`), an empty fault schedule, and software
rendering through `llvmpipe`. Nav2 returned `SUCCEEDED` after all three ordered
waypoints. The authoritative UUID-matched action-status acceptance stamp was
`68.308 s`; the Jazzy `rclcpp_action` response stamp was retained separately as
zero/unavailable, and the terminal observation was `156.994 s`.

Headline observations were 213 tests with zero errors and failures, calculated
RTF median `0.9863`, calculated RTF p5 `0.9302`, and peak owned-process-group
RSS `1,372,244 KiB`. All 111 checksum-manifest entries validated. Source/install
binding, source/configuration non-mutation, bounded process cleanup, mission
JSON/CSV reconciliation, action ownership, command/ground-truth correlation,
and an independent post-run evidence audit passed.

Evidence:

- [full Phase 2 report](docs/results/phase-2/20260826T010218Z-466.md)
- [compact canonical JSON](docs/results/phase-2/20260826T010218Z-466.json)
- [matching one-row CSV](docs/results/phase-2/20260826T010218Z-466.csv)

This is bounded mission/action acceptance, not full Scenario 1 acceptance. The
dirty worktree makes it development evidence only. Fault injection and recovery
were not exercised by that Phase 2 run; their authoritative acceptance, along
with collision count, path length, path efficiency, and repeated-run evidence,
remains in the separate Phase 3 gate. Fast DDS
reported live history/depth as `UNKNOWN`/`0`; bounded queues retain static
source-and-test proof. Required TF edges and endpoint sets were observed, but
Jazzy callbacks did not provide per-edge publisher-GID attribution.

## Phase 3-5 static gates

These commands exercise source, build, unit, policy, and short non-live checks.
They do not authorize the Phase 3 campaign, the privileged Phase 4 lifecycle
run, GitHub publication, or a release verdict.

```bash
cd /home/hasan/robotest-lab
taskset -c 0-5 scripts/verify_phase3.sh
```

```bash
cd /home/hasan/robotest-lab
taskset -c 0-5 scripts/verify_phase4.sh
```

```bash
cd /home/hasan/robotest-lab
scripts/verify_phase5.sh --local
```

The Phase 3 campaign and Phase 4 `--apply` run have explicit authorization,
clean-source, and evidence-binding requirements. Follow the canonical commands
and order in the [verification matrix](docs/testing/verification-matrix.md),
then use the [Phase 5 CI gate](docs/testing/phase5-ci.md) for candidate and
evidence-commit proof. A successful static gate is not runtime or release
acceptance.

## Safety properties of setup

- Mutating setup scripts require an explicit `--apply` argument.
- The apt manifest is parsed as literal package-name arguments; shell
  evaluation and `xargs` interpolation are not used.
- The ROS apt-source package is pinned and SHA-256 verified before installation.
- Dependency installation is blocked if projected usage exceeds 18 GiB or
  would leave less than 20 GiB free on the Windows host drive.
- No script unregisters, exports, imports, terminates, or changes the default
  of any WSL distribution.

## Roadmap and evidence gates

<!-- ROBOTEST_RELEASE_ROADMAP_START -->
| Phase | Status | Deliverable and gate |
| --- | --- | --- |
| 0 | Verified locally | Environment foundation, exact install manifest, versions, and resource gates |
| 1 | Verified development | Original robot/world, pass-through proxy, headless gates, and separate RViz evidence |
| 2 | Verified development | Bounded seeded Nav2 mission/action result, command chain, matching JSON/CSV, and global resource/isolation gates |
| 3 | Implemented; authoritative status is the bounded final-status block | Repeatable LiDAR-dropout and odometry-drift campaigns with metrics |
| 4 | Implemented; authoritative status is the bounded final-status block | Go supervisor, systemd and Debian package with lifecycle proof |
| 5 | Static-ready; authoritative status is the bounded final-status block | Public CI and portfolio evidence on a documented clean commit |
<!-- ROBOTEST_RELEASE_ROADMAP_END -->

## Repository layout

```text
src/                 ROS 2 packages
supervisor/          Go supervisor
scenarios/           Versioned mission and fault scenarios
config/              Shared manifests and resource policy
scripts/             Setup and evidence-producing verification
packaging/debian/    Debian packaging
tests/               Cross-package and system tests
benchmarks/          Benchmark definitions and bounded outputs
docs/                Architecture, decisions, testing, and results
artifacts/evidence/  Machine-readable verification evidence
```

## Documentation map

- [Architecture contracts](docs/architecture/metrics-contract.md) define the
  evidence and metric semantics.
- [Acceptance criteria](docs/testing/acceptance-criteria.md) freeze pass/fail
  targets; the [verification matrix](docs/testing/verification-matrix.md) maps
  each target to proof.
- [Troubleshooting](docs/troubleshooting.md) preserves failed evidence and
  diagnoses the first failing layer.
- [Case study](docs/case-study.md) explains the engineering choices and the
  current evidence boundary.
- [Portfolio notes](docs/portfolio.md) preserve source-linked Phase 1 and Phase
  2 development bullets and condition every later-phase claim on the exact
  final release gate.
- [Phase 5 CI and release order](docs/testing/phase5-ci.md) defines local CI,
  public proof, documentation replay, genuine-media projection, and the final
  read-only decision.

## Contributing and security

See [CONTRIBUTING.md](CONTRIBUTING.md) for the evidence-first development
workflow and [SECURITY.md](SECURITY.md) for the local-only security boundary.

## License

Original RoboTest Lab code is licensed under the
[Apache License 2.0](LICENSE). Third-party software retains its own license;
see [NOTICE.md](NOTICE.md).
