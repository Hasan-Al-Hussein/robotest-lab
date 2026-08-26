# RoboTest Lab Verification Matrix

Status: **Phase 0 normative verification plan**

All commands are launched from Windows unless the row says otherwise. The
canonical Linux working directory is `/home/hasan/robotest-lab`.

Common wrapper:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && <command>'
```

Phase verifiers use fail-fast shell settings, preserve command/exit metadata,
emit a machine-readable phase manifest, and run
`colcon test-result --verbose` whenever colcon tests are involved. Expected
evidence is observed output, never a prewritten success message.

## Phase 0 — environment and dependency gate

| Criterion | Proof method | Exact command/check | Expected result and evidence | Owner/reviewer | Failure signal | Recovery trigger |
| --- | --- | --- | --- | --- | --- | --- |
| P0-01 Verified WSL target and preservation | Host inventory plus guest identity | `wsl --version`; `wsl --status`; `wsl -l -v`; then guest `cat /etc/os-release`, `uname -a`, `ps -p 1 -o comm=` | Ubuntu 24.04.x, WSL2, PID 1 systemd; pre-install inventory stored in `docs/environment-audit.md` | Master agent | Wrong distro/version, WSL1, systemd absent, or any command targets `Ubuntu-20.04` | Stop. Correct command targeting or request user action; never unregister a distro |
| P0-02 Exact dependency and disk preflight | Mandatory simulation before install | `scripts/verify_phase0_preinstall.sh` | Manifest `config/apt-packages.txt` resolves; projected project/dependencies <=18 GiB and host reserve >=20 GiB; JSON/text preflight evidence | Master agent | Missing package, untrusted repository, projection/reserve failure, or installer starts before pass | Reduce exact dependencies/artifact budget and repeat preflight; install nothing |
| P0-03 Installed versions | Package/version inventory and tool probes | `scripts/verify_phase0.sh` | Exact resolved `dpkg` manifest plus verified ROS, Gazebo, ros_gz, Nav2, colcon, C++, Python, and Go versions; phase manifest reports pass | Master agent | Missing/mixed distro package, unsupported version, or command failure | Fix only the dependency defect, rerun P0-02 if size changes, then rerun P0-03 |
| P0-04 Repository isolation | Filesystem and Git checks | `findmnt -T /home/hasan/robotest-lab`; `git status --short`; `git branch --show-current` after initialization | Project is on Linux filesystem; branch `codex/robotest-lab-build`; OneDrive repository unchanged | Master agent | Project resolves through `/mnt/c`, wrong repository/branch, or unrelated changes | Stop and return to the canonical root without moving/deleting unrelated files |

Phase 0 passes only after P0-02 passes before installation and P0-03 passes
after installation.

## Phase 1 — minimal simulation and pass-through plane

Canonical command:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_phase1.sh'
```

| Criterion | Proof method | Expected result and evidence | Failure signal | Recovery trigger |
| --- | --- | --- | --- | --- |
| P1-01 Build and static model validity | Four-worker colcon build, package tests, Xacro/URDF/SDF checks | Build/test logs, `colcon test-result --verbose`, validated generated model/world | Warning-as-error, malformed model, invalid inertia/frame, missing dependency | Fix the smallest model/build defect; clean only affected build outputs and rerun |
| P1-02 Headless launch and clock | Timed `gz sim -s -r --headless-rendering` launch through ROS launch | Gazebo, bridge, proxy, robot state publisher start; `/clock` advances; all participating nodes report `use_sim_time=true` | Timeout, render/sensor failure, missing/duplicate clock, mixed time | Lower rendering load or fix launch/bridge; do not proceed to Nav2 |
| P1-03 Data-plane/QoS contract | Live endpoint introspection and rate/stamp probes | Topic graph and QoS evidence match `topic-and-tf-contract.md`; raw and validated pass-through messages correspond | KEEP_ALL, incompatible QoS, unexpected publisher/subscriber, stale/nonmonotonic stamps | Correct QoS/remaps/ownership and repeat graph capture |
| P1-04 TF ownership | Enumerate TF publishers and inspect graph | One publisher per contracted Phase 1 edge; proxy alone owns `odom -> base_footprint`; stored TF graph | Duplicate edge, broken chain, wrong frame or timestamp | Disable original Gazebo odom TF or fix proxy/RSP ownership |
| P1-05 Bounded motion | Publish bounded command, observe validation truth, publish zero | Measured nonzero displacement, final zero command, no continuing motion command, command/pose trace | No motion, wrong direction, multiple command publishers, missing final zero | Stop stack, fix command bridge/kinematics, repeat with conservative command |
| P1-06 Resource target | Process-tree sampler during headless run | CPU affinity 0–5; peak RSS sum <=6 GiB; median RTF >=0.80 and p5 >=0.50; resource CSV | OOM, affinity escape, or target failure | Reduce rays/rates/physics/world complexity before changing hardware assumptions |
| P1-07 RViz inspection | Launch RViz only for bounded manual inspection | Genuine screenshot shows robot model, TF, scan, odometry, and expected frame alignment; command and run ID recorded | Missing display, wrong frame/alignment, stale sensor, or screenshot not tied to the run | Fix model/bridge/TF, rerun headless gates, then repeat inspection |

Phase 1 evidence summary belongs under `docs/results/phase-1/`; large raw
logs/bags remain in the ignored artifact area with tracked hashes.

## Phase 2 — navigation and missions

Phase 2 implements the command boundary in
[ADR 0004](../decisions/0004-phase2-command-ownership.md). It runs an empty,
no-fault baseline and preserves the future PRELOAD/ARM ordering in
[ADR 0003](../decisions/0003-two-phase-fault-schedule-arming.md) without
claiming that fault arming, goal binding, or the future proxy states exist.

Canonical command:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && ROBOTEST_SIM_SEED=42 ROBOTEST_CPUSET=0-5 scripts/verify_phase2.sh'
```

| Criterion | Proof method | Expected result and evidence | Failure signal | Recovery trigger |
| --- | --- | --- | --- | --- |
| P2-01 Map, localization, and Nav2 readiness | Launch test plus lifecycle, map, scan, costmap, TF, and simulation-time probes | Required lifecycle nodes active; map/world alignment and costmap observations valid; exactly one allowed owner for each required TF publisher set; no validation subscriber in autonomy | Inactive node, plugin/config failure, bad map alignment, mixed time, duplicate TF owner, or validation leak | Fix navigation contract before submitting a mission |
| P2-02 Mission schema and exits | Pytest parameterized valid/invalid YAML and terminal outcomes | Unknown/missing/invalid fields rejected; success/cancel/reject/abort/timeout/infrastructure codes distinct; artifact-write failure is nonzero | Invalid mission accepted, exception swallowed, incomplete returns zero, or an artifact failure reports success | Fix schema/state machine and rerun unit tests |
| P2-03 Baseline action integration | Three-waypoint headless action run before the metrics package exists | Seed 42, empty fault schedule, all three waypoints complete, terminal action result is succeeded, and mission YAML, bounded action trace, matching mission-result JSON/CSV, command/exit evidence are preserved | Any missed waypoint, timeout, false success, non-empty fault path, missing trace, or JSON/CSV mismatch | Diagnose first failed layer; rerun only after unit/launch gate is green |
| P2-04 Cancellation and escape timeout | Bounded cancellation fixture plus a frozen-simulation-clock fixture | Goal cancellation is acknowledged and reaches a non-success terminal result; a steady wall escape ends the stopped-clock case; fixture logs and test-result XML are retained | Hanging verifier, success after cancellation/timeout, or no executable fixture evidence | Repair bounded wait/cancel logic before fault work |
| P2-05 Acceptance freeze | Hash target criteria and scenario inputs before Phase 3 | Target-set hash and numerical scenario configurations are frozen before fault results; provenance binds the exact scenario and source files | Target changes after observation, missing hash, or source/install mismatch | Invalidate affected results; create prior decision record and rerun |
| P2-06 Command-chain ownership | Static launch/config tests plus live endpoint, QoS, process, and bounded command-trace probes | Exact ADR 0004 chain: controller only publishes `cmd_vel_nav`, smoother only publishes `cmd_vel_smoothed`, collision monitor only publishes final `cmd_vel`; expected connected subscribers; behavior output has no subscriber; final zero observed | Extra/missing owner, bypass, behavior subscriber, incompatible QoS, `KEEP_ALL`, or missing final zero | Stop the owned process group; repair remaps/plugins/queues and repeat readiness before motion |
| P2-07 Resource, isolation, and evidence integrity | Process-tree sampler, world-statistics windows, graph audit, cleanup audit, canonical artifact finalization | Affinity remains within CPUs 0–5; peak RSS sum <=6 GiB; median RTF >=0.80 and p5 >=0.50; no autonomy validation subscriber; owned PGID is gone; hashes and provenance reconcile | Affinity escape, memory/RTF miss, validation leak, orphan, checksum mismatch, or missing source/version/seed evidence | Preserve failed run, clean only owned processes, fix the first failed layer, and rerun the full Phase 2 verifier |

Phase 2 passes the mission/action integration subset plus the global
resource/isolation gates. Full Scenario 1 collision, path, efficiency, and
repeated-trial acceptance is proved in P3-03 after `robotest_metrics` exists.
Phase 2 evidence must mark those metrics `DEFERRED_TO_PHASE3`; it must not copy
their frozen targets into measured-result fields.

## Phase 3 — metrics and scenarios 1–5

Canonical commands:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_phase3.sh'
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/run_benchmarks.sh'
```

| Criterion | Proof method | Expected result and evidence | Failure signal | Recovery trigger |
| --- | --- | --- | --- | --- |
| P3-01 Fault logic | GTest for schedules, boundaries, deterministic transforms, queues, reset, and events | Same seed/input produces same output; half-open intervals and counts correct; queues bounded | Wall-time dependence, off-by-one interval, odom/TF mismatch, nondeterminism | Isolate pure logic; no simulation run until unit proof passes |
| P3-02 Metric formulas | Pytest fixtures for every formula and invalid edge case | Results match `metrics-contract.md`; nulls and quality failures preserved | Zero substituted for missing, bad angular wrap, contact overcount, hidden sample drop | Correct pure calculation and regenerate all affected reports |
| P3-03 Baseline/static/dynamic | Repeated Scenario 1–3 benchmark runs | Three of three trials per scenario satisfy frozen criteria; individual and aggregate artifacts retained | Missing interaction, collision, timeout, unrecorded trial, threshold miss | Keep failed evidence; simplify/tune within frozen contract, then rerun full set |
| P3-04 LiDAR dropout | Raw/validated/event/cmd/action bag and repeated runs | Raw scans continue, validated scans absent in interval, safe stop/recovery and mission outcome meet Scenario 4 targets | Substitute scan, count mismatch, unsafe motion, false recovery/success | Fix one fault path; repeat Scenario 4 from clean state |
| P3-05 Odometry drift | Raw/validated odom, TF, truth, localization, action and event comparison | Injected offsets and odom/TF consistency meet Scenario 5; localization metrics have required coverage | Duplicate/bypassed TF, wrong drift, raw mutation, missing coverage | Fix TF/data ownership and invalidate affected drift results |
| P3-06 Report integrity | Regenerate Markdown/HTML/PNG from canonical JSON and verify hashes | Every displayed number resolves to JSON field and run ID; target and measurement sections distinct | Hand-entered/untraceable number or hash mismatch | Delete/regenerate only derived artifacts from preserved canonical data |

## Phase 4 — supervision, service, and package

Canonical command:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_phase4.sh'
```

The verifier includes the literal Go checks:

```bash
cd supervisor
go test -race ./...
go vet ./...
```

| Criterion | Proof method | Expected result and evidence | Failure signal | Recovery trigger |
| --- | --- | --- | --- | --- |
| P4-01 Go unit/concurrency behavior | Fake-clock/process tests, race detector, vet | Backoff/exhaustion, signals, groups, heartbeat, endpoints and parsing pass with no race/vet finding | Race, leaked goroutine/process, nondeterministic timing, ignored signal | Fix supervisor before staging or packaging |
| P4-02 Runtime staging and systemd | `scripts/stage_runtime_overlay.sh`, package install, `systemd-analyze verify`, controlled service start/stop | Overlay at `/opt/robotest-lab`; unit disabled by default; condition/hardening valid; journal evidence | Service autostarts, missing condition, home dependency, hardening blocks required path | Stop/disable service, preserve logs/config, repair unit/staging |
| P4-03 HTTP semantics | Loopback HTTP integration tests | Health remains live, readiness tracks children/heartbeat, status JSON and metrics valid; bind is loopback only | False-ready, non-loopback listener, mutation endpoint, unbounded labels | Stop service and repair API/state calculation |
| P4-04 Scenario 6 | Kill configured managed child during active mission | Scenario 6 meets every frozen detection, orphan, restart, readiness, follow-up mission, and evidence criterion | Orphan, duplicate supervisor, infinite restart, interrupted mission called success, recovery target miss | Disable service, diagnose structured timeline, rerun from clean process tree |
| P4-05 Debian lifecycle | lintian plus install/upgrade/remove/purge in target distro | Package manifest correct; modified conffile behavior proven; no unrelated path removed | Package overwrite, unsafe maintainer script, lintian project error, data loss | Stop distribution of package; repair lifecycle and repeat in clean fixture |

Any package lifecycle command requiring elevated privileges must be surfaced
before execution. It may not hide a password prompt or weaken system security.

## Phase 5 — portfolio and remote CI

Local command before the candidate commit:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && scripts/verify_phase5.sh --local'
```

After pushing, remote verification uses the exact pushed SHA:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && scripts/verify_phase5.sh --remote <sha>'
```

| Criterion | Proof method | Expected result and evidence | Failure signal | Recovery trigger |
| --- | --- | --- | --- | --- |
| P5-01 Local CI equivalent | Formatting, lint, unit/build/short smoke workflow locally | Local workflow command log and clean exit | Local/CI command divergence or failure | Fix locally before commit/push |
| P5-02 Public standard-runner CI | Candidate commit, pushed SHA, workflow wait by SHA | Public repository URL and successful workflow URL for exact SHA | Missing auth/repo, wrong SHA, canceled/failed workflow, paid/larger runner | Leave remote criterion incomplete or fix workflow; never claim CI pass |
| P5-03 Evidence commit | Record candidate workflow evidence, create evidence-only commit, push, verify its SHA | Second workflow succeeds; recorded URLs/SHA and repository state reconcile | Evidence file points to another commit/run or second CI fails | Correct evidence and repeat remote verification |
| P5-04 Documentation replay | Execute README setup/run/test commands in clean-environment matrix | Command transcript matches documented paths and expected behavior | Stale command, hidden prerequisite, untested platform claim | Correct documentation or implementation and replay from start |
| P5-05 Media and diagrams | Render Mermaid; capture genuine run media tied to run ID | Valid diagrams and media checksum; displayed behavior matches passing evidence | Mock/generated runtime image represented as evidence, broken diagram, no run link | Re-record/re-render; leave criterion incomplete if capture unavailable |
| P5-06 Licenses and claims | License inventory and claim-to-evidence audit | Apache-2.0 original code; third-party notices; README/CV values resolve to measurements | Unsupported license/claim or placeholder filled without measurement | Remove claim/dependency or document compatible treatment |

## Final gate

Canonical command:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_all.sh'
```

`verify_all.sh` orchestrates the non-manual phase checks and verifies evidence
integrity. Long benchmarks still run through `scripts/run_benchmarks.sh`;
public CI and genuine visual inspection retain their own evidence.

The final reviewer confirms:

1. every required criterion has evidence at the stated verification level;
2. every failure remains visible and resolved before the next phase;
3. README commands and public CI refer to the tested commit;
4. measured numbers resolve to canonical run JSON;
5. Ubuntu 20.04 and unrelated Windows/WSL workloads were not modified; and
6. the tag is created only after all Definition-of-Done items pass.

If any criterion lacks proof, final status is incomplete. Near-completion,
elapsed time, or user expectations never override the gate.
