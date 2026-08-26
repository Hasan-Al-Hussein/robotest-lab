# RoboTest Lab Verification Matrix

Status: **Phase 3 normative verification plan, revision 2**

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

Phase 3 uses the exact 15-trial cold-stack order, bounded evidence streams,
canonical result ownership, and artifact byte limits frozen in the
[metrics contract](../architecture/metrics-contract.md). Fault control follows
[ADR 0005](../decisions/0005-phase3-deterministic-fault-protocol.md), while
obstacle and trial mechanics follow
[ADR 0006](../decisions/0006-phase3-scenario-mechanics.md). Candidate benchmark
commands refuse a dirty worktree; development smoke runs are labeled
non-candidate and cannot be aggregated with the clean suite.

Static/build verification only; this command never starts Gazebo or a candidate
trial:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_phase3.sh'
```

The verifier prints the exact `build-binding.json` path. A binding produced
from a dirty development tree is static evidence only and is intentionally
rejected by every runtime stage after a commit changes the Git identity. For
an authoritative run, first commit the final source, require a clean worktree,
rerun this verifier on that exact commit, and use its new binding for four
explicit, non-interchangeable stages with the same candidate ID, domain base,
output root, and build binding:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && CANDIDATE_ID=phase3-candidate-001 && DOMAIN_BASE=100 && OUTPUT_ROOT=artifacts/evidence/phase3-benchmarks && BUILD_BINDING=artifacts/evidence/phase3/VERIFY_RUN_ID/build-binding.json && scripts/run_benchmarks.sh --mode prepare --candidate-id "$CANDIDATE_ID" --domain-base "$DOMAIN_BASE" --output-root "$OUTPUT_ROOT" --build-binding "$BUILD_BINDING" && scripts/run_benchmarks.sh --mode positive-control --candidate-id "$CANDIDATE_ID" --domain-base "$DOMAIN_BASE" --output-root "$OUTPUT_ROOT" --build-binding "$BUILD_BINDING" && scripts/run_benchmarks.sh --mode smoke --candidate-id "$CANDIDATE_ID" --domain-base "$DOMAIN_BASE" --output-root "$OUTPUT_ROOT" --build-binding "$BUILD_BINDING" && scripts/run_benchmarks.sh --mode campaign --candidate-id "$CANDIDATE_ID" --domain-base "$DOMAIN_BASE" --output-root "$OUTPUT_ROOT" --build-binding "$BUILD_BINDING" --authorization I_AUTHORIZE_EXACTLY_15_COLD_STACK_TRIALS_NO_RETRIES'
```

`prepare` is non-runtime provenance freezing, `positive-control` is the
separate collision-pipeline prerequisite, and `smoke` is non-candidate. Only
the authorized `campaign` stage may start the exact ordered 15 cold-stack
candidate trials; it has no replacement-retry path.

| Criterion | Proof method | Expected result and evidence | Failure signal | Recovery trigger |
| --- | --- | --- | --- | --- |
| P3-01 Fault protocol and transforms | GTest plus cross-language known-answer hash fixtures for canonical schedules, RESET/PREPARED/ARMED transitions, boundaries, idempotency, counts, dropout, drift, queues, and reset | Python/C++ canonical bytes and SHA-256 match; preload is inert; exact UUID/T0/hash/generation arm precedes the first fault by >=0.50 s; half-open intervals, event counts, left-composed SE(2), odom/TF identity, and bounded QoS pass | Wall-time dependence, hash mismatch, partial state, conflicting replay accepted, off-by-one interval, odom/TF mismatch, nondeterminism, or pre-arm effect | Isolate pure logic and interface contract; no simulation run until unit and service fixtures pass |
| P3-02 Metric formulas and bounds | Pytest fixtures for every formula, invalid edge case, collector capacity, artifact cap, collision coverage, positive control, plan leg/hash, TF alignment, and recovery candidate | Results match `metrics-contract.md`; nulls and quality failures preserved; every first overflow fails closed; JSON/CSV reconcile | Zero substituted for missing, bad angular wrap, leg mix-up, contact overcount, silent topic called collision-free, hidden sample drop/truncation, or permissive recovery | Correct pure calculation/collector first and regenerate all derived artifacts |
| P3-03 Contact-pipeline qualification | Manifest-v3 topology/source inventory, installed-ELF digest binding, strict gate/runtime liveness evidence, and a bounded positive-control run before the candidate suite | Sole private bridge feeds only the compiled gate; sole public gate emits seeded, strict, nonempty authoritative delivered-state snapshots with <=0.22 s gaps; the running ELF digest matches the manifest source inventory; all rendered collisions are covered; the expected contact stops within 0.10 s; a wall-absent snapshot `q` strictly beyond final-zero+0.25 s is retained and collector-acknowledged; final zero, hashes, buffers, topology continuity, and cleanup pass | Extra/missing endpoint, stale/conflicting ELF digest, bridge/gate exit, unseeded/silent/regressing/oversize stream, missing/unknown/non-robot collision, chassis-only coverage, only ground/internal contact, fabricated/equal-boundary drain, stale control hash, overflow, or orphan | Fix the private bridge/gate/source-binary/snapshot or collector-ack path, rebuild from current source, regenerate the manifest, and repeat the positive control before any candidate trial |
| P3-04 Baseline/static/dynamic | Repeated Scenario 1–3 cold-stack runs with bounded controllers and validation world-pose evidence | Three of three trials per scenario satisfy frozen mission/path/collision/resource criteria; Scenario 2 proves post-plan insertion plus a changed safe leg-0 path; Scenario 3 proves the exact trajectory and collision-monitor stop response; individual and aggregate artifacts retained | Missing interaction/pose, collision, timeout, unrecorded trial, action retry, threshold miss, or source/config mismatch | Keep failed evidence; fix the first product or harness layer without changing targets, commit, then start a new full candidate set |
| P3-05 LiDAR dropout | Raw/validated/event/command/action traces and three cold-stack runs | Raw scans continue, validated scans are absent only in the half-open interval, counts reconcile, source timeout is exactly 0.60 s, final command becomes/remains safe, the complete 1.0 s recovery window passes, and the mission outcome meets Scenario 4 | Substitute/stale scan, count mismatch, unsafe motion, incomplete stability evidence, false recovery/success, or missing arm binding | Fix one fault/control/metric path; invalidate the affected candidate suite and restart from clean state |
| P3-06 Odometry drift | Raw/validated odom, proxy TF, truth, localization, action, and event comparison across three cold-stack runs | `T_validated * inverse(T_raw)` matches the configured end offset at a sample <=0.25 s before interval end; raw odom, twist, covariance, stamps remain valid; odom/TF agree; localization coverage and mission criteria pass | Duplicate/bypassed TF, wrong composition/endpoint, raw mutation, covariance/twist change, missing coverage, or false action verdict | Fix TF/data ownership and invalidate affected drift results before a new candidate suite |
| P3-07 Suite independence, resources, and cleanup | Orchestrator ledger, source/install binding, per-run process/resource probes, exact ordered run IDs, and post-run process/graph audit | Clean commit; exactly 15 immutable runs in order; unique domains/partitions/PGIDs; reset confirmed; retries zero; every run meets affinity/RSS/RTF/isolation/integrity gates; no survivor or reused state | Dirty source, replacement attempt, warm state, resource miss, validation leak, orphan, checksum or provenance mismatch | Preserve the failed suite; clean only owned processes; fix/commit and restart all 15 candidate runs |
| P3-08 Report integrity | Regenerate JSON/CSV/Markdown/HTML/PNG and aggregate tables from canonical run JSON, then verify hashes and caps | Sole per-run verdict and every displayed number resolve to source JSON/run ID; nearest-rank aggregation, target/measurement separation, byte caps, and full 3/3 denominators pass | Hand-entered/untraceable number, dropped failure, default percentile, JSON/CSV mismatch, cap overrun, or hash mismatch | Delete and regenerate only derived artifacts from preserved canonical data; never edit benchmark values manually |

## Phase 4 — supervision, service, and package

Non-mutating preflight only (never a Phase 4 release verdict):

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_phase4.sh'
```

Authoritative Phase 4 command, after the Phase 3 source tree and audited
package/lifecycle evidence are stable:

```powershell
wsl -d Ubuntu -u root -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_phase4.sh --apply'
```

The bare command proves only static/unit/package-source readiness and emits no
canonical Phase 4 verdict. Only the explicit privileged `--apply` run may emit
`scenario6-result.json` with `verdict.status = PASS` and
`verdict.accepted = true`; that compositor requires the live staging, package
lifecycle binding, service, crash/recovery, follow-up mission, manifest, source
stability, and owned-cleanup gates all to pass.

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
| P4-04 Scenario 6 | Exact executable/lineage revalidation and signal of the nested controller during an active mission; exact sequence-bounded event-chain and mission-completion reconciliation | Scenario 6 meets every frozen detection, orphan, restart, readiness, follow-up mission, event-causality, and evidence criterion; every published original/replacement/harness PGID is empty at cleanup | Identity spoof/drift, unrelated or duplicate transition, orphan, duplicate supervisor, infinite restart, missing/mismatched mission completion, interrupted mission called success, recovery target miss | Retain runtime resources unless service inactivity is proven; diagnose structured timeline, then rerun from a clean process tree |
| P4-05 Debian lifecycle | Exact-schema lifecycle evidence bound to both package bytes, plus direct installed-state re-observation after lintian and install/upgrade/remove/purge in the target distro | Modified conffile is preserved on upgrade/remove and removed on purge; sentinel/state/log survive; final vendor config, version, payload hashes, modes/owners, groups, installed smoke, zero stale dpkg artifacts, disabled/inactive service, and clean dpkg verification all match | Truncated/extra/tampered PASS JSON, package overwrite, unsafe maintainer script, lintian project error, wrong final state, stale artifact, or data loss | Stop distribution of package; repair lifecycle and repeat in clean fixture |

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

Canonical local aggregate command (always exits 3/INCOMPLETE after successful
static/local routing):

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && taskset -c 0-5 scripts/verify_all.sh'
```

Final read-only evidence command, with every path selected explicitly by the
reviewer:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && scripts/verify_all.sh --release-evidence --local-aggregate <exact-prior-verify-all.json> --phase3-candidate-root <exact-phase3-candidate-root> --phase3-aggregate <exact-phase3-candidate-root>/aggregate/aggregate-result.json --phase4-run-directory <exact-phase4-run-directory> --phase4-scenario6 <exact-phase4-run-directory>/scenario6-result.json --phase5-remote-proof docs/results/phase-5/remote-<candidate-sha>.json --phase5-evidence-commit-remote-proof artifacts/evidence/phase5/remote-evidence-commit/remote-<evidence-commit-sha>.json'
```

Bare `verify_all.sh` is the only mode that orchestrates phase verifiers, and it
invokes Phase 5 explicitly as `--local`. `--release-evidence` invokes no phase
verifier and performs no live, campaign, privileged, remote, or publication
operation. It does not discover `latest`: it revalidates only the exact prior
local aggregate, Phase 3 candidate/aggregate, Phase 4 run/Scenario 6, and
tracked candidate proof plus ignored second-CI proof supplied by the reviewer.
The prior aggregate records the bounded tracked Phase 0 refresh delta; any
source/configuration change remains a failure. Before evidence commit E, run
the deterministic `tests/phase5_release_docs.py` producer on those exact Phase
3/4 inputs. E must add exactly the six derived Phase 3/4 JSON/CSV/Markdown
documents, the two bounded README region replacements, the exact
`config/release-claims.json` extension, the prior local/Phase 0 evidence, and
the candidate remote-proof trio; every changed path must be a regular `100644`
blob. Release mode reconstructs README and claims from `git show C`, rejects
extra or missing E paths, and never discovers a latest run. Only one clean
candidate SHA, complete canonical PASS evidence, an exact evidence-only child
commit, and a successful exact-SHA CI run for that child can produce
`release_eligible=true`.
Long benchmarks still run through
`scripts/run_benchmarks.sh`; public CI and genuine visual inspection retain
their own evidence.

The final reviewer confirms:

1. every required criterion has evidence at the stated verification level;
2. every failure remains visible and resolved before the next phase;
3. README commands and public CI refer to the tested commit;
4. measured numbers resolve to canonical run JSON;
5. Ubuntu 20.04 and unrelated Windows/WSL workloads were not modified; and
6. the tag is created only after all Definition-of-Done items pass.

If any criterion lacks proof, final status is incomplete. Near-completion,
elapsed time, or user expectations never override the gate.
