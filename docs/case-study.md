# RoboTest Lab case study

## The problem

Autonomous-navigation demos often stop at visible motion. RoboTest Lab treats
the robot, simulator, fault layer, mission, metrics, recovery supervisor,
package, and release evidence as one testable system. The central question is
not only “did it move?” but “which exact source, scenario, process tree, clock,
and artifact prove the outcome?”

The target is a CPU-only ROS 2 Jazzy and Gazebo Harmonic workflow in WSL2. It
uses Nav2 for autonomy, C++ fault proxies for deterministic sensor effects,
Python for mission control and evidence, and Go for bounded process
supervision. The [system-boundary decision](decisions/0001-system-boundaries.md)
keeps validation data out of autonomy and assigns one owner to each command and
TF boundary.

## Engineering approach

### Determinism before fault injection

Scenarios freeze seeds, routes, fault schedules, timeouts, and numerical
targets before a candidate runs. Fault schedules follow a two-stage
`RESET -> PREPARED -> ARMED` protocol so parsing a schedule cannot activate it.
Simulation time drives simulated events; steady wall time provides bounded
escape deadlines.

### Evidence as a product surface

Each accepted run is designed to produce canonical JSON and a matching CSV,
command and exit metadata, source/configuration identities, resource samples,
and a checksum manifest. Derived reports and charts must resolve back to those
canonical fields. Missing values remain unavailable rather than becoming zero.

### Recovery without a second owner

The Go supervisor owns child process groups, bounded exponential backoff,
loopback health/readiness endpoints, and graceful-then-forced termination. The
Phase 4 design requires the failed group to be empty before replacement and
keeps the systemd unit disabled by default. These surfaces are implemented,
while their privileged authoritative acceptance is deliberately decided by a
separate evidence gate rather than inferred from source. The README final-status
block states whether that gate has passed in the checkout being viewed.

## Historical development evidence

Before final release, the only published portfolio measurements come from the
two checked-in bounded development reports below. Both runs used dirty
worktrees, so neither becomes a release benchmark when later evidence is
added; they remain historical development results.

| Evidence | Scope | Selected measured result |
| --- | --- | --- |
| [Phase 1 run `20260825T200725Z-1333`](results/phase-1/20260825T200725Z-1333.md) | Original model/world, pass-through proxy, bounded motion, headless graph/resource gate | 103 tests with 0 errors/failures; calculated RTF median `0.9999`, p5 `0.9162`; peak launch-group RSS `795,476 KiB`; displacement `0.2962 m` |
| [Phase 2 run `20260826T010218Z-466`](results/phase-2/20260826T010218Z-466.md) | Seeded three-waypoint Nav2 mission/action integration and global resource/isolation gate | Nav2 `SUCCEEDED` after 3/3 waypoints; 213 tests with 0 errors/failures; calculated RTF median `0.9863`, p5 `0.9302`; peak owned-process-group RSS `1,372,244 KiB`; all 111 manifest entries validated |

Phase 1 also has a genuine
[RViz screenshot](results/phase-1/rviz-phase1.png) from a separate bounded
manual inspection. It shows RobotModel, TF, LiDAR, and odometry alignment, but
it is not the headless performance run and does not prove Phase 3 fault or
collision acceptance.

## Release boundary

The historical Phase 1/2 runs never prove the following later scopes. Their
status must be read from the bounded README final-status block and the exact
final release-evidence result in the checkout being reviewed:

- the exact 15-trial Phase 3 candidate campaign for Scenarios 1–5;
- fault injection, recovery metrics, collision-free acceptance, and repeated
  scenario stability;
- the privileged Phase 4 package lifecycle and Scenario 6 recovery acceptance;
  and
- public CI for exact clean candidate `C` and evidence-only child `E`, README
  replay, genuine Phase 3 portfolio media, and the final read-only decision.

If the final result is absent or incomplete, these remain unproven. If it
reports `release_eligible=true`, cite the generated Phase 3/4 result documents,
portfolio proof, public-CI proofs, and final result for later-phase claims; do
not retrofit those claims into the two development rows above.

The [acceptance criteria](testing/acceptance-criteria.md) define the frozen
targets. The [verification matrix](testing/verification-matrix.md) maps each
target to its proof and failure handling.

## Chart versus screenshot

The P5-05 portfolio chart is data-derived evidence: it must be byte-identical
to the accepted Scenario 5 `runs/12` `localization-error.png`, with the run ID,
candidate SHA, canonical PASS result, artifact manifest, dimensions, and hash
revalidated before projection. A screenshot is a capture of a visual tool. The
tracked Phase 1 RViz screenshot is genuine, but it cannot substitute for the
run-derived Scenario 5 chart or be relabeled as benchmark output.

The two README Mermaid diagrams are explanatory source. Portfolio prepare
hash-binds their candidate fences and emits a render request; the operator then
renders outside the attempt. Only after inspecting those exact SVG bytes may a
human create the canonical review consumed by finalization. They are not
simulation evidence.

## Release discipline

Candidate commit `C` remains the source identity for runtime and public-CI
proof. Deterministic results, documentation replay, and media are projected
into one allowlisted evidence-only child `E`. A second successful public-CI run
for `E` is captured in ignored evidence, and only the final read-only
`--release-evidence` gate may declare `release_eligible=true`.

See the [Phase 5 CI guide](testing/phase5-ci.md) and
[portfolio notes](portfolio.md) for the exact publication boundary.
