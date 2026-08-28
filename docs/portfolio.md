# RoboTest Lab portfolio notes

## Status and use

The bounded final-status block in the [README](../README.md) is authoritative
for the checkout being viewed. This file deliberately preserves the historical
Phase 1 and Phase 2 development-evidence bullets and does not prewrite a Phase
3, Phase 4, or release result. Those bullets remain valid only with their stated
scope, regardless of whether a later evidence-only commit closes the release
gate.

When the final gate is incomplete, use only the development bullets below. If
an exact final result reports `release_eligible=true`, derive any later-phase
bullet from that result and its generated Phase 3/4 documents; do not turn a
target, implemented feature, or this conditional guidance into a measurement.

## Evidence-bounded project summary

RoboTest Lab is an evidence-first, CPU-only software-in-the-loop robotics test
platform built around ROS 2 Jazzy, Gazebo Harmonic, Nav2, deterministic fault
interfaces, bounded metrics, and process supervision. The two historical runs
below prove the Phase 1 simulation/pass-through scope and the Phase 2
mission/action integration scope at their stated development level. They do
not decide the separate later-phase release gates.

## Development-evidence CV bullets

- Built a CPU-only ROS 2 Jazzy/Gazebo Harmonic robot simulation whose bounded
  Phase 1 development run completed 103 tests with 0 errors or failures,
  measured calculated real-time factor median `0.9999` and p5 `0.9162`, held
  peak launch-group RSS to `795,476 KiB`, and recorded `0.2962 m` of bounded
  displacement in run `20260825T200725Z-1333`.
- Integrated a seeded three-waypoint Nav2 mission/action path whose bounded
  Phase 2 development run returned `SUCCEEDED` for 3/3 ordered waypoints,
  completed 213 tests with 0 errors or failures, measured calculated real-time
  factor median `0.9863` and p5 `0.9302`, and held peak owned-process-group RSS
  to `1,372,244 KiB` in run `20260826T010218Z-466`.
- Produced canonical JSON/CSV and checksum-bound development evidence for both
  scopes: all 45 Phase 1 manifest entries and all 111 Phase 2 manifest entries
  validated in their respective runs.

Every value above resolves to the
[Phase 1 report](results/phase-1/20260825T200725Z-1333.md) or
[Phase 2 report](results/phase-2/20260826T010218Z-466.md). Both reports state
that their worktrees were dirty and their results are development evidence,
not clean-commit release benchmarks.

## Media inventory and semantics

| Asset | What it is | What it may prove |
| --- | --- | --- |
| [Phase 1 RViz screenshot](results/phase-1/rviz-phase1.png) | Genuine cropped screenshot from separate bounded manual run `rviz-20260825T200856Z-2873` | Visual inspection of RobotModel, TF, LiDAR, odometry, and frame alignment for that development run |
| P5-05 Scenario 5 chart | Release projection `docs/results/phase-5/scenario5-localization-error-<C>.png`, added to `E` only from deterministic Phase 3 `runs/12` | The plotted localization-error fields for that exact PASS run, after result/suite/aggregate/manifest/hash/dimension/candidate joins |
| Two rendered Mermaid diagrams | Release projections `architecture-<C>.svg` and `release-flow-<C>.svg`, added to `E` only after render and later hash-bound human review of the exact candidate README fences | Explanatory architecture and release-flow visuals only; never runtime behavior |

A run-derived chart is generated from canonical measurements. A screenshot is
a visual capture of a running tool. Neither may be relabeled as the other, and
generated or mocked robot imagery is not accepted as demonstration evidence.

## Conditional final-release block

This block is active whenever the exact final release-evidence result is absent
or does not report `release_eligible=true`. While it is active, **do not publish
a “completed RoboTest Lab” CV bullet, release badge, or fault-tolerance
benchmark claim.** It is satisfied only when all of the following evidence has
passed:

1. an accepted exact 15-trial Phase 3 campaign for Scenarios 1–5 on one clean
   candidate commit;
2. an accepted privileged Phase 4 package lifecycle and Scenario 6 recovery
   run bound to that candidate;
3. successful public standard-runner CI for candidate `C`;
4. P5-04 clean-checkout README replay and P5-05 genuine chart plus two Mermaid
   renders, with immutable raw attempts and one exact PASS-attempt projection;
5. one allowlisted evidence-only child commit `E`, successful public CI for
   `E`, and its ignored exact-SHA proof; and
6. a final read-only `scripts/verify_all.sh --release-evidence` result with
   `release_eligible=true`.

While any item is unproven, later-phase capability may be described as
implemented or statically verified only where repository evidence supports
that wording. Once the exact final gate proves all six, release claims must cite
the generated evidence-commit documents, run IDs, and final proof rather than
silently broadening the historical bullets above.

The authoritative sequence lives in the
[Phase 5 CI guide](testing/phase5-ci.md). See the
[case study](case-study.md) for the design narrative and
[acceptance criteria](testing/acceptance-criteria.md) for the frozen targets.
