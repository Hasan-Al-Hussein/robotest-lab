# Evidence ledger

This ledger separates what exists in source, what was actually exercised, and
what remains unqualified. A feature being implemented is not the same as a
runtime or release result.

## Status by workstream

| Workstream | Implemented in the repository | Verified evidence | Planned or unverified |
| --- | --- | --- | --- |
| Robot and simulation boundary | Differential-drive Xacro model, Gazebo world and models, sensors, ROS-Gazebo bridge, TF, and a validating fault-proxy boundary | [Phase 1 bounded simulation result](results/phase-1/20260825T200725Z-1333.md): seeded development run with build/model checks, 103 tests, live graph/sensor checks, bounded motion, resource sampling, and a separate RViz inspection | Hardware behavior, collision-free operation, hardware rendering, and release reproducibility were not established by that run |
| Navigation mission | Map, Nav2 configuration, three-waypoint mission runner, action/provenance capture, and command-chain observations | [Phase 2 bounded mission result](results/phase-2/20260826T010218Z-466.md): Nav2 completed all 3 ordered waypoints in a seeded development run; 213 tests and the scoped runtime gates passed | This is simulation evidence, not field or hardware acceptance |
| Fault campaigns and metrics | Deterministic LiDAR dropout and odometry-drift protocol, static/dynamic obstacle scenarios, collectors, analyzers, aggregation, and the 15-trial runner surface | The clean-source [hosted L2 receipt](results/hosted-ci/README.md) passed the Phase 3 static/interface checks and the non-live test surface | No accepted 15-trial cold-stack campaign is tracked; no Phase 3 benchmark PASS is claimed |
| Supervision and packaging | Go supervisor, owned process-group handling, health/readiness API, systemd unit, Debian packaging, and Phase 4 verification tooling | Hosted L2 checks include Go formatting, vet/race tests, and the Phase 4 static/interface surface | The privileged Phase 4 `--apply` lifecycle and Scenario 6 recovery acceptance were not completed; no installed-service acceptance is claimed |
| Portfolio and release gate | Evidence schemas, checksum/provenance tooling, documentation contracts, and the release-verification pipeline | GitHub Actions run `33274853279` recorded `PASS` for 27/27 L2 checks at clean commit `039effa496abbf69d006ccca729c6ed26116c2eb` | The verifier records `remote_ci_verified=false`; no final release qualification or production-readiness claim is made |

## Clean-source hosted receipt

The archived GitHub Actions artifact was produced on a GitHub-hosted Ubuntu
24.04 runner for exact clean source commit
`039effa496abbf69d006ccca729c6ed26116c2eb`.

| Evidence | Result |
| --- | --- |
| Aggregate checks | `PASS`: 27 passed, 0 failed |
| Pure-Python tests | 1,266 passed in 1,960.12 seconds |
| Colcon result | 1,045 tests, 0 errors, 0 failures, 11 skipped |
| Go race tests | All 4 listed packages passed |
| Ruff | Check passed; 138 files already formatted |
| Live/privileged scope | Gazebo false; hardware false; Phase 1/2 live verifier false; Phase 3 campaign false; Phase 4 apply false; systemd mutation false |

See the [receipt guide](results/hosted-ci/README.md),
[`verification-summary.json`](results/hosted-ci/20260829T210238Z-19597/verification-summary.json),
and [`provenance.json`](results/hosted-ci/20260829T210238Z-19597/provenance.json).

## Editable source and media inventory

The simulation and test-bay assets are source-controlled and editable in text
form:

- seven Xacro files in [`src/robotest_description/urdf/`](../src/robotest_description/urdf/);
- five SDF files across [`src/robotest_sim/worlds/`](../src/robotest_sim/worlds/)
  and [`src/robotest_sim/models/`](../src/robotest_sim/models/);
- the occupancy-grid PGM and YAML in
  [`src/robotest_navigation/maps/`](../src/robotest_navigation/maps/);
- six scenario YAML files in [`scenarios/`](../scenarios/), covering the Phase 2
  baseline and five Phase 3 cases;
- Nav2, bridge, fault-proxy, RViz, and launch configuration under the relevant
  packages in [`src/`](../src/).

The tracked raster media consists of three Gazebo showcase PNGs in
[`docs/showcase/`](showcase/) and one technical RViz PNG in
[`docs/results/phase-1/`](results/phase-1/). A repository inventory found no
tracked video, animated GIF, layered artwork, or CAD/mesh asset. The Xacro,
SDF, map, scenario, and configuration files are therefore the original
editable assets; the PNGs are presentation or inspection outputs, not proof of
an engineering result.

## Environment and preservation boundary

The core local workflow targets ROS 2 Jazzy on Ubuntu 24.04 under WSL2. The
hosted receipt used a native GitHub-hosted Ubuntu 24.04 runner. This repository
contains no tracked Dockerfile or Compose definition, and the preserved CI
artifact does not identify any RoboTest Docker container, image, or volume.
Docker storage on the same computer must therefore be audited by the project
that owns it; it is not RoboTest evidence.

The hosted artifact is preserved as documentation and reproducibility proof,
not as an offline reinstall bundle. It contains logs, summaries, provenance,
source snapshots, and checksums; it intentionally does not contain dependency
caches, a WSL virtual disk, or a runnable environment image.
