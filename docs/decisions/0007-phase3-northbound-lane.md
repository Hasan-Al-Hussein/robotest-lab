# ADR 0007: Phase 3 Northbound Lane Revision

- Status: Accepted for Phase 3 target-set revision 3; runtime evidence pending
- Date: 2026-08-27
- Decision owners: RoboTest Lab navigation, safety, scenario, and benchmark gate

## Context

ADR 0006 froze the original Phase 3 route with waypoint 1 at `(0.0, 0.0)`
and waypoint 2 at `(0.0, 3.5)`. Development smoke
`phase3-0d86ff3-002`, bound to Git `0d86ff3baa904db6094992fc4c61df0548c86086`,
used that route and reached the final leg but exposed a deterministic
safety-margin fragility in the `1.2 m` divider opening. At simulation time
`84.760 s`, ground truth placed the robot centre at approximately `x=0.340 m`.
The east divider begins at `x=0.600 m`, while the enabled collision-monitor
stop polygon extends `0.270 m` laterally. The resulting overlap was
approximately `0.010 m`, so the collision monitor correctly commanded STOP
and Nav2 could not finish the leg.

This observation is diagnostic development evidence, not an accepted result.
The affected run remains immutable and cannot qualify under the revised target
set. The correction must be frozen before any replacement candidate is run.

Shrinking the stop polygon would remove safety margin from the padded physical
footprint. Widening the divider would also require changing the mechanically
generated map and the Scenario 3 actor's deliberately embedded parking poses.
Neither change is justified by a route-clearance defect.

## Decision

Phase 3 target-set revision 3 retains the start pose and first waypoint, but
defines a west-offset northbound lane:

```yaml
start_pose: {x: 0.0, y: -3.5, yaw: 0.0}
waypoints:
  - {x: -2.0, y: -3.5, yaw: 0.0}
  - {x: -0.2, y: 0.0, yaw: 0.0}
  - {x: -0.2, y: 3.5, yaw: 0.0}
```

Both final-leg endpoints move together so the intended passage segment is
vertical at `x=-0.2`; moving only the final goal would produce a diagonal whose
centre shifts by only about `0.086 m` at divider latitude. All five Phase 3
scenarios use the revised route. The Phase 2 baseline remains unchanged at
`(0.0, 0.0)` and `(0.0, 3.5)` because this decision revises only the Phase 3
fault-benchmark target set.

The following contracts remain unchanged:

- the world SDF, generated PGM/YAML map, and `1.2 m` divider opening;
- the robot footprint, costmap padding, and `+/-0.27 m` PolygonStop half-width;
- three ordered waypoints, seed `42`, timeouts, zero-collision requirement,
  and every scenario-specific fault/actor schedule;
- Scenario 3's actor path through `x=0`, which still intersects the revised
  lane's stop envelope; and
- the real-time-factor, resource, localization, and evidence-quality targets.

## Verification and evidence consequences

Static verification must prove that all five Phase 3 scenario documents and
both strict schemas encode the revised route, while the Phase 2 baseline still
encodes its historical route. The navigation geometry test must also prove
that the configured PolygonStop envelope for the revised vertical segment
retains at least `0.10 m` nominal west clearance. With a conservative
`0.35 m` eastward centre-error allowance and `0.10 rad` heading-error
allowance, it must retain at least `0.15 m` east clearance. These guards cover
the direction and magnitude exposed by the diagnostic smoke without changing
the runtime safety response.

Every scenario SHA-256 and the acceptance-criteria target-set SHA-256 changes.
The new ADR is part of both the source configuration and source-tree build
binding. Therefore no result produced under revision 2, including the smoke
that motivated this decision, may be rebound or promoted. A clean commit,
fresh build binding, positive control, smoke, and the complete no-retry cold
campaign are required.

## Consequences

The route correction preserves the safety monitor instead of weakening it and
avoids a world/map/actor cascade. It deliberately changes the benchmark target,
so comparison across revisions must use the recorded scenario and target-set
hashes. It does not change or waive the independent real-time-factor target,
which the diagnostic smoke also missed. If repeated cold runs still enter
PolygonStop at the static divider, the route is not widened post hoc; the
candidate fails and navigation or localization must be diagnosed under a later
prior decision.
