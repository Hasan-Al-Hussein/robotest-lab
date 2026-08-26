# robotest_interfaces

Bounded ROS 2 interfaces for the RoboTest Lab deterministic fault boundary.

Phase 3 uses `PreloadFaultSchedule` to validate an inert canonical schedule,
then `ArmFaultSchedule` to bind that exact generation to one accepted
`FollowWaypoints` UUID and authoritative `T0`. `FaultEvent` version 2 carries
machine-readable control, application, counter, and restoration evidence.

The normative wire and hashing rules are in
`docs/decisions/0005-phase3-deterministic-fault-protocol.md`.
