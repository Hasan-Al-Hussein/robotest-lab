# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

# Version 1 numeric values are wire-contract values. Do not renumber them.
uint32 SCHEMA_VERSION=1

uint8 TARGET_UNSPECIFIED=0
uint8 TARGET_SCAN=1
uint8 TARGET_ODOM=2
uint8 TARGET_IMU=3

uint8 MODE_PASS_THROUGH=0
uint8 MODE_LIDAR_DROPOUT=1
uint8 MODE_LIDAR_FROZEN=2
uint8 MODE_LIDAR_GAUSSIAN_NOISE=3
uint8 MODE_ODOM_DRIFT=4
uint8 MODE_ODOM_DELAY=5
uint8 MODE_IMU_DROPOUT=6
uint8 MODE_SENSOR_OUTAGE=7

# Append-only schema version. Consumers must reject unsupported versions.
uint32 schema_version

# Stable within one canonical schedule.
string<=64 fault_id
uint8 target
uint8 mode

# Both values are measured in simulation time from the supplied mission start.
builtin_interfaces/Duration start_offset
builtin_interfaces/Duration duration
uint64 seed

# Canonical JSON object containing parameters defined by the selected mode.
# Keeping extension data in one bounded field avoids ambiguous positional values.
string<=1024 parameters_json
