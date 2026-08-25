#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Ee

SCRIPT_PATH="$(readlink -f -- "$0")"
WORKSPACE="$(cd -- "$(dirname -- "$SCRIPT_PATH")/.." && pwd -P)"
ROBOTEST_SIM_SEED="${ROBOTEST_SIM_SEED:-42}"
if [[ ! "$ROBOTEST_SIM_SEED" =~ ^(0|[1-9][0-9]{0,9})$ ]] ||
  (( 10#$ROBOTEST_SIM_SEED > 4294967295 )); then
  echo "ROBOTEST_SIM_SEED must be an unsigned 32-bit integer" >&2
  exit 2
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"

set -u
set -o pipefail
IFS=$'\n\t'

RUN_ID="rviz-$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIR="$WORKSPACE/artifacts/evidence/phase1/$RUN_ID"
mkdir -p -- "$RUN_DIR"

export ROS_DOMAIN_ID="$((200 + ($$ % 20)))"
export GZ_PARTITION="robotest_phase1_${RUN_ID//[^A-Za-z0-9_]/_}"
GIT_SHA="$(git -C "$WORKSPACE" rev-parse HEAD 2>/dev/null || printf unavailable)"
if [[ -n "$(git -C "$WORKSPACE" status --porcelain=v1 --untracked-files=all 2>/dev/null)" ]]; then
  GIT_DIRTY=true
else
  GIT_DIRTY=false
fi

LAUNCH_PID=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    local launch_pgid=""
    launch_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ' || true)"
    if [[ -n "$launch_pgid" && "$launch_pgid" == "$LAUNCH_PID" ]]; then
      kill -TERM -- "-$launch_pgid" 2>/dev/null || true
      for _ in {1..50}; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 0.1
      done
      kill -KILL -- "-$launch_pgid" 2>/dev/null || true
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

setsid timeout --signal=TERM --kill-after=10s 120s \
  taskset -c 0-5 \
  ros2 launch robotest_sim sim.launch.py \
  headless:=true render_sensors:=true rviz:=true seed:="$ROBOTEST_SIM_SEED" \
  > "$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

printf '%s\n' \
  "run_id=$RUN_ID" \
  "launch_pid=$LAUNCH_PID" \
  "ros_domain_id=$ROS_DOMAIN_ID" \
  "gz_partition=$GZ_PARTITION" \
  "simulator_seed=$ROBOTEST_SIM_SEED" \
  "git_sha=$GIT_SHA" \
  "git_dirty=$GIT_DIRTY" \
  "sim_launch_sha256=$(sha256sum "$WORKSPACE/src/robotest_sim/launch/sim.launch.py" | awk '{print $1}')" \
  "rviz_config_sha256=$(sha256sum "$WORKSPACE/src/robotest_sim/rviz/robotest.rviz" | awk '{print $1}')" \
  | tee "$RUN_DIR/inspection.txt"

deadline=$((SECONDS + 60))
ready=0
while (( SECONDS < deadline )); do
  kill -0 "$LAUNCH_PID" 2>/dev/null || break
  nodes="$(ros2 node list 2>/dev/null || true)"
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -Fxq '/robotest/rviz2' <<< "$nodes" &&
    grep -Fxq '/robotest/scan' <<< "$topics" &&
    grep -Fxq '/robotest/odom' <<< "$topics"; then
    ready=1
    break
  fi
  sleep 1
done

if (( ready != 1 )); then
  echo 'RViz inspection stack did not become ready' >&2
  tail -100 "$RUN_DIR/launch.log" >&2
  exit 1
fi

ros2 node list > "$RUN_DIR/nodes.txt"
ros2 topic list -t > "$RUN_DIR/topics.txt"
ros2 node info /robotest/rviz2 > "$RUN_DIR/rviz-node.txt"
ros2 param get /robotest/rviz2 use_sim_time > "$RUN_DIR/rviz-use-sim-time.txt"
echo "RVIZ_READY run_id=$RUN_ID"

while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  sleep 1
done

set +e
wait "$LAUNCH_PID"
launch_status=$?
set -e
LAUNCH_PID=""

# GNU timeout returns 124 when the deliberately bounded inspection window
# expires. Readiness above proves only that the inspection stack was usable;
# the human visual verdict and screenshot remain separate required evidence.
if [[ "$launch_status" -ne 0 && "$launch_status" -ne 124 ]]; then
  echo "RViz inspection stack exited with status $launch_status" >&2
  exit "$launch_status"
fi

printf '%s\n' \
  "{\"git_dirty\":$GIT_DIRTY,\"git_sha\":\"$GIT_SHA\",\"run_id\":\"$RUN_ID\"," \
  "\"simulator_seed\":$ROBOTEST_SIM_SEED,\"stack_ready\":true," \
  '"visual_verdict":"not_recorded_by_script"}' \
  | tr -d '\n' > "$RUN_DIR/inspection-summary.json"
printf '\n' >> "$RUN_DIR/inspection-summary.json"
echo "RViz inspection window complete; record the visual verdict separately: $RUN_DIR"
