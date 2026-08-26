#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

# This sourced helper never signals a numeric launch PID or PGID. Launch TERM,
# KILL, and existence probes use PIDFD_SIGNAL_PROCESS_GROUP through the retained
# proc-directory descriptor. Bounded waits may signal only their own exact timer
# child; the verifier likewise bounds shutdown of its exact sampler child.
# The PHASE1_GROUP_* assignments are the sourced helper's result API.
# shellcheck disable=SC2034

phase1_record_signal() {
  local signal_status="$1"

  : "${PENDING_SIGNAL_STATUS:=}" "${SIGNAL_DEFER_DEPTH:=0}" "${FINALIZED:=0}"
  [[ -n "$PENDING_SIGNAL_STATUS" ]] || PENDING_SIGNAL_STATUS="$signal_status"
  if (( SIGNAL_DEFER_DEPTH == 0 && FINALIZED == 0 )); then
    exit "$PENDING_SIGNAL_STATUS"
  fi
}

phase1_begin_signal_deferral() {
  : "${SIGNAL_DEFER_DEPTH:=0}"
  SIGNAL_DEFER_DEPTH=$((SIGNAL_DEFER_DEPTH + 1))
}

phase1_end_signal_deferral() {
  : "${PENDING_SIGNAL_STATUS:=}" "${SIGNAL_DEFER_DEPTH:=0}" "${FINALIZED:=0}"
  (( SIGNAL_DEFER_DEPTH > 0 )) || return 2
  SIGNAL_DEFER_DEPTH=$((SIGNAL_DEFER_DEPTH - 1))
  if (( SIGNAL_DEFER_DEPTH == 0 && FINALIZED == 0 )) \
    && [[ -n "$PENDING_SIGNAL_STATUS" ]]; then
    exit "$PENDING_SIGNAL_STATUS"
  fi
}

phase1_release_launch_gate() {
  local go_fd="$1"
  local token_variable="$2"
  local timestamp_variable="$3"
  local release_status=0
  local release_timestamp=""

  if [[ ! "$go_fd" =~ ^[0-9]+$ \
    || ! "$token_variable" =~ ^[a-zA-Z_][a-zA-Z0-9_]*$ \
    || ! "$timestamp_variable" =~ ^[a-zA-Z_][a-zA-Z0-9_]*$ \
    || "$token_variable" == "$timestamp_variable" ]]; then
    return 2
  fi
  local -n token_ref="$token_variable"
  local -n timestamp_ref="$timestamp_variable"

  phase1_begin_signal_deferral
  timestamp_ref=""
  if [[ -z "$token_ref" ]]; then
    release_status=2
  elif ! release_timestamp="$(date -u +%Y-%m-%dT%H:%M:%SZ)"; then
    release_status=1
  elif ! printf '%s\n' "$token_ref" >&"$go_fd"; then
    release_status=1
  else
    timestamp_ref="$release_timestamp"
    token_ref=""
  fi
  phase1_end_signal_deferral
  return "$release_status"
}

phase1_process_group_snapshot() {
  local pgid="$1"
  local process_table=""
  process_table="$(LC_ALL=C ps -eo pid=,pgid=,sid=,stat=,comm=)" || return 2
  awk -v group="$pgid" '$2 == group {print $1, $2, $3, $4, $5}' \
    <<< "$process_table"
}

phase1_process_group_has_live_members() {
  local pgid="$1"
  local process_table=""
  process_table="$(LC_ALL=C ps -eo pgid=,stat=)" || return 2
  awk -v group="$pgid" \
    '$1 == group && substr($2, 1, 1) != "Z" {found=1} END {exit !found}' \
    <<< "$process_table"
}

phase1_cleanup_record_event() {
  local evidence_path="$1"
  local stage="$2"
  local detail="$3"
  local event_time=""

  event_time="$(date -u +%Y-%m-%dT%H:%M:%SZ)" || return 1
  printf '%s\tevent\t%s\t-\t-\t-\t-\t-\t%s\n' \
    "$event_time" "$stage" "$detail" >> "$evidence_path"
}

phase1_cleanup_record_snapshot() {
  local evidence_path="$1"
  local stage="$2"
  local pgid="$3"
  local snapshot=""
  local snapshot_time=""
  snapshot_time="$(date -u +%Y-%m-%dT%H:%M:%SZ)" || return 1
  snapshot="$(phase1_process_group_snapshot "$pgid")" || return 1
  awk -v now="$snapshot_time" -v stage="$stage" \
    'NF >= 5 {print now "\tprocess\t" stage "\t" $1 "\t" $2 "\t" $3 "\t" $4 "\t" $5 "\t-"}' \
    <<< "$snapshot" >> "$evidence_path"
}

phase1_cleanup_prepare_evidence() {
  local evidence_path="$1"

  [[ ! -L "$evidence_path" ]] || return 3
  if [[ ! -e "$evidence_path" ]]; then
    printf 'utc\ttype\tstage\tpid\tpgid\tsid\tstate\tcommand\tdetail\n' \
      > "$evidence_path" || return 3
  elif [[ ! -f "$evidence_path" ]]; then
    return 3
  fi
}

phase1_cleanup_record_event_if_ready() {
  local evidence_ready="$1"
  shift

  (( evidence_ready == 1 )) || return 1
  phase1_cleanup_record_event "$@"
}

phase1_cleanup_record_snapshot_if_ready() {
  local evidence_ready="$1"
  shift

  (( evidence_ready == 1 )) || return 1
  phase1_cleanup_record_snapshot "$@"
}

phase1_wait_for_owned_child_bounded() {
  local child_pid="$1"
  local timeout_seconds="$2"
  local completed_pid=""
  local result_status=2
  local timer_pid=""
  local wait_status=0

  PHASE1_GATE_WAIT_STATUS=""
  [[ "$child_pid" =~ ^[1-9][0-9]*$ \
    && "$timeout_seconds" =~ ^[1-9][0-9]*$ ]] || return 2

  # With no task at the captured PID, wait can only consume Bash's cached
  # direct-child status (or report it unavailable); it cannot block here.
  if ! kill -0 "$child_pid" 2>/dev/null; then
    if wait "$child_pid" 2>/dev/null; then
      wait_status=0
    else
      wait_status=$?
    fi
    PHASE1_GATE_WAIT_STATUS="$wait_status"
    return 0
  fi

  phase1_begin_signal_deferral
  (
    trap '' INT TERM
    exec sleep "$timeout_seconds"
  ) &
  timer_pid=$!
  if wait -n -p completed_pid "$child_pid" "$timer_pid" 2>/dev/null; then
    wait_status=0
  else
    wait_status=$?
  fi
  : "${completed_pid:=}"
  if [[ "$completed_pid" == "$child_pid" ]]; then
    if kill -0 "$timer_pid" 2>/dev/null; then
      kill -KILL "$timer_pid" 2>/dev/null || true
    fi
    wait "$timer_pid" 2>/dev/null || true
    PHASE1_GATE_WAIT_STATUS="$wait_status"
    result_status=0
  elif [[ "$completed_pid" == "$timer_pid" ]]; then
    result_status=1
  else
    if kill -0 "$timer_pid" 2>/dev/null; then
      kill -KILL "$timer_pid" 2>/dev/null || true
    fi
    wait "$timer_pid" 2>/dev/null || true
  fi
  phase1_end_signal_deferral
  return "$result_status"
}

phase1_stop_exact_child_bounded() {
  local child_pid="$1"
  local followup_wait_status=0
  local initial_wait_status=0

  PHASE1_CHILD_DRAINED=false
  PHASE1_CHILD_OUTCOME_STATUS=125
  [[ "$child_pid" =~ ^[1-9][0-9]*$ ]] || return 2

  if phase1_wait_for_owned_child_bounded "$child_pid" 3; then
    PHASE1_CHILD_OUTCOME_STATUS="$PHASE1_GATE_WAIT_STATUS"
    PHASE1_CHILD_DRAINED=true
    return 0
  else
    initial_wait_status=$?
  fi
  if (( initial_wait_status == 1 )); then
    PHASE1_CHILD_OUTCOME_STATUS=124
  else
    PHASE1_CHILD_OUTCOME_STATUS=125
  fi

  if kill -0 "$child_pid" 2>/dev/null; then
    kill -TERM "$child_pid" 2>/dev/null || true
  fi
  if phase1_wait_for_owned_child_bounded "$child_pid" 2; then
    PHASE1_CHILD_DRAINED=true
    return 0
  else
    followup_wait_status=$?
  fi
  (( followup_wait_status == 1 )) || PHASE1_CHILD_OUTCOME_STATUS=125

  if kill -0 "$child_pid" 2>/dev/null; then
    kill -KILL "$child_pid" 2>/dev/null || true
  fi
  if phase1_wait_for_owned_child_bounded "$child_pid" 2; then
    PHASE1_CHILD_DRAINED=true
    return 0
  fi

  PHASE1_CHILD_OUTCOME_STATUS=125
  return 1
}

phase1_abort_unreleased_launch_gate() {
  local role="$1"
  local leader_pid="$2"
  local go_fd="$3"
  local evidence_path="$4"
  local timeout_seconds="$5"
  local abort_status=0
  local evidence_error=0
  local evidence_ready=0

  PHASE1_GROUP_TERM_SENT=false
  PHASE1_GROUP_KILL_SENT=false
  PHASE1_GROUP_DRAINED=false
  PHASE1_GROUP_WAIT_STATUS=""

  if [[ ! "$leader_pid" =~ ^[1-9][0-9]*$ \
    || ! "$timeout_seconds" =~ ^[1-9][0-9]*$ ]]; then
    return 3
  fi
  if phase1_cleanup_prepare_evidence "$evidence_path"; then
    evidence_ready=1
  else
    evidence_error=1
  fi
  phase1_cleanup_record_event_if_ready "$evidence_ready" \
    "$evidence_path" cleanup-start \
    "role=$role leader=$leader_pid gated=true" || evidence_error=1

  if [[ "$go_fd" =~ ^[0-9]+$ ]] \
    && printf '%s\n' ROBOTEST_PHASE1_ABORT >&"$go_fd"; then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" gate-abort-sent invalid-release-token || evidence_error=1
  else
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed gate-abort-write || evidence_error=1
    abort_status=3
  fi

  if phase1_wait_for_owned_child_bounded "$leader_pid" "$timeout_seconds"; then
    PHASE1_GROUP_WAIT_STATUS="$PHASE1_GATE_WAIT_STATUS"
    PHASE1_GROUP_DRAINED=true
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-pass gate-abort-reaped || evidence_error=1
  else
    abort_status=$?
    PHASE1_GROUP_DRAINED=false
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed gate-abort-timeout || evidence_error=1
    (( abort_status == 1 )) && abort_status=4
    (( abort_status == 2 )) && abort_status=3
  fi

  (( abort_status == 0 )) || return "$abort_status"
  (( evidence_error == 0 )) || return 3
  return 0
}

phase1_json_status() {
  python3 - "$1" "$2" "$3" <<'PY'
import json
import sys

payload = json.loads(sys.argv[1])
command = sys.argv[2]
expected_signal = sys.argv[3]
status = payload.get('status')
if payload.get('group_signal_flag') != 4:
    raise SystemExit(1)
if command == 'probe' and status not in {'empty', 'present'}:
    raise SystemExit(1)
if command == 'send' and (
    status not in {'empty', 'sent'} or payload.get('signal') != expected_signal
):
    raise SystemExit(1)
if command not in {'probe', 'send'}:
    raise SystemExit(1)
print(status)
PY
}

phase1_run_pidfd_command() {
  local helper="$1"
  local retained_fd="$2"
  local command="$3"
  shift 3

  [[ -f "$helper" && ! -L "$helper" && "$retained_fd" =~ ^[0-9]+$ ]] || return 3
  (
    trap '' INT TERM
    python3 "$helper" "$command" --fd 9 "$@" 9<&"$retained_fd"
  )
}

phase1_pidfd_action() {
  local helper="$1"
  local retained_fd="$2"
  local output="$3"
  local command="$4"
  local action_exit_status=0
  local action_hard_error=0
  local action_payload=""
  local parsed_status=""
  local requested_signal=""
  local action_status=""
  shift 4

  PHASE1_PIDFD_ACTION_STATUS=""
  : "${PHASE1_PIDFD_EVIDENCE_ERROR:=false}"
  if action_payload="$(
      trap '' INT TERM
      phase1_run_pidfd_command \
        "$helper" "$retained_fd" "$command" "$@"
    )"; then
    action_exit_status=0
  else
    action_exit_status=$?
  fi
  case "$command:$action_exit_status" in
    probe:0) action_status=present ;;
    probe:20) action_status=empty ;;
    send:0) action_status=sent ;;
    send:20) action_status=empty ;;
    *) action_hard_error=1 ;;
  esac
  if [[ "$command" == send && "${1:-}" == --signal \
    && "${2:-}" =~ ^(KILL|TERM)$ ]]; then
    requested_signal="$2"
  fi
  if (( ${#action_payload} > 16384 )); then
    PHASE1_PIDFD_EVIDENCE_ERROR=true
  elif parsed_status="$(phase1_json_status \
      "$action_payload" "$command" "$requested_signal")"; then
    [[ "$parsed_status" == "$action_status" ]] || PHASE1_PIDFD_EVIDENCE_ERROR=true
  else
    PHASE1_PIDFD_EVIDENCE_ERROR=true
  fi
  if [[ -L "$output" || ( -e "$output" && ! -f "$output" ) ]] \
    || ! printf '%s\n' "$action_payload" > "$output"; then
    PHASE1_PIDFD_EVIDENCE_ERROR=true
  fi
  (( action_hard_error == 0 )) || return 3
  PHASE1_PIDFD_ACTION_STATUS="$action_status"
  return 0
}

phase1_wait_for_numeric_live_drain() {
  local pgid="$1"
  local iteration=0
  local membership_status=0

  for ((iteration = 0; iteration < 50; iteration += 1)); do
    if phase1_process_group_has_live_members "$pgid"; then
      sleep 0.1
      continue
    else
      membership_status=$?
    fi
    (( membership_status == 1 )) && return 0
    return 2
  done
  return 1
}

phase1_sample_numeric_group_until_drain() {
  local pgid="$1"
  local sample_callback="$2"
  local sample_interval="$3"
  local membership_status=0

  [[ "$pgid" =~ ^[1-9][0-9]*$ \
    && "$sample_callback" =~ ^[a-zA-Z_][a-zA-Z0-9_]*$ \
    && "$sample_interval" =~ ^[0-9]+([.][0-9]+)?$ ]] || return 2
  declare -F "$sample_callback" >/dev/null || return 2

  while true; do
    sleep "$sample_interval" || return 2
    if phase1_process_group_has_live_members "$pgid"; then
      "$sample_callback" || return 2
      continue
    else
      membership_status=$?
    fi
    (( membership_status == 1 )) && return 0
    return 2
  done
}

phase1_wait_for_pidfd_empty() {
  local helper="$1"
  local retained_fd="$2"
  local output="$3"
  local iterations="$4"
  local iteration=0

  for ((iteration = 0; iteration < iterations; iteration += 1)); do
    phase1_pidfd_action "$helper" "$retained_fd" "$output" probe || return 2
    [[ "$PHASE1_PIDFD_ACTION_STATUS" == empty ]] && return 0
    [[ "$PHASE1_PIDFD_ACTION_STATUS" == present ]] || return 2
    sleep 0.1
  done
  return 1
}

phase1_wait_for_leader() {
  local leader_pid="$1"
  local timeout_seconds="${2:-2}"
  local wait_status=0

  [[ -z "$PHASE1_GROUP_WAIT_STATUS" ]] || return 0
  if phase1_wait_for_owned_child_bounded "$leader_pid" "$timeout_seconds"; then
    PHASE1_GROUP_WAIT_STATUS="$PHASE1_GATE_WAIT_STATUS"
    return 0
  else
    wait_status=$?
  fi
  return "$wait_status"
}

phase1_stop_owned_process_group() {
  local role="$1"
  local leader_pid="$2"
  local pgid="$3"
  local sid="$4"
  local leader_start_ticks="$5"
  local evidence_path="$6"
  local pidfd_helper="$7"
  local retained_fd="$8"
  local run_directory="$9"
  local known_wait_status="${10:-}"
  local evidence_error=0
  local evidence_ready=0
  local leader_reap_status=0
  local numeric_drain_status=0
  local pidfd_drain_status=0

  PHASE1_GROUP_TERM_SENT=false
  PHASE1_GROUP_KILL_SENT=false
  PHASE1_GROUP_DRAINED=false
  PHASE1_GROUP_WAIT_STATUS="$known_wait_status"
  PHASE1_PIDFD_EVIDENCE_ERROR=false

  if [[ ! "$leader_pid" =~ ^[1-9][0-9]*$ \
    || "$leader_pid" != "$pgid" \
    || "$leader_pid" != "$sid" \
    || ! "$leader_start_ticks" =~ ^[1-9][0-9]*$ \
    || ! "$retained_fd" =~ ^[0-9]+$ \
    || ! -f "$pidfd_helper" \
    || -L "$pidfd_helper" \
    || ( -n "$known_wait_status" && ! "$known_wait_status" =~ ^[0-9]+$ ) \
    || ! -d "$run_directory" ]]; then
    return 3
  fi
  if phase1_cleanup_prepare_evidence "$evidence_path"; then
    evidence_ready=1
  else
    evidence_error=1
  fi

  phase1_cleanup_record_event_if_ready "$evidence_ready" \
    "$evidence_path" cleanup-start \
    "role=$role leader=$leader_pid pinned_fd=$retained_fd start_ticks=$leader_start_ticks" \
    || evidence_error=1
  if ! phase1_cleanup_record_snapshot_if_ready \
      "$evidence_ready" "$evidence_path" before-probe "$pgid"; then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" snapshot-unavailable before-probe || evidence_error=1
  fi

  if ! phase1_pidfd_action \
      "$pidfd_helper" "$retained_fd" "$run_directory/pidfd-probe-before.json" probe; then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed pidfd-probe-before || evidence_error=1
    return 3
  fi
  if [[ "$PHASE1_PIDFD_ACTION_STATUS" == empty ]]; then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" already-empty stable-pidfd-group || evidence_error=1
    if phase1_wait_for_leader "$leader_pid" 2; then
      leader_reap_status=0
    else
      leader_reap_status=$?
    fi
    if (( leader_reap_status != 0 )); then
      if (( leader_reap_status == 1 )); then
        phase1_cleanup_record_event_if_ready "$evidence_ready" \
          "$evidence_path" cleanup-failed leader-reap-timeout || evidence_error=1
        return 4
      fi
      phase1_cleanup_record_event_if_ready "$evidence_ready" \
        "$evidence_path" cleanup-failed leader-reap-inspection-unavailable \
        || evidence_error=1
      return 3
    fi
    PHASE1_GROUP_DRAINED=true
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-pass already-empty || evidence_error=1
    [[ "$PHASE1_PIDFD_EVIDENCE_ERROR" != true ]] || evidence_error=1
    (( evidence_error == 0 )) || return 3
    return 0
  fi

  if ! phase1_pidfd_action \
      "$pidfd_helper" "$retained_fd" "$run_directory/pidfd-term.json" \
      send --signal TERM; then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed pidfd-term || evidence_error=1
    return 3
  fi
  if [[ "$PHASE1_PIDFD_ACTION_STATUS" == sent ]]; then
    PHASE1_GROUP_TERM_SENT=true
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" term-sent stable-pidfd-group || evidence_error=1
  else
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" term-esrch stable-pidfd-group || evidence_error=1
  fi

  if phase1_wait_for_numeric_live_drain "$pgid"; then
    numeric_drain_status=0
  else
    numeric_drain_status=$?
  fi
  if (( numeric_drain_status == 2 )); then
    phase1_cleanup_record_event_if_ready "$evidence_ready" "$evidence_path" \
      numeric-inspection-unavailable escalate-kill || evidence_error=1
  elif (( numeric_drain_status == 1 )); then
    if ! phase1_cleanup_record_snapshot_if_ready \
        "$evidence_ready" "$evidence_path" term-timeout "$pgid"; then
      phase1_cleanup_record_event_if_ready "$evidence_ready" \
        "$evidence_path" snapshot-unavailable term-timeout || evidence_error=1
    fi
  else
    if phase1_wait_for_leader "$leader_pid" 2; then
      leader_reap_status=0
    else
      leader_reap_status=$?
    fi
    if (( leader_reap_status == 1 )); then
      phase1_cleanup_record_event_if_ready "$evidence_ready" \
        "$evidence_path" leader-reap-timeout escalate-kill || evidence_error=1
    elif (( leader_reap_status == 2 )); then
      phase1_cleanup_record_event_if_ready "$evidence_ready" \
        "$evidence_path" leader-reap-inspection-unavailable escalate-kill \
        || evidence_error=1
      evidence_error=1
    else
      if phase1_wait_for_pidfd_empty \
          "$pidfd_helper" "$retained_fd" "$run_directory/pidfd-probe-after-term.json" 20; then
        pidfd_drain_status=0
      else
        pidfd_drain_status=$?
      fi
      if (( pidfd_drain_status == 0 )); then
        PHASE1_GROUP_DRAINED=true
        phase1_cleanup_record_event_if_ready "$evidence_ready" \
          "$evidence_path" cleanup-pass term-drained || evidence_error=1
        [[ "$PHASE1_PIDFD_EVIDENCE_ERROR" != true ]] || evidence_error=1
        (( evidence_error == 0 )) || return 3
        return 0
      elif (( pidfd_drain_status == 1 )); then
        phase1_cleanup_record_event_if_ready "$evidence_ready" \
          "$evidence_path" stable-group-present escalate-kill || evidence_error=1
      else
        phase1_cleanup_record_event_if_ready "$evidence_ready" \
          "$evidence_path" pidfd-inspection-unavailable escalate-kill || evidence_error=1
      fi
    fi
  fi

  if ! phase1_pidfd_action \
      "$pidfd_helper" "$retained_fd" "$run_directory/pidfd-kill.json" \
      send --signal KILL; then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed pidfd-kill || evidence_error=1
    return 3
  fi
  if [[ "$PHASE1_PIDFD_ACTION_STATUS" == sent ]]; then
    PHASE1_GROUP_KILL_SENT=true
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" kill-sent stable-pidfd-group || evidence_error=1
  else
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" kill-esrch stable-pidfd-group || evidence_error=1
  fi

  if phase1_wait_for_leader "$leader_pid" 2; then
    leader_reap_status=0
  else
    leader_reap_status=$?
  fi
  if (( leader_reap_status != 0 )); then
    if (( leader_reap_status == 1 )); then
      phase1_cleanup_record_event_if_ready "$evidence_ready" \
        "$evidence_path" cleanup-failed leader-reap-timeout-after-kill \
        || evidence_error=1
      return 4
    fi
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed leader-reap-inspection-unavailable-after-kill \
      || evidence_error=1
    return 3
  fi
  if phase1_wait_for_pidfd_empty \
      "$pidfd_helper" "$retained_fd" "$run_directory/pidfd-probe-after-kill.json" 50; then
    pidfd_drain_status=0
  else
    pidfd_drain_status=$?
  fi
  if (( pidfd_drain_status == 1 )); then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed residual-stable-group || evidence_error=1
    return 4
  elif (( pidfd_drain_status != 0 )); then
    phase1_cleanup_record_event_if_ready "$evidence_ready" \
      "$evidence_path" cleanup-failed pidfd-inspection-unavailable-after-kill \
      || evidence_error=1
    return 3
  fi

  PHASE1_GROUP_DRAINED=true
  phase1_cleanup_record_event_if_ready "$evidence_ready" \
    "$evidence_path" cleanup-pass kill-drained || evidence_error=1
  [[ "$PHASE1_PIDFD_EVIDENCE_ERROR" != true ]] || evidence_error=1
  (( evidence_error == 0 )) || return 3
  return 0
}
