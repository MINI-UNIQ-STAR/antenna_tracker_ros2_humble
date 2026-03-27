#!/usr/bin/env bash
set -eo pipefail

LAUNCH_LOG=/tmp/antenna_tracker_sim_auto_e2e.log

cleanup() {
  local exit_code=$?
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi

  if [[ $exit_code -ne 0 ]]; then
    echo "=== AUTO E2E launch log tail ==="
    tail -n 200 "${LAUNCH_LOG}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ensure_launch_alive() {
  if [[ -n "${LAUNCH_PID:-}" ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "Simulation launch exited unexpectedly"
    return 1
  fi
}

wait_for_service() {
  local service_name="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))

  while (( SECONDS < deadline )); do
    ensure_launch_alive
    if ros2 service type "${service_name}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  echo "Timed out waiting for service ${service_name}"
  return 1
}

wait_for_message() {
  local topic_name="$1"
  local timeout_s="$2"
  local output_file="$3"

  ensure_launch_alive
  if ! timeout "${timeout_s}" ros2 topic echo "${topic_name}" --once >"${output_file}" 2>/dev/null; then
    echo "Timed out waiting for topic ${topic_name}"
    return 1
  fi
}

echo "=== Starting AUTO-mode end-to-end simulation ==="
set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u

ros2 launch antenna_tracker_simulation sim.launch.py launch_rviz:=false ${SIM_LAUNCH_ARGS:-} >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
sleep 2

wait_for_service /antenna/set_mode 60
wait_for_message /imu/raw 60 /tmp/imu_raw_auto.txt
wait_for_message /antenna/encoder_feedback 60 /tmp/encoder_feedback_auto.txt

python3 /ros2_ws/scripts/test_sim_auto_e2e.py \
  --timeout "${SIM_AUTO_TIMEOUT:-180}" \
  --az-tol "${SIM_AUTO_AZ_TOL:-1}" \
  --el-tol "${SIM_AUTO_EL_TOL:-1}"
