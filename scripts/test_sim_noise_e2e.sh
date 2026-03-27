#!/usr/bin/env bash
set -eo pipefail

SIM_LAUNCH_ARGS="encoder_noise_deg:=0.35 velocity_noise_dps:=1.50 command_noise_hz:=2.50 disturbance_dps2:=1.25" \
SIM_AUTO_TIMEOUT="${SIM_AUTO_TIMEOUT:-180}" \
SIM_AUTO_AZ_TOL="${SIM_AUTO_AZ_TOL:-2}" \
SIM_AUTO_EL_TOL="${SIM_AUTO_EL_TOL:-2}" \
  bash /ros2_ws/scripts/test_sim_auto_e2e.sh
