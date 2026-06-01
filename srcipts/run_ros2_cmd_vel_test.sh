#!/usr/bin/env bash
set -euo pipefail

BRIDGE_SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/novel/apps/isaacsim/5.1.0/isaacsim}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [ -f "$ISAAC_SIM_ROOT/setup_ros_env.sh" ]; then
  set +u
  source "$ISAAC_SIM_ROOT/setup_ros_env.sh"
  set -u
fi

BRIDGE_PY="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/$ROS_DISTRO/rclpy"
BRIDGE_LIB="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib"
export PYTHONPATH="$BRIDGE_PY${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$BRIDGE_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec "$ISAAC_SIM_ROOT/python.sh" "$BRIDGE_SCRIPT_DIR/ros2_cmd_vel_test.py" "$@"
