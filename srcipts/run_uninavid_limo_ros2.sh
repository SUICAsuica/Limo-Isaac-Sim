#!/usr/bin/env bash
set -euo pipefail

INSTRUCTION="${1:?usage: $0 'navigation instruction' [image_topic]}"
IMAGE_TOPIC="${2:-/camera/color/image_raw}"

BRIDGE_SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/novel/apps/isaacsim/5.1.0/isaacsim}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [ -f "$ISAAC_SIM_ROOT/setup_ros_env.sh" ]; then
  # shellcheck disable=SC1090
  set +u
  source "$ISAAC_SIM_ROOT/setup_ros_env.sh"
  set -u
fi

BRIDGE_PY="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/$ROS_DISTRO/rclpy"
BRIDGE_LIB="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib"
if [ -d "$BRIDGE_PY" ]; then
  export PYTHONPATH="$BRIDGE_PY${PYTHONPATH:+:$PYTHONPATH}"
fi
if [ -d "$BRIDGE_LIB" ]; then
  export LD_LIBRARY_PATH="$BRIDGE_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

PYTHON_BIN="$ISAAC_SIM_ROOT/python.sh"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN="python3"
fi

exec "$PYTHON_BIN" "$BRIDGE_SCRIPT_DIR/uninavid_limo_ros2_bridge.py" \
  --instruction "$INSTRUCTION" \
  --image-topic "$IMAGE_TOPIC"
