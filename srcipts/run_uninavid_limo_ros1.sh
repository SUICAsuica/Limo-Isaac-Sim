#!/usr/bin/env bash
set -euo pipefail

INSTRUCTION="${1:?usage: $0 'navigation instruction' [image_topic]}"
IMAGE_TOPIC="${2:-/camera/color/image_raw}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$SCRIPT_DIR/uninavid_limo_ros1_bridge.py" \
  --instruction "$INSTRUCTION" \
  --image-topic "$IMAGE_TOPIC"
