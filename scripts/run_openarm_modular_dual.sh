#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="/home/user/rl_ws"
WS_DIR="${ROOT_DIR}/urdf"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="${WS_DIR}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"
set -u

if ! python3 -c "import xacro" >/dev/null 2>&1; then
  echo "Missing Python module 'xacro'. Install ros-humble-xacro first." >&2
  echo "Suggested command: sudo apt install ros-humble-xacro" >&2
  exit 1
fi

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  echo "No workspace install found. Building openarm_description and dg_description..."
  (
    cd "${WS_DIR}"
    colcon build \
      --base-paths \
      "${WS_DIR}/vendor/openarm_description" \
      "${WS_DIR}/vendor/delto_m_ros2/dg_description"
  )
fi

source "${INSTALL_SETUP}"

exec ros2 launch "${WS_DIR}/launch/display_openarm_modular_dual.launch.py" "$@"
