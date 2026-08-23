#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 ${ROS_DISTRO_NAME} is not installed (${ROS_SETUP} not found)." >&2
  echo "Install ROS 2 desktop, colcon, and libpugixml-dev." >&2
  exit 1
fi
set +u; source "${ROS_SETUP}"; set -u
for command in colcon rviz2; do
  command -v "${command}" >/dev/null 2>&1 || { echo "Required command '${command}' is missing." >&2; exit 1; }
done
if ! dpkg-query -W -f='${Status}' libpugixml-dev 2>/dev/null | grep -q "install ok installed"; then
  echo "Missing dependency: libpugixml-dev" >&2
  echo "Install it with: sudo apt update && sudo apt install libpugixml-dev" >&2
  exit 1
fi
cd "${PROJECT_DIR}"
colcon build --symlink-install
set +u; source "${PROJECT_DIR}/install/setup.bash"; set -u
export FILEPATH="${FILEPATH:-${PROJECT_DIR}/src/bjb_sangjani_full.xodr}"
VEHICLE_SPEED_MPS="${VEHICLE_SPEED_MPS:-6.0}"

# VS Code installed through Snap exports GTK/GIO module paths from the Snap
# runtime. Native RViz can then load Snap's incompatible glibc/libpthread.
# Remove only those GUI-runtime overrides before starting the native ROS apps.
unset GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR
unset GSETTINGS_SCHEMA_DIR GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_PATH LOCPATH
if [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]]; then
  export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}"
fi
NODE_PID=""
cleanup() { if [[ -n "${NODE_PID}" ]] && kill -0 "${NODE_PID}" 2>/dev/null; then kill "${NODE_PID}" 2>/dev/null || true; wait "${NODE_PID}" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
ros2 run traffic_map road_visualization --ros-args -p vehicle_speed_mps:="${VEHICLE_SPEED_MPS}" &
NODE_PID=$!
# Synchronize RViz with the display refresh to avoid XWayland/OpenGL tearing.
# The first variable covers NVIDIA and the second covers Mesa/Intel/AMD.
env __GL_SYNC_TO_VBLANK=1 vblank_mode=1 QT_OPENGL=desktop \
  rviz2 -d "${PROJECT_DIR}/rviz/traffic_map.rviz"
