#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Snap applications can inject an incompatible Core20 glibc into ROS/Qt tools.
unset LD_LIBRARY_PATH LD_PRELOAD PYTHONPATH QT_PLUGIN_PATH QML2_IMPORT_PATH GTK_PATH GTK_MODULES
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${PROJECT_DIR}/install/setup.bash" ]]; then
  source "${PROJECT_DIR}/install/setup.bash"
fi
set -u

exec ros2 run rqt_image_view rqt_image_view
