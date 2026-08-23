#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u; source /opt/ros/humble/setup.bash; set -u
cd "${PROJECT_DIR}"
colcon build --symlink-install
set +u; source install/setup.bash; set -u
export FILEPATH="${FILEPATH:-${PROJECT_DIR}/src/bjb_sangjani_full.xodr}"
ASSET_DIR="${PROJECT_DIR}/build/gazebo_assets"
mkdir -p "${ASSET_DIR}"
ros2 run traffic_map generate_gazebo_assets "${ASSET_DIR}"
export TRAFFIC_ROUTE_CSV="${ASSET_DIR}/route.csv"
SOURCE_CAR_MESH="${TRAFFIC_CAR_MESH:-${PROJECT_DIR}/cars/Audi_Q7_2009.glb}"
export TRAFFIC_VEHICLE_COUNT="${TRAFFIC_VEHICLE_COUNT:-24}"
export WIND_X_MPS="${WIND_X_MPS:-3.0}"
export WIND_Y_MPS="${WIND_Y_MPS:-1.0}"
export VISIBILITY_M="${VISIBILITY_M:-700}"
export SUN_LEVEL="${SUN_LEVEL:-0.9}"
if [[ ! -f "${SOURCE_CAR_MESH}" ]]; then
  echo "Car mesh not found: ${SOURCE_CAR_MESH}" >&2
  exit 1
fi
GAZEBO_CAR_MESH="${ASSET_DIR}/Audi_Q7_2009.obj"
ros2 run traffic_map convert_glb_to_obj "${SOURCE_CAR_MESH}" "${GAZEBO_CAR_MESH}"
export TRAFFIC_CAR_MESH="${GAZEBO_CAR_MESH}"
export GAZEBO_PLUGIN_PATH="${PROJECT_DIR}/install/traffic_map/lib:${GAZEBO_PLUGIN_PATH:-}"
WORLD_FILE="${ASSET_DIR}/traffic.world"
sed -e "s|ROAD_SURFACE_PATH|${ASSET_DIR}/road_surface.obj|g" \
    -e "s|ROAD_WHITE_PATH|${ASSET_DIR}/road_white.obj|g" \
    -e "s|ROAD_YELLOW_PATH|${ASSET_DIR}/road_yellow.obj|g" \
    -e "s|VEGETATION_PATH|${ASSET_DIR}/vegetation.obj|g" \
    -e "s|SIDEWALKS_PATH|${ASSET_DIR}/sidewalks.obj|g" \
    -e "s|GUARDRAILS_PATH|${ASSET_DIR}/guardrails.obj|g" \
    -e "s|WIND_X|${WIND_X_MPS}|g" \
    -e "s|WIND_Y|${WIND_Y_MPS}|g" \
    -e "s|FOG_END|${VISIBILITY_M}|g" \
    -e "s|SUN_LEVEL|${SUN_LEVEL}|g" \
    "${PROJECT_DIR}/gazebo/traffic.world" > "${WORLD_FILE}"
export GAZEBO_MODEL_DATABASE_URI="file://${PROJECT_DIR}/gazebo/model_database"
gzserver --verbose "${WORLD_FILE}" &
GAZEBO_SERVER_PID=$!
cleanup_gazebo() {
  if kill -0 "${GAZEBO_SERVER_PID}" 2>/dev/null; then
    kill "${GAZEBO_SERVER_PID}" 2>/dev/null || true
    wait "${GAZEBO_SERVER_PID}" 2>/dev/null || true
  fi
}
trap cleanup_gazebo EXIT INT TERM
sleep 2
if ! kill -0 "${GAZEBO_SERVER_PID}" 2>/dev/null; then
  echo "Gazebo server stopped before the GUI could start." >&2
  wait "${GAZEBO_SERVER_PID}"
fi
gzclient --verbose
