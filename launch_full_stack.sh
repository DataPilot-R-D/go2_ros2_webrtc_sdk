#!/bin/bash
# Launch Go2 Pro with full ROS2 stack: driver + SLAM + Nav2 + Foxglove
# Requires ros2_m1_native to be built alongside this repo

set -e

cd "$(dirname "$0")"
SDK_DIR="$(pwd)/go2_robot_sdk"
CONFIG_DIR="${SDK_DIR}/config"

# Find ros2_m1_native (sibling directory)
ROS2_DIR=""
for candidate in "../ros2_m1_native" "../../ros2_m1_native"; do
  [ -d "$candidate" ] && ROS2_DIR="$(cd "$candidate" && pwd)" && break
done
if [ -z "$ROS2_DIR" ]; then
  echo "ERROR: ros2_m1_native not found"
  exit 1
fi

echo "=== Go2 ROS2 Full Stack ==="
echo "Robot IP: ${ROBOT_IP:-192.168.1.139}"
echo "SDK: $SDK_DIR"
echo "ROS2: $ROS2_DIR"

# Activate ros2_m1_native environment
cd "$ROS2_DIR"
source scripts/activate_env.sh

# Set up ROS2 paths
EXTRA_PATHS=""
for link in install/lib/python3.11/site-packages/*.egg-link; do
  [ -f "$link" ] && EXTRA_PATHS="${EXTRA_PATHS}:$(head -1 "$link")"
done
export PYTHONPATH="${ROS2_DIR}/install/lib/python3.11/site-packages${EXTRA_PATHS}:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="${ROS2_DIR}/install"
export DYLD_LIBRARY_PATH="${ROS2_DIR}/install/lib:${ROS2_DIR}/.local/deps/lib:${DYLD_LIBRARY_PATH:-}"
export PATH="${ROS2_DIR}/install/bin:${PATH}"
export ROBOT_IP="${ROBOT_IP:-192.168.1.139}"
export CONN_TYPE="webrtc"

# Make go2_robot_sdk findable by ament
export AMENT_PREFIX_PATH="${SDK_DIR}:${AMENT_PREFIX_PATH}"
SHARE_DIR="${SDK_DIR}/share/go2_robot_sdk"
if [ ! -d "$SHARE_DIR" ]; then
  mkdir -p "$SHARE_DIR"
  ln -sf "${SDK_DIR}/config" "$SHARE_DIR/config" 2>/dev/null || true
  ln -sf "${SDK_DIR}/urdf" "$SHARE_DIR/urdf" 2>/dev/null || true
  ln -sf "${SDK_DIR}/launch" "$SHARE_DIR/launch" 2>/dev/null || true
  mkdir -p "${SDK_DIR}/share/ament_index/resource_index/packages"
  touch "${SDK_DIR}/share/ament_index/resource_index/packages/go2_robot_sdk"
fi

PIDFILE="/tmp/go2_ros2_stack.pids"
echo "" > "$PIDFILE"

cleanup() {
  echo ""
  echo "Shutting down..."
  while read -r pid; do
    [ -n "$pid" ] && kill "$pid" 2>/dev/null
  done < "$PIDFILE"
  wait 2>/dev/null
  rm -f "$PIDFILE"
  echo "All stopped."
}
trap cleanup EXIT INT TERM

echo ""
echo "--- Starting Go2 driver ---"
PYTHONUNBUFFERED=1 .venv/bin/python -c "
import sys, os
sys.path.insert(0, '${SDK_DIR}')
from go2_robot_sdk.go2_driver_node import main
main()
" &
echo $! >> "$PIDFILE"
echo "Driver PID: $!"

echo "Waiting for driver to connect (~20s)..."
sleep 20

echo ""
echo "--- Starting PointCloud2 to LaserScan converter ---"
PYTHONUNBUFFERED=1 .venv/bin/python -c "
import sys
sys.path.insert(0, '${SDK_DIR}')
from go2_robot_sdk.pointcloud_to_scan import main
main()
" &
echo $! >> "$PIDFILE"
echo "PC2->Scan PID: $!"

sleep 2

echo ""
echo "--- Starting robot_state_publisher ---"
# Write params YAML to avoid shell escaping issues with URDF XML
URDF_CONTENT="$(cat ${SDK_DIR}/urdf/go2.urdf)"
cat > /tmp/rsp_params.yaml << YAMLEOF
robot_state_publisher:
  ros__parameters:
    robot_description: |
$(echo "$URDF_CONTENT" | sed 's/^/      /')
YAMLEOF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file /tmp/rsp_params.yaml &
echo $! >> "$PIDFILE"

sleep 2

echo ""
echo "--- Starting SLAM Toolbox ---"
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="${CONFIG_DIR}/mapper_params_online_async.yaml" \
  use_sim_time:=false &
echo $! >> "$PIDFILE"

sleep 3

echo ""
echo "--- Starting Nav2 ---"
ros2 launch nav2_bringup navigation_launch.py \
  params_file:="${CONFIG_DIR}/nav2_params.yaml" \
  use_sim_time:=false &
echo $! >> "$PIDFILE"

sleep 3

echo ""
echo "--- Starting Foxglove Bridge ---"
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
echo $! >> "$PIDFILE"

echo ""
echo "============================================"
echo "  Full stack running!"
echo "============================================"
echo ""
echo "  Foxglove Studio: https://app.foxglove.dev"
echo "    -> Open connection -> ws://localhost:8765"
echo ""
echo "  Topics: /image_raw /point_cloud2 /scan /odom /imu /map"
echo "  Nav2:   Send goals via Foxglove 2D Nav Goal"
echo ""
echo "  Press Ctrl+C to stop everything"
echo ""

wait
