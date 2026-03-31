# Go2 ROS2 Full Stack Launch Guide

## Prerequisites

- Go2 robot powered on and connected to the same WiFi network
- `ros2_m1_native` built at `../../ros2_m1_native` (relative to this repo)
- `unitree_webrtc_connect` installed in the ros2_m1_native venv (`uv pip install unitree-webrtc-connect`)
- Robot IP known (default: `192.168.1.139`)

## Quick Start

```bash
cd go2_ros2_webrtc_sdk
ROBOT_IP=192.168.1.139 bash launch_full_stack.sh
```

This starts (in order):
1. **Go2 driver** — WebRTC connection, camera, odom, IMU, lidar
2. **PointCloud2 to LaserScan converter** — converts Go2 lidar to `/scan` for SLAM
3. **robot_state_publisher** — publishes URDF TF tree
4. **SLAM Toolbox** — builds 2D map from `/scan`
5. **Nav2** — autonomous navigation stack
6. **Foxglove Bridge** — WebSocket bridge on port 8765

## Foxglove Studio

Open https://app.foxglove.dev and connect to `ws://localhost:8765`.

### Key topics

| Topic | Type | Description |
|---|---|---|
| `/camera/image_raw` | Image | Front camera 1280x720 BGR8 |
| `/camera/camera_info` | CameraInfo | Camera calibration |
| `/odom` | Odometry | Robot odometry |
| `/joint_states` | JointState | Leg joint positions |
| `/imu` | IMU | Accelerometer + gyroscope |
| `/point_cloud2` | PointCloud2 | 3D lidar point cloud |
| `/scan` | LaserScan | 2D lidar scan (converted from point cloud) |
| `/map` | OccupancyGrid | SLAM map |
| `/go2_states` | Go2State | Robot mode, gait, velocity |
| `/tf` | TFMessage | Transform tree |

### Teleop (manual driving)

Add a **Teleop** panel in Foxglove. Set the topic to `/cmd_vel_out` (not `/cmd_vel`).
Use the arrow buttons to drive the robot and build the SLAM map.

### 3D view

In the 3D panel, enable these topics for a full view:
- `/point_cloud2` — 3D lidar
- `/scan` — 2D laser scan
- `/map` — SLAM occupancy grid
- `/tf` — transforms (robot model, frames)
- `/robot_description` — URDF model

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `ROBOT_IP` | `192.168.1.139` | Go2 robot IP address |
| `CONN_TYPE` | `webrtc` | Connection type (set automatically) |

## Troubleshooting

### Driver doesn't connect
- Check robot is on: `ping $ROBOT_IP`
- Check WebRTC port: `nc -z $ROBOT_IP 9991` (should succeed)
- Robot needs ~30s after power on for WebRTC service to start

### No camera image in Foxglove
- Disconnect and reconnect in Foxglove Studio
- Verify topic exists: the image panel should show `/camera/image_raw`
- Check driver is alive: `pgrep -f go2_driver_node`

### No point cloud or scan data
- The lidar decoder (`LibVoxelDecoder`) must load — check for "Using unitree_webrtc_connect LibVoxelDecoder" in logs
- Verify: subscribe to `/point_cloud2` and `/scan` in Foxglove

### Nav2 inactive
- Known `bad_weak_ptr` issue on macOS during Nav2 activation
- SLAM still works without Nav2 — drive manually with teleop
- Try restarting Nav2 nodes after all topics are publishing

### Foxglove shows topics but no data
- Kill and restart Foxglove Bridge: `pkill -9 foxglove_bridge`
- The launch script restarts it automatically
- Or run manually: `$ROS2_DIR/install/lib/foxglove_bridge/foxglove_bridge`

## Stopping

Press `Ctrl+C` in the terminal running `launch_full_stack.sh`. All processes are cleaned up automatically.
