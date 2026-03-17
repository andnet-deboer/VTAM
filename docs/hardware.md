# Hardware

## Robot

**Hello Robot Stretch 3** with the following sensors active during recording:

| Sensor | ROS2 Topic | Notes |
|--------|-----------|-------|
| Gripper camera (RGB) | `/gripper_camera/color/image_rect_raw` | 640×480, compressed to JPEG at 85% for ZMQ |
| Joint states | `/joint_states` | Lift, arm, wrist (yaw/pitch/roll), gripper |
| UMI cube pose | `/umi_cube_pose` | ArUco-based EE tracking via `umi_detector_node` |
| Tactile left | `/tactile_left` | 15D (5 MLX90393 sensors) |
| Tactile right | `/tactile_right` | 15D (5 MLX90393 sensors) |

## Tactile Sensor (eFlesh)

5 MLX90393 3-axis magnetometers per gripper finger, streamed by a **QT Py microcontroller** over USB serial. Firmware is in `firmware/`. See [firmware README](../firmware/README.md) for setup and credits.

<video controls width="640">
    <source src="images/SAMD21.mp4" type="video/mp4">
</video>

The firmware includes a **recording trigger button on pin A3** — pressing it toggles episode recording without needing to call the ROS2 service manually.

**Serial device check:**
```bash
ls -l /dev/serial/by-path/
```

## UMI Hand Tracker

ArUco cube mounted on the gripper. Detected by `umi_detector_node.py` using tag definitions in `config/tags_umi.yaml` and calibration in `config/cube_corners.json`.

The detected pose is published on `/umi_cube_pose` and consumed by `process_demo.py` during dataset creation to extract EE trajectories from `/tf`.

## Network

| Machine | Role | Notes |
|---------|------|-------|
| Stretch 3 | Robot + sensor host | Runs `vtam_robot_node.py` in Docker |
| GPU workstation ("sheep") | Inference server | Runs `vtam_server_inference.py` |

IPs and ZMQ ports are set in `config/inference.yaml`.

Robot firewall:
```bash
sudo ufw allow 4405
sudo ufw allow 4406
```
