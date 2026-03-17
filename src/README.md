# ROS2 Packages

## vtam_core

Main ROS2 package for recording demonstrations and running onboard sensors.

### Build

```bash
cd ~/VTAM
colcon build --packages-select vtam_core
source install/setup.bash
```

Requires `libvisp-dev` for ArUco detection:
```bash
sudo apt update && sudo apt install -y libvisp-dev
```

### Launch

```bash
# Full system (sensors + recording)
ros2 launch vtam_core vtam.launch.xml

# Recording only
ros2 launch vtam_core vtam_record.launch.py
```

### Nodes

| Node | Entry Point | Purpose |
|------|------------|---------|
| `record_node` | `nodes/record_node.py` | Manages session MCAP bags and episode markers |
| `umi_gripper_node` | `nodes/umi_gripper_node.py` | Publishes normalised gripper width |
| `head_tracker_node` | `nodes/head_tracker_node.py` | Head camera pose tracking |
| `eflesh_node` | `drivers/eflesh_node.py` | eFlesh tactile sensor serial driver (30D) |
| `umi_detector_node` | `drivers/umi_detector_node.py` | ArUco cube pose estimator, publishes `/umi_cube_pose` |

### Recording Services

```bash
# Start session bag (3s warmup, then ready)
ros2 service call /start_session std_srvs/srv/SetBool "{data: true}"

# Toggle episode start/stop (or use physical button on A3)
ros2 service call /record_demo std_srvs/srv/SetBool "{data: true}"
```

Session bags are saved to `data/raw/<task>/session_<timestamp>/`. Episodes are delimited by `/recording/active` Bool messages and split offline by `training/scripts/bag_chunker.py`.

### Config

| File | Purpose |
|------|---------|
| `config/record.yaml` | Recording parameters |
| `config/tags_umi.yaml` | ArUco tag IDs for UMI hand detection |
| `config/umi_trigger_calibration.yaml` | UMI trigger calibration |