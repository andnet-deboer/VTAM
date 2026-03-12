# VTAM
### Visuo-Tactile Assistive Manipulation

A UMI platform for the Hello Robot Stretch 3 that records and replays visuo-tactile demonstrations using imitation learning. The system synchronizes RGB, depth, joint, and tactile (eFlesh) sensor data during teleoperated demonstrations, then uses those recordings to train and run learned policies.

---

## Repository Structure

```
VTAM/
├── src/
│   ├── umi_cube_tracker/       # ROS2 package for UMI cube pose tracking
│   └── vtam_core/              # Core ROS2 package
│       ├── config/
│       ├── launch/
│       ├── urdf/
│       └── vtam_core/          # Python source (drivers, recording, teach mode)
├── training/
│   ├── scripts/                # Demo processing pipeline
│   │   ├── process_demo.py
│   │   ├── replay_demo.py
│   │   ├── validate_demos.py
│   │   ├── bag_chunker.py
│   │   ├── detect_umi.py
│   │   ├── fuse_imu.py
│   │   └── goto_demo_pose.py
│   ├── utils/                  # IK, kinematics, dataset tools
│   │   ├── differential_ik.py
│   │   ├── kinematics.py
│   │   ├── replay_on_robot.py
│   │   ├── validate_pipeline.py
│   │   └── workspace_projection.py
│   ├── main.py
│   ├── Training.md
│   └── old/                    # Archived scripts
├── pyproject.toml
├── uv.lock
└── mkdocs.yml
```

---

## Setup

### Install UV
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc  # or ~/.zshrc
```

### Environment
```bash
cd ~/VTAM
uv sync
source .venv/bin/activate
uv pip install -e dependencies/diffusion_policy --config-setting editable_mode=compat
```

### ROS2 Workspace
```bash
cd ros_ws/src
ln -s ../../dependencies/stretch_ros2/stretch_description .
```

### LeRobot
```bash
git clone https://github.com/hello-robot/lerobot
cd lerobot
git switch stretch-act
uv pip install av --no-build-isolation
uv pip install -e . --no-deps
uv pip install datasets safetensors huggingface-hub torch torchvision
```

### Stretch AI
```bash
python3 -m pip install --user sophuspy
python -m pip install -e ./src --no-cache-dir
python -m pip uninstall av -y
```

### Tactile Sensor
```bash
pip3 install --user anyskin --break-system-packages
```

---

## Recording Demonstrations

```bash
ros2 run vtam_core record_node --ros-args -p demo_name:=my_demo

# Start/stop recording via service
ros2 service call /record_demo std_srvs/srv/SetBool "{data: True}"
```

## Syncing Data
```bash
rsync -av --progress user@<robot-ip>:~/VTAM/data/training/my_demo.zarr ~/VTAM/data/training/
```

---

## Hardware Notes

- Sensor firmware lives in `hardware/` (QT Py microcontroller)
- Check serial devices: `ls -l /dev/serial/by-path/`
- Serial tools: `sudo apt install python3-serial`
