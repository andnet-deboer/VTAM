<div align="center">
  
# VTAM
### Visuo-Tactile Assistive Manipulation

<img width="1110" height="479" alt="image" src="https://github.com/user-attachments/assets/c212ab2d-570e-411c-bf9f-7420078956de" />

**Andnet DeBoer** | *Northwestern University MSR*
</div>

---

## Overview
VTAM is a **Visuo-Tactile Assistive Manipulation Platform** designed for the Hello Robot Stretch 3. Unlike single-task robots, VTAM features a "Teach Mode" that allows users to record visuo-tactile demonstrations for any task (e.g., making a sandwich, pouring coffee) and replay them using imitation learning.

**Core Capabilities:**
* **Teach Mode:** Few-shot demonstration recording using a custom teleoperation device.
* **Visuo-Tactile Learning:** Synchronized logging of RGB, Depth, Joint, and Tactile (eFlesh) data.
* **Open-Ended Architecture:** A skill-agnostic policy manager capable of switching tasks on the fly.

---
for lerobot
git clone https://github.com/hello-robot/lerobot
cd lerobot
git switch stretch-act
uv pip install av --no-build-isolation
uv pip install -e . --no-deps
uv pip install datasets safetensors huggingface-hub torch torchvision

for stretch ai
python3 -m pip install --user sophuspy
python -m pip install -e ./src --no-cache-dir
python -m pip uninstall av -y
##  Repository Structure

```text
VTAM/
├── docker/                 # Infrastructure & Environment
│   ├── docker-compose.yaml # Orchestrates the dev container
│   └── Dockerfile          # Builds the robot OS & Vendor dependencies
├── ros_ws/
│   └── src/
│       └── vtam_core/      # The core application
│           ├── drivers/    # Custom hardware drivers (eFlesh/Tactile)
│           ├── recording/  # Data acquisition & formatting
│           └── teach.py    # The "Teach Mode" CLI entry point
├── dependencies/           # External libraries
│   └── stretch_ai/         # Forked Hello Robot drivers
├── training/               # ML Training Stack (PyTorch/LeRobot)
└── hardware/               # Firmware for custom sensors (QT py)


# Build stretch_description Symlink 
cd ros_ws/src
ln -s ../../dependencies/stretch_ros2/stretch_description .

### Install UV
curl -LsSf https://astral.sh/uv/install.sh | sh

### Refresh path
source ~/.bashrc or source ~/.zshrc

cd ~/VTAM

uv sync

source .venv/bin/activate

uv pip install -e dependencies/diffusion_policy --config-setting editable_mode=compat

rsync -av --progress leogray@10.106.29.84:~/VTAM/data/training/pickup_cup.zarr ~/VTAM/data/training/



#### Check the Link
ls -la stretch_description 

cd ..

ls -l /dev/serial/by-path/

sudo apt install python3-serial

pip3 install --user anyskin --break-system-packages


ros2 run vtam_core record_node --ros-args -p demo_name:=wipetable_demo


ros2 service call /record_demo std_srvs/srv/SetBool "{data: True}"