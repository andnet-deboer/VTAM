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