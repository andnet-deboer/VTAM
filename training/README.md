# VTAM Training Pipeline

End-to-end pipeline for recording, processing, and training robot manipulation policies on Stretch 3.

## Pipeline Overview

```
Robot Demo
     record_demo_node.py
data/raw/<task>/session_<timestamp>.mcap        - full session bag
     scripts/bag_chunker.py
data/processed/<task>/episode_000.mcap          - one bag per episode
     scripts/process_demo.py
data/lerobot/<task>/                            - HuggingFace dataset
     lerobot/scripts/train.py
checkpoints/
```

---

## Directory Structure

```
training/
├── scripts/
│   ├── bag_chunker.py       — Split session bag → episode bags
│   ├── process_demo.py      — Episode bags → LeRobot dataset (EE pose + projection)
│   └── zarr_to_lerobot.py   — Legacy: zarr → LeRobot dataset (joint space)
├── utils/
│   ├── workspace_projection.py   — Project UMI hand poses → robot canonical frame
│   ├── differential_ik.py        — Full retargeting pipeline (used at inference)
│   ├── kinematics.py             — Analytical IK solver (seed for diff IK)
│   ├── stretch_omni_mobile_ik.urdf  — 7-DOF URDF for pinocchio
│   ├── replay_on_robot.py        — Replay a recorded trajectory on the robot
│   ├── validate_pipeline.py      — Validate zarr episode integrity
│   ├── dataset_report.py         — Print dataset statistics
│   ├── debug_tf.py               — Debug TF frame extraction from bags
│   ├── visualize_ik.py           — Visualize IK results
│   └── foxglove_viz.py           — Foxglove bridge utilities
└── experiments/                  — Experiment notes and plots
```

---

## Step 1 — Record

```bash
# Launch the full system
ros2 launch vtam_core vtam.launch.py

# Start a recording session (3s warmup, then ready)
ros2 service call /start_session std_srvs/srv/SetBool "{data: true}"

# Toggle episode start/stop (or use physical button)
ros2 service call /record_demo std_srvs/srv/SetBool "{data: true}"
```

Session bags are saved to `data/raw/<task>/session_<timestamp>/`.

---

## Step 2 — Chunk

Split one session bag into individual episode bags.

```bash
# All sessions for a demo
python3 ~/VTAM/training/scripts/bag_chunker.py default_demo

# One specific session
python3 ~/VTAM/training/scripts/bag_chunker.py default_demo --session session_20260303_230109
```

Output: `data/processed/pickup_cup/episode_000.mcap`, `episode_001.mcap`, ...

Episodes under 30 frames are discarded automatically (accidental button presses).

---

## Step 3 — Process

make sure to login to hugging face 

uvx hf auth logout

uvx hf auth login

Extract EE poses from `/tf`, run workspace projection, build LeRobot dataset.

```bash
python3 ~/VTAM/training/scripts/process_demo.py default_demo --push-to-hub
```

**What this does:**
- Extracts UMI hand pose from `/tf` at each camera frame timestamp
- Calls `workspace_projection.project()` → canonical EE pose (N, 7)
- Extracts `/gripper_width_normalized` → (N,)
- Extracts `/camera_arm` images → (N, 320, 320, 3)
- Computes progress token (0→1) per episode
- Packages as HuggingFace dataset with `action = [x, y, z, qx, qy, qz, qw, gripper]` (8D)

---

## Step 4 — Train

```bash
cd dependencies/lerobot
python3 lerobot/scripts/train.py \
    policy=stretch_diffusion \
    env=stretch_real \
    dataset_repo_id=leogray/vtam_pickup_cup \
    training.root=../../data/lerobot \
    training.batch_size=64 \
    training.num_workers=8 \
    wandb.enable=false
```

---

## Action / State Space

The new EE pose pipeline replaces the old 9D joint space representation.

| Field | Dim | Description |
|-------|-----|-------------|
| `x, y, z` | 3 | EE position in robot canonical frame (metres) |
| `qx, qy, qz, qw` | 4 | EE orientation as XYZW quaternion |
| `gripper` | 1 | Normalised gripper width [0, 1] |
| **Total** | **8** | |

Progress token `[0.0 → 1.0]` is available as an observation but not part of the action.

---

## Inference

At inference, policy outputs target EE pose → `differential_ik.py` converts to joint commands live:

```python
from utils.differential_ik import TrajectoryRetargeter

retargeter = TrajectoryRetargeter()
q_current  = read_joint_states_from_robot()
ee_target  = policy.predict(obs)               # (8,) [x,y,z,qx,qy,qz,qw,gripper]

q_new, pos_err, ori_err = retargeter.step_ik(
    q_current, ee_target[:3], ee_target[3:7]
)
send_joint_commands(q_new)
gripper_cmd = ee_target[7]
```

---

## Data Directory

```
data/
├── raw/          - session bags off the robot
├── processed/    - episode bags (chunked, ready for processing)
└── lerobot/      - HuggingFace datasets (ready for training)
```

---
