# teleoperation/

Phone-based leader-follower teleoperation for VTAM data collection.

## Paradigm

The iPhone (ARKit via TeleTool2) is the leader. The robot arm is the follower.
Each frame, the SE(3) delta between the current and previous phone pose is fed
into the differential IK solver to produce joint commands. The robot executes
those commands and its actual `/joint_states` are what gets recorded — **no IK
at training or inference time**.

```
/phone_teleop/pose  (teleop_receiver_node)
        │
        ▼
  T_delta = T_now @ inv(T_prev)   ← phone motion this frame
        │
        ▼
  T_target = FK(q_current) @ T_delta_scaled
        │
        ▼
  differential IK  →  joint commands  →  robot
        │
        ▼
  /joint_states  ←  this is the training data
```

## Files

| File | Purpose |
|------|---------|
| `run.py` | Main script. `--mode teleop` for collection, `--mode inference` for policy rollout |

## Prerequisites

All three must be running before `run.py`:

1. **teleop_receiver_node** — receives TeleTool2 socket stream, publishes `/phone_teleop/pose`
   ```bash
   ros2 run vtam_core teleop_receiver_node
   ```

2. **vtam_robot_node** — reads `/joint_states`, camera; bridges to ZMQ
   ```bash
   ros2 run vtam_core vtam_robot_node   # or python3 inference/robot_inference.py
   ```

3. **TeleTool2 on iPhone** — connected to this machine's IP on port 8080

## Running teleoperation

```bash
# Basic (1:1 scale)
python3 teleoperation/run.py --mode teleop

# With motion scaling (phone moves 1 unit → robot moves 1.5 units)
python3 teleoperation/run.py --mode teleop --scale 1.5

# Dry run — IK runs but no commands sent to robot (safe for testing)
python3 teleoperation/run.py --mode teleop --dry-run

# Slower control rate (default 10 Hz)
python3 teleoperation/run.py --mode teleop --fps 5
```

Move the phone to move the robot. Stop moving the phone → robot holds position.
Ctrl+C to stop. Diagnostics written to `/tmp/vtam_diag/`.

## Running policy inference (unchanged)

```bash
python3 teleoperation/run.py --mode inference
```

## Frame math

The phone poses from `teleop_receiver_node` are in `arkit_world` frame with
ROS axis conventions (X forward, Y left, Z up). The SE(3) delta between
consecutive frames is frame-origin-independent, so the arbitrary ARKit world
origin does not matter.

The translation delta is applied directly in the robot's EE frame
(`T_target = T_ee_current @ T_delta`), which means phone forward/back maps
to EE forward/back. For best results, hold the phone with the same orientation
as the robot's gripper is facing.

## Motion scale tuning

Start with `--scale 1.0`. If the robot motion is too small relative to phone
motion, increase. If too large/jerky, decrease or reduce `--fps`.

## What to record

Use the existing `record_node` with updated `SESSION_TOPICS` (remove
`/umi_cube_pose`, keep `/joint_states`). The rosbag + offline `process_demo.py`
pipeline is unchanged — only the EE pose source changes (joint states instead
of TF chain).
