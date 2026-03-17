# VTAM Training Pipeline

End-to-end pipeline: robot recording to dataset to policy.

```
data/raw/<task>/session_<timestamp>.mcap
    bag_chunker.py
data/chunked/<task>/episode_000.mcap  episode_001.mcap  ...
    detect_umi.py
data/processed/<task>/episode_000.mcap  episode_001.mcap  ...
    process_demo.py
data/lerobot/<task>/                        (HuggingFace dataset)
    lerobot/scripts/train.py
dependencies/lerobot/outputs/train/*/checkpoints/
```

---

## Step 1 — Record

```bash
ros2 launch vtam_core vtam.launch.xml
ros2 service call /start_session std_srvs/srv/SetBool "{data: true}"
ros2 service call /record_demo std_srvs/srv/SetBool "{data: true}"
```

Session bags saved to `data/raw/<task>/session_<timestamp>/`.

---

## Step 2 — Chunk

Split a session bag into per-episode bags. Episodes under 30 frames are discarded.

```bash
python3 training/scripts/bag_chunker.py <task>
# single session:
python3 training/scripts/bag_chunker.py <task> --session session_20260303_230109
```

---

## Step 3 — Detect UMI

Run offline AprilTag detection on each chunked episode. Reads `/tf_static` and live `/tf` from the bag to transform the detected cube pose into `base_link` frame, and writes `umi_disconnect` and `umi_gripper` TF frames into new bags in `data/processed/`.

```bash
python3 training/scripts/detect_umi.py <task>
```

---

## Step 4 — Process

Login to HuggingFace first:
```bash
uvx hf auth login
```

Extract EE poses from `/tf`, build LeRobot dataset:

**With tactile** (includes 30D eFlesh observations):
```bash
python3 training/scripts/process_demo.py <task> \
    --fps 10 --force --tactile \
    --repo-id <hf_user>/<dataset_name>_tactile \
    --push-to-hub
```

**Without tactile** (RGB + joints only):
```bash
python3 training/scripts/process_demo.py <task> \
    --fps 10 --force \
    --repo-id <hf_user>/<dataset_name> \
    --push-to-hub
```

Reads from `data/processed/<task>/` (output of `detect_umi.py`). What this does:
- Extracts UMI hand pose from `/tf` at each camera frame timestamp to get canonical EE pose (N, 7)
- Extracts `/gripper_width_normalized` (N,)
- Extracts `/camera_arm` images (N, 320, 320, 3)
- Computes progress token (0 to 1) per episode
- Packages as HuggingFace dataset with `action = [x, y, z, qx, qy, qz, qw, gripper]` (8D)

---

## Step 5 — Train

```bash
cd dependencies/lerobot
CUDA_VISIBLE_DEVICES=1 python3 lerobot/scripts/train.py \
    policy=stretch_act_real_vtam_rel \
    env=stretch_real_vtam \
    dataset_repo_id=<hf_user>/<dataset_name> \
    training.batch_size=256 \
    training.num_workers=16 \
    training.image_transforms.enable=true \
    training.save_freq=1000 \
    training.log_freq=50 \
    wandb.enable=true
```

---

## Action Space (8D)

| Field | Dim | Description |
|-------|-----|-------------|
| `x, y, z` | 3 | EE position in robot canonical frame (metres) |
| `qx, qy, qz, qw` | 4 | EE orientation as XYZW quaternion |
| `gripper` | 1 | Normalised gripper width [0, 1] |

Actions are relative deltas, invariant to robot starting configuration. Progress token [0.0 to 1.0] is an observation only.

---

## Scripts

| Script | Purpose |
|--------|---------|
| `scripts/bag_chunker.py` | Split session bag into episode bags on `/recording/active` markers |
| `scripts/process_demo.py` | Episode bags to HuggingFace LeRobot dataset |
| `inference/run.py` | Replay a recorded trajectory or run live policy inference on the robot |
| `scripts/validate_demos.py` | Validate episode integrity |
| `scripts/detect_umi.py` | Offline AprilTag detection: chunked bags to processed bags with UMI TF injected |
| `scripts/fuse_imu.py` | IMU sensor fusion |
| `scripts/goto_demo_pose.py` | Move robot to neutral demo pose |

## Utils

| Module | Purpose |
|--------|---------|
| `utils/differential_ik.py` | Jacobian-based 6-DOF IK (damped least-squares); used at inference |
| `utils/kinematics.py` | Analytical IK for trajectory seeding |
| `utils/visualize_ik.py` | Visualise IK results |
| `utils/stretch_omni_mobile_ik.urdf` | 7-DOF URDF for pinocchio |