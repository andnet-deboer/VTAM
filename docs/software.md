# Software

## Setup

### Dependencies

```bash
cd ~/VTAM
uv sync
source .venv/bin/activate
uv pip install -e dependencies/diffusion_policy --config-setting editable_mode=compat
```

### LeRobot (stretch-act branch)

```bash
cd dependencies/lerobot
git switch stretch-act
uv pip install av --no-build-isolation
uv pip install -e . --no-deps
```

### HuggingFace login (required for `--push-to-hub`)

```bash
uvx hf auth login
```

### Environment variables

```bash
export ROS_DOMAIN_ID=31
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export PYTHONPATH="/home/kmy2091/VTAM/dependencies/lerobot:$PYTHONPATH"
```

---

## Recording Pipeline

### 1. Launch

```bash
ros2 launch vtam_core vtam.launch.py
ros2 service call /start_session std_srvs/srv/SetBool "{data: true}"
```

Sessions are saved to `data/raw/<task>/session_<timestamp>/`.

### 2. Chunk

Split a session bag into per-episode bags (episodes < 30 frames are discarded).

```bash
python3 training/scripts/bag_chunker.py <task>
# single session:
python3 training/scripts/bag_chunker.py <task> --session session_20260303_230109
```

Output: `data/processed/<task>/episode_000.mcap`, `episode_001.mcap`, ...

### 3. Process

Extract EE poses from `/tf`, run workspace projection, build a HuggingFace dataset.

```bash
python3 training/scripts/process_demo.py <task> \
    --fps 10 --force --tactile \
    --repo-id <hf_user>/<dataset_name> \
    --push-to-hub
```

Output: `data/lerobot/<task>/` with 8D actions `[x, y, z, qx, qy, qz, qw, gripper]`.

### 4. Train

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
    wandb.enable=true
```

Checkpoints land in `dependencies/lerobot/outputs/train/`.

---

## Inference

### Robot (Docker)

```bash
docker run -it --rm --name vtam_production \
  --network host --ipc host --privileged --shm-size=1g \
  -e ROS_DOMAIN_ID=31 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v /home/leogray/VTAM:/home/hello-robot/VTAM \
  hellorobotinc/stretch-ai-ros2-bridge:latest \
  /home/hello-robot/VTAM/inference/robot_server.sh
```

### GPU Machine

```bash
python3 inference/vtam_server_inference.py \
    --policy-path outputs/train/.../checkpoints/last/pretrained_model
```

---

## Key Modules

| Module | Purpose |
|--------|---------|
| `training/utils/differential_ik.py` | Jacobian-based 6-DOF IK (damped least-squares); used at inference |
| `training/utils/kinematics.py` | Analytical IK for trajectory seeding |
| `training/utils/workspace_projection.py` | UMI frame → robot canonical frame transform |
| `training/scripts/process_demo.py` | MCAP → HuggingFace LeRobot dataset |
| `training/scripts/bag_chunker.py` | Session bag → per-episode bags |
| `inference/vtam_server_inference.py` | ZMQ server; runs ACTPolicy on GPU |
| `inference/vtam_robot_node.py` | Robot-side ZMQ bridge; ROS2 sensors → obs, actions → motors |
| `src/vtam_core/nodes/record_node.py` | ROS2 recording node; manages session bags and episode markers |
| `src/vtam_core/drivers/eflesh_node.py` | eFlesh tactile sensor serial driver |
| `src/vtam_core/drivers/umi_detector_node.py` | ArUco cube pose estimator |
