#!/usr/bin/env python3
"""
zarr_to_lerobot.py — Convert VTAM zarr episodes to LeRobot v1.6 format.

Verified against hellorobotinc/kitchen_cabinet_diagonal:
  - action: 9D (no progress token)
  - state:  9D
  - images: PIL Image (video=False)
  - meta:   info.json + stats.safetensors + episode_data_index.safetensors

Joint mapping (retargeter → Hello Robot ACTION_ORDER):
  retargeter idx:  [0=base_rot, 1=base_trans, 2=lift, 3=arm, 4=wrist_yaw, 5=wrist_pitch, 6=wrist_roll]
  ACTION_ORDER:    [base_x,    base_y(0),   base_theta, lift, arm, wrist_roll, wrist_pitch, wrist_yaw, gripper]
  mapping:          1           0.0          0           2     3    6           5            4          gripper

Usage:
    cd /home/leogray/VTAM
    python3 training/scripts/zarr_to_lerobot.py \
        --task pickup_cup \
        --repo-id leogray/vtam_pickup_cup \
        --local-dir data/lerobot/leogray/vtam_pickup_cup \
        --fps 10
"""

import argparse
import json
import os
import sys
import glob
import shutil
from pathlib import Path

import numpy as np
import torch
import zarr
from datasets import Dataset, Features, Image, Sequence, Value
from PIL import Image as PILImage
from safetensors.torch import save_file
from tqdm import tqdm

# ── Path setup ─────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT   = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
LEROBOT_DIR = os.path.join(VTAM_ROOT, 'dependencies', 'lerobot')
sys.path.insert(0, LEROBOT_DIR)

from lerobot.common.datasets.push_dataset_to_hub.utils import concatenate_episodes
from lerobot.common.datasets.utils import hf_transform_to_torch, flatten_dict
from lerobot.common.datasets.compute_stats import compute_stats
from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset

# ── Constants (verified against hellorobotinc/kitchen_cabinet_diagonal) ────────
ACTION_ORDER = [
    "base_x_joint",        # idx 0 ← retargeter base_translation (idx 1)
    "base_y_joint",        # idx 1 ← always 0.0 (Stretch can't strafe)
    "base_theta_joint",    # idx 2 ← retargeter base_rotation (idx 0)
    "joint_lift",          # idx 3 ← retargeter lift (idx 2)
    "joint_arm_l0",        # idx 4 ← retargeter arm (idx 3)
    "joint_wrist_roll",    # idx 5 ← retargeter wrist_roll (idx 6)
    "joint_wrist_pitch",   # idx 6 ← retargeter wrist_pitch (idx 5)
    "joint_wrist_yaw",     # idx 7 ← retargeter wrist_yaw (idx 4)
    "stretch_gripper",     # idx 8 ← gripper_position
]

STATE_ORDER = [
    "base_x",
    "base_y",
    "base_theta",
    "lift",
    "arm",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
    "gripper_finger_right",
]

ACTION_DIM = 9
STATE_DIM  = 9


def remap_to_action(joint_states, gripper):
    """(T,7) joint_states + (T,1) gripper → (T,9) action in ACTION_ORDER."""
    T = len(joint_states)
    action = np.zeros((T, ACTION_DIM), dtype=np.float32)
    action[:, 0] = joint_states[:, 1]   # base_x      ← base_translation
    action[:, 1] = 0.0                   # base_y      ← always 0
    action[:, 2] = joint_states[:, 0]   # base_theta  ← base_rotation
    action[:, 3] = joint_states[:, 2]   # lift
    action[:, 4] = joint_states[:, 3]   # arm
    action[:, 5] = joint_states[:, 6]   # wrist_roll
    action[:, 6] = joint_states[:, 5]   # wrist_pitch
    action[:, 7] = joint_states[:, 4]   # wrist_yaw
    action[:, 8] = gripper.flatten()    # gripper
    return action


def remap_to_state(joint_states, gripper):
    """(T,7) joint_states + (T,1) gripper → (T,9) state in STATE_ORDER."""
    T = len(joint_states)
    state = np.zeros((T, STATE_DIM), dtype=np.float32)
    state[:, 0] = joint_states[:, 1]
    state[:, 1] = 0.0
    state[:, 2] = joint_states[:, 0]
    state[:, 3] = joint_states[:, 2]
    state[:, 4] = joint_states[:, 3]
    state[:, 5] = joint_states[:, 6]
    state[:, 6] = joint_states[:, 5]
    state[:, 7] = joint_states[:, 4]
    state[:, 8] = gripper.flatten()
    return state


def load_episode(zarr_path):
    """Load one zarr, validate, remap. Returns None if invalid."""
    r = zarr.open(zarr_path, mode='r')

    for key in ['obs/joint_states', 'obs/gripper_position', 'obs/video_wrist']:
        if key not in r:
            print(f"  SKIP {os.path.basename(zarr_path)}: missing {key}")
            return None

    joint_states = np.array(r['obs/joint_states'],      dtype=np.float32)
    gripper      = np.array(r['obs/gripper_position'],  dtype=np.float32)
    images       = np.array(r['obs/video_wrist'],       dtype=np.uint8)

    if joint_states.ndim != 2 or joint_states.shape[1] != 7:
        print(f"  SKIP {os.path.basename(zarr_path)}: bad joint_states {joint_states.shape}")
        return None

    T = len(joint_states)
    if T < 30:
        print(f"  SKIP {os.path.basename(zarr_path)}: too short T={T}")
        return None

    if images.shape[1:3] != (320, 320):
        print(f"  SKIP {os.path.basename(zarr_path)}: bad image size {images.shape}")
        return None

    return {
        'images':  images,
        'action':  remap_to_action(joint_states, gripper),
        'state':   remap_to_state(joint_states, gripper),
        'T':       T,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task',      type=str,  required=True)
    parser.add_argument('--repo-id',   type=str,  required=True,
                        help='e.g. leogray/vtam_pickup_cup')
    parser.add_argument('--local-dir', type=Path, required=True,
                        help='e.g. data/lerobot/leogray/vtam_pickup_cup')
    parser.add_argument('--fps',       type=int,  default=10)
    parser.add_argument('--force',     type=int,  default=0)
    args = parser.parse_args()

    local_dir  = Path(args.local_dir)
    meta_dir   = local_dir / 'meta_data'
    train_dir  = local_dir / 'train'

    if local_dir.exists():
        if args.force:
            shutil.rmtree(local_dir)
        else:
            print(f"ERROR: {local_dir} already exists. Use --force 1 to overwrite.")
            sys.exit(1)

    task_dir   = os.path.join(VTAM_ROOT, 'data', 'processed', args.task)
    zarr_paths = sorted(glob.glob(os.path.join(task_dir, '*.zarr')))
    print(f"Found {len(zarr_paths)} zarrs in {task_dir}\n")

    # ── Process episodes ───────────────────────────────────────────────────────
    ep_dicts           = []
    episode_data_index = {'from': [], 'to': []}
    cursor             = 0

    for ep_idx, zpath in enumerate(tqdm(zarr_paths, desc='Converting')):
        ep = load_episode(zpath)
        if ep is None:
            continue

        T = ep['T']
        pil_images = [PILImage.fromarray(ep['images'][i]) for i in range(T)]

        done = torch.zeros(T, dtype=torch.bool)
        done[-1] = True

        ep_dict = {
            'observation.images.gripper': pil_images,
            'observation.state':          torch.from_numpy(ep['state']),
            'action':                     torch.from_numpy(ep['action']),
            'episode_index':              torch.tensor([ep_idx] * T),
            'frame_index':                torch.arange(0, T),
            'timestamp':                  torch.arange(0, T, dtype=torch.float32) / args.fps,
            'next.done':                  done,
        }

        ep_dicts.append(ep_dict)
        episode_data_index['from'].append(cursor)
        episode_data_index['to'].append(cursor + T)
        cursor += T

    print(f"\nConverted {len(ep_dicts)} episodes, {cursor} total frames")

    # ── Build HuggingFace dataset ──────────────────────────────────────────────
    data_dict          = concatenate_episodes(ep_dicts)
    data_dict['index'] = torch.arange(0, cursor)

    features = Features({
        'observation.images.gripper': Image(),
        'observation.state':          Sequence(length=STATE_DIM,  feature=Value('float32')),
        'action':                     Sequence(length=ACTION_DIM, feature=Value('float32')),
        'episode_index':              Value('int64'),
        'frame_index':                Value('int64'),
        'timestamp':                  Value('float32'),
        'next.done':                  Value('bool'),
        'index':                      Value('int64'),
    })

    hf_dataset = Dataset.from_dict(data_dict, features=features)
    hf_dataset.set_transform(hf_transform_to_torch)

    # ── Info ───────────────────────────────────────────────────────────────────
    info = {
        'codebase_version': CODEBASE_VERSION,
        'fps':              args.fps,
        'video':            False,
        'action_order':     ACTION_ORDER,
        'state_order':      STATE_ORDER,
        'image_size':       {'gripper': [320, 320]},
        'num_episodes':     len(ep_dicts),
        'num_frames':       cursor,
        'task':             args.task,
        'episode_metadata': [
            {'task': args.task, 'num_frames': int(ep_dicts[i]['action'].shape[0])}
            for i in range(len(ep_dicts))
        ],
    }

    # ── Compute stats ──────────────────────────────────────────────────────────
    print("\nComputing dataset statistics...")
    lerobot_dataset = LeRobotDataset.from_preloaded(
        repo_id=args.repo_id,
        hf_dataset=hf_dataset,
        episode_data_index=episode_data_index,
        info=info,
        videos_dir=None,
    )
    stats = compute_stats(lerobot_dataset, batch_size=32, num_workers=4)

    # ── Save to disk ───────────────────────────────────────────────────────────
    hf_dataset.with_format(None).save_to_disk(str(train_dir))

    meta_dir.mkdir(parents=True, exist_ok=True)
    with open(meta_dir / 'info.json', 'w') as f:
        json.dump(info, f, indent=4)

    save_file({k: torch.tensor(v) for k, v in episode_data_index.items()},
              meta_dir / 'episode_data_index.safetensors')
    save_file(flatten_dict(stats), meta_dir / 'stats.safetensors')

    print(f"\n✓ Dataset saved to: {local_dir}")
    print(f"  Episodes : {len(ep_dicts)}")
    print(f"  Frames   : {cursor}")
    print(f"\nTo train:")
    print(f"  cd {VTAM_ROOT}/dependencies/lerobot")
    print(f"  python3 lerobot/scripts/train.py \\")
    print(f"    policy=stretch_diffusion \\")
    print(f"    env=stretch_real \\")
    print(f"    dataset_repo_id={args.repo_id} \\")
    print(f"    training.root={local_dir.parent.parent} \\")
    print(f"    training.batch_size=64 \\")
    print(f"    training.num_workers=8 \\")
    print(f"    wandb.enable=false")


if __name__ == '__main__':
    main()