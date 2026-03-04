#!/usr/bin/env python3
"""
process_demo.py — Convert episode MCAP bags → LeRobot dataset.

Pipeline position:
    chunk_bag.py → data/processed/episode_000.mcap → process_demo.py → data/lerobot/

What this does per episode:
    1. Read sync timestamps from /sync_pulse (JointState header.stamp)
    2. Read /progress directly from bag (injected by chunk_bag.py)
    3. Extract EE pose via TF chain: base_link → umi_disconnect → umi_gripper
    4. Snap all topics to /sync_pulse timestamps (nearest-neighbour)
    5. Apply workspace_projection.project()  canonical EE pose (N, 7)
    6. Pack into HuggingFace LeRobot dataset

Action / State space (8D):
    [x, y, z, qx, qy, qz, qw, gripper]

Usage:
    python3 training/scripts/process_demo.py \
        --task pickup_cup \
        --processed-dir data/processed/pickup_cup/ \
        --lerobot-dir data/lerobot/pickup_cup/ \
        --fps 10
"""

import argparse
import glob
import json
import os
import shutil
import sys
import struct
from pathlib import Path

import cv2
import numpy as np
import torch
from datasets import Dataset, Features, Image, Sequence, Value
from PIL import Image as PILImage
from safetensors.torch import save_file
from scipy.spatial.transform import Rotation
from tqdm import tqdm
import yaml

# ── Path setup ─────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT   = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
UTILS_DIR   = os.path.join(SCRIPT_DIR, '..', 'utils')
LEROBOT_DIR = os.path.join(VTAM_ROOT, 'dependencies', 'lerobot')
CONFIG_PATH = os.path.expanduser("~/VTAM/vtam_core/config/record.yaml")


def load_topics(config_path: str) -> list[str]:
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['/**']['ros__parameters']['session_topics']

SESSION_TOPICS = load_topics(CONFIG_PATH)

# Derive specific topics from config (with fallbacks)
def find_topic(topics, keyword):
    matches = [t for t in topics if keyword in t]
    return matches[0] if matches else None

SYNC_TOPIC     = find_topic(SESSION_TOPICS, 'sync_pulse')
IMAGE_TOPIC    = find_topic(SESSION_TOPICS, 'color/image_rect_raw')
DEPTH_TOPIC = find_topic(SESSION_TOPICS, 'depth/image_rect_raw')
GRIPPER_TOPIC  = find_topic(SESSION_TOPICS, 'gripper_width')
PROGRESS_TOPIC = "/progress"  # injected by chunker, not in record.yaml

sys.path.insert(0, UTILS_DIR)
sys.path.insert(0, LEROBOT_DIR)

from workspace_projection import WorkspaceProjector
from mcap_ros2.reader import read_ros2_messages
from lerobot.common.datasets.push_dataset_to_hub.utils import concatenate_episodes
from lerobot.common.datasets.utils import hf_transform_to_torch, flatten_dict
from lerobot.common.datasets.compute_stats import compute_stats
from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset

# ── Configuration ──────────────────────────────────────────────────────────────

IMAGE_SIZE    = (320, 320)
MIN_FRAMES    = 30
ACTION_DIM    = 8   # [x, y, z, qx, qy, qz, qw, gripper]
STATE_DIM     = 8

# TF chain
TF_CHAIN = [
    ("base_link",      "umi_disconnect"),
    ("umi_disconnect", "umi_gripper"),
]

ACTION_ORDER = ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]
STATE_ORDER  = ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]


# ── TF utilities ───────────────────────────────────────────────────────────────

def tf_to_matrix(pose7: np.ndarray) -> np.ndarray:
    """(7,) [x,y,z,qx,qy,qz,qw] → (4,4) SE3 matrix."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(pose7[3:]).as_matrix()
    T[:3, 3]  = pose7[:3]
    return T


def extract_ee_pose(tf_data: dict, tf_ts: dict) -> tuple[list, list]:
    """
    Stitch TF chain into EE poses.
    Identical logic to original ZarrSynchronizer.extract_ee_pose().

    Returns:
        ee_poses:      list of (7,) float32 [x,y,z,qx,qy,qz,qw]
        ee_timestamps: list of float (seconds)
    """
    drive_pair = TF_CHAIN[0]

    if not tf_data[drive_pair]:
        print("  WARNING: No UMI detections in this bag.")
        return [], []

    ts_arrays = {p: np.array(tf_ts[p]) for p in TF_CHAIN}
    ee_poses, ee_timestamps = [], []

    for i in range(len(tf_data[drive_pair])):
        t       = tf_ts[drive_pair][i]
        T_total = tf_to_matrix(tf_data[drive_pair][i])

        for pair in TF_CHAIN[1:]:
            idx = np.clip(
                np.searchsorted(ts_arrays[pair], t) - 1,
                0, len(tf_data[pair]) - 1
            )
            T_total = T_total @ tf_to_matrix(tf_data[pair][idx])

        pos  = T_total[:3, 3]
        quat = Rotation.from_matrix(T_total[:3, :3]).as_quat()
        ee_poses.append(np.concatenate([pos, quat]).astype(np.float32))
        ee_timestamps.append(t)

    return ee_poses, ee_timestamps


# ── Per-episode processing ─────────────────────────────────────────────────────

def process_episode(mcap_path: str, projector: WorkspaceProjector) -> dict | None:
    """
    Process one episode bag → dict ready for LeRobot packaging.
    Returns None if episode is invalid.
    """
    sync_ts      = []
    progress_buf = []
    tf_data      = {p: [] for p in TF_CHAIN}
    tf_ts        = {p: [] for p in TF_CHAIN}
    img_buf,     img_ts      = [], []
    gripper_buf, gripper_ts  = [], []
    depth_buf, depth_ts = [], []

    all_topics = [t for t in [
        SYNC_TOPIC, PROGRESS_TOPIC, "/tf",
        IMAGE_TOPIC, DEPTH_TOPIC, GRIPPER_TOPIC,
    ] if t is not None]

    for msg in read_ros2_messages(mcap_path, topics=all_topics):
        topic = msg.channel.topic
        t     = msg.publish_time_ns / 1e9

        if topic == SYNC_TOPIC:
            stamp = msg.ros_msg.header.stamp
            sync_ts.append(stamp.sec + stamp.nanosec / 1e9)

        elif topic == PROGRESS_TOPIC:
            progress_buf.append(float(msg.ros_msg.data))

        elif topic == "/tf":
            for tf in msg.ros_msg.transforms:
                pair = (tf.header.frame_id, tf.child_frame_id)
                if pair in tf_data:
                    tr, ro = tf.transform.translation, tf.transform.rotation
                    tf_data[pair].append(
                        np.array([tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w])
                    )
                    tf_ts[pair].append(t)

        elif topic == IMAGE_TOPIC:
            buf = np.frombuffer(msg.ros_msg.data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            h, w = img.shape[:2]
            s = min(h, w)
            y0, x0 = (h - s) // 2, (w - s) // 2
            img = img[y0:y0+s, x0:x0+s]
            img = cv2.resize(img, IMAGE_SIZE).astype(np.uint8)
            img_buf.append(img)
            img_ts.append(t)

        elif topic == GRIPPER_TOPIC:
            gripper_buf.append(float(msg.ros_msg.data))
            gripper_ts.append(t)


        elif topic == DEPTH_TOPIC:
            raw = bytes(msg.ros_msg.data)
            png_data = raw[12:]
            buf = np.frombuffer(png_data, dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_ANYDEPTH)
            h, w = img.shape[:2]
            s = min(h, w)
            y0, x0 = (h - s) // 2, (w - s) // 2
            img = img[y0:y0+s, x0:x0+s]
            img = cv2.resize(img, IMAGE_SIZE)

            valid = img[img > 0]
            if len(valid) == 0:
                img8 = np.zeros(IMAGE_SIZE, dtype=np.uint8)
            else:
                d_min, d_max = np.percentile(valid, 5), np.percentile(valid, 95)
                img8 = np.clip((img.astype(np.float32) - d_min) / (d_max - d_min + 1e-6) * 255, 0, 255).astype(np.uint8)

            img3 = np.stack([img8, img8, img8], axis=-1)
            depth_buf.append(img3)
            depth_ts.append(t)
            
            

    # ── Validate ───────────────────────────────────────────────────────────────
    if len(sync_ts) < MIN_FRAMES:
        print(f"  SKIP: only {len(sync_ts)} sync frames (min {MIN_FRAMES})")
        return None

    if not tf_data[TF_CHAIN[0]]:
        print("  SKIP: no TF data for UMI chain")
        return None

    if not img_buf:
        print("  SKIP: no camera images")
        return None

    # ── Extract EE poses ───────────────────────────────────────────────────────
    ee_poses_raw, ee_ts = extract_ee_pose(tf_data, tf_ts)
    if not ee_poses_raw:
        print("  SKIP: EE pose extraction failed")
        return None

    # ── Snap everything to /sync_pulse timestamps ──────────────────────────────
    sync_arr    = np.array(sync_ts)
    ee_ts_arr   = np.array(ee_ts)
    img_ts_arr  = np.array(img_ts)
    grip_ts_arr = np.array(gripper_ts)

    def snap(buf, ts_arr):
        idx = np.clip(np.searchsorted(ts_arr, sync_arr) - 1, 0, len(buf) - 1)
        return [buf[i] for i in idx]
    
    N = len(sync_ts)
    ee_synced      = snap(ee_poses_raw, ee_ts_arr)
    images_synced  = snap(img_buf,      img_ts_arr)
    gripper_synced = snap(gripper_buf,  grip_ts_arr)
    depth_ts_arr   = np.array(depth_ts) if depth_ts else np.array([0.0])
    depth_synced   = snap(depth_buf, depth_ts_arr) if depth_buf else [np.zeros((*IMAGE_SIZE, 3), dtype=np.uint8)] * N

    # Progress: use from bag if available, otherwise recompute
    if len(progress_buf) == N:
        progress = np.array(progress_buf, dtype=np.float32)
    else:
        progress = np.linspace(0.0, 1.0, N, dtype=np.float32)

    # ── Workspace projection ───────────────────────────────────────────────────
    positions   = np.array([p[:3] for p in ee_synced], dtype=np.float64)
    quaternions = np.array([p[3:] for p in ee_synced], dtype=np.float64)

    result       = projector.project(positions, quaternions, sync_arr)
    ee_canonical = result['ee_poses'].astype(np.float32)  # (N, 7)

    # ── Build action / state (8D) ──────────────────────────────────────────────
    gripper_arr = np.array(gripper_synced, dtype=np.float32).reshape(N, 1)
    action      = np.concatenate([ee_canonical, gripper_arr], axis=1)  # (N, 8)
    state       = action.copy()

    print(f"  {N} frames | "
          f"x[{ee_canonical[:,0].min():.3f}, {ee_canonical[:,0].max():.3f}] "
          f"z[{ee_canonical[:,2].min():.3f}, {ee_canonical[:,2].max():.3f}] | "
          f"gripper[{gripper_arr.min():.2f}, {gripper_arr.max():.2f}]")

    return {
        'images':   images_synced,
        'depth_images': depth_synced,
        'action':   action,           # (N, 8) float32
        'state':    state,            # (N, 8) float32
        'progress': progress,         # (N,)   float32
        'T':        N,
    }


# ── Main ───────────────────────────────────────────────────────────────────────

def main(args):

    lerobot_dir = Path(args.lerobot_dir)
    meta_dir    = lerobot_dir / 'meta_data'
    train_dir   = lerobot_dir / 'train'
    repo_id     = args.repo_id or f"leogray/vtam_{args.task}"

    if lerobot_dir.exists():
        if args.force:
            shutil.rmtree(lerobot_dir)
        else:
            print(f"ERROR: {lerobot_dir} exists. Use --force to overwrite.")
            sys.exit(1)

    mcap_paths = sorted(glob.glob(str(args.processed_dir / '*.mcap')))
    if not mcap_paths:
        print(f"No .mcap files found in {args.processed_dir}")
        sys.exit(1)
    print(f"Found {len(mcap_paths)} episode bags\n")

    # Load projector once — reused across all episodes (self.neutral_q never mutated)
    projector = WorkspaceProjector()

    # ── Process episodes ───────────────────────────────────────────────────────
    ep_dicts           = []
    episode_data_index = {'from': [], 'to': []}
    cursor             = 0

    for ep_idx, mcap_path in enumerate(tqdm(mcap_paths, desc='Processing')):
        print(f"\nEpisode {ep_idx}: {os.path.basename(mcap_path)}")
        ep = process_episode(mcap_path, projector)
        if ep is None:
            continue

        T          = ep['T']
        pil_images = [PILImage.fromarray(ep['images'][i]) for i in range(T)]
        done       = torch.zeros(T, dtype=torch.bool)
        done[-1]   = True

        ep_dict = {
            'observation.images.gripper': pil_images,
            'observation.images.gripper_depth': [PILImage.fromarray(ep['depth_images'][i]) for i in range(T)],
            'observation.state':          torch.from_numpy(ep['state']),
            'observation.progress':       torch.from_numpy(ep['progress']),
            'action':                     torch.from_numpy(ep['action']),
            'episode_index':              torch.tensor([ep_idx] * T),
            'frame_index':                torch.arange(T),
            'timestamp':                  torch.arange(T, dtype=torch.float32) / args.fps,
            'next.done':                  done,
        }

        ep_dicts.append(ep_dict)
        episode_data_index['from'].append(cursor)
        episode_data_index['to'].append(cursor + T)
        cursor += T

    if not ep_dicts:
        print("No valid episodes processed. Exiting.")
        sys.exit(1)

    print(f"\nConverted {len(ep_dicts)} episodes, {cursor} total frames")

    # ── Build HuggingFace dataset ──────────────────────────────────────────────
    data_dict          = concatenate_episodes(ep_dicts)
    data_dict['index'] = torch.arange(cursor)

    features = Features({
        'observation.images.gripper': Image(),
        'observation.images.gripper_depth': Image(),
        'observation.state':          Sequence(length=STATE_DIM,  feature=Value('float32')),
        'observation.progress':       Value('float32'),
        'action':                     Sequence(length=ACTION_DIM, feature=Value('float32')),
        'episode_index':              Value('int64'),
        'frame_index':                Value('int64'),
        'timestamp':                  Value('float32'),
        'next.done':                  Value('bool'),
        'index':                      Value('int64'),
    })

    hf_dataset = Dataset.from_dict(data_dict, features=features)

    # ── Info ───────────────────────────────────────────────────────────────────
    info = {
        'codebase_version': CODEBASE_VERSION,
        'fps':              args.fps,
        'video':            False,
        'action_order':     ACTION_ORDER,
        'state_order':      STATE_ORDER,
        'image_size':       {'gripper': list(IMAGE_SIZE)},
        'num_episodes':     len(ep_dicts),
        'num_frames':       cursor,
        'task':             args.task,
    }

    # ── Compute stats ──────────────────────────────────────────────────────────
    print("\nComputing dataset statistics...")
    hf_dataset_for_stats = hf_dataset.with_transform(hf_transform_to_torch)
    lerobot_dataset = LeRobotDataset.from_preloaded(
        repo_id=repo_id,
        hf_dataset=hf_dataset_for_stats,
        episode_data_index=episode_data_index,
        info=info,
        videos_dir=None,
    )
    stats = compute_stats(lerobot_dataset, batch_size=32, num_workers=4)

    # ── Save ───────────────────────────────────────────────────────────────────────
    clean_dataset = Dataset.from_dict(hf_dataset.to_dict(), features=features)

    # Always save locally
    clean_dataset.save_to_disk(str(train_dir))
    meta_dir.mkdir(parents=True, exist_ok=True)

    with open(meta_dir / 'info.json', 'w') as f:
        json.dump(info, f, indent=4)

    save_file(
        {k: torch.tensor(v) for k, v in episode_data_index.items()},
        meta_dir / 'episode_data_index.safetensors'
    )
    save_file(flatten_dict(stats), meta_dir / 'stats.safetensors')

    # Push to HuggingFace Hub if requested
    if args.push_to_hub:
        print("\nPushing to HuggingFace Hub...")
        from huggingface_hub import HfApi
        api = HfApi()
        api.create_repo(repo_id=repo_id, repo_type="dataset", exist_ok=True)
        clean_dataset.push_to_hub(repo_id, split="train")
        # Also upload meta_data
        api.upload_folder(
            folder_path=str(meta_dir),
            repo_id=repo_id,
            repo_type="dataset",
            path_in_repo="meta_data",
            ignore_patterns=["*.lock"],
        )
        print(f"Published: https://huggingface.co/datasets/{repo_id}")
    
    print(f"\nDataset saved to: {lerobot_dir}")
    print(f"  Episodes : {len(ep_dicts)}")
    print(f"  Frames   : {cursor}")
    print(f"\nTo train:")
    print(f"  cd {VTAM_ROOT}/dependencies/lerobot")
    print(f"  python3 lerobot/scripts/train.py \\")
    print(f"    policy=stretch_diffusion \\")
    print(f"    env=stretch_real \\")
    print(f"    dataset_repo_id={repo_id} \\")
    print(f"    training.root={lerobot_dir.parent} \\")
    print(f"    training.batch_size=64 \\")
    print(f"    training.num_workers=8 \\")
    print(f"    wandb.enable=false")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('demo_name', type=str)
    parser.add_argument('--fps',         type=int, default=10)
    parser.add_argument('--force',       action='store_true')
    parser.add_argument('--push-to-hub', action='store_true')
    args = parser.parse_args()

    vtam_root = Path(__file__).resolve().parents[2]
    args.task          = args.demo_name
    args.processed_dir = vtam_root / 'data' / 'processed' / args.demo_name
    args.lerobot_dir   = vtam_root / 'data' / 'lerobot'   / args.demo_name
    args.repo_id       = f"andnetdeboer/vtam_{args.demo_name}"

    main(args)