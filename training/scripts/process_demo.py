#!/usr/bin/env python3
import argparse
import glob
import json
import os
import shutil
import sys
from pathlib import Path
import shutil
import cv2
import numpy as np
import torch
from datasets import Dataset, Features, Image, Sequence, Value
from lerobot.common.datasets.video_utils import encode_video_frames
from PIL import Image as PILImage
from safetensors.torch import save_file
from scipy.spatial.transform import Rotation
from tqdm import tqdm
import yaml
from mcap.reader import make_reader
from rosbags.typesys import Stores, get_typestore
_typestore = get_typestore(Stores.ROS2_HUMBLE)

# ── Path setup ─────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT   = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
UTILS_DIR   = os.path.join(SCRIPT_DIR, '..', 'utils')
LEROBOT_DIR = os.path.join(VTAM_ROOT, 'dependencies', 'lerobot')
CONFIG_PATH = os.path.expanduser("~/VTAM/src/vtam_core/config/record.yaml")

sys.path.insert(0, UTILS_DIR)
sys.path.insert(0, LEROBOT_DIR)

from mcap_ros2.reader import read_ros2_messages
from lerobot.common.datasets.utils import hf_transform_to_torch, flatten_dict
from lerobot.common.datasets.compute_stats import compute_stats
from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset

# ── Configuration ──────────────────────────────────────────────────────────────
IMAGE_SIZE    = (320, 320)
MIN_FRAMES    = 30
ACTION_DIM    = 8   # [dx, dy, dz, dqx, dqy, dqz, dqw, gripper]
STATE_DIM     = 8   # [x, y, z, qx, qy, qz, qw, gripper] (RAW Robot Base Frame)
TACTILE_DIM   = 30  # 15 Left + 15 Right

TF_CHAIN = [("base_link", "umi_disconnect"), ("umi_disconnect", "umi_gripper")]
ACTION_ORDER = ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]
STATE_ORDER  = ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]

# ── TF & Topic helpers ────────────────────────────────────────────────────────
def load_topics(config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['/**']['ros__parameters']['session_topics']

SESSION_TOPICS = load_topics(CONFIG_PATH)
def find_topic(kw): return next((t for t in SESSION_TOPICS if kw in t), None)

SYNC_TOPIC     = find_topic('sync_pulse')
IMAGE_TOPIC    = find_topic('camera_arm/color/image_rect_raw')
GRIPPER_TOPIC  = find_topic('gripper_width')
TACTILE_LEFT   = find_topic('tactile_left')
TACTILE_RIGHT  = find_topic('tactile_right')

def tf_to_matrix(pose7):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(pose7[3:]).as_matrix()
    T[:3, 3]  = pose7[:3]
    return T

def extract_ee_pose(tf_data, tf_ts):
    p1 = TF_CHAIN[0]
    if not tf_data[p1]: return [], []
    ts_arrs = {p: np.array(tf_ts[p]) for p in TF_CHAIN}
    ee_p, ee_t = [], []
    for i in range(len(tf_data[p1])):
        t, T = tf_ts[p1][i], tf_to_matrix(tf_data[p1][i])
        for pair in TF_CHAIN[1:]:
            idx = np.clip(np.searchsorted(ts_arrs[pair], t)-1, 0, len(tf_data[pair])-1)
            T = T @ tf_to_matrix(tf_data[pair][idx])
        ee_p.append(np.concatenate([T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_quat()]))
        ee_t.append(t)
    return ee_p, ee_t

# ── Per-episode processing ─────────────────────────────────────────────────────
def process_episode(mcap_path, use_tactile=False):
    sync_ts, img_buf, img_ts, grip_buf, grip_ts = [], [], [], [], []
    tf_data = {p: [] for p in TF_CHAIN}; tf_ts = {p: [] for p in TF_CHAIN}
    tl_buf, tl_ts, tr_buf, tr_ts = [], [], [], []

    with open(mcap_path, 'rb') as f:
        r = make_reader(f)
        for _, channel, message in r.iter_messages():
            if channel.topic == SYNC_TOPIC:
                sync_ts.append(message.publish_time / 1e9)
            elif channel.topic == '/tf':
                t = message.publish_time / 1e9
                tf_msg = _typestore.deserialize_cdr(message.data, 'tf2_msgs/msg/TFMessage')
                for tf in tf_msg.transforms:
                    pair = (tf.header.frame_id, tf.child_frame_id)
                    if pair in tf_data:
                        tr, ro = tf.transform.translation, tf.transform.rotation
                        tf_data[pair].append(np.array([tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w]))
                        tf_ts[pair].append(t)

    other_topics = [t for t in [IMAGE_TOPIC, GRIPPER_TOPIC, TACTILE_LEFT, TACTILE_RIGHT] if t]
    for msg in read_ros2_messages(mcap_path, topics=other_topics):
        t, topic = msg.publish_time_ns / 1e9, msg.channel.topic
        if topic == IMAGE_TOPIC:
            img = cv2.cvtColor(cv2.imdecode(np.frombuffer(msg.ros_msg.data, np.uint8), 1), cv2.COLOR_BGR2RGB)
            h, w = img.shape[:2]; s = min(h, w); y, x = (h-s)//2, (w-s)//2
            img_buf.append(cv2.resize(img[y:y+s, x:x+s], IMAGE_SIZE).astype(np.uint8)); img_ts.append(t)
        elif topic == GRIPPER_TOPIC: grip_buf.append(float(msg.ros_msg.data)); grip_ts.append(t)
        elif topic == TACTILE_LEFT: tl_buf.append(np.array(msg.ros_msg.data, np.float32)); tl_ts.append(t)
        elif topic == TACTILE_RIGHT: tr_buf.append(np.array(msg.ros_msg.data, np.float32)); tr_ts.append(t)

    if len(sync_ts) < MIN_FRAMES or not tf_data[TF_CHAIN[0]]: return None
    sync_arr = np.array(sync_ts)
    def snap(buf, ts):
        if not buf: return [np.zeros_like(buf[0]) if buf else 0] * len(sync_arr)
        idx = np.clip(np.searchsorted(np.array(ts), sync_arr)-1, 0, len(buf)-1)
        return [buf[i] for i in idx]

    ee_raw, ee_t = extract_ee_pose(tf_data, tf_ts)
    ee_synced, imgs, grips = snap(ee_raw, ee_t), snap(img_buf, img_ts), snap(grip_buf, grip_ts)

    actions_rel = [np.array(p, dtype=np.float32) for p in ee_synced]

    N = len(sync_ts)
    ep = {
        'images': [PILImage.fromarray(img) for img in imgs],  # PIL images, not numpy
        'action': np.concatenate([np.array(actions_rel, np.float32), np.array(grips, np.float32)[:, None]], axis=1),
        'state': np.concatenate([np.array(ee_synced, np.float32), np.array(grips, np.float32)[:, None]], axis=1),
        'T': N
    }

    if use_tactile:
        tls, trs = snap(tl_buf, tl_ts), snap(tr_buf, tr_ts)
        ep['tactile'] = np.concatenate([np.array(tls), np.array(trs)], axis=1).astype(np.float32)
    return ep

# ── Main ───────────────────────────────────────────────────────────────────────
def main(args):
    import gc
    from datasets import concatenate_datasets, load_from_disk

    lerobot_dir = Path(args.lerobot_dir); meta_dir = lerobot_dir/'meta_data'; train_dir = lerobot_dir/'train'
    if lerobot_dir.exists():
        if args.force: shutil.rmtree(lerobot_dir)
        else: print("Dir exists, use --force"); sys.exit(1)

    mcap_paths = sorted(glob.glob(str(args.processed_dir/'*.mcap')))

    # Use datasets.Image()
    feat_dict = {
        'observation.images.gripper': Image(),
        'observation.state': Sequence(length=STATE_DIM, feature=Value('float32')),
        'action': Sequence(length=ACTION_DIM, feature=Value('float32')),
        'episode_index': Value('int64'), 'frame_index': Value('int64'),
        'timestamp': Value('float32'), 'next.done': Value('bool'), 'index': Value('int64'),
    }
    if args.tactile: feat_dict['observation.tactile'] = Sequence(length=TACTILE_DIM, feature=Value('float32'))
    features = Features(feat_dict)

    shard_dir = lerobot_dir/'_shards'; shard_dir.mkdir(parents=True, exist_ok=True)
    episode_data_index = {'from': [], 'to': []}; shard_paths = []; cursor = 0

    for ep_idx, path in enumerate(tqdm(mcap_paths, desc='Processing')):
        ep = process_episode(path, use_tactile=args.tactile)
        if ep is None: continue
        T = ep['T']; done = torch.zeros(T, dtype=torch.bool); done[-1] = True

        ep_dict = {
            'observation.images.gripper': ep['images'],  # list of PIL Images
            'observation.state': torch.from_numpy(ep['state']),
            'action': torch.from_numpy(ep['action']),
            'episode_index': torch.tensor([ep_idx]*T), 'frame_index': torch.arange(T),
            'timestamp': torch.arange(T, dtype=torch.float32)/args.fps,
            'next.done': done, 'index': torch.arange(cursor, cursor+T)
        }
        if args.tactile: ep_dict['observation.tactile'] = torch.from_numpy(ep['tactile'])
        Dataset.from_dict(ep_dict, features=features).save_to_disk(str(shard_dir/f'ep_{ep_idx:04d}'))
        shard_paths.append(str(shard_dir/f'ep_{ep_idx:04d}'))
        episode_data_index['from'].append(cursor); episode_data_index['to'].append(cursor+T); cursor += T
        gc.collect()

    hf_dataset = concatenate_datasets([load_from_disk(p) for p in shard_paths])

    # video=False: LeRobot will load images directly from the HF dataset, no decode step
    info = {
        'codebase_version': CODEBASE_VERSION,
        'fps': args.fps,
        'video': False,
        'action_order': ACTION_ORDER,
        'state_order': STATE_ORDER,
        'image_size': {'gripper': list(IMAGE_SIZE)},
        'num_episodes': len(shard_paths),
        'num_frames': cursor,
        'task': args.task
    }

    # Stats — action only (images don't need computed stats, ImageNet values injected below)
    chunk_size = 15
    delta_ts = {'action': [i / args.fps for i in range(chunk_size)]}
    episode_data_index_t = {k: torch.tensor(v) for k, v in episode_data_index.items()}
    stats = compute_stats(LeRobotDataset.from_preloaded(
        repo_id=args.repo_id,
        hf_dataset=hf_dataset.with_transform(hf_transform_to_torch),
        episode_data_index=episode_data_index_t,
        info=info,
        videos_dir=lerobot_dir/'videos',
        delta_timestamps=delta_ts,
    ))

    # Inject ImageNet normalization stats for gripper image key.
    # factory.py override_dataset_stats requires these keys to exist.
    stats['observation.images.gripper'] = {
        'mean': torch.tensor([[[0.485]], [[0.456]], [[0.406]]]),
        'std':  torch.tensor([[[0.229]], [[0.224]], [[0.225]]]),
        'min':  torch.tensor([[[0.0]],  [[0.0]],  [[0.0]]]),
        'max':  torch.tensor([[[1.0]],  [[1.0]],  [[1.0]]]),
    }

    # CLEANUP shards
    if shard_dir.exists():
        shutil.rmtree(str(shard_dir))

    # Export to Parquet
    data_dir = lerobot_dir / 'data'
    data_dir.mkdir(parents=True, exist_ok=True)
    hf_dataset.to_parquet(str(data_dir / 'train-00000-of-00001.parquet'))
    meta_dir.mkdir(parents=True, exist_ok=True)
    with open(meta_dir/'info.json', 'w') as f: json.dump(info, f, indent=4)
    save_file({k: torch.tensor(v) for k, v in episode_data_index.items()}, meta_dir/'episode_data_index.safetensors')
    save_file(flatten_dict(stats), meta_dir/'stats.safetensors')

    if args.push_to_hub:
        from huggingface_hub import HfApi
        api = HfApi(); api.upload_large_folder(folder_path=str(lerobot_dir), repo_id=args.repo_id, repo_type="dataset")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('demo_name', type=str)
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--push-to-hub', action='store_true')
    parser.add_argument('--tactile', action='store_true')
    parser.add_argument('--repo-id', type=str, default=None)
    args = parser.parse_args()
    vtam_root = Path(__file__).resolve().parents[2]
    args.task = args.demo_name
    args.processed_dir = vtam_root/'data'/'processed'/args.demo_name
    args.lerobot_dir = vtam_root/'data'/'lerobot'/args.demo_name
    args.repo_id = args.repo_id or f"andnetdeboer/vtam_{args.demo_name}"
    main(args)