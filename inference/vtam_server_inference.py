#!/usr/bin/env python3
"""
vtam_server_inference.py — runs on sheep (kmy2091@sheep)

Pipeline:
  1. Recv {rgb, state} from robot (4406)
  2. Preprocess → run ACT policy
  3. Send action chunk (15, 8) to robot (4405)

Usage:
    python3 vtam_server_inference.py
    python3 vtam_server_inference.py --policy-path /path/to/pretrained_model
"""

import os
import sys
import time
import pickle
import argparse
import numpy as np
import torch
import cv2
import zmq
import yaml
from pathlib import Path

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
LEROBOT    = os.path.join(VTAM_ROOT, 'dependencies', 'lerobot')
sys.path.insert(0, LEROBOT)

from lerobot.common.policies.act.modeling_act import ACTPolicy

# ── Config ────────────────────────────────────────────────────────────────────
CONFIG_PATH = os.path.join(VTAM_ROOT, 'config', 'inference.yaml')

with open(CONFIG_PATH) as f:
    CFG = yaml.safe_load(f)

ROBOT_IP     = CFG['robot_ip']
PORTS        = CFG['ports']
POLICY_PATH  = CFG['policy_path']
DEVICE       = CFG.get('device', 'cuda:1')
CHUNK_SIZE   = CFG['chunk_size']

# ImageNet normalization (matches training config)
IMG_MEAN = torch.tensor([0.485, 0.456, 0.406], dtype=torch.float32)
IMG_STD  = torch.tensor([0.229, 0.224, 0.225], dtype=torch.float32)
IMG_SIZE = (320, 320)


def load_policy(policy_path: str, device: str) -> ACTPolicy:
    print(f"Loading policy from {policy_path}...")
    policy = ACTPolicy.from_pretrained(policy_path)
    policy.to(device)
    policy.eval()
    print(f"  Loaded. chunk_size={policy.config.chunk_size} "
          f"n_action_steps={policy.config.n_action_steps}")
    return policy


def preprocess_rgb(rgb_bytes: bytes, device: str) -> torch.Tensor:
    """Decode JPEG → (1, 3, 320, 320) float32 tensor normalized for ResNet."""
    arr = np.frombuffer(rgb_bytes, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)          # BGR uint8
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, IMG_SIZE)
    t   = torch.from_numpy(img).float() / 255.0        # (320,320,3) [0,1]
    t   = (t - IMG_MEAN) / IMG_STD                     # normalize
    t   = t.permute(2, 0, 1).unsqueeze(0)              # (1,3,320,320)
    return t.to(device)


def preprocess_state(state_8d: np.ndarray, device: str) -> torch.Tensor:
    """(8,) float32 → (1, 8) tensor."""
    return torch.from_numpy(state_8d).float().unsqueeze(0).to(device)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy-path', type=str, default=POLICY_PATH)
    parser.add_argument('--device', type=str, default=DEVICE)
    args = parser.parse_args()

    print(f"\n{'='*55}")
    print(f"VTAM Server Inference")
    print(f"  Robot:      {ROBOT_IP}")
    print(f"  Obs port:   {PORTS['chunk'] + 1}  Chunk port: {PORTS['chunk']}")
    print(f"  Device:     {args.device}")
    print(f"  chunk_size: {CHUNK_SIZE}")
    print(f"{'='*55}\n")

    policy = load_policy(args.policy_path, args.device)

    ctx = zmq.Context()

    # Recv obs from robot
    obs_sock = ctx.socket(zmq.SUB)
    obs_sock.setsockopt(zmq.SUBSCRIBE, b"")
    obs_sock.setsockopt(zmq.CONFLATE, 1)
    obs_sock.setsockopt(zmq.RCVTIMEO, 5000)
    obs_sock.connect(f"tcp://{ROBOT_IP}:{PORTS['chunk'] + 1}")  # 4406

    # Send chunk to robot
    chunk_sock = ctx.socket(zmq.PUB)
    chunk_sock.connect(f"tcp://{ROBOT_IP}:{PORTS['chunk']}")    # 4405

    time.sleep(0.5)
    print("Sockets ready. Waiting for observations...\n")

    step        = 0
    t_inference = []

    try:
        while True:
            # ── 1. Recv obs ───────────────────────────────────────────────
            try:
                obs = pickle.loads(obs_sock.recv())
            except zmq.Again:
                print("WARNING: timeout waiting for obs from robot")
                continue

            # ── 2. Preprocess ─────────────────────────────────────────────
            img   = preprocess_rgb(obs["rgb"], args.device)
            state = preprocess_state(obs["state"], args.device)

            batch = {
                "observation.images.gripper": img,
                "observation.state":          state,
            }

            # ── 3. Run policy ─────────────────────────────────────────────
            t0 = time.time()
            with torch.inference_mode():
                action = policy.select_action(batch)  # (1, 15, 8) or (1, 8)
            dt_inf = time.time() - t0
            t_inference.append(dt_inf)

            # ── 4. Extract chunk (15, 8) ──────────────────────────────────
            action_np = action.cpu().numpy()
            if action_np.ndim == 3:
                chunk = action_np[0]   # (15, 8)
            elif action_np.ndim == 2:
                chunk = action_np      # (15, 8) already
            else:
                chunk = action_np.reshape(CHUNK_SIZE, -1)

            # ── 5. Send chunk to robot ────────────────────────────────────
            chunk_sock.send(pickle.dumps(chunk.astype(np.float32)),
                            flags=zmq.NOBLOCK)

            if step % 10 == 0:
                mean_inf = np.mean(t_inference[-20:]) * 1000
                print(f"  step={step} inference={dt_inf*1000:.1f}ms "
                      f"mean={mean_inf:.1f}ms "
                      f"chunk[0]={chunk[0,:3]}")

            step += 1

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        obs_sock.close()
        chunk_sock.close()
        ctx.term()


if __name__ == '__main__':
    main()