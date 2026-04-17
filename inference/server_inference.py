#!/usr/bin/env python3
"""
vtam_server_inference.py

Pipeline:
  1. Recv {rgb, state} from robot (4406)
  2. policy.select_action() - (1, 8) — policy manages chunk queue internally
  3. Send (8,) action to robot (4405) every step

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

# ── Path setup
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
LEROBOT    = os.path.join(VTAM_ROOT, 'dependencies', 'lerobot')
sys.path.insert(0, LEROBOT)

from lerobot.common.policies.act.modeling_act import ACTPolicy

# ── Config
CONFIG_PATH = os.path.join(VTAM_ROOT, 'config', 'inference.yaml')
with open(CONFIG_PATH) as f:
    CFG = yaml.safe_load(f)

ROBOT_IP    = CFG['robot_ip']
PORTS       = CFG['ports']
POLICY_PATH = CFG['policy_path']
DEVICE      = CFG.get('device', 'cuda:1')

OBS_PORT   = PORTS['chunk'] + 1   # 4406 — recv obs from robot
CHUNK_PORT = PORTS['chunk']        # 4405 — send action to robot

# ImageNet normalization 
IMG_MEAN = torch.tensor([0.485, 0.456, 0.406], dtype=torch.float32)
IMG_STD  = torch.tensor([0.229, 0.224, 0.225], dtype=torch.float32)
IMG_SIZE = (320, 320)

def preprocess_rgb(rgb_bytes: bytes, device: str) -> torch.Tensor:
    arr = np.frombuffer(rgb_bytes, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]
    s = min(h, w)
    y, x = (h - s) // 2, (w - s) // 2
    img = img[y:y+s, x:x+s]
    img = cv2.resize(img, IMG_SIZE)
    t   = torch.from_numpy(img).float() / 255.0
    t   = (t - IMG_MEAN) / IMG_STD
    t   = t.permute(2, 0, 1).unsqueeze(0)   # (1, 3, 320, 320)
    return t.to(device)

def preprocess_state(state_8d: np.ndarray, device: str) -> torch.Tensor:
    return torch.from_numpy(state_8d).float().unsqueeze(0).to(device)  # (1, 8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy-path', type=str, default=POLICY_PATH)
    parser.add_argument('--device',      type=str, default=DEVICE)
    args = parser.parse_args()

    print(f"\n{'='*55}")
    print(f"VTAM Server Inference")
    print(f"  Robot:     {ROBOT_IP}")
    print(f"  obs←{OBS_PORT}  action→{CHUNK_PORT}")
    print(f"  Device:    {args.device}")
    print(f"{'='*55}\n")

    print(f"Loading policy from {args.policy_path}...")
    policy = ACTPolicy.from_pretrained(args.policy_path, local_files_only=True)
    policy.to(args.device)
    policy.eval()

    print(f"  Loaded. chunk_size={policy.config.chunk_size}\n")

    ctx = zmq.Context()

    obs_sock = ctx.socket(zmq.SUB)
    obs_sock.setsockopt(zmq.SUBSCRIBE, b"")
    obs_sock.setsockopt(zmq.CONFLATE, 1)
    obs_sock.setsockopt(zmq.RCVTIMEO, 5000)
    obs_sock.bind(f"tcp://*:{OBS_PORT}")
    
    action_sock = ctx.socket(zmq.PUB)
    action_sock.bind(f"tcp://*:{CHUNK_PORT}")

    time.sleep(0.5)
    print("Sockets ready. Waiting for observations...\n")

    step        = 0
    t_inference = []

    try:
        while True:

            try:
                raw = obs_sock.recv()
                # Drain any buffered messages, keep only the last
                while True:
                    try:
                        raw = obs_sock.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                obs = pickle.loads(raw)
            except zmq.Again:
                print("WARNING: timeout waiting for obs")
                continue

            # Preprocess ─────────────────────────────────────────────
            batch = {
                "observation.images.gripper": preprocess_rgb(obs["rgb"], args.device),
                "observation.state":          preprocess_state(obs["state"], args.device),
            }

            # Run policy — returns (1, 8), queue managed internally ──
            t0 = time.time()
            with torch.inference_mode():
                action = policy.select_action(batch)
            dt_inf = time.time() - t0
            t_inference.append(dt_inf)

            # Send (8,) to robot ─────────────────────────────────────
            action_np = action.cpu().numpy()[0].astype(np.float32)  # (8,)
            action_sock.send(pickle.dumps(action_np), flags=zmq.NOBLOCK)

            if step % 10 == 0:
                mean_inf = np.mean(t_inference[-20:]) * 1000
                print(f"  step={step} inf={dt_inf*1000:.1f}ms "
                      f"mean={mean_inf:.1f}ms action={action_np[:3]}")

            step += 1

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        obs_sock.close()
        action_sock.close()
        ctx.term()


if __name__ == '__main__':
    main()