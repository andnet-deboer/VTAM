#!/usr/bin/env python3
"""
vtam_robot_inference.py — runs on robot (stretch-se3-3047)

Pipeline:
  1. Recv RGB + state from vtam_robot_node (4401, 4403)
  2. FK → real 8D EE state
  3. Forward {rgb, state} to sheep inference server (4406)
  4. Recv action chunk (15, 8) from sheep (4405)
  5. PD2.1 + IK → 9D joints → vtam_robot_node (4402)

Usage:
    python3 vtam_robot_inference.py
    python3 vtam_robot_inference.py --dry-run
"""

import os
import sys
import time
import pickle
import argparse
import numpy as np
import zmq
import yaml
from scipy.spatial.transform import Rotation

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
sys.path.insert(0, VTAM_ROOT)
sys.path.insert(0, os.path.dirname(VTAM_ROOT))

from training.scripts.replay_demo import OnlineIK, normalized_to_hardware

# ── Config ────────────────────────────────────────────────────────────────────
CONFIG_PATH = os.path.join(VTAM_ROOT, 'config', 'inference.yaml')
with open(CONFIG_PATH) as f:
    CFG = yaml.safe_load(f)

PORTS      = CFG['ports']
FPS        = CFG['fps']
CHUNK_SIZE = CFG['chunk_size']
DT         = 1.0 / FPS

OBS_PORT    = PORTS['obs']       # 4401 — recv from vtam_robot_node
STATE_PORT  = PORTS['state']     # 4403 — recv from vtam_robot_node
ACTION_PORT = PORTS['action']    # 4402 — send to vtam_robot_node
CHUNK_PORT  = PORTS['chunk']     # 4405 — recv chunk from sheep
FWD_PORT    = CHUNK_PORT + 1     # 4406 — send obs to sheep


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    print(f"\n{'='*55}")
    print(f"VTAM Robot Inference Node")
    print(f"  obs←{OBS_PORT} state←{STATE_PORT} action→{ACTION_PORT}")
    print(f"  obs→{FWD_PORT}(sheep)  chunk←{CHUNK_PORT}(sheep)")
    print(f"  FPS={FPS}  chunk_size={CHUNK_SIZE}  dry_run={args.dry_run}")
    print(f"{'='*55}\n")

    ik = OnlineIK()

    ctx = zmq.Context()

    # Recv obs from vtam_robot_node
    obs_sock = ctx.socket(zmq.SUB)
    obs_sock.setsockopt(zmq.SUBSCRIBE, b"")
    obs_sock.setsockopt(zmq.CONFLATE, 1)
    obs_sock.setsockopt(zmq.RCVTIMEO, 3000)
    obs_sock.connect(f"tcp://localhost:{OBS_PORT}")

    # Recv state from vtam_robot_node
    state_sock = ctx.socket(zmq.SUB)
    state_sock.setsockopt(zmq.SUBSCRIBE, b"")
    state_sock.setsockopt(zmq.CONFLATE, 1)
    state_sock.setsockopt(zmq.RCVTIMEO, 3000)
    state_sock.connect(f"tcp://localhost:{STATE_PORT}")

    # Forward obs to sheep
    fwd_sock = ctx.socket(zmq.PUB)
    fwd_sock.bind(f"tcp://*:{FWD_PORT}")

    # Recv chunk from sheep
    chunk_sock = ctx.socket(zmq.SUB)
    chunk_sock.setsockopt(zmq.SUBSCRIBE, b"")
    chunk_sock.setsockopt(zmq.CONFLATE, 1)
    chunk_sock.setsockopt(zmq.RCVTIMEO, 5000)
    chunk_sock.bind(f"tcp://*:{CHUNK_PORT}")

    # Send joints to vtam_robot_node
    action_sock = ctx.socket(zmq.PUB)
    action_sock.connect(f"tcp://localhost:{ACTION_PORT}")

    time.sleep(0.5)
    print("Sockets ready. Waiting for first obs...")

    step       = 0
    chunk      = None
    chunk_step = 0
    T_t0_robot = np.eye(4)

    try:
        while True:
            t_start = time.time()

            # ── 1. Recv obs + state ───────────────────────────────────────
            try:
                obs   = pickle.loads(obs_sock.recv())
                state = pickle.loads(state_sock.recv())
            except zmq.Again:
                print("WARNING: timeout waiting for obs/state")
                continue

            # ── 2. FK → real 8D EE state ──────────────────────────────────
            ik.seed_from_robot_state(state, silent=True)
            cur_pos, cur_rot = ik.forward_kinematics(ik.q_current)
            quat = Rotation.from_matrix(cur_rot).as_quat()
            jp   = state.get("joint_positions", [])
            grip = float(jp[5]) if len(jp) >= 6 else 0.0
            state_8d = np.concatenate([cur_pos, quat, [grip]]).astype(np.float32)

            # ── 3. Forward obs to sheep ───────────────────────────────────
            try:
                fwd_sock.send(
                    pickle.dumps({"rgb": obs["rgb"], "state": state_8d, "step": step}),
                    flags=zmq.NOBLOCK)
            except zmq.ZMQError:
                pass

            # ── 4. Check for new chunk from sheep ─────────────────────────
            try:
                chunk      = pickle.loads(chunk_sock.recv(flags=zmq.NOBLOCK))
                chunk_step = 0

                # Re-seed and capture T_t0_robot at chunk start
                ik.seed_from_robot_state(state, silent=True)
                cur_pos, cur_rot   = ik.forward_kinematics(ik.q_current)
                T_t0_robot         = np.eye(4)
                T_t0_robot[:3, :3] = cur_rot
                T_t0_robot[:3,  3] = cur_pos
                print(f"  New chunk at step={step}")

            except zmq.Again:
                pass

            # ── 5. Execute current chunk step ─────────────────────────────
            if chunk is not None and chunk_step < CHUNK_SIZE:
                rel       = chunk[chunk_step]
                pos_rel   = rel[:3]
                quat_rel  = rel[3:7]
                grip_norm = float(rel[7])

                # PD2.1: T_target = T_t0_robot @ T_chunk_rel
                T_rel          = np.eye(4)
                T_rel[:3, :3]  = Rotation.from_quat(quat_rel).as_matrix()
                T_rel[:3,  3]  = pos_rel
                T_target       = T_t0_robot @ T_rel
                target_pos     = T_target[:3, 3]
                target_quat    = Rotation.from_matrix(T_target[:3, :3]).as_quat()

                q_prev = ik.q_current.copy()
                q_new, pos_err, ori_err = ik.step_ik(
                    ik.q_current, target_pos, target_quat)
                ik.q_current = q_new

                action_9d = ik.joints_to_action(q_new, grip_norm, q_prev, DT)

                if step % 10 == 0:
                    print(f"  step={step} chunk_step={chunk_step} "
                          f"pos_err={pos_err*1000:.1f}mm "
                          f"ori_err={np.degrees(ori_err):.1f}deg")

                if not args.dry_run:
                    action_sock.send(
                        pickle.dumps({"joint": action_9d.tolist()}),
                        flags=zmq.NOBLOCK)

                chunk_step += 1

            step += 1
            sleep_t = DT - (time.time() - t_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for s in [obs_sock, state_sock, fwd_sock, chunk_sock, action_sock]:
            s.close()
        ctx.term()


if __name__ == '__main__':
    main()