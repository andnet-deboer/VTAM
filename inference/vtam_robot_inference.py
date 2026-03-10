#!/usr/bin/env python3
"""
vtam_robot_inference.py — replay_demo.py with ZMQ instead of MCAP.

Sheep sends (8,) chunk-relative EE actions on port 4405.
This script applies PD2.1 + IK and sends 9D joints to vtam_robot_node.
Everything else is verbatim from replay_demo.py.
"""

import os, sys, time, pickle, argparse
import numpy as np
import zmq
import yaml
from scipy.spatial.transform import Rotation

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
sys.path.insert(0, VTAM_ROOT)
sys.path.insert(0, os.path.dirname(VTAM_ROOT))

from training.scripts.replay_demo import (
    OnlineIK, send_action, get_robot_state, normalized_to_hardware,
    DEFAULT_FPS, CHUNK_SIZE, ZMQ_ACTION_PORT, ZMQ_STATE_PORT,
    WARN_POS_ERROR_M, WARN_ORI_ERROR_RAD
)

CONFIG_PATH = os.path.join(VTAM_ROOT, 'config', 'inference.yaml')
with open(CONFIG_PATH) as f:
    CFG = yaml.safe_load(f)

SERVER_IP  = CFG['server_ip']
FWD_PORT   = 4406   # send obs to sheep
CHUNK_PORT = CFG['ports']['chunk']  # 4405 recv action from sheep
OBS_PORT   = CFG['ports']['obs']    # 4401 recv rgb from vtam_robot_node
CHUNK_SIZE = 15


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--fps', type=float, default=DEFAULT_FPS)
    args = parser.parse_args()

    dt = 1.0 / args.fps

    ik  = OnlineIK()
    ctx = zmq.Context()

    # recv obs from vtam_robot_node
    obs_sock = ctx.socket(zmq.SUB)
    obs_sock.setsockopt(zmq.SUBSCRIBE, b"")
    obs_sock.setsockopt(zmq.CONFLATE, 1)
    obs_sock.connect(f"tcp://localhost:{OBS_PORT}")

    # forward obs to sheep
    fwd_sock = ctx.socket(zmq.PUB)
    fwd_sock.bind(f"tcp://*:{FWD_PORT}")

    # recv chunk-relative action from sheep
    action_in = ctx.socket(zmq.SUB)
    action_in.setsockopt(zmq.SUBSCRIBE, b"")
    action_in.setsockopt(zmq.CONFLATE, 1)
    action_in.bind(f"tcp://*:{CHUNK_PORT}")

    # send joints to vtam_robot_node
    action_sock = ctx.socket(zmq.PUB)
    action_sock.connect(f"tcp://localhost:{ZMQ_ACTION_PORT}")
    time.sleep(0.5)

    # seed IK — identical to replay_demo
    state = get_robot_state(ctx)
    ik.seed_from_robot_state(state)

    print("Connected. Starting in 3 seconds...")
    time.sleep(3.0)

    step = 0
    T_t0_robot = np.eye(4)

    # ── Main loop — identical structure to replay_demo ─────────────────────
    while True:
        t_start = time.time()

        # forward obs to sheep
        try:
            obs = pickle.loads(obs_sock.recv(flags=zmq.NOBLOCK))
            state = get_robot_state(ctx)
            ik.seed_from_robot_state(state, silent=True)
            cur_pos, cur_rot = ik.forward_kinematics(ik.q_current)
            state_8d = np.concatenate([
                cur_pos,
                Rotation.from_matrix(cur_rot).as_quat(),
                [float(state.get("joint_positions", [0]*6)[5])]
            ]).astype(np.float32)
            fwd_sock.send(pickle.dumps({"rgb": obs["rgb"], "state": state_8d}),
                          flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

        # re-seed once per chunk — identical to replay_demo PD2.1
        if step % CHUNK_SIZE == 0:
            state = get_robot_state(ctx)
            ik.seed_from_robot_state(state, silent=True)
            cur_pos, cur_rot   = ik.forward_kinematics(ik.q_current)
            T_t0_robot         = np.eye(4)
            T_t0_robot[:3, :3] = cur_rot
            T_t0_robot[:3,  3] = cur_pos

        # recv action from sheep
        try:
            action_8d = pickle.loads(action_in.recv(flags=zmq.NOBLOCK))
        except zmq.Again:
            step += 1
            time.sleep(max(0, dt - (time.time() - t_start)))
            continue

        # PD2.1 — verbatim from replay_demo inner loop
        T_chunk_rel        = np.eye(4)
        T_chunk_rel[:3,:3] = Rotation.from_quat(action_8d[3:7]).as_matrix()
        T_chunk_rel[:3, 3] = action_8d[:3]
        T_target    = T_t0_robot @ T_chunk_rel
        target_pos  = T_target[:3, 3]
        target_quat = Rotation.from_matrix(T_target[:3,:3]).as_quat()

        q_prev = ik.q_current.copy()
        q_new, pos_err, ori_err = ik.step_ik(ik.q_current, target_pos, target_quat)
        ik.q_current = q_new

        action_9d = ik.joints_to_action(q_new, float(action_8d[7]), q_prev, dt)
        send_action(action_sock, action_9d, dry_run=args.dry_run)

        if step % 10 == 0:
            status = "OK"
            if pos_err > WARN_POS_ERROR_M:
                status = f"WARN pos={pos_err*1000:.0f}mm"
            print(f"step={step} pos_err={pos_err*1000:.1f}mm "
                  f"ori_err={np.degrees(ori_err):.1f}deg {status}")

        step += 1
        time.sleep(max(0, dt - (time.time() - t_start)))


if __name__ == '__main__':
    main()