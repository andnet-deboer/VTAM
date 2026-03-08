#!/usr/bin/env python3
"""
replay_demo.py — Replay a recorded MCAP demonstration on the Stretch robot.

PURPOSE:
    Validate the full inference pipeline (TF extraction → IK → ZMQ execution)
    BEFORE training any policy. If the robot faithfully replays a demonstration,
    the IK and action convention are correct. If not, fix them here first.

PIPELINE:
    1. Extract absolute EE poses from /tf in the MCAP
    2. Compute T_rel[t] = inv(T_t) @ T_{t+1} for each frame pair
    3. Each frame:
        a. Read actual robot joint state from ZMQ port 4403
        b. FK to get current EE pose in base_link
        c. Apply T_rel in current EE local frame: T_target = T_current @ T_rel
        d. IK solve to T_target → joint commands
        e. Send to robot via ZMQ port 4402

USAGE:
    python3 replay_demo.py place_coffee_cup/episode_000.mcap --dry-run
    python3 replay_demo.py place_coffee_cup/episode_000.mcap
    python3 replay_demo.py place_coffee_cup/episode_000.mcap --fps 5

RUN ON:
    Robot (stretch-se3-3047) — needs ZMQ connection to vtam_robot_node.py
"""

import argparse
import os
import sys
import time
import pickle
import numpy as np
import zmq
from pathlib import Path
from scipy.spatial.transform import Rotation

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
UTILS_DIR  = os.path.join(VTAM_ROOT, 'training', 'utils')

sys.path.insert(0, VTAM_ROOT)
sys.path.insert(0, os.path.dirname(VTAM_ROOT))
sys.path.insert(0, UTILS_DIR)
from training.utils.differential_ik import TrajectoryRetargeter

# ── Config ────────────────────────────────────────────────────────────────────
ROBOT_IP           = "localhost"
ZMQ_STATE_PORT     = 4403
ZMQ_ACTION_PORT    = 4402
DEFAULT_FPS        = 10

GRIPPER_OPEN       = 1.1
GRIPPER_CLOSED     = -0.5
GRIPPER_TRAVEL     = GRIPPER_OPEN - GRIPPER_CLOSED  # 1.6

TF_CHAIN = [("base_link", "umi_disconnect"), ("umi_disconnect", "umi_gripper")]

WARN_POS_ERROR_M   = 0.03
WARN_ORI_ERROR_RAD = 0.2
ZMQ_TIMEOUT_MS     = 3000


def normalized_to_hardware(normalized: float) -> float:
    return GRIPPER_OPEN - (normalized * GRIPPER_TRAVEL)


def get_robot_state(ctx: zmq.Context, timeout_ms: int = ZMQ_TIMEOUT_MS) -> dict:
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.connect(f"tcp://{ROBOT_IP}:{ZMQ_STATE_PORT}")
    try:
        raw = sock.recv()
        return pickle.loads(raw)
    except zmq.Again:
        raise RuntimeError(
            f"Timeout waiting for robot state on port {ZMQ_STATE_PORT}. "
            "Is vtam_robot_node.py running?"
        )
    finally:
        sock.close()


def send_action(sock: zmq.Socket, targets_9d: np.ndarray, dry_run: bool = False):
    if dry_run:
        return
    msg = {"joint": targets_9d.tolist()}
    sock.send(pickle.dumps(msg), flags=zmq.NOBLOCK)


def extract_episode(mcap_path: str):
    from mcap_ros2.reader import read_ros2_messages

    sync_ts  = []
    tf_data  = {p: [] for p in TF_CHAIN}
    tf_ts    = {p: [] for p in TF_CHAIN}
    grip_buf = []
    grip_ts  = []

    for msg in read_ros2_messages(mcap_path, topics=["/sync_pulse", "/tf", "/gripper_width_normalized"]):
        t     = msg.publish_time_ns / 1e9
        topic = msg.channel.topic

        if topic == "/sync_pulse":
            sync_ts.append(
                msg.ros_msg.header.stamp.sec + msg.ros_msg.header.stamp.nanosec / 1e9
            )
        elif topic == "/tf":
            for tf in msg.ros_msg.transforms:
                pair = (tf.header.frame_id, tf.child_frame_id)
                if pair in tf_data:
                    tr = tf.transform.translation
                    ro = tf.transform.rotation
                    tf_data[pair].append(np.array([tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w]))
                    tf_ts[pair].append(t)
        elif topic == "/gripper_width_normalized":
            grip_buf.append(float(msg.ros_msg.data))
            grip_ts.append(t)

    if len(sync_ts) < 2:
        raise ValueError(f"Not enough sync pulses: {len(sync_ts)}")

    sync_arr  = np.array(sync_ts)
    ts_arrays = {p: np.array(tf_ts[p]) for p in TF_CHAIN}

    def tf_to_matrix(pose7):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(pose7[3:]).as_matrix()
        T[:3, 3]  = pose7[:3]
        return T

    ee_poses = []
    for sync_t in sync_arr:
        T = np.eye(4)
        valid = True
        for pair in TF_CHAIN:
            arr = ts_arrays[pair]
            if len(arr) == 0:
                valid = False
                break
            idx = int(np.clip(np.searchsorted(arr, sync_t) - 1, 0, len(tf_data[pair]) - 1))
            T = T @ tf_to_matrix(tf_data[pair][idx])
        if not valid:
            continue
        pos  = T[:3, 3]
        quat = Rotation.from_matrix(T[:3, :3]).as_quat()
        ee_poses.append(np.concatenate([pos, quat]).astype(np.float32))

    if grip_buf:
        grip_arr = np.array(grip_ts)
        grip_idx = np.clip(np.searchsorted(grip_arr, sync_arr) - 1, 0, len(grip_buf) - 1)
        grippers = [grip_buf[i] for i in grip_idx]
    else:
        print("WARNING: No gripper data found, using zeros")
        grippers = [0.0] * len(ee_poses)

    n_frames = min(len(ee_poses), len(grippers))
    return ee_poses[:n_frames], grippers[:n_frames], n_frames


class OnlineIK(TrajectoryRetargeter):

    def __init__(self, urdf_path=None):
        super().__init__(urdf_path)
        self.q_current = self.neutral_q.copy()

    def seed_from_robot_state(self, state: dict, silent: bool = False):
        jp = state.get("joint_positions", None)
        if jp is None or len(jp) < 9:
            print("WARNING: Could not read joint_positions, using neutral config")
            self.q_current = self.neutral_q.copy()
            return

        self.q_current = np.array([
            jp[2],   # base_theta  → base_rotation
            jp[0],   # base_x      → base_translation
            jp[3],   # joint_lift
            jp[4]*4,   # joint_arm_l0
            jp[8],   # joint_wrist_yaw
            jp[7],   # joint_wrist_pitch
            jp[6],   # joint_wrist_roll
        ])
        if not silent:
            print(f"Seeded IK: lift={jp[3]:.3f} arm={jp[4]:.3f} "
                  f"yaw={jp[8]:.3f} pitch={jp[7]:.3f} roll={jp[6]:.3f}")

    def joints_to_action(self, q7: np.ndarray, gripper_norm: float,
                         q_prev: np.ndarray, dt: float) -> np.ndarray:
        gripper_hw       = normalized_to_hardware(gripper_norm)
        base_linear_vel  = (q7[1] - q_prev[1]) / dt
        base_angular_vel = (q7[0] - q_prev[0]) / dt

        return np.array([
            base_linear_vel,
            0.0,
            base_angular_vel,
            q7[2],   # lift
            q7[3],   # arm
            q7[6],   # wrist_roll
            q7[5],   # wrist_pitch
            q7[4],   # wrist_yaw
            gripper_hw,
        ], dtype=np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mcap", type=str)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    args = parser.parse_args()

    mcap_path = Path(args.mcap)
    if not mcap_path.exists():
        candidate = Path(VTAM_ROOT) / "data" / "processed" / args.mcap
        if candidate.exists():
            mcap_path = candidate
        else:
            print(f"ERROR: MCAP file not found: {args.mcap}")
            sys.exit(1)

    print(f"\n{'='*60}")
    print(f"VTAM Demo Replay")
    print(f"{'='*60}")
    print(f"MCAP:    {mcap_path}")
    print(f"FPS:     {args.fps}")
    print(f"Dry run: {args.dry_run}")
    print(f"{'='*60}\n")

    print("Extracting EE poses from MCAP...")
    ee_poses, grippers, n_frames = extract_episode(str(mcap_path))
    print(f"Extracted {n_frames} frames")
    print(f"EE pose[0]:    {ee_poses[0]}")
    print(f"Gripper range: min={min(grippers):.3f} max={max(grippers):.3f}\n")

    print("Computing absolute EP matrices (PD2.1)...")

    def tf_to_mat(p):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
        T[:3, 3]  = p[:3]
        return T

    ep_mats = [tf_to_mat(p) for p in ee_poses]
    CHUNK_SIZE = 15
    print(f"Loaded {len(ep_mats)} absolute poses, chunk_size={CHUNK_SIZE}\n")

    print("Initialising IK solver...")
    ik = OnlineIK()

    if not args.dry_run:
        print(f"Connecting to robot state on port {ZMQ_STATE_PORT}...")
        ctx = zmq.Context()
        try:
            state = get_robot_state(ctx)
            ik.seed_from_robot_state(state)
        except RuntimeError as e:
            print(f"ERROR: {e}")
            sys.exit(1)

        action_sock = ctx.socket(zmq.PUB)
        action_sock.connect(f"tcp://{ROBOT_IP}:{ZMQ_ACTION_PORT}")
        time.sleep(0.5)
        print("Connected to robot. Starting replay in 3 seconds...")
        time.sleep(3.0)
    else:
        ctx = None
        action_sock = None
        print("DRY RUN — using neutral IK seed\n")

    dt         = 1.0 / args.fps
    pos_errors = []
    ori_errors = []

    print(f"{'Frame':<7} {'PosErr(mm)':<13} {'OriErr(deg)':<13} {'Lift':<8} {'Arm':<8} {'Gripper':<10} {'Status'}")
    print("-" * 75)

    for chunk_start in range(0, n_frames - 1, CHUNK_SIZE):
        chunk_end = min(chunk_start + CHUNK_SIZE, n_frames - 1)

        # PD2.1: re-seed once per chunk from real robot state
        if not args.dry_run:
            state = get_robot_state(ctx)
            ik.seed_from_robot_state(state, silent=True)

        # Record robot EE at chunk start as anchor
        cur_pos, cur_rot = ik.forward_kinematics(ik.q_current)
        T_t0_robot = np.eye(4)
        T_t0_robot[:3,:3] = cur_rot
        T_t0_robot[:3,3]  = cur_pos

        # Episode anchor at chunk start
        T_t0_ep_inv = np.linalg.inv(ep_mats[chunk_start])

        for i in range(chunk_start, chunk_end):
            t_start = time.time()

            # PD2.1: all targets relative to same chunk anchor
            T_chunk_rel = T_t0_ep_inv @ ep_mats[i + 1]
            T_target    = T_t0_robot @ T_chunk_rel
            target_pos  = T_target[:3, 3]
            target_quat = Rotation.from_matrix(T_target[:3,:3]).as_quat()

            q_prev = ik.q_current.copy()
            q_new, pos_err, ori_err = ik.step_ik(ik.q_current, target_pos, target_quat)
            ik.q_current = q_new

            gripper_norm = grippers[i + 1]
            action_9d    = ik.joints_to_action(q_new, gripper_norm, q_prev, dt)

            pos_errors.append(pos_err)
            ori_errors.append(ori_err)

            status = "OK"
            if pos_err > WARN_POS_ERROR_M:
                status = f"WARN pos={pos_err*1000:.0f}mm"
            elif ori_err > WARN_ORI_ERROR_RAD:
                status = f"WARN ori={np.degrees(ori_err):.1f}deg"

            print(
                f"{i:<7} "
                f"{pos_err*1000:<13.1f} "
                f"{np.degrees(ori_err):<13.1f} "
                f"{action_9d[3]:<8.3f} "
                f"{action_9d[4]:<8.3f} "
                f"{gripper_norm:<10.3f} "
                f"{status}"
            )

            send_action(action_sock, action_9d, dry_run=args.dry_run)

            elapsed = time.time() - t_start
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
            elif elapsed > dt * 1.5:
                print(f"  WARNING: Frame {i} took {elapsed*1000:.0f}ms (target {dt*1000:.0f}ms)")

    print("\n" + "=" * 60)
    print("Replay complete")
    print(f"  Frames:           {len(ep_mats) - 1}")
    print(f"  Mean pos error:   {np.mean(pos_errors)*1000:.1f} mm")
    print(f"  Max pos error:    {np.max(pos_errors)*1000:.1f} mm")
    print(f"  Mean ori error:   {np.degrees(np.mean(ori_errors)):.1f} deg")
    print(f"  Max ori error:    {np.degrees(np.max(ori_errors)):.1f} deg")
    warn_frames = sum(1 for e in pos_errors if e > WARN_POS_ERROR_M)
    print(f"  Frames > {WARN_POS_ERROR_M*1000:.0f}mm:  {warn_frames}/{len(ep_mats) - 1}")
    print("=" * 60)

    if action_sock:
        action_sock.close()
    if ctx:
        ctx.term()


if __name__ == "__main__":
    main()