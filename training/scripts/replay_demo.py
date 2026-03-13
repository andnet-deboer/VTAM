#!/usr/bin/env python3
"""
replay_demo.py — Replay a recorded MCAP demonstration on the Stretch robot,
                 or run live policy inference (--mode inference).
replay_demo.py — Replay a recorded MCAP demonstration on the Stretch robot,
                 or run live policy inference (--mode inference).

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
    python3 replay_demo.py --mode inference

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
ZMQ_OBS_PORT       = 4401   # recv rgb from vtam_robot_node  (inference only)
ZMQ_FWD_PORT       = 4406   # send obs to sheep              (inference only)
ZMQ_CHUNK_PORT     = 4405   # recv action from sheep         (inference only)
DEFAULT_FPS        = 10
CHUNK_SIZE         = 15

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
    from mcap.reader import make_reader
    from rosbags.typesys import Stores, get_typestore
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    sync_ts  = []
    tf_data  = {p: [] for p in TF_CHAIN}
    tf_ts    = {p: [] for p in TF_CHAIN}
    grip_buf = []
    grip_ts  = []

    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)
        for _, channel, message in reader.iter_messages(
                topics=["/sync_pulse", "/tf", "/gripper_width_normalized"]):
            t     = message.publish_time / 1e9
            topic = channel.topic

            if topic == "/sync_pulse":
                sync_ts.append(t)

            elif topic == "/tf":
                msg = typestore.deserialize_cdr(message.data, 'tf2_msgs/msg/TFMessage')
                for tf in msg.transforms:
                    pair = (tf.header.frame_id, tf.child_frame_id)
                    if pair in tf_data:
                        tr, ro = tf.transform.translation, tf.transform.rotation
                        tf_data[pair].append(
                            np.array([tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w]))
                        tf_ts[pair].append(t)

            elif topic == "/gripper_width_normalized":
                msg = typestore.deserialize_cdr(message.data, 'std_msgs/msg/Float32')
                grip_buf.append(float(msg.data))
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
            jp[2],     # base_theta  → base_rotation
            jp[0],     # base_x      → base_translation
            jp[3],     # joint_lift
            jp[4] * 4, # joint_arm_l0
            jp[8],     # joint_wrist_yaw
            jp[7],     # joint_wrist_pitch
            jp[6],     # joint_wrist_roll
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
            q7[2],  # lift
            q7[3],  # arm
            q7[6],  # wrist_roll
            q7[5],  # wrist_pitch
            q7[4],  # wrist_yaw
            gripper_hw,
        ], dtype=np.float32)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mcap", type=str, nargs="?", default=None)
    parser.add_argument("--mode", choices=["replay", "inference"], default="replay")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    args = parser.parse_args()

    if args.mode == "replay" and args.mcap is None:
        print("ERROR: mcap path required for replay mode")
        sys.exit(1)

    dt = 1.0 / args.fps

    # ── Mode-specific setup ───────────────────────────────────────────────────
    if args.mode == "replay":
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
        print("Computing absolute EP matrices (PD2.1)...")

        def tf_to_mat(p):
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
            T[:3, 3]  = p[:3]
            return T
        def tf_to_mat(p):
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
            T[:3, 3]  = p[:3]
            return T

        ep_mats = [tf_to_mat(p) for p in ee_poses]
        print(f"Loaded {len(ep_mats)} absolute poses, chunk_size={CHUNK_SIZE}\n")
        ep_mats = [tf_to_mat(p) for p in ee_poses]
        print(f"Loaded {len(ep_mats)} absolute poses, chunk_size={CHUNK_SIZE}\n")

    else:
        print(f"\n{'='*60}")
        print(f"VTAM Policy Inference")
        print(f"{'='*60}")
        print(f"FPS: {args.fps}  dry_run: {args.dry_run}")
        print(f"{'='*60}\n")

    # ── Shared setup ──────────────────────────────────────────────────────────
    print("Initialising IK solver...")
    ik = OnlineIK()

    ctx = zmq.Context()

    if not args.dry_run:
        print(f"Connecting to robot state on port {ZMQ_STATE_PORT}...")
        try:
            state = get_robot_state(ctx)
            ik.seed_from_robot_state(state)
        except RuntimeError as e:
            print(f"ERROR: {e}")
            sys.exit(1)

        action_sock = ctx.socket(zmq.PUB)
        action_sock.connect(f"tcp://{ROBOT_IP}:{ZMQ_ACTION_PORT}")
        time.sleep(0.5)
    else:
        action_sock = None
        print("DRY RUN — using neutral IK seed\n")

    # ── Inference-only sockets (set up once, before the loop) ────────────────
    if args.mode == "inference":
        obs_sock = ctx.socket(zmq.SUB)
        obs_sock.setsockopt(zmq.SUBSCRIBE, b"")
        obs_sock.setsockopt(zmq.CONFLATE, 1)
        obs_sock.connect(f"tcp://localhost:{ZMQ_OBS_PORT}")

        SHEEP_IP = "129.105.69.11"

        fwd_sock = ctx.socket(zmq.PUB)
        fwd_sock.connect(f"tcp://{SHEEP_IP}:{ZMQ_FWD_PORT}")

        action_in = ctx.socket(zmq.SUB)
        action_in.setsockopt(zmq.SUBSCRIBE, b"")
        action_in.setsockopt(zmq.CONFLATE, 1)
        action_in.connect(f"tcp://{SHEEP_IP}:{ZMQ_CHUNK_PORT}")
        time.sleep(0.5)
        print("Sockets ready. Waiting for actions from sheep...\n")

    # ── Main loop ─────────────────────────────────────────────────────────────
    pos_errors = []
    ori_errors = []

    print(f"{'Frame':<7} {'PosErr(mm)':<13} {'OriErr(deg)':<13} "
          f"{'Lift':<8} {'Arm':<8} {'Gripper':<10} {'Status'}")
    print("-" * 75)

    if args.mode == "replay":
        print("Connected. Starting replay in 3 seconds...")
        time.sleep(3.0)
        chunk_iter = range(0, n_frames - 1, CHUNK_SIZE)
    else:
        chunk_iter = iter(int, 1)  # infinite

    step = 0
    for chunk_start in chunk_iter:

        # PD2.1: re-seed once per chunk from real robot state
        if not args.dry_run:
            state = get_robot_state(ctx)
            ik.seed_from_robot_state(state, silent=True)
        else:
            state = {}  # <-- NEW: Dummy state for dry-run

        # Record robot EE at chunk start as anchor
        cur_pos, cur_rot    = ik.forward_kinematics(ik.q_current)
        T_t0_robot          = np.eye(4)
        T_t0_robot[:3, :3]  = cur_rot
        T_t0_robot[:3,  3]  = cur_pos

        if args.mode == "replay": 
            chunk_end   = min(chunk_start + CHUNK_SIZE, n_frames - 1)
            T_t0_ep_inv = np.linalg.inv(ep_mats[chunk_start])
            inner_range = range(chunk_start, chunk_end)

        else: # Inference mode
            T_t0_robot_inv = np.linalg.inv(T_t0_robot)
            inner_range = range(CHUNK_SIZE)

        for i in inner_range:
            t_start = time.time()

            # ── Only these lines differ between modes ──────────────────────
            if args.mode == "replay":
                T_chunk_rel  = T_t0_ep_inv @ ep_mats[i + 1]
                gripper_norm = grippers[i + 1]
            else:
                obs = pickle.loads(obs_sock.recv())  # block until obs arrives
                # jp  = state.get("joint_positions", [])
                
                # Get current absolute UMI pose
                cur_pos_now, cur_rot_now = ik.forward_kinematics(ik.q_current)
                T_curr = np.eye(4)
                T_curr[:3,:3] = cur_rot_now
                T_curr[:3,3] = cur_pos_now
                T_rel = T_t0_robot_inv @ T_curr
                #TODO rel_pos = T_rel[:3, 3]
                rel_pos = T_rel[:3, 3]
                rel_pos[0] = -rel_pos[0]
                rel_quat = Rotation.from_matrix(T_rel[:3, :3]).as_quat()

                # state_8d = np.concatenate([
                #     rel_pos,
                #     rel_quat,
                #     [max(0.0, min(1.0, (1.05 - jp[5]) / 1.25)) if len(jp) > 5 else 0.0]
                # ]).astype(np.float32)

                live_joints = obs.get("joint", {})
                current_gripper = live_joints.get("joint_gripper_finger_right", 1.1)

                state_8d = np.concatenate([
                    rel_pos,
                    rel_quat,
                    [max(0.0, min(1.0, (1.05 - current_gripper) / 1.25))]
                ]).astype(np.float32)
                
                fwd_sock.send(pickle.dumps({"rgb": obs["rgb"], "state": state_8d}), flags=zmq.NOBLOCK)
                
                # --- Receive action from the server ---
                action_8d           = pickle.loads(action_in.recv())
                T_chunk_rel         = np.eye(4)
                T_chunk_rel[:3, :3] = Rotation.from_quat(action_8d[3:7]).as_matrix()
                T_chunk_rel[:3,  3] = action_8d[:3]
                
                gripper_norm        = float(action_8d[7])
                
                #TODO: Debug
                # 1. Compute target first so action_9d exists for the print
                T_target    = T_t0_robot @ T_chunk_rel
                target_pos  = T_target[:3, 3]
                target_quat = Rotation.from_matrix(T_target[:3, :3]).as_quat()

                q_prev = ik.q_current.copy()
                q_new, pos_err, ori_err = ik.step_ik(ik.q_current, target_pos, target_quat)
                ik.q_current = q_new
                action_9d = ik.joints_to_action(q_new, gripper_norm, q_prev, dt)

                # 2. Enhanced Diagnostic Dashboard
                if (step % CHUNK_SIZE) < 5:
                    print(f"\n--- [FRAME {step}] COORDINATE ANALYSIS ---")
                    # Model's output in UMI Frame (usually X is forward/out)
                    print(f"POLICY DELTA:  x={action_8d[0]:.4f} (Model wants to move this far from anchor)")
                    
                    # Robot's physical state in UMI Frame
                    print(f"ROBOT DELTA:   x={rel_pos[0]:.4f} (Robot thinks it has moved this far from anchor)")
                    
                    # The Physical Hardware result
                    # action_9d[4] is the total arm extension in meters
                    print(f"ARM COMMAND:   {action_9d[4]:.4f}m (Calculated extension for hardware)")

                    # THE "SMOKING GUN" CHECK
                    # If action_8d[0] is positive and ARM COMMAND is decreasing, your axis is flipped.
                    if action_8d[0] > 0.005 and action_9d[4] < (q_prev[3] - 0.001):
                         print("!!! AXIS FLIP DETECTED: Model wants to extend, but IK is retracting arm!")
                    elif action_8d[0] < -0.005 and action_9d[4] > (q_prev[3] + 0.001):
                         print("!!! AXIS FLIP DETECTED: Model wants to retract, but IK is extending arm!")
                    
                    print(f"-------------------------------------------\n")

            # ── PD2.1 + IK — identical for both modes ─────────────────────
            T_target    = T_t0_robot @ T_chunk_rel
            target_pos  = T_target[:3, 3]
            target_quat = Rotation.from_matrix(T_target[:3, :3]).as_quat()

            q_prev = ik.q_current.copy()
            q_new, pos_err, ori_err = ik.step_ik(ik.q_current, target_pos, target_quat)
            ik.q_current = q_new

            action_9d = ik.joints_to_action(q_new, gripper_norm, q_prev, dt)
            #TODO
            if step < 5:
                print(f"  [DBG joints] lift={action_9d[3]:.4f} arm={action_9d[4]:.4f} yaw={action_9d[7]:.4f} pitch={action_9d[6]:.4f} roll={action_9d[5]:.4f}")
                print(f"  [DBG target] pos=({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f})")
            
            pos_errors.append(pos_err)
            ori_errors.append(ori_err)

            status = "OK"
            if pos_err > WARN_POS_ERROR_M:
                status = f"WARN pos={pos_err*1000:.0f}mm"
            elif ori_err > WARN_ORI_ERROR_RAD:
                status = f"WARN ori={np.degrees(ori_err):.1f}deg"

            print(
                f"{step:<7} "
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
                print(f"  WARNING: Frame {step} took {elapsed*1000:.0f}ms (target {dt*1000:.0f}ms)")

            step += 1

    if args.mode == "replay":
        print("\n" + "=" * 60)
        print("Replay complete")
        print(f"  Frames:           {step}")
        print(f"  Mean pos error:   {np.mean(pos_errors)*1000:.1f} mm")
        print(f"  Max pos error:    {np.max(pos_errors)*1000:.1f} mm")
        print(f"  Mean ori error:   {np.degrees(np.mean(ori_errors)):.1f} deg")
        print(f"  Max ori error:    {np.degrees(np.max(ori_errors)):.1f} deg")
        warn_frames = sum(1 for e in pos_errors if e > WARN_POS_ERROR_M)
        print(f"  Frames > {WARN_POS_ERROR_M*1000:.0f}mm:  {warn_frames}/{step}")
        print("=" * 60)

    if action_sock:
        action_sock.close()
    ctx.term()


if __name__ == "__main__":
    main()