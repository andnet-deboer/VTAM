#!/usr/bin/env python3
"""
run.py — Phone teleoperation or policy inference for Stretch.

Modes:
    teleop    — Phone (ARKit via teleop_receiver_node) drives robot via differential IK.
                Each frame: delta from /phone_teleop/pose → IK → joint commands.
                /joint_states from the robot IS the training data — no IK at inference.

    inference — Policy on sheep GPU drives robot (unchanged from original run.py).

USAGE:
    python3 run.py --mode teleop
    python3 run.py --mode teleop --dry-run
    python3 run.py --mode teleop --scale 1.5
    python3 run.py --mode inference

REQUIRES:
    teleop mode:    teleop_receiver_node running, vtam_robot_node.py running
    inference mode: vtam_robot_node.py running, sheep reachable

RUN ON: Robot (stretch-se3-3047)
"""

import argparse
import os
import sys
import time
import pickle
import threading
import numpy as np
import zmq
import cv2
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
UTILS_DIR  = os.path.join(VTAM_ROOT, 'training', 'utils')

sys.path.insert(0, VTAM_ROOT)
sys.path.insert(0, os.path.dirname(VTAM_ROOT))
sys.path.insert(0, UTILS_DIR)
from training.utils.differential_ik import TrajectoryRetargeter

# ── Config ────────────────────────────────────────────────────────────────────
ROBOT_IP           = "localhost"
ZMQ_STATE_PORT     = 4403
ZMQ_ACTION_PORT    = 4402
ZMQ_OBS_PORT       = 4401
ZMQ_FWD_PORT       = 4406
ZMQ_CHUNK_PORT     = 4405
DEFAULT_FPS        = 10
CHUNK_SIZE         = 15

GRIPPER_OPEN       = 1.05
GRIPPER_CLOSED     = -0.2
GRIPPER_TRAVEL     = GRIPPER_OPEN - GRIPPER_CLOSED

WARN_POS_ERROR_M   = 0.03
WARN_ORI_ERROR_RAD = 0.2
ZMQ_TIMEOUT_MS     = 3000

DIAG_DIR = "/tmp/vtam_diag"


# ── Phone pose subscriber (background ROS thread) ─────────────────────────────

class _PhoneSubscriber(Node):
    """Minimal rclpy node — stores the latest phone pose as a 4x4 matrix."""
    def __init__(self):
        super().__init__('teleop_run_phone_sub')
        self.latest_T = None
        self._lock    = threading.Lock()
        self.create_subscription(PoseStamped, '/phone_teleop/pose', self._cb, 10)

    def _cb(self, msg):
        p, o = msg.pose.position, msg.pose.orientation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        T[:3, 3]  = [p.x, p.y, p.z]
        with self._lock:
            self.latest_T = T

    def get(self):
        with self._lock:
            return None if self.latest_T is None else self.latest_T.copy()


def _spin_ros(node):
    rclpy.spin(node)


# ── Helpers ───────────────────────────────────────────────────────────────────

def normalized_to_hardware(normalized: float) -> float:
    return GRIPPER_CLOSED + (normalized * GRIPPER_TRAVEL)


def tf_to_mat(p):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
    T[:3, 3]  = p[:3]
    return T


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


# ── IK wrapper (unchanged from original) ─────────────────────────────────────

class OnlineIK(TrajectoryRetargeter):

    def __init__(self, urdf_path=None):
        super().__init__(urdf_path)
        self.q_current = self.neutral_q.copy()

    def _dynamic_weights(self, q_current, target_pos):
        return self.joint_weights.copy()

    def seed_from_robot_state(self, state: dict, silent: bool = False):
        jp = state.get("joint_positions", None)
        if jp is None or len(jp) < 9:
            print("WARNING: Could not read joint_positions, using neutral config")
            self.q_current = self.neutral_q.copy()
            return

        self.q_current = np.array([
            jp[2],     # base_theta
            jp[0],     # base_x
            jp[3],     # joint_lift
            jp[4] * 4, # joint_arm_l0
            jp[8],     # joint_wrist_yaw
            jp[7],     # joint_wrist_pitch
            jp[6],     # joint_wrist_roll
        ])
        if not silent:
            print(f"Seeded IK: base_x={jp[0]:.4f} base_theta={jp[2]:.4f} "
                  f"lift={jp[3]:.3f} arm={jp[4]:.3f}")

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


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode",  choices=["teleop", "inference"], default="teleop")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--fps",   type=float, default=DEFAULT_FPS)
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Teleop motion scale: phone translation → robot translation")
    args = parser.parse_args()

    dt = 1.0 / args.fps

    print(f"\n{'='*60}")
    print(f"VTAM {'Phone Teleoperation' if args.mode == 'teleop' else 'Policy Inference'}")
    print(f"{'='*60}")
    print(f"FPS: {args.fps}  dry_run: {args.dry_run}")
    if args.mode == "teleop":
        print(f"Scale: {args.scale}  topic: /phone_teleop/pose")
    print(f"{'='*60}\n")

    # ── Teleop: start ROS subscriber in background thread ────────────────────
    phone_sub = None
    if args.mode == "teleop":
        rclpy.init()
        phone_sub  = _PhoneSubscriber()
        ros_thread = threading.Thread(target=_spin_ros, args=(phone_sub,), daemon=True)
        ros_thread.start()
        print("Waiting for first phone pose on /phone_teleop/pose ...")
        while phone_sub.get() is None:
            time.sleep(0.05)
        print("Phone pose received.\n")

    # ── IK + ZMQ shared setup ─────────────────────────────────────────────────
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

    # ── Persistent state socket (re-seed IK from real robot each frame) ───────
    state_sock = ctx.socket(zmq.SUB)
    state_sock.setsockopt(zmq.SUBSCRIBE, b"")
    state_sock.setsockopt(zmq.CONFLATE, 1)
    state_sock.setsockopt(zmq.RCVTIMEO, ZMQ_TIMEOUT_MS)
    state_sock.connect(f"tcp://{ROBOT_IP}:{ZMQ_STATE_PORT}")

    # ── Inference-only sockets ────────────────────────────────────────────────
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

    # ── Anchor EE pose once before the loop ───────────────────────────────────
    cur_pos, cur_rot   = ik.forward_kinematics(ik.q_current)
    T_t0_robot         = np.eye(4)
    T_t0_robot[:3, :3] = cur_rot
    T_t0_robot[:3, 3]  = cur_pos
    T_t0_robot_inv     = np.linalg.inv(T_t0_robot)
    print(f"Anchored at EE: [{cur_pos[0]:.4f}, {cur_pos[1]:.4f}, {cur_pos[2]:.4f}]\n")

    # ── Diagnostics ───────────────────────────────────────────────────────────
    os.makedirs(DIAG_DIR, exist_ok=True)
    diag_log = open(os.path.join(DIAG_DIR, "diag.csv"), "w")
    diag_log.write("step,roundtrip_ms,state_x,state_y,state_z,action_x,action_y,action_z,"
                   "state_grip,action_grip,pos_err_mm,real_grip\n")

    print(f"{'Frame':<7} {'PosErr(mm)':<13} {'OriErr(deg)':<13} "
          f"{'Lift':<8} {'Arm':<8} {'Gripper':<10} {'Status'}")
    print("-" * 75)

    # ── Per-frame state ───────────────────────────────────────────────────────
    T_phone_prev    = phone_sub.get() if phone_sub else None
    roundtrip_ms    = 0.0
    state_8d        = np.zeros(8, dtype=np.float32)
    action_8d       = np.zeros(8, dtype=np.float32)
    current_gripper = 1.1

    step = 0
    try:
        while True:
            t_start = time.time()

            if args.mode == "teleop":
                # Re-seed IK from actual robot state to prevent drift
                try:
                    state_from_robot = pickle.loads(state_sock.recv())
                    ik.seed_from_robot_state(state_from_robot, silent=True)
                    jp = state_from_robot.get("joint_positions", [])
                    current_gripper = jp[5] if len(jp) > 5 else current_gripper
                except zmq.Again:
                    pass

                # Phone delta: how the phone moved since the last frame
                T_phone_now = phone_sub.get()
                if T_phone_prev is None:
                    T_phone_prev = T_phone_now

                T_delta           = T_phone_now @ np.linalg.inv(T_phone_prev)
                T_delta[:3, 3]   *= args.scale
                T_phone_prev      = T_phone_now

                # Apply delta to current robot EE
                cur_pos_now, cur_rot_now = ik.forward_kinematics(ik.q_current)
                T_ee = np.eye(4)
                T_ee[:3, :3] = cur_rot_now
                T_ee[:3, 3]  = cur_pos_now

                T_target    = T_ee @ T_delta
                target_pos  = T_target[:3, 3]
                target_quat = Rotation.from_matrix(T_target[:3, :3]).as_quat()
                gripper_norm = np.clip(
                    (current_gripper - GRIPPER_CLOSED) / GRIPPER_TRAVEL, 0.0, 1.0)

            else:
                # ── Inference (unchanged) ─────────────────────────────────────
                obs    = pickle.loads(obs_sock.recv())
                t_send = time.time()

                state_from_robot = pickle.loads(state_sock.recv())
                ik.seed_from_robot_state(state_from_robot, silent=True)
                cur_pos_now, cur_rot_now = ik.forward_kinematics(ik.q_current)

                T_curr = np.eye(4)
                T_curr[:3, :3] = cur_rot_now
                T_curr[:3, 3]  = cur_pos_now
                T_rel    = T_t0_robot_inv @ T_curr
                rel_pos  = T_rel[:3, 3]
                rel_quat = Rotation.from_matrix(T_rel[:3, :3]).as_quat()

                live_joints     = obs.get("joint", {})
                current_gripper = live_joints.get("joint_gripper_finger_left", 1.05)

                state_8d = np.concatenate([
                    rel_pos, rel_quat,
                    [np.clip((current_gripper - GRIPPER_CLOSED) / GRIPPER_TRAVEL, 0.0, 1.0)],
                ]).astype(np.float32)
                if step == 0:
                    print(f"  [STATE-8D] full state: {state_8d}")
                if step in [14, 15, 29, 30]:
                    print(f"  [STATE-SENT step={step}] state_8d[:3]={state_8d[:3]}")

                fwd_sock.send(pickle.dumps({"rgb": obs["rgb"], "state": state_8d}),
                              flags=zmq.NOBLOCK)

                action_8d     = pickle.loads(action_in.recv())
                raw_delta     = action_8d[:3].copy()
                action_8d[0]  = raw_delta[2]
                action_8d[1]  = raw_delta[0]
                action_8d[2]  = raw_delta[1]
                t_recv        = time.time()
                roundtrip_ms  = (t_recv - t_send) * 1000

                if step % 20 == 0:
                    try:
                        arr = np.frombuffer(obs["rgb"], np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            cv2.imwrite(os.path.join(DIAG_DIR, f"frame_{step:04d}.jpg"), img)
                    except Exception as e:
                        print(f"  [DIAG] image save failed: {e}")

                if step % 10 == 0:
                    print(f"  [DIAG-TIME] roundtrip={roundtrip_ms:.0f}ms")

                print(f"[DIAG] step={step} state_pos={state_8d[:3]} state_grip={state_8d[7]:.3f}")
                print(f"[DIAG] step={step} action_pos={action_8d[:3]} action_grip={action_8d[7]:.3f}")
                print(f"[DIAG] step={step} real_grip={current_gripper:.3f} ik_q={ik.q_current[:4]}")

                gripper_norm   = float(action_8d[7])
                target_rel_pos = rel_pos + action_8d[:3]
                R_rel_target   = Rotation.from_quat(rel_quat) * Rotation.from_quat(action_8d[3:7])

                T_target_rel             = np.eye(4)
                T_target_rel[:3, 3]      = target_rel_pos
                T_target_rel[:3, :3]     = R_rel_target.as_matrix()
                T_target                 = T_t0_robot @ T_target_rel
                target_pos               = T_target[:3, 3]
                target_quat              = Rotation.from_matrix(T_target[:3, :3]).as_quat()

            # ── IK (shared) ───────────────────────────────────────────────────
            q_prev = ik.q_current.copy()
            q      = ik.q_current.copy()
            for _ in range(20):
                q_new, pos_err, ori_err = ik.step_ik(q, target_pos, target_quat)
                if pos_err < 0.001 and ori_err < 0.001:
                    break
                q = q_new
            ik.q_current = q_new

            if step < 10:
                print(f"  [BASE-DIAG] rot={q_new[0]:.4f} trans={q_new[1]:.4f}  "
                      f"Δrot={(q_new[0]-q_prev[0]):.4f} Δtrans={(q_new[1]-q_prev[1]):.4f}")

            action_9d = ik.joints_to_action(q_new, gripper_norm, q_prev, dt)

            if step < 5:
                print(f"  [DBG joints] lift={action_9d[3]:.4f} arm={action_9d[4]:.4f} "
                      f"yaw={action_9d[7]:.4f} pitch={action_9d[6]:.4f} roll={action_9d[5]:.4f}")
                print(f"  [DBG target] pos=({target_pos[0]:.4f}, "
                      f"{target_pos[1]:.4f}, {target_pos[2]:.4f})")

            status = "OK"
            if pos_err > WARN_POS_ERROR_M:
                status = f"WARN pos={pos_err*1000:.0f}mm"
            elif ori_err > WARN_ORI_ERROR_RAD:
                status = f"WARN ori={np.degrees(ori_err):.1f}deg"

            print(f"{step:<7} {pos_err*1000:<13.1f} {np.degrees(ori_err):<13.1f} "
                  f"{action_9d[3]:<8.3f} {action_9d[4]:<8.3f} "
                  f"{gripper_norm:<10.3f} {status}")

            send_action(action_sock, action_9d, dry_run=args.dry_run)

            diag_log.write(f"{step},{roundtrip_ms:.1f},"
                           f"{target_pos[0]:.6f},{target_pos[1]:.6f},{target_pos[2]:.6f},"
                           f"{action_9d[3]:.6f},{action_9d[4]:.6f},{action_9d[5]:.6f},"
                           f"{gripper_norm:.4f},{gripper_norm:.4f},"
                           f"{pos_err*1000:.1f},{current_gripper:.4f}\n")
            diag_log.flush()

            elapsed = time.time() - t_start
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
            elif elapsed > dt * 1.5:
                print(f"  WARNING: Frame {step} took {elapsed*1000:.0f}ms "
                      f"(target {dt*1000:.0f}ms)")
            step += 1

    except KeyboardInterrupt:
        print(f"\nStopped at step {step}.")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    diag_log.close()
    print(f"Diagnostics saved to {DIAG_DIR}/")
    if action_sock:
        action_sock.close()
    ctx.term()
    if phone_sub is not None:
        phone_sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
