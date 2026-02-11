#!/usr/bin/env python3
"""
bag_to_zarr.py

Converts ROS2 bags of DexUMI end-effector poses into LeRobot-compatible .zarr datasets.

Pipeline:
    ROS2 bag (/dexumi/ee_pose) → Simple IK → MuJoCo headless → .zarr

Topic:
    /dexumi/ee_pose  (geometry_msgs/PoseStamped, in robot base frame)

Usage:
    python3 bag_to_zarr.py --bag path/to/rosbag --output path/to/output.zarr
    python3 bag_to_zarr.py --bag path/to/rosbag --output output.zarr --no-mujoco
    python3 bag_to_zarr.py --bag-dir path/to/bags/ --output dataset.zarr  # multi-episode

Dependencies:
    pip install rosbags zarr numpy scipy
    # simple_ik.py from stretch_dex_teleop (set path below)
    # stretch_mujoco (optional, for physics validation)
"""

import argparse
import sys
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path

# ============================================================
# PATH CONFIG 
# Where simple_ik.py and its dependencies live
SIMPLE_IK_DIR = os.path.expanduser(
    "~/VTAM/dependencies/stretch_dex_teleop"
)
sys.path.insert(0, SIMPLE_IK_DIR)

# ============================================================
# IMPORTS
# ============================================================
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
import zarr
import simple_ik as si

# Topic to read from bag
EE_POSE_TOPIC = "/dexumi/ee_pose"

# Joint names in order - this defines the columns of your .zarr arrays
JOINT_NAMES = [
    "joint_mobile_base_rotation",
    "joint_lift",
    "joint_arm_l0",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
    "stretch_gripper",
]

# Target control frequency for LeRobot (Hz)
# If bag rate differs, we resample to this
TARGET_HZ = 15.0


# ============================================================
# IK: pose joint config
# ============================================================
def pose_to_joints(
    position: np.ndarray,
    rotation_matrix: np.ndarray,
    grip_width: float = 0.5,
    ik: si.SimpleIK = None,
) -> dict | None:
    """
    Convert a 6DOF EE pose to Stretch joint configuration.

    Position → simple_ik.ik_rotary_base() for lift, arm, base rotation.
    Orientation → Euler ZXY decomposition for wrist yaw, pitch, roll.
    Lifted directly from gripper_to_goal.py.

    Args:
        position: [x, y, z] in robot base frame
        rotation_matrix: 3x3 rotation matrix of end-effector
        grip_width: 0.0 (closed) to 1.0 (open), default 0.5
        ik: SimpleIK instance (reuse across calls)

    Returns:
        dict of joint_name → value, or None if IK fails
    """
    if ik is None:
        ik = si.SimpleIK()

    # --- Position IK ---
    config = ik.ik_rotary_base(position)
    if config is None:
        return None

    # --- Orientation: Euler decomposition (from gripper_to_goal.py) ---
    r = Rotation.from_matrix(rotation_matrix)
    ypr = r.as_euler("ZXY", degrees=False)

    wrist_yaw = _angle_diff(ypr[0] + np.pi, 0.0)
    wrist_pitch = ypr[1]
    wrist_roll = _angle_diff(ypr[2] + np.pi, 0.0)

    # --- Assemble full joint config ---
    joints = {
        "joint_mobile_base_rotation": config["joint_mobile_base_rotation"],
        "joint_lift": config["joint_lift"],
        "joint_arm_l0": config["joint_arm_l0"],
        "joint_wrist_yaw": wrist_yaw,
        "joint_wrist_pitch": wrist_pitch,
        "joint_wrist_roll": wrist_roll,
        "stretch_gripper": grip_width - 0.5,  # center around 0
    }

    ik.clip_with_joint_limits(joints)
    return joints


def _angle_diff(a: float, b: float) -> float:
    """Compute shortest angular difference, result in [-pi, pi]."""
    diff = a - b
    while diff > np.pi:
        diff -= 2.0 * np.pi
    while diff < -np.pi:
        diff += 2.0 * np.pi
    return diff


# ============================================================
# BAG READING
# ============================================================
def read_poses_from_bag(bag_path: str, topic: str = EE_POSE_TOPIC):
    """
    Read all PoseStamped messages from a ROS2 bag.

    Returns:
        timestamps: list of float (seconds)
        positions: list of np.ndarray [x,y,z]
        rotation_matrices: list of np.ndarray (3x3)
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    timestamps = []
    positions = []
    rotation_matrices = []

    with Reader(bag_path) as reader:
        # Check topic exists
        available_topics = [c.topic for c in reader.connections]
        if topic not in available_topics:
            print(f"ERROR: Topic '{topic}' not found in bag.")
            print(f"Available topics: {available_topics}")
            sys.exit(1)

        connections = [c for c in reader.connections if c.topic == topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            t = timestamp * 1e-9  # nanoseconds → seconds
            pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])
            quat = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
            rot = Rotation.from_quat(quat).as_matrix()

            timestamps.append(t)
            positions.append(pos)
            rotation_matrices.append(rot)

    print(f"Read {len(timestamps)} poses from '{topic}' in {bag_path}")
    return timestamps, positions, rotation_matrices


# ============================================================
# MUJOCO HEADLESS VALIDATION
# ============================================================
def validate_with_mujoco(joint_trajectories: np.ndarray, dt: float) -> np.ndarray:
    """
    Run joint commands through MuJoCo headless to get physically-valid trajectories.

    Steps the sim one frame per joint config, reads back achieved joint states.
    What comes out is what the robot would actually do.

    Args:
        joint_trajectories: (N, 7) array of commanded joint positions
        dt: timestep in seconds

    Returns:
        achieved_trajectories: (N, 7) array of achieved joint positions
    """
    try:
        from stretch_mujoco import StretchMujocoSimulator
    except ImportError:
        print("ERROR: stretch_mujoco not installed. Use --no-mujoco to skip.")
        sys.exit(1)

    print("Starting MuJoCo headless validation...")

    # Joint name mapping to stretch_mujoco names TODO: check thse names match stretch_mujoco, adjust if needed
    MUJOCO_JOINT_MAP = {
        "joint_mobile_base_rotation": "joint_mobile_base_rotation",
        "joint_lift": "joint_lift",
        "joint_arm_l0": "joint_arm_l0",
        "joint_wrist_yaw": "joint_wrist_yaw",
        "joint_wrist_pitch": "joint_wrist_pitch",
        "joint_wrist_roll": "joint_wrist_roll",
        "stretch_gripper": "joint_gripper_finger_left",
    }

    sim = StretchMujocoSimulator(headless=True)
    sim.start()

    # Let sim settle
    time.sleep(0.5)

    achieved = np.zeros_like(joint_trajectories)
    n_frames = joint_trajectories.shape[0]

    for i in range(n_frames):
        # Command each joint
        for j, joint_name in enumerate(JOINT_NAMES):
            mujoco_name = MUJOCO_JOINT_MAP.get(joint_name, joint_name)
            sim.set_joint_position(mujoco_name, joint_trajectories[i, j])

        # Step sim forward
        sim.step(dt)

        # Read back achieved positions
        for j, joint_name in enumerate(JOINT_NAMES):
            mujoco_name = MUJOCO_JOINT_MAP.get(joint_name, joint_name)
            achieved[i, j] = sim.get_joint_position(mujoco_name)

        if (i + 1) % 100 == 0:
            print(f"  MuJoCo: {i+1}/{n_frames} frames")

    sim.stop()

    # Report tracking error
    error = np.abs(joint_trajectories - achieved)
    print(f"MuJoCo tracking error (mean): {error.mean():.4f} rad")
    print(f"MuJoCo tracking error (max):  {error.max():.4f} rad")

    return achieved


# ============================================================
# RESAMPLING
# ============================================================
def resample_uniform(timestamps, data, target_hz):
    """
    Resample irregular timestamps to uniform rate via linear interpolation.

    Args:
        timestamps: (N,) array of original times
        data: (N, D) array of values
        target_hz: desired output frequency

    Returns:
        new_timestamps: (M,) uniform timestamps
        new_data: (M, D) interpolated values
    """
    t_start = timestamps[0]
    t_end = timestamps[-1]
    dt = 1.0 / target_hz
    new_timestamps = np.arange(t_start, t_end, dt)

    new_data = np.zeros((len(new_timestamps), data.shape[1]))
    for col in range(data.shape[1]):
        new_data[:, col] = np.interp(new_timestamps, timestamps, data[:, col])

    print(f"Resampled {len(timestamps)} frames → {len(new_timestamps)} frames at {target_hz} Hz")
    return new_timestamps, new_data


# ============================================================
# ZARR WRITING (LeRobot format)
# ============================================================
def write_zarr(
    output_path: str,
    joint_states: np.ndarray,
    timestamps: np.ndarray,
    episode_ends: list[int],
    hz: float,
):
    """
    Write to .zarr in LeRobot-compatible format.

    Structure:
        data/
            state:   (N, 7) float32 - joint positions at each timestep
            action:  (N, 7) float32 - joint positions at next timestep (shifted by 1)
        meta/
            episode_ends: array of indices where episodes end
    """
    store = zarr.DirectoryStore(output_path)
    root = zarr.group(store=store, overwrite=True)

    data_group = root.create_group("data")
    meta_group = root.create_group("meta")

    N = joint_states.shape[0]

    # State: joint positions at each timestep
    data_group.create_dataset(
        "state",
        data=joint_states.astype(np.float32),
        chunks=(min(1000, N), len(JOINT_NAMES)),
        dtype=np.float32,
    )

    # Action: next joint positions (shift state by 1, repeat last frame)
    actions = np.roll(joint_states, -1, axis=0)
    actions[-1] = joint_states[-1]  # last action = hold position
    data_group.create_dataset(
        "action",
        data=actions.astype(np.float32),
        chunks=(min(1000, N), len(JOINT_NAMES)),
        dtype=np.float32,
    )

    # Timestamps
    data_group.create_dataset(
        "timestamp",
        data=timestamps.astype(np.float64),
        chunks=(min(1000, N),),
        dtype=np.float64,
    )

    # Episode boundaries
    meta_group.create_dataset(
        "episode_ends",
        data=np.array(episode_ends, dtype=np.int64),
        dtype=np.int64,
    )

    # Metadata attrs
    root.attrs["fps"] = hz
    root.attrs["joint_names"] = JOINT_NAMES
    root.attrs["n_episodes"] = len(episode_ends)
    root.attrs["n_frames"] = N

    print(f"Wrote .zarr to {output_path}")
    print(f"  {N} frames, {len(episode_ends)} episode(s), {hz} Hz")
    print(f"  state shape:  {joint_states.shape}")
    print(f"  action shape: {actions.shape}")


# ============================================================
# MAIN
# ============================================================
def process_single_bag(bag_path: str, ik: si.SimpleIK) -> tuple:
    """Process one bag file. Returns (timestamps, joint_configs) or (None, None)."""

    timestamps, positions, rotations = read_poses_from_bag(bag_path)
    if len(timestamps) == 0:
        print(f"WARNING: No poses found in {bag_path}, skipping.")
        return None, None

    # Run IK on every frame
    joint_configs = []
    ik_failures = 0

    for i, (pos, rot) in enumerate(zip(positions, rotations)):
        joints = pose_to_joints(pos, rot, grip_width=0.5, ik=ik)
        if joints is None:
            ik_failures += 1
            if len(joint_configs) > 0:
                joint_configs.append(joint_configs[-1])  # hold last valid
            else:
                joint_configs.append({name: 0.0 for name in JOINT_NAMES})
        else:
            joint_configs.append(joints)

    if ik_failures > 0:
        pct = 100.0 * ik_failures / len(timestamps)
        print(f"WARNING: IK failed on {ik_failures}/{len(timestamps)} frames ({pct:.1f}%)")

    # Convert to numpy array
    joint_array = np.array([[cfg[name] for name in JOINT_NAMES] for cfg in joint_configs])

    return np.array(timestamps), joint_array


def main():
    parser = argparse.ArgumentParser(description="Convert DexUMI ROS2 bags to LeRobot .zarr")
    parser.add_argument("--bag", type=str, help="Path to single ROS2 bag directory")
    parser.add_argument("--bag-dir", type=str, help="Path to directory of bags (one per episode)")
    parser.add_argument("--output", type=str, required=True, help="Output .zarr path")
    parser.add_argument("--topic", type=str, default=EE_POSE_TOPIC, help="Pose topic name")
    parser.add_argument("--hz", type=float, default=TARGET_HZ, help="Target frequency (Hz)")
    parser.add_argument("--no-mujoco", action="store_true", help="Skip MuJoCo validation")
    parser.add_argument("--visualize", action="store_true", help="Open MuJoCo viewer after processing")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed for --visualize")
    args = parser.parse_args()

    global EE_POSE_TOPIC
    EE_POSE_TOPIC = args.topic

    if not args.bag and not args.bag_dir:
        parser.error("Provide --bag or --bag-dir")

    # Collect bag paths
    if args.bag:
        bag_paths = [args.bag]
    else:
        bag_paths = sorted([
            str(p) for p in Path(args.bag_dir).iterdir()
            if p.is_dir() and (p / "metadata.yaml").exists()
        ])
        if not bag_paths:
            print(f"ERROR: No ROS2 bags found in {args.bag_dir}")
            sys.exit(1)
        print(f"Found {len(bag_paths)} bags in {args.bag_dir}")

    # Init IK once
    print("Initializing SimpleIK...")
    ik = si.SimpleIK()

    # Process each bag as one episode ---
    all_timestamps = []
    all_joints = []
    episode_ends = []

    for bag_path in bag_paths:
        print(f"\n--- Processing: {bag_path} ---")
        ts, joints = process_single_bag(bag_path, ik)
        if ts is None:
            continue

        # Resample to uniform rate
        ts_uniform, joints_uniform = resample_uniform(ts, joints, args.hz)

        all_timestamps.append(ts_uniform)
        all_joints.append(joints_uniform)
        episode_ends.append(
            sum(len(j) for j in all_joints)  # cumulative index
        )

    if not all_joints:
        print("ERROR: No valid data processed.")
        sys.exit(1)

    # Stack all episodes
    all_timestamps = np.concatenate(all_timestamps)
    all_joints = np.concatenate(all_joints)

    # MuJoCo validation ---
    if not args.no_mujoco:
        dt = 1.0 / args.hz
        all_joints = validate_with_mujoco(all_joints, dt)

    # Write .zarr
    write_zarr(args.output, all_joints, all_timestamps, episode_ends, args.hz)

    print("\nDone!")
    print(f"  Total frames: {len(all_timestamps)}")
    print(f"  Episodes:     {len(episode_ends)}")
    print(f"  Output:       {args.output}")

    # Visualize if requested
    if args.visualize:
        from visualize_zarr import replay_in_mujoco
        print("\nLaunching MuJoCo viewer...")
        replay_in_mujoco(all_joints, args.hz, speed=args.speed)


if __name__ == "__main__":
    main()