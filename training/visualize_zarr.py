#!/usr/bin/env python3
"""
visualize_zarr.py

Replay a .zarr trajectory through MuJoCo with the viewer.
Use this to visually validate IK output before training.

Usage:
    python3 visualize_zarr.py --zarr dataset.zarr
    python3 visualize_zarr.py --zarr dataset.zarr --episode 0
    python3 visualize_zarr.py --zarr dataset.zarr --episode 2 --speed 0.5
    python3 visualize_zarr.py --zarr dataset.zarr --headless  # render to video

Dependencies:
    pip install zarr numpy
    stretch_mujoco
"""

import argparse
import sys
import time
import numpy as np
import zarr


# Joint name mapping: zarr columns → stretch_mujoco joint names
# Adjust if stretch_mujoco uses different names
JOINT_NAMES = [
    "joint_mobile_base_rotation",
    "joint_lift",
    "joint_arm_l0",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
    "stretch_gripper",
]

MUJOCO_JOINT_MAP = {
    "joint_mobile_base_rotation": "joint_mobile_base_rotation",
    "joint_lift": "joint_lift",
    "joint_arm_l0": "joint_arm_l0",
    "joint_wrist_yaw": "joint_wrist_yaw",
    "joint_wrist_pitch": "joint_wrist_pitch",
    "joint_wrist_roll": "joint_wrist_roll",
    "stretch_gripper": "joint_gripper_finger_left",
}


def load_episode(zarr_path: str, episode_idx: int = 0):
    """Load a single episode from .zarr."""
    root = zarr.open(zarr_path, mode="r")

    states = np.array(root["data/state"])
    episode_ends = np.array(root["meta/episode_ends"])
    fps = root.attrs.get("fps", 15.0)

    # Get episode boundaries
    start = 0 if episode_idx == 0 else episode_ends[episode_idx - 1]
    end = episode_ends[episode_idx]

    episode_states = states[start:end]
    n_episodes = len(episode_ends)

    print(f"Loaded episode {episode_idx}/{n_episodes - 1}")
    print(f"  Frames: {len(episode_states)}")
    print(f"  FPS:    {fps}")
    print(f"  Duration: {len(episode_states) / fps:.1f}s")

    return episode_states, fps, n_episodes


def print_frame_info(frame_idx: int, n_frames: int, joints: np.ndarray):
    """Print current joint values."""
    print(f"\rFrame {frame_idx:4d}/{n_frames} | ", end="")
    for name, val in zip(JOINT_NAMES, joints):
        short = name.replace("joint_", "").replace("mobile_base_", "base_")[:8]
        print(f"{short}:{val:+.3f} ", end="")
    print("", end="", flush=True)


def replay_in_mujoco(
    states: np.ndarray,
    fps: float,
    speed: float = 1.0,
    headless: bool = False,
    video_path: str = None,
):
    """Play trajectory through MuJoCo."""
    try:
        from stretch_mujoco import StretchMujocoSimulator
    except ImportError:
        print("ERROR: stretch_mujoco not installed.")
        print("Install with: pip install stretch-mujoco")
        sys.exit(1)

    sim = StretchMujocoSimulator(headless=headless)
    sim.start()
    time.sleep(1.0)  # let sim settle

    dt = 1.0 / (fps * speed)
    n_frames = states.shape[0]
    frames = []  # for video export

    print(f"\nPlaying back {n_frames} frames at {speed}x speed...")
    print("Press Ctrl+C to stop\n")

    try:
        for i in range(n_frames):
            # Command each joint
            for j, joint_name in enumerate(JOINT_NAMES):
                mujoco_name = MUJOCO_JOINT_MAP.get(joint_name, joint_name)
                sim.set_joint_position(mujoco_name, states[i, j])

            sim.step(1.0 / fps)

            # Capture frame if recording video
            if video_path and hasattr(sim, "get_camera_image"):
                frame = sim.get_camera_image("navigation_camera")
                if frame is not None:
                    frames.append(frame)

            print_frame_info(i, n_frames, states[i])
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    print(f"\n\nPlayback complete.")

    # Save video if requested
    if video_path and frames:
        save_video(frames, video_path, fps * speed)

    # Keep viewer open until user closes it
    if not headless:
        print("Close the MuJoCo viewer window to exit.")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    sim.stop()


def save_video(frames: list, path: str, fps: float):
    """Save captured frames as mp4."""
    try:
        import cv2
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
        for frame in frames:
            writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        writer.release()
        print(f"Saved video to {path} ({len(frames)} frames)")
    except ImportError:
        print("WARNING: opencv not installed, skipping video export")


def list_episodes(zarr_path: str):
    """Print info about all episodes in a .zarr."""
    root = zarr.open(zarr_path, mode="r")
    episode_ends = np.array(root["meta/episode_ends"])
    fps = root.attrs.get("fps", 15.0)
    n_total = root["data/state"].shape[0]

    print(f"\n{zarr_path}")
    print(f"  Total frames: {n_total}")
    print(f"  FPS: {fps}")
    print(f"  Episodes: {len(episode_ends)}\n")

    for i, end in enumerate(episode_ends):
        start = 0 if i == 0 else episode_ends[i - 1]
        n = end - start
        dur = n / fps
        print(f"  Episode {i}: frames {start}-{end} ({n} frames, {dur:.1f}s)")

    print()


def main():
    parser = argparse.ArgumentParser(description="Visualize .zarr trajectories in MuJoCo")
    parser.add_argument("--zarr", type=str, required=True, help="Path to .zarr dataset")
    parser.add_argument("--episode", type=int, default=0, help="Episode index to replay")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--headless", action="store_true", help="Run without viewer (for video)")
    parser.add_argument("--video", type=str, default=None, help="Save replay as .mp4")
    parser.add_argument("--list", action="store_true", help="List episodes and exit")
    args = parser.parse_args()

    if args.list:
        list_episodes(args.zarr)
        return

    states, fps, n_episodes = load_episode(args.zarr, args.episode)

    if args.episode >= n_episodes:
        print(f"ERROR: Episode {args.episode} doesn't exist (max: {n_episodes - 1})")
        sys.exit(1)

    replay_in_mujoco(
        states,
        fps,
        speed=args.speed,
        headless=args.headless,
        video_path=args.video,
    )


if __name__ == "__main__":
    main()