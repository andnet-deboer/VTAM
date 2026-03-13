#!/usr/bin/env python3
"""
validate_demos.py — Validate processed MCAP episodes before dataset creation.

Classification:
  PASS    — episode looks normal
  WARNING — length outlier (mean ± 2 std) OR gripper barely moved
  ERROR   — gripper never moved (< 5% of dataset median range)

Usage:
  python validate_demos.py place_coffee_cup
  python validate_demos.py place_coffee_cup --delete   # remove ERROR files
  python validate_demos.py place_coffee_cup --verbose  # print per-episode gripper stats
"""

import argparse
import sys
import os
import glob
import numpy as np
from pathlib import Path

# ── Path setup  ───────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VTAM_ROOT  = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))
UTILS_DIR  = os.path.join(SCRIPT_DIR, '..', 'utils')
QUAT_JUMP_THRESHOLD = 0.3  # dot product below this = large jump
QUAT_JUMP_ERROR_FRACTION = 0.05  # >5% of frames with jumps = ERROR
QUAT_JUMP_WARNING_FRACTION = 0.02  # >2% = WARNING

sys.path.insert(0, UTILS_DIR)
sys.path.insert(0, os.path.join(VTAM_ROOT, 'dependencies', 'lerobot'))

import yaml
from mcap_ros2.reader import read_ros2_messages

# ── Config ──────────────────────────────────────────
CONFIG_PATH = os.path.expanduser("~/VTAM/src/vtam_core/config/record.yaml")
MIN_FRAMES  = 30   # episodes shorter than this are always an ERROR

# Gripper thresholds (fraction of dataset median gripper range)
GRIPPER_ERROR_THRESHOLD   = 0.05   # < 5%  of median range → ERROR (never opened/closed)
GRIPPER_WARNING_THRESHOLD = 0.20   # < 20% of median range → WARNING (barely moved)


def load_topics(config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['/**']['ros__parameters']['session_topics']


def find_topic(topics, kw):
    return next((t for t in topics if kw in t), None)


def read_episode(mcap_path, sync_topic, gripper_topic):
    """
    Read a single MCAP file and return:
      - n_frames: number of sync pulses (= frame count)
      - gripper_values: list of gripper width floats
    Returns None if the file cannot be read.
    """
    sync_ts     = []
    grip_buf    = []
    grip_ts     = []

    try:
        for msg in read_ros2_messages(mcap_path, topics=[sync_topic, gripper_topic]):
            topic = msg.channel.topic
            t     = msg.publish_time_ns / 1e9

            if topic == sync_topic:
                sync_ts.append(
                    msg.ros_msg.header.stamp.sec
                    + msg.ros_msg.header.stamp.nanosec / 1e9
                )
            elif topic == gripper_topic:
                grip_buf.append(float(msg.ros_msg.data))
                grip_ts.append(t)
    except Exception as e:
        return None, None, str(e)

    if len(sync_ts) < 1:
        return 0, [], None

    # Snap gripper values to sync timestamps
    sync_arr = np.array(sync_ts)
    if grip_buf:
        grip_arr = np.array(grip_ts)
        idx      = np.clip(np.searchsorted(grip_arr, sync_arr) - 1, 0, len(grip_buf) - 1)
        gripper  = [grip_buf[i] for i in idx]
    else:
        gripper  = []

    return len(sync_ts), gripper, None

def count_quat_jumps(mcap_path, sync_topic):
    from mcap.reader import make_reader
    from rosbags.typesys import Stores, get_typestore
    from scipy.spatial.transform import Rotation as R
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    TF_CHAIN = [("base_link", "umi_disconnect"), ("umi_disconnect", "umi_gripper")]
    tf_data = {p: [] for p in TF_CHAIN}
    tf_ts = {p: [] for p in TF_CHAIN}
    sync_ts = []

    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)
        for _, channel, message in reader.iter_messages(topics=['/tf', sync_topic]):
            t = message.publish_time / 1e9
            if channel.topic == sync_topic:
                sync_ts.append(t)
            elif channel.topic == '/tf':
                msg = typestore.deserialize_cdr(message.data, 'tf2_msgs/msg/TFMessage')
                for tf in msg.transforms:
                    pair = (tf.header.frame_id, tf.child_frame_id)
                    if pair in tf_data:
                        ro = tf.transform.rotation
                        tf_data[pair].append(np.array([ro.x, ro.y, ro.z, ro.w]))
                        tf_ts[pair].append(t)

    if not tf_data[TF_CHAIN[0]] or not sync_ts:
        return 0, 0

    # Snap to sync timestamps
    sync_arr = np.array(sync_ts)
    ts_arr = np.array(tf_ts[TF_CHAIN[0]])
    idx = np.clip(np.searchsorted(ts_arr, sync_arr) - 1, 0, len(tf_data[TF_CHAIN[0]]) - 1)
    quats = [tf_data[TF_CHAIN[0]][i] for i in idx]

    jumps = 0
    for i in range(1, len(quats)):
        dot = abs(np.dot(quats[i], quats[i-1]))
        if dot < QUAT_JUMP_THRESHOLD:
            jumps += 1

    return jumps, len(quats)

def classify_episode(n_frames, gripper_values, length_mean, length_std,
                     gripper_median_range, quat_jumps=0, quat_total=0, verbose=False):
    """
    Returns (status, reasons) where status is 'PASS', 'WARNING', or 'ERROR'
    and reasons is a list of human-readable strings.
    """
    status  = 'PASS'
    reasons = []

    # ── Length checks ──────────────────────────────────────────────────────────
    if n_frames < MIN_FRAMES:
        status = 'ERROR'
        reasons.append(f"Too short: {n_frames} frames (min={MIN_FRAMES})")
    else:
        low  = length_mean - 2 * length_std
        high = length_mean + 2 * length_std
        if n_frames < low:
            if status != 'ERROR':
                status = 'WARNING'
            reasons.append(
                f"Short outlier: {n_frames} frames "
                f"(mean={length_mean:.0f}, threshold={low:.0f})"
            )
        elif n_frames > high:
            if status != 'ERROR':
                status = 'WARNING'
            reasons.append(
                f"Long outlier: {n_frames} frames "
                f"(mean={length_mean:.0f}, threshold={high:.0f})"
            )

    # ── Gripper checks ─────────────────────────────────────────────────────────
    if not gripper_values:
        status = 'ERROR'
        reasons.append("No gripper data found in episode")
    else:
        g_range = max(gripper_values) - min(gripper_values)

        if verbose:
            reasons.append(
                f"  gripper range={g_range*1000:.1f}mm "
                f"min={min(gripper_values)*1000:.1f}mm "
                f"max={max(gripper_values)*1000:.1f}mm"
            )
        
        if gripper_median_range > 0:
            fraction = g_range / gripper_median_range
            if fraction < GRIPPER_ERROR_THRESHOLD:
                status = 'ERROR'
                reasons.append(
                    f"Gripper never moved: range={g_range*1000:.1f}mm "
                    f"({fraction*100:.1f}% of median {gripper_median_range*1000:.1f}mm)"
                )
            elif fraction < GRIPPER_WARNING_THRESHOLD:
                if status != 'ERROR':
                    status = 'WARNING'
                reasons.append(
                    f"Gripper barely moved: range={g_range*1000:.1f}mm "
                    f"({fraction*100:.1f}% of median {gripper_median_range*1000:.1f}mm)"
                )
        # ── Orientation jump checks ────────────────────────────────────────────────
    if quat_total > 0:
        fraction = quat_jumps / quat_total
        if fraction > QUAT_JUMP_ERROR_FRACTION:
            status = 'ERROR'
            reasons.append(f"Too many orientation jumps: {quat_jumps}/{quat_total} ({fraction*100:.1f}%)")
        elif fraction > QUAT_JUMP_WARNING_FRACTION:
            if status != 'ERROR':
                status = 'WARNING'
            reasons.append(f"Orientation jumps: {quat_jumps}/{quat_total} ({fraction*100:.1f}%)")
    
    return status, reasons


def main():
    parser = argparse.ArgumentParser(
        description="Validate processed MCAP episodes before dataset creation."
    )
    parser.add_argument(
        'demo_name',
        type=str,
        help="Demo name, e.g. place_coffee_cup"
    )
    parser.add_argument(
        '--delete',
        action='store_true',
        help="Delete MCAP files classified as ERROR"
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help="Print per-episode gripper stats"
    )
    args = parser.parse_args()

    processed_dir = Path(VTAM_ROOT) / 'data' / 'processed' / args.demo_name
    if not processed_dir.exists():
        print(f"ERROR: Directory not found: {processed_dir}")
        sys.exit(1)

    mcap_paths = sorted(glob.glob(str(processed_dir / '*.mcap')))
    if not mcap_paths:
        print(f"ERROR: No MCAP files found in {processed_dir}")
        sys.exit(1)

    # Load topic names from config
    topics        = load_topics(CONFIG_PATH)
    sync_topic    = find_topic(topics, 'sync_pulse')
    gripper_topic = find_topic(topics, 'gripper_width')

    if not sync_topic or not gripper_topic:
        print(f"ERROR: Could not find required topics in {CONFIG_PATH}")
        print(f"  sync_pulse found: {sync_topic}")
        print(f"  gripper_width found: {gripper_topic}")
        sys.exit(1)

    print(f"\nValidating {len(mcap_paths)} episodes in: {processed_dir}")
    print(f"Sync topic:    {sync_topic}")
    print(f"Gripper topic: {gripper_topic}\n")

    # ── Pass 1: read all episodes ──────────────────────────────────────────────
    print("Reading episodes...")
    episodes = []  # list of (path, n_frames, gripper_values, read_error)
    for path in mcap_paths:
        n_frames, gripper, err = read_episode(path, sync_topic, gripper_topic)
        episodes.append((path, n_frames, gripper, err))
        print(f"  {Path(path).name}: {n_frames} frames", end='')
        print(f" [READ ERROR: {err}]" if err else "")

    # ── Compute dataset-level statistics ──────────────────────────────────────
    valid_lengths = [n for _, n, _, e in episodes if e is None and n >= MIN_FRAMES]
    if not valid_lengths:
        print("\nERROR: No valid episodes to compute statistics from.")
        sys.exit(1)

    length_mean = np.mean(valid_lengths)
    length_std  = np.std(valid_lengths)

    gripper_ranges = []
    for _, _, gripper, err in episodes:
        if err is None and gripper:
            gripper_ranges.append(max(gripper) - min(gripper))

    gripper_median_range = np.median(gripper_ranges) if gripper_ranges else 0.0

    print(f"\nDataset statistics:")
    print(f"  Frame count  — mean: {length_mean:.1f}  std: {length_std:.1f}  "
          f"low threshold: {length_mean - 2*length_std:.0f}  "
          f"high threshold: {length_mean + 2*length_std:.0f}")
    print(f"  Gripper range median: {gripper_median_range*1000:.1f}mm")

    # ── Pass 2: classify ───────────────────────────────────────────────────────
    print(f"\n{'─'*70}")
    print(f"{'Episode':<45} {'Status':<10} Notes")
    print(f"{'─'*70}")

    counts   = {'PASS': 0, 'WARNING': 0, 'ERROR': 0}
    to_delete = []

    for path, n_frames, gripper, read_error in episodes:
        name = Path(path).name

        if read_error:
            status  = 'ERROR'
            reasons = [f"Read failed: {read_error}"]
        else:
            jumps, total = count_quat_jumps(path, sync_topic)
            status, reasons = classify_episode(
                n_frames, gripper,
                length_mean, length_std,
                gripper_median_range,
                quat_jumps=jumps, quat_total=total,
                verbose=args.verbose
            )

        counts[status] += 1
        if status == 'ERROR':
            to_delete.append(path)

        # Print line
        label = {'PASS': 'PASS', 'WARNING': 'WARN', 'ERROR': 'ERROR'}[status]
        note  = reasons[0] if reasons else ''
        # filter out verbose gripper lines from first note
        display_reasons = [r for r in reasons if not r.strip().startswith('gripper range')]
        note = display_reasons[0] if display_reasons else ''
        print(f"  {name:<43} [{label:<5}]  {note}")

        # Print additional reasons indented
        for r in display_reasons[1:]:
            print(f"  {'':43}          {r}")

        # Print verbose gripper info if requested
        if args.verbose:
            for r in reasons:
                if r.strip().startswith('gripper range'):
                    print(f"  {'':43}          {r.strip()}")

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"{'─'*70}")
    print(f"\nSummary: {counts['PASS']} PASS  |  "
          f"{counts['WARNING']} WARNING  |  {counts['ERROR']} ERROR\n")

    # ── Delete ────────────────────────────────────────────────────────────────
    if args.delete:
        if not to_delete:
            print("No ERROR episodes to delete.")
        else:
            print(f"Deleting {len(to_delete)} ERROR episode(s):")
            for path in to_delete:
                print(f"  Deleting {Path(path).name}")
                os.remove(path)
            print("Done.")
    elif to_delete:
        print(f"Run with --delete to remove {len(to_delete)} ERROR episode(s).")


if __name__ == '__main__':
    main()