#!/usr/bin/env python3
"""
retarget_demo.py Apply differential IK to one or all .zarr demos.

Reads obs/ee_pose, writes obs/joint_states + error metadata.

Usage:
    # Single demo
    python3 retarget_demo.py --zarr data/processed/demo/demo_20260216_020448_0.zarr

    # All demos in a task
    python3 retarget_demo.py --task demo

    # All demos in a task with custom URDF
    python3 retarget_demo.py --task demo --urdf path/to/stretch.urdf
"""

import argparse
import os
import sys
import glob
import time
import numpy as np
import zarr

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(SCRIPT_DIR, '..', 'utils')
sys.path.insert(0, UTILS_DIR)

from differential_ik import TrajectoryRetargeter


def find_data_root():
    """Walk up from this script to find the VTAM data directory."""
    search = SCRIPT_DIR
    for _ in range(6):
        candidate = os.path.join(search, 'data', 'processed')
        if os.path.exists(candidate):
            return candidate
        search = os.path.dirname(search)
    raise FileNotFoundError("Could not find data/processed directory.")


def retarget_single(zarr_path, retargeter):
    """
    Retarget one zarr demo. Returns True on success, False on failure.
    
    Importable for use in other scripts or record_node's background queue.
    """
    root = zarr.open(zarr_path, mode='a')
    
    if 'obs/ee_pose' not in root:
        print(f"  SKIP: No obs/ee_pose in {os.path.basename(zarr_path)}")
        return False
    
    ee_pose = np.array(root['obs/ee_pose'])
    if ee_pose.ndim != 2 or ee_pose.shape[1] != 7:
        print(f"  SKIP: Unexpected ee_pose shape {ee_pose.shape}")
        return False
    
    positions = ee_pose[:, :3]
    quaternions = ee_pose[:, 3:]
    timestamps = np.arange(len(ee_pose)) / 10.0  # Assume 10 Hz sync
    
    try:
        result = retargeter.retarget(positions, quaternions, timestamps)
    except (ValueError, RuntimeError) as e:
        print(f"  FAIL: IK failed — {e}")
        return False
    
    # Write results
    if 'obs/joint_states' in root:
        del root['obs/joint_states']
    root.create_dataset(
        'obs/joint_states',
        data=result['joint_states'],
        chunks=(100, 6),
        overwrite=True
    )
    
    if 'meta' not in root:
        root.create_group('meta')
    meta = root['meta']
    for key in ['pos_error', 'ori_error']:
        if key in meta:
            del meta[key]
        meta.create_dataset(key, data=result[key], overwrite=True)
    
    meta.attrs['ik_seed_frames'] = result['seed_frames']
    meta.attrs['ik_fallback_count'] = result['fallback_count']
    meta.attrs['ik_joint_names'] = result['joint_names']
    
    mean_err = np.mean(result['pos_error']) * 1000
    max_err = np.max(result['pos_error']) * 1000
    fallbacks = result['fallback_count']
    status = "OK" if max_err < 20 and fallbacks == 0 else "WARN"
    
    print(f"  [{status}] {os.path.basename(zarr_path)}: "
          f"mean={mean_err:.1f}mm max={max_err:.1f}mm fallbacks={fallbacks}")
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Batch IK retargeting for .zarr demos')
    parser.add_argument('--zarr', type=str, default=None, help='Single .zarr path')
    parser.add_argument('--task', type=str, default=None, help='Task name (processes all zarrs)')
    parser.add_argument('--urdf', type=str, default=None, help='Path to Stretch URDF')
    args = parser.parse_args()
    
    if args.zarr is None and args.task is None:
        parser.error("Provide either --zarr or --task")
    
    retargeter = TrajectoryRetargeter(urdf_path=args.urdf)
    
    if args.zarr:
        zarr_paths = [args.zarr]
    else:
        data_root = find_data_root()
        task_dir = os.path.join(data_root, args.task)
        if not os.path.exists(task_dir):
            print(f"ERROR: Task directory not found: {task_dir}")
            sys.exit(1)
        zarr_paths = sorted(glob.glob(os.path.join(task_dir, '*.zarr')))
        print(f"Found {len(zarr_paths)} demos in {args.task}/\n")
    
    t_start = time.time()
    success = 0
    for path in zarr_paths:
        if retarget_single(path, retargeter):
            success += 1
    
    elapsed = time.time() - t_start
    print(f"\nDone: {success}/{len(zarr_paths)} demos retargeted in {elapsed:.1f}s")


if __name__ == '__main__':
    main()