#!/usr/bin/env python3
# python3 test_ik.py ../../data/processed/demo/demo_20260216_020448_0.zarr
"""
Quick test script for differential_ik.py
"""
import sys
import numpy as np
import zarr
from differential_ik import TrajectoryRetargeter

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_ik.py <path_to_zarr>")
        print("Example: python3 test_ik.py ../../data/processed/demo/demo_20260216_020448_0.zarr")
        sys.exit(1)
    
    zarr_path = sys.argv[1]
    
    # Load data
    print(f"Loading: {zarr_path}")
    root = zarr.open(zarr_path, mode='r')
    ee_pose = np.array(root['obs/ee_pose'])
    
    positions = ee_pose[:, :3]
    quaternions = ee_pose[:, 3:]  # XYZW format
    timestamps = np.arange(len(positions)) / 10.0
    
    print(f"  Loaded {len(positions)} frames")
    print(f"  Position range: X[{positions[:,0].min():.3f}, {positions[:,0].max():.3f}] "
          f"Y[{positions[:,1].min():.3f}, {positions[:,1].max():.3f}] "
          f"Z[{positions[:,2].min():.3f}, {positions[:,2].max():.3f}]")
    
    # Run IK
    print("\nInitializing IK solver...")
    retargeter = TrajectoryRetargeter()
    
    print("\nRunning retargeting pipeline...")
    result = retargeter.retarget(positions, quaternions, timestamps)
    
    # Print results
    print("\n" + "="*60)
    print("RESULTS")
    print("="*60)
    print(f"Frames processed:    {len(result['joint_states'])}")
    print(f"Seed frames used:    {result['seed_frames']}")
    print(f"Fallback resets:     {result['fallback_count']}")
    print(f"\nTracking Errors:")
    print(f"  Position - Mean: {np.mean(result['pos_error'])*1000:.2f} mm, Max: {np.max(result['pos_error'])*1000:.2f} mm")
    print(f"  Orientation - Mean: {np.degrees(np.mean(result['ori_error'])):.2f} deg, Max: {np.degrees(np.max(result['ori_error'])):.2f} deg")
    
    print(f"\nBase Transform (workspace projection):")
    print(result['T_transform'])
    
    print(f"\nFirst 5 frames of joint states:")
    print(f"{'Frame':<6} {'BaseRot':<10} {'BaseTrans':<10} {'Lift':<10} {'Arm':<10}")
    for i in range(min(5, len(result['joint_states']))):
        js = result['joint_states'][i]
        print(f"{i:<6} {js[0]:>9.3f} {js[1]:>9.3f} {js[2]:>9.3f} {js[3]:>9.3f}")
    
    print(f"\nLast 5 frames of joint states:")
    start_idx = max(0, len(result['joint_states']) - 5)
    for i in range(start_idx, len(result['joint_states'])):
        js = result['joint_states'][i]
        print(f"{i:<6} {js[0]:>9.3f} {js[1]:>9.3f} {js[2]:>9.3f} {js[3]:>9.3f}")
    
    # Check for anomalies
    print("\n" + "="*60)
    print("ANOMALY DETECTION")
    print("="*60)
    
    lift_vals = result['joint_states'][:, 2]
    arm_vals = result['joint_states'][:, 3]
    
    if np.any(lift_vals < 0.0) or np.any(lift_vals > 1.1):
        print("WARNING: LIFT OUT OF BOUNDS! Range: [0.0, 1.1]")
        print(f"   Actual range: [{lift_vals.min():.3f}, {lift_vals.max():.3f}]")
    else:
        print(f"OK: Lift within bounds: [{lift_vals.min():.3f}, {lift_vals.max():.3f}]")
    
    if np.any(arm_vals < 0.0) or np.any(arm_vals > 0.52):
        print("WARNING: ARM OUT OF BOUNDS! Range: [0.0, 0.52]")
        print(f"   Actual range: [{arm_vals.min():.3f}, {arm_vals.max():.3f}]")
    else:
        print(f"OK: Arm within bounds: [{arm_vals.min():.3f}, {arm_vals.max():.3f}]")
    
    if np.max(result['pos_error']) > 0.05:
        print(f"WARNING: LARGE POSITION ERROR! Max: {np.max(result['pos_error'])*1000:.1f}mm")
    else:
        print(f"OK: Position errors acceptable")

if __name__ == '__main__':
    main()