#!/usr/bin/env python3
import os
import argparse
from pathlib import Path
import numpy as np

# Import modules
from kinematics import StretchRetargeter
import process_demo  
# import bag_to_zarr # Placeholder for the final export step

def run_pipeline(task_name, episode_id, raw_path, output_root):
    """
    Orchestrates the end-to-end flow for a single demo episode.
    """
    print(f"\n>>> Processing: {task_name} | Episode: {episode_id}")
    

    # Extract raw data from MCAP and sync Tactile + Pose + Gripper to 10Hz
    try:
        synced_data = process_demo.sync_mcap_to_dict(raw_path, target_hz=10.0)
    except Exception as e:
        print(f"  [!] Sync Failed: {e}")
        return False

    # Instantiate the solver and process the trajectory
    retargeter = StretchRetargeter()
    
    # Set calibration based on the first frame's orientation
    retargeter.set_calibration(synced_data['quaternions'][0])
    
    try:
        # retarget_trajectory logic handles the resampled joint generation
        ik_results = retargeter.process_trajectory(synced_data, hz=10.0)
    except Exception as e:
        print(f"  [!] IK Processing Failed: {e}")
        return False

    # Report retargeting success metrics
    total_frames = len(synced_data['timestamps'])
    solved_frames = len(ik_results['joints'])
    reachability = (solved_frames / total_frames) * 100

    print(f"  - Frames: {total_frames}")
    print(f"  - IK Reachability: {reachability:.1f}%")

    #Set minimal threshold for a 'valid' demo
    if reachability < 90.0:
        print(f"  [REJECTED] Reachability below 90%. Check task workspace.")
        return False

    # Export to Zarr
    # Create the task-specific output directory
    zarr_out = output_root / task_name / f"episode_{episode_id}.zarr"
    zarr_out.parent.mkdir(parents=True, exist_ok=True)
    
    # bag_to_zarr.save(ik_results, synced_data, zarr_out)
    print(f"  [ACCEPTED] Saved to: {zarr_out}")
    return True

def main():
    parser = argparse.ArgumentParser(description="VTAM Retargeting Pipeline")
    parser.add_argument("--task", type=str, required=True, help="Task name (e.g., wipe_counter)")
    args = parser.parse_args()

    # Setup Paths
    root = Path(__file__).parent.parent # VTAM root
    raw_dir = root / "data" / "raw" / args.task
    processed_dir = root / "data" / "processed"

    # Discovery: Find all .mcap files in the task folder (nested structure)
    mcap_files = list(raw_dir.glob("**/*.mcap"))
    
    if not mcap_files:
        print(f"No MCAP files found in {raw_dir}")
        return

    print(f"Found {len(mcap_files)} episodes for task: {args.task}")

    stats = {"success": 0, "fail": 0}
    for mcap in mcap_files:
        # Extract episode ID from parent folder name
        episode_id = mcap.parent.name 
        
        success = run_pipeline(args.task, episode_id, mcap, processed_dir)
        
        if success: stats["success"] += 1
        else: stats["fail"] += 1

    print("\n" + "="*30)
    print(f"PIPELINE COMPLETE: {args.task}")
    print(f"  Accepted: {stats['success']}")
    print(f"  Rejected: {stats['fail']}")
    print("="*30)

if __name__ == "__main__":
    main()