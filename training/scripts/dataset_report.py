#!/usr/bin/env python3
import os
import zarr
import glob
import argparse
import numpy as np

def check_episode(p):
    r = zarr.open(p, 'r')
    name = p.split('/')[-1].replace('.zarr', '')
    res = {'name': name, 'flags': [], 'stats': {}}
    
    # EE_POSE: Must move > 10cm total to be a real demo
    try:
        ee = np.array(r['obs/ee_pose'], dtype=np.float32)
        xyz = ee[:, :3]
        # Calculate total distance between start and end
        dist = np.linalg.norm(xyz[-1] - xyz[0])
        # Calculate variation
        std = np.std(xyz)
        res['stats']['pose_drift'] = dist
        
        if dist < 0.10: # Less than 10cm of movement is likely a failed track
            res['flags'].append(f'EE_STATIONARY({dist:.3f}m)')
        if np.all(ee == 0) or np.isnan(ee).any():
            res['flags'].append('EE_INVALID')
    except:
        res['flags'].append('EE_MISSING')

    # VIDEO: Pixels must actually change
    try:
        img = np.array(r['obs/video_wrist'], dtype=np.float32)
        img_std = np.std(img)
        res['stats']['img_std'] = img_std
        
        if img_std < 1.0: # If variance is this low, the camera was capped or frozen
            res['flags'].append(f'IMG_FROZEN({img_std:.2f})')
    except:
        res['flags'].append('IMG_MISSING')

    # TACTILE: Must have signals
    try:
        t = np.array(r['obs/tactile_left'])
        if np.std(t) < 1e-4:
            res['flags'].append('TACTILE_DEAD')
    except:
        pass

    return res

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, required=True)
    args = parser.parse_args()
    
    eps = sorted(glob.glob(f"data/processed/{args.task}/*.zarr"))
    print(f"\nDEEP AUDIT: {args.task}\n" + "="*100)
    print(f"{'Episode':<45} | {'Move(m)':<10} | {'ImgStd':<8} | Status")
    print("-" * 100)
    
    flagged = []
    for p in eps:
        res = check_episode(p)
        status = "REJECT" if res['flags'] else "PASS"
        move = f"{res['stats'].get('pose_drift', 0):.3f}"
        istd = f"{res['stats'].get('img_std', 0):.2f}"
        
        line = f"{res['name']:<45} | {move:<10} | {istd:<8} | {status}"
        if res['flags']:
            line += f" !! {', '.join(res['flags'])}"
            flagged.append(res['name'])
        print(line)
            
    print("=" * 100)
    print(f"TOTAL: {len(eps)} | CLEAN: {len(eps)-len(flagged)} | REJECTED: {len(flagged)}")
    
    if flagged:
        print("\n# COMMAND TO PURGE ALL REJECTS:")
        p_cmds = [f"rm -rf data/processed/{args.task}/{f}.zarr" for f in flagged]
        r_cmds = [f"rm -rf data/raw/{args.task}/{f.rsplit('_', 1)[0]}" for f in flagged]
        print(" && ".join(p_cmds + r_cmds))

if __name__ == '__main__':
    main()
