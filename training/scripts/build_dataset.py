#!/usr/bin/env python3
import os, zarr, glob, argparse, numpy as np, json

def build_dataset(task, chunk_size=10):
    eps = sorted(glob.glob(f"data/processed/{task}/*.zarr"))
    print(f"Found {len(eps)} episodes for task: {task}")

    # Pass 1: collect raw data
    raw = {'ee': [], 'grip': [], 'tl': [], 'tr': [], 'imgs': [], 'act': []}
    for ep_path in eps:
        r = zarr.open(ep_path, 'r')
        raw['ee'].append(  np.array(r['obs/ee_pose'],          dtype=np.float32))
        raw['grip'].append(np.array(r['obs/gripper_position'], dtype=np.float32))
        raw['tl'].append(  np.array(r['obs/tactile_left'],     dtype=np.float32))
        raw['tr'].append(  np.array(r['obs/tactile_right'],    dtype=np.float32))
        raw['imgs'].append(np.array(r['obs/video_wrist'],      dtype=np.uint8))

    # Compute normalization stats from full dataset
    def stats(arrays):
        x = np.concatenate(arrays, axis=0)
        mean = x.mean(axis=0)
        std  = x.std(axis=0) + 1e-8
        return mean, std

    ee_mean,   ee_std   = stats(raw['ee'])
    grip_mean, grip_std = stats(raw['grip'])
    tl_mean,   tl_std   = stats(raw['tl'])
    tr_mean,   tr_std   = stats(raw['tr'])

    print(f"EE pose std range:     [{ee_std.min():.4f}, {ee_std.max():.4f}]")
    print(f"Tactile L std range:   [{tl_std.min():.4f}, {tl_std.max():.4f}]")
    print(f"Tactile R std range:   [{tr_std.min():.4f}, {tr_std.max():.4f}]")

    # Save stats for inference-time unnormalization
    norm_stats = {
        'ee_mean': ee_mean.tolist(), 'ee_std': ee_std.tolist(),
        'grip_mean': grip_mean.tolist(), 'grip_std': grip_std.tolist(),
        'tl_mean': tl_mean.tolist(), 'tl_std': tl_std.tolist(),
        'tr_mean': tr_mean.tolist(), 'tr_std': tr_std.tolist(),
    }
    os.makedirs("data/training", exist_ok=True)
    with open(f"data/training/{task}_norm_stats.json", 'w') as f:
        json.dump(norm_stats, f, indent=2)
    print(f"Norm stats saved to data/training/{task}_norm_stats.json")

    # Pass 2: build normalized arrays
    all_images, all_agent_pos, all_actions = [], [], []
    episode_ends = []
    total_steps = 0

    for i, ep_path in enumerate(eps):
        T = raw['ee'][i].shape[0]
        ee   = (raw['ee'][i]   - ee_mean)   / ee_std
        grip = (raw['grip'][i] - grip_mean) / grip_std
        tl   = (raw['tl'][i]  - tl_mean)   / tl_std
        tr   = (raw['tr'][i]  - tr_mean)   / tr_std

        agent_pos = np.concatenate([ee, grip, tl, tr], axis=-1)  # (T, 38)

        # Actions: normalized ee + grip only (what policy predicts)
        raw_action = np.concatenate([ee, grip], axis=-1)  # (T, 8)
        actions = np.zeros((T, chunk_size, 8), dtype=np.float32)
        for t in range(T):
            end = min(t + chunk_size, T)
            actual = raw_action[t:end]
            pad = chunk_size - len(actual)
            if pad > 0:
                actual = np.concatenate([actual, np.tile(actual[-1:], (pad, 1))], axis=0)
            actions[t] = actual

        all_images.append(raw['imgs'][i])
        all_agent_pos.append(agent_pos)
        all_actions.append(actions)
        total_steps += T
        episode_ends.append(total_steps)
        print(f"  {os.path.basename(ep_path)}: T={T}")

    all_images    = np.concatenate(all_images,    axis=0)
    all_agent_pos = np.concatenate(all_agent_pos, axis=0)
    all_actions   = np.concatenate(all_actions,   axis=0)
    episode_ends  = np.array(episode_ends, dtype=np.int64)

    print(f"\nTotal steps: {total_steps}")
    print(f"Agent pos range (normalized): [{all_agent_pos.min():.3f}, {all_agent_pos.max():.3f}]")
    print(f"Action range (normalized):    [{all_actions.min():.3f}, {all_actions.max():.3f}]")

    out_path = f"data/training/{task}.zarr"
    root = zarr.open(out_path, mode='w')
    root.create_dataset('data/image',       data=all_images,    chunks=(1, 96, 96, 3),        dtype='uint8')
    root.create_dataset('data/agent_pos',   data=all_agent_pos, chunks=(1000, 38),             dtype='float32')
    root.create_dataset('data/action',      data=all_actions,   chunks=(1000, chunk_size, 8),  dtype='float32')
    root.create_dataset('meta/episode_ends',data=episode_ends,  dtype='int64')

    print(f"\nDataset written to {out_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', default='pickup_cup')
    parser.add_argument('--chunk_size', type=int, default=10)
    args = parser.parse_args()
    build_dataset(args.task, args.chunk_size)
