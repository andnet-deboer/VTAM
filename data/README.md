# Data

All data is gitignored. Choose one of two storage strategies — or use both.

---

## Storage Options

### Option A — Local Only (rsync)

Keep all data on your development machine. Sync raw bags directly from the robot:

```bash
rsync -av --progress <user>@<robot-ip>:~/VTAM/data/raw/<task>/ ~/VTAM/data/raw/<task>/
```

Process locally and train from `data/lerobot/`.

---

### Option B — HuggingFace Hub

Upload processed datasets to the HuggingFace Hub for storage, sharing, and training from any machine:

```bash
uvx hf auth login

python3 training/scripts/process_demo.py <task> \
    --fps 10 --force \
    --repo-id <hf_user>/<dataset_name> \
    --push-to-hub
```

The dataset is pushed automatically after processing. Training then pulls directly from the Hub via `dataset_repo_id=<hf_user>/<dataset_name>`.

---

## Local Layout

```
data/
  raw/                          # Session MCAP bags recorded on the robot
    <task>/
      session_<timestamp>/
        *.mcap
  chunked/                      # Per-episode bags (output of bag_chunker.py)
    <task>/
      episode_000.mcap
      episode_001.mcap
      ...
  processed/                    # Episodes staged for process_demo.py
    <task>/
      *.mcap
  lerobot/                      # HuggingFace datasets (output of process_demo.py)
    <task>/
      data/
      meta_data/
      videos/
```

See `training/README.md` for the full pipeline.

