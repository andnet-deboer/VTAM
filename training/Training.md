#### Single episode, with MuJoCo validation
python3 bag_to_zarr.py --bag ~/bags/episode_001 --output dataset.zarr

#### Single episode, skip MuJoCo (faster, IK only)
python3 bag_to_zarr.py --bag ~/bags/episode_001 --output dataset.zarr --no-mujoco

#### Multi-episode (one bag per episode)
python3 bag_to_zarr.py --bag-dir ~/bags/ --output dataset.zarr