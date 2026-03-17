# VTAM Inference

Three-process pipeline across two machines.

## Architecture

| Robot | GPU Machine |
|-------|-------------|
| `robot_inference.py` | `server_inference.py` |
| `run.py --mode inference` | `ACTPolicy.select_action()` |
| ROS2: `/gripper_camera`, `/joint_states`, `/odom` | |
| obs out 4401, state out 4403, actions in 4402 | obs in 4406, actions out 4405 |

| Port | From | To | Contents |
|------|------|----|----------|
| 4401 | robot_inference.py | run.py | rgb jpeg + joint dict |
| 4403 | robot_inference.py | run.py | 9D joint positions |
| 4406 | run.py | server_inference.py | rgb jpeg + 8D state |
| 4405 | server_inference.py | run.py | 8D delta action |
| 4402 | run.py | robot_inference.py | 9D joint targets |

Ports and IPs are configured in `../config/inference.yaml`.

## Running

Start all three processes:

### 1. Robot
```bash
python3 robot_inference.py
```

### 2. Robot (or laptop on same network)
```bash
# Live policy inference
python3 run.py --mode inference

# Replay a recorded demo (validate IK before deploying a policy)
python3 run.py data/processed/<task>/episode_000.mcap --dry-run
python3 run.py data/processed/<task>/episode_000.mcap
```

### 3. GPU Machine
```bash
python3 server_inference.py
# or override the policy path from config:
python3 server_inference.py --policy-path /path/to/pretrained_model
```

## Setup (GPU machine only)

```bash
bash install.sh
source .venv/bin/activate
```

## Firewall (robot)

```bash
sudo ufw allow 4405
sudo ufw allow 4406
```

## Credits

The ZMQ-based distributed inference pipeline: streaming observations from the robot to a GPU machine over the network and sending actions back — is  adapted from [Stretch AI](https://github.com/hello-robot/stretch_ai) by Hello Robot. The two-machine architecture, socket patterns, and ROS2 bridge approach in `robot_inference.py` are a modified version of their work. See their repository for the original implementation and broader set of Stretch 3 intelligent behaviors.
