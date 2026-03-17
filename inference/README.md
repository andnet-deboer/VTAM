# VTAM Inference

Two-process pipeline: the robot streams observations over ZMQ; the GPU machine runs the policy and sends back actions.

## Architecture

| Robot | GPU Machine |
|-------|-------------|
| `vtam_robot_node.py` | `vtam_server_inference.py` |
| `/gripper_camera`  obs (4401)  | `ACTPolicy.select_action()` |
| `/joint_states`  state (4403)  | 8D EE pose |
|  actions (4402)  |  joint cmds |

Ports and IPs are configured in `../config/inference.yaml`.

## Running

### Robot 

```bash
python3 robot_inference.py
```

### GPU Machine

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
