# on robot
sudo ufw status
sudo ufw allow 4405
sudo ufw allow 4406

Training Diffusion model

CUDA_VISIBLE_DEVICES=1 python3 lerobot/scripts/train.py \
    policy=stretch_diffusion_vtam \
    env=stretch_real_vtam \
    dataset_repo_id=andnetdeboer/vtam_place_coffee_cup \
    training.batch_size=64 \
    training.num_workers=8 \
    wandb.enable=true



./scripts/run_vtam_bridge.sh

docker run -it --rm \ --name vtam_production \ --network host --ipc host --privileged --shm-size=1g \ -e ROS_DOMAIN_ID=31 \ -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \ -v /home/leogray/VTAM:/home/hello-robot/VTAM \ hellorobotinc/stretch-ai-ros2-bridge:latest \ /home/hello-robot/VTAM/inference/robot_server.sh


docker run -it --rm --name vtam_production --network host --ipc host --privileged --shm-size=1g -e ROS_DOMAIN_ID=31 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -v /home/leogray/VTAM:/home/hello-robot/VTAM hellorobotinc/stretch-ai-ros2-bridge:latest /home/hello-robot/VTAM/inference/robot_server.sh




When restarting the robot


SHEEP
export PYTHONPATH="/home/kmy2091/VTAM/dependencies/lerobot:$PYTHONPATH"

Then
python3 -m stretch.app.lfd.ros2_lfd_leader   --policy_path /home/kmy2091/VTAM/dependencies/lerobot/outputs/train/2026-02-26/02-02-41_stretch_real_diffusion_default/checkpoints/200000/pretrained_model   --policy_name diffusion



ROBOT

Terminal 1:

 docker run -it --rm --name vtam_production --network host --ipc host --privileged --shm-size=1g -e ROS_DOMAIN_ID=31 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -v /home/leogray/VTAM:/home/hello-robot/VTAM hellorobotinc/stretch-ai-ros2-bridge:latest /home/hello-robot/VTAM/inference/robot_server.sh

Terminal 2:



Terminal 3: 










 What the Training Script does (The Model's Job)
When you call python3 lerobot/scripts/train.py, the code does the following:
Temporal Sampling: It randomly picks an index $t$.
Input Gathering: It grabs the image at $t$ and the proprioception at $t$.
Action Retrieval: It grabs the pre-calculated action chunk for that index.
Diffusion Training: It adds noise to that chunk and trains the model to "denoise" it back to the relative path you calculated.
















ACT Inference 

On Robot:

docker run -it --rm --name vtam_production --network host --ipc host --privileged --shm-size=1g \
  -e ROS_DOMAIN_ID=31 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v /home/leogray/VTAM:/home/hello-robot/VTAM \
  hellorobotinc/stretch-ai-ros2-bridge:latest \
  /home/hello-robot/VTAM/inference/robot_server.sh

On Server:


export PYTHONPATH="/home/kmy2091/VTAM/dependencies/lerobot:$PYTHONPATH"

python3 inference.py \
  --checkpoint /home/kmy2091/VTAM/dependencies/lerobot/outputs/train/stretch_act_stationary/checkpoints/060000/pretrained_model \
  --robot_ip <robot_ip> \
  --n_chunks 10

START COFFEE
CKPT=~/VTAM/dependencies/lerobot/outputs/train/2026-03-11/14-50-01_stretch_real_act_stretch_act_stationary/checkpoints/last/pretrained_model 

SETUP CUP
CKPT=~/VTAM/dependencies/lerobot/outputs/train/2026-03-11/14-40-55_stretch_real_act_stretch_act_stationary/checkpoints/last/pretrained_model

python3 inference.py \ --checkpoint $CKPT \ --robot_ip 10.106.29.84 \ --n_chunks 10


source ~/miniforge3/etc/profile.d/conda.sh conda activate stretch_ai

The core issue is that workspace reprojection anchors the UMI trajectory to neutral_q in absolute robot base frame, then IK produces absolute joint positions q[t] which are stored as actions. At inference, ros2_lfd_leader sends these directly via arm_to(q[t]). This only works if the robot starts in exactly neutral_q at inference time — any deviation means every commanded position is wrong in absolute space. The policy learns a mapping f(image, state) → q_absolute where q_absolute is meaningful only relative to one fixed starting configuration, not relative to what the camera sees.
The fix is to store joint deltas Δq[t] = q[t+1] - q[t] as actions instead of absolute positions. The IK pipeline and workspace reprojection stay identical — they are still needed to get a valid joint trajectory in robot base frame. Only the final storage changes. At inference: q[t+1] = q_robot_current + Δq_predicted, where q_robot_current comes from the live robot state. The policy now learns f(image, state) → Δq, which is object-centric — the wrist camera encodes the relative gripper-to-object pose, so the policy learns "given what I see, move this much" regardless of absolute starting configuration. This is consistent with the UMI formulation and removes the requirement that the robot start at neutral_q.

mamba install -c conda-forge portaudio