
<div align="center">

# VTAM
### Visuo-Tactile Assistive Manipulation

<img width="317" height="402" alt="UMI (1)" src="https://github.com/user-attachments/assets/8415a7d1-ee2e-4f89-b267-54d740070a63" />

</div>

A UMI-style imitation learning platform for the Hello Robot Stretch 3. VTAM records teleoperated demonstrations with synchronized RGB, joint, and tactile ([eFlesh](https://github.com/notvenky/eFlesh)) sensor data, processes them into training datasets, and deploys learned policies via a distributed ZMQ inference pipeline.

This project builds on [UMI](https://umi-gripper.github.io),  [stretch_ai](https://tonyzhaozh.github.io/aloha/), and [eFlesh](https://github.com/notvenky/eFlesh). See [Citations](#citations) below.

---

## Setup

### Install UV
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
```

### Environment
```bash
cd ~/VTAM
uv sync
source .venv/bin/activate
uv pip install -e dependencies/diffusion_policy --config-setting editable_mode=compat
```

### LeRobot (stretch-act branch)
```bash
cd dependencies/lerobot
git switch stretch-act
uv pip install av --no-build-isolation
uv pip install -e . --no-deps
```

---

## Pipeline

### 1. Record
```bash
ros2 launch vtam_core vtam.launch.py
ros2 service call /start_session std_srvs/srv/SetBool "{data: true}"
ros2 service call /record_demo std_srvs/srv/SetBool "{data: true}"
```

### 2. Chunk
```bash
python3 training/scripts/bag_chunker.py <task>
```

### 3. Process
```bash
python3 training/scripts/process_demo.py <task> \
    --fps 10 --force --tactile \
    --repo-id <hf_user>/<dataset_name> --push-to-hub
```

### 4. Train
```bash
cd dependencies/lerobot
CUDA_VISIBLE_DEVICES=1 python3 lerobot/scripts/train.py \
    policy=stretch_act_real_vtam_rel \
    env=stretch_real_vtam \
    dataset_repo_id=<hf_user>/<dataset_name> \
    training.batch_size=256 \
    training.image_transforms.enable=true \
    wandb.enable=true
```

### 5. Infer

**Robot:**
```bash
docker run -it --rm --name vtam_production \
  --network host --ipc host --privileged --shm-size=1g \
  -e ROS_DOMAIN_ID=31 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v /home/leogray/VTAM:/home/hello-robot/VTAM \
  hellorobotinc/stretch-ai-ros2-bridge:latest \
  /home/hello-robot/VTAM/inference/robot_server.sh
```

**GPU machine:**
```bash
export PYTHONPATH="/home/kmy2091/VTAM/dependencies/lerobot:$PYTHONPATH"
python3 inference/vtam_server_inference.py --policy-path /path/to/pretrained_model
```

---

## Hardware Notes

- Tactile firmware: `firmware/` (QT Py microcontroller, eFlesh-based)
- Check serial devices: `ls -l /dev/serial/by-path/`
- Ports/IPs: `config/inference.yaml`

---

## Citations

If you use this project, please also cite the works it builds on:

**eFlesh**
```bibtex
@article{pattabiraman2025eflesh,
    title={eFlesh: Highly customizable Magnetic Touch Sensing using Cut-Cell Microstructures},
    author={Venkatesh Pattabiraman and Zizhou Huang and Daniele Panozzo and Denis Zorin and Lerrel Pinto and Raunaq Bhirangi},
    journal={arXiv preprint arXiv:2506.09994},
    year={2025}
}
```

**UMI**
```bibtex
@inproceedings{chi2024universal,
    title={Universal Manipulation Interface: In-The-Wild Robot Teaching Without In-The-Wild Robots},
    author={Cheng Chi and Zhenjia Xu and Chuer Pan and Eric Cousineau and Benjamin Burchfiel and Siyuan Feng and Russ Tedrake and Shuran Song},
    booktitle={Robotics: Science and Systems},
    year={2024},
    url={https://umi-gripper.github.io}
}
```

**ACT**
```bibtex
@inproceedings{zhao2023learning,
    title={Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware},
    author={Tony Z. Zhao and Vikash Kumar and Sergey Levine and Chelsea Finn},
    booktitle={Robotics: Science and Systems},
    year={2023},
    url={https://tonyzhaozh.github.io/aloha}
}
```

**Diffusion Policy**
```bibtex
@inproceedings{chi2023diffusion,
    title={Diffusion Policy: Visuomotor Policy Learning via Action Diffusion},
    author={Cheng Chi and Siyuan Feng and Yilun Du and Zhenjia Xu and Eric Cousineau and Benjamin Burchfiel and Shuran Song},
    booktitle={Robotics: Science and Systems},
    year={2023},
    url={https://diffusion-policy.cs.columbia.edu}
}
```

