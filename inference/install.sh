#!/bin/bash

# Path Setup
VTAM_DIR=~/VTAM
cd $VTAM_DIR || { echo "Error: $VTAM_DIR not found"; exit 1; }

echo "Initializing Submodules"
git submodule sync --recursive
# Allow failure for broken submodules like 'anyskin'
git submodule update --init --recursive || echo "Some submodules failed, proceeding with critical ones..."

echo "Setting up Conda (for System Headers/Binaries)"
# We need Conda to provide Python.h and FFmpeg since we can't use apt-get
source $(conda info --base)/etc/profile.d/conda.sh
conda activate base
conda install -c conda-forge ffmpeg scikit-fmm av portaudio -y

echo "Setting up UV Environment ---"
uv venv --python 3.12 --seed
source .venv/bin/activate

echo "Installing Core Machine Learning Stack"
uv pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
uv pip install pyzmq pyyaml opencv-python-headless lzfse open3d-cpu \
               transforms3d yacs h5py scikit-image trimesh pyrender \
               tqdm setproctitle pynput diffusers modern_robotics pinocchio \
               numba einops gymnasium termcolor hydra-core

echo "Linking Robot Drivers & Submodules"


uv pip install hello-robot-stretch-body
source ~/VTAM/.venv/bin/activate
# Link LeRobot and Stretch AI
uv pip install -e ./dependencies/lerobot
uv pip install -e ./dependencies/stretch_ai/src

DEX_PATH="$VTAM_DIR/dependencies/stretch_dex_teleop"
touch "$DEX_PATH/__init__.py"

echo "export PYTHONPATH=\$PYTHONPATH:$DEX_PATH" >> .venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$DEX_PATH

echo "------------------------------------------------"
echo "VTAM setup complete!"
echo "To start, run: source .venv/bin/activate"
echo "------------------------------------------------"