FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    git-lfs \
    curl \
    build-essential \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Install miniforge/mamba inside the container
RUN curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh" \
    && bash Miniforge3-Linux-x86_64.sh -b -p /opt/miniforge \
    && rm Miniforge3-Linux-x86_64.sh

ENV PATH="/opt/miniforge/bin:$PATH"

# Initialize mamba
RUN mamba init bash

# Clone and install stretch-ai
WORKDIR /app
RUN git lfs install \
    && git clone https://github.com/hello-robot/stretch_ai.git --recursive

WORKDIR /app/stretch_ai

# Run install script
RUN ./install.sh --cuda=12.4 --no-version

# Set up entrypoint to activate conda environment
RUN echo "mamba activate stretch_ai" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/miniforge/etc/profile.d/conda.sh && mamba activate stretch_ai && exec bash"]