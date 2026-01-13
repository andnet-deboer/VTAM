# Start with NVIDIA CUDA (for Training/AI)
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install Basic Tools & Setup ROS 2 Sources
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    git-lfs \
    build-essential \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble 
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-foxglove-bridge \
    ros-humble-cv-bridge \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Mamba
RUN curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh" \
    && bash Miniforge3-Linux-x86_64.sh -b -p /opt/miniforge \
    && rm Miniforge3-Linux-x86_64.sh
ENV PATH="/opt/miniforge/bin:$PATH"
RUN mamba init bash

# Install Stretch_AI
# Copy fork from 'dependencies/' folder.
WORKDIR /app
COPY dependencies/stretch_ai /app/stretch_ai

WORKDIR /app/stretch_ai
# Run their heavy install script
RUN ./install.sh --cuda=12.4 --no-version

# Install VTAM Core
# We copy your ROS workspace and install Python dependencies
WORKDIR /root/vtam_ws
COPY ros_ws /root/vtam_ws
# Install VTAM in "editable" mode to accept live tweaks
RUN pip install -e src/vtam_core

# Entrypoint Setup
# Activates Conda and sources ROS 2
RUN echo "#!/bin/bash" > /entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /entrypoint.sh \
    && echo "source /opt/miniforge/etc/profile.d/conda.sh" >> /entrypoint.sh \
    && echo "mamba activate stretch_ai" >> /entrypoint.sh \
    && echo "exec \"\$@\"" >> /entrypoint.sh \
    && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]