FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/London

# Install system dependencies
RUN apt-get update && \
	ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    wget \
    sudo \
    cargo \
    clang \
    clang-tidy \
	software-properties-common \
    apt-transport-https

# Install ROS 2 Eloquent
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get install -y \
    ros-eloquent-desktop

# Install CUDA 10.2 (for Jetson Nano compatibility)
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin && \
    mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub && \
    add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /" && \
    apt-get update && \
    apt-get install -y cuda-toolkit-10-2

# Install OpenCV with CUDA support
RUN apt-get install -y libopencv-dev

# Set up environment variables
ENV CUDA_HOME /usr/local/cuda-10.2
ENV PATH ${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH ${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# Source ROS 2 setup.bash
RUN echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc

# Install run-clang-tidy
RUN git clone https://github.com/lmapii/run-clang-tidy.git && \
    cd run-clang-tidy && \
    cargo build --release --target-dir /tmp/target && \
    mv /tmp/target/release/run-clang-tidy /usr/local/bin/run-clang-tidy-rust

# Set up workspace
WORKDIR /workspace
