# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# Jetpack 4.6.1
# Docker file for aarch64 based Jetson device
FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1
# L4T variables
ENV L4T=32.7
ENV L4T_MINOR_VERSION=7.1
# Configuration CUDA
ENV CUDA=10.2

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install OpenCV dependencies
RUN apt-get update && apt-get install -y \
    libavformat-dev \
    libjpeg-dev \
    libopenjp2-7-dev \
    libpng-dev \
    libpq-dev \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libtiff-dev \
    pkg-config \
    yasm && \
    rm -rf /var/lib/apt/lists/*

# Install additional packages needed for ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-distutils \
    libboost-all-dev \
    libboost-dev \
    libpcl-dev \
    libode-dev \
    lcov \
    python3-zmq \
    libxaw7-dev \
    libgraphicsmagick++1-dev \
    graphicsmagick-libmagick-dev-compat \
    libceres-dev \
    libsuitesparse-dev \
    libncurses5-dev \
    libassimp-dev \
    libyaml-cpp-dev \
    libpcap-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Git-LFS and other packages
RUN apt-get update && apt-get install -y \
    git-lfs \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Fix cuda info
ARG DPKG_STATUS
# Add nvidia repo/public key and install VPI libraries
RUN echo "$DPKG_STATUS" >> /var/lib/dpkg/status && \
    curl https://repo.download.nvidia.com/jetson/jetson-ota-public.asc > /etc/apt/trusted.gpg.d/jetson-ota-public.asc && \
    echo "deb https://repo.download.nvidia.com/jetson/common r${L4T} main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    apt-get update && apt-get install -y libnvvpi1 vpi1-dev && \
    rm -rf /var/lib/apt/lists/*

# Update environment
ENV LD_LIBRARY_PATH="/opt/nvidia/vpi1/lib64:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-${CUDA}/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}"

# Install pcl_conversions & sensor_msgs_py
ARG ROSINSTALL=isaac_ros_base.rosinstall
# Copy wstool rosinstall
COPY ${ROSINSTALL} ${ROSINSTALL}

# Fix image transport to v3.0.0
# https://github.com/stereolabs/zed-ros2-wrapper/issues/66
RUN apt-get update && \
    apt-get install python3-vcstool python3-pip -y && \
    mkdir -p ${ROS_ROOT}/src && \
    vcs import ${ROS_ROOT}/src < ${ROSINSTALL} && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd ${ROS_ROOT} && \
    rosdep install -y --ignore-src --from-paths src --rosdistro foxy \
    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    colcon build --merge-install --packages-up-to pcl_conversions sensor_msgs_py diagnostic_updater xacro \
    camera_calibration_parsers image_transport image_common camera_info_manager \
    image_publisher depth_image_proc image_proc image_view image_rotate stereo_image_proc compressed_image_transport compressed_depth_image_transport && \
    rm -Rf src logs build

# Install gcc8 for cross-compiled binaries from Ubuntu 20.04
RUN apt-get update && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    apt-get install -y gcc-8 g++-8 libstdc++6 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8 && \
    rm -rf /usr/bin/aarch64-linux-gnu-gcc /usr/bin/aarch64-linux-gnu-g++ \
        /usr/bin/aarch64-linux-gnu-g++-7 /usr/bin/aarch64-linux-gnu-gcc-7 && \
    update-alternatives --install /usr/bin/aarch64-linux-gnu-gcc aarch64-linux-gnu-gcc \
        /usr/bin/gcc-8 8 && \
    update-alternatives --install /usr/bin/aarch64-linux-gnu-g++ aarch64-linux-gnu-g++ \
        /usr/bin/g++-8 8 && \
    rm -rf /var/lib/apt/lists/*

################ Final enviroment setup ####################

# Restore using the default Foxy DDS middleware: FastRTPS
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_ROOT/install/setup.bash"' \
      /ros_entrypoint.sh

# https://docs.docker.com/engine/reference/builder/#stopsignal
# https://hynek.me/articles/docker-signals/
STOPSIGNAL SIGINT
