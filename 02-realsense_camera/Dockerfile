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

# Jetpack 4.6
# Docker file for aarch64 based Jetson device
ARG BASE_IMAGE="dustynv/ros:foxy-ros-base-l4t-r32.6.1"
FROM ${BASE_IMAGE}
# Configuration CUDA
ARG CUDA=10.2
ARG L4T=r32.6

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
    libpcl-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Git-LFS and other packages
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
    apt-get update && apt-get install -y git-lfs software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Fix cuda info
ARG DPKG_STATUS
# Add nvidia repo/public key and install VPI libraries
RUN echo "$DPKG_STATUS" >> /var/lib/dpkg/status && \
    curl https://repo.download.nvidia.com/jetson/jetson-ota-public.asc > /etc/apt/trusted.gpg.d/jetson-ota-public.asc && \
    echo "deb https://repo.download.nvidia.com/jetson/common ${L4T} main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    apt-get update && apt-get install -y libnvvpi1 vpi1-dev && \
    rm -rf /var/lib/apt/lists/*

# Update environment
ENV LD_LIBRARY_PATH="/opt/nvidia/vpi1/lib64:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-${CUDA}/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}"

# Install pcl_conversions & sensor_msgs_py
RUN mkdir -p /opt/ros/foxy/src && \
    cd /opt/ros/foxy/src && \
    git clone --branch ros2 https://github.com/ros-perception/pcl_msgs.git && \
    git clone https://github.com/ros-perception/perception_pcl.git && \
    git clone --branch 2.2.3 https://github.com/ros2/common_interfaces.git && \
    git clone --branch foxy https://github.com/ros/diagnostics.git && \
    git clone --branch ros2 https://github.com/ros/xacro.git && \
    . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd /opt/ros/foxy && \
    rosdep install -y --ignore-src --from-paths src --rosdistro foxy && \
    colcon build --merge-install --packages-up-to pcl_conversions sensor_msgs_py diagnostic_updater xacro && \
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

################ INSTALL REALSENSE #######################

# Install Realsense
RUN apt-get update && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    apt-get install -y rsync librealsense2-utils librealsense2-dev && \
    rm -rf /var/lib/apt/lists/*

# Install ros2 realsense pkg
RUN mkdir -p /opt/ros/foxy/src && \
    cd /opt/ros/foxy/src && \
    git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git && \
    . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd /opt/ros/foxy && \
    colcon build --merge-install --packages-up-to realsense2_camera realsense2_description && \
    rm -Rf src logs build

################ BUILD & INSTALL ISAAC ROS packages ####################

# Build Isaac ROS package
ENV ISAAC_ROS_WS /opt/isaac_ros_ws
ARG ROSINSTALL=realsense_camera.rosinstall
# Copy wstool rosinstall
COPY ${ROSINSTALL} ${ROSINSTALL}

RUN apt-get update && \
    apt-get install python3-vcstool python3-pip -y && \
    mkdir -p ${ISAAC_ROS_WS}/src && \
    vcs import ${ISAAC_ROS_WS}/src < ${ROSINSTALL} && \
    rm -rf /var/lib/apt/lists/*
# Pull LFS files
RUN cd ${ISAAC_ROS_WS}/src/isaac_ros_common && git lfs pull && \
    cd ${ISAAC_ROS_WS}/src/isaac_ros_image_pipeline && git lfs pull && \
    cd ${ISAAC_ROS_WS}/src/isaac_ros_visual_odometry && git lfs pull

# Change workdir
WORKDIR $ISAAC_ROS_WS

# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

################ PROJECT PKG ###############################

# Download and build nanosaur_perception
ENV ROS_WS /opt/ros_ws
RUN mkdir -p ${ROS_WS}/src

ADD isaac_ros_realsense $ROS_WS/src/isaac_ros_realsense

# Change workdir
WORKDIR $ROS_WS

# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    . $ISAAC_ROS_WS/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# Restore using the default Foxy DDS middleware: FastRTPS
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

################ Final enviroment setup ####################

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# https://docs.docker.com/engine/reference/builder/#stopsignal
# https://hynek.me/articles/docker-signals/
STOPSIGNAL SIGINT
# run ros package launch file
CMD ["ros2", "launch", "isaac_ros_realsense", "isaac_ros_realsense.launch.py"]