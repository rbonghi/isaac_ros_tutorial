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

# Start from Isaac ROS base
ARG BASE_IMAGE="rbonghi/isaac-ros-tutorial:isaac-ros-base"
FROM ${BASE_IMAGE}
# ZED version
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=6

################ INSTALL ZED Camera drivers #################################

#This environment variable is needed to use the streaming features on Jetson inside a container
# install options: silent, runtime_only, skip_tools, skip_od_module, skip_python
ENV LOGNAME root
RUN apt-get update -y && \
    apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https build-essential cmake -y && \
    echo "# R32 (release), REVISION: ${L4T_MINOR_VERSION}" > /etc/nv_tegra_release && \
    wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/jp${JETPACK_MAJOR}${JETPACK_MINOR}/jetsons && \
    chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools skip_python skip_od_module && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux_JP.run && \
    rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# Download and build zed ROS2
RUN mkdir -p /opt/ros/foxy/src && \
    cd /opt/ros/foxy/src && \
    git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd /opt/ros/foxy && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -Rf src logs build

################ BUILD & INSTALL ISAAC ROS packages ####################

# Build Isaac ROS package
ENV ISAAC_ROS_WS /opt/isaac_ros_ws
ARG ROSINSTALL=zed_camera.rosinstall
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

# Download and build ros workspace
ENV ROS_WS /opt/ros_ws
RUN mkdir -p ${ROS_WS}/src

# Download and build ros workspace
ADD isaac_ros_zed $ROS_WS/src/isaac_ros_zed

# Change workdir
WORKDIR $ROS_WS

# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    . $ISAAC_ROS_WS/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

################ Final enviroment setup ####################

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run ros package launch file
CMD ["ros2", "launch", "isaac_ros_zed", "isaac_ros_zed.launch.py"]