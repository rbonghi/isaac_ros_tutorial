#!/bin/bash
# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
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

# Get the entry in the dpkg status file corresponding to the provied package name
# Prepend two newlines so it can be safely added to the end of any existing
# dpkg/status file.
get_dpkg_status() {
    echo -e "\n"
    awk '/Package: '"$1"'/,/^$/' /var/lib/dpkg/status
}

main()
{
    local PLATFORM="$(uname -m)"
    # Check if is running on NVIDIA Jetson platform
    if [[ $PLATFORM != "aarch64" ]]; then
        echo "Run this script only on NVIDIA Jetson platform"
        exit 33
    fi

    local REPO_NAME="rbonghi/isaac_ros_tutorial"
    local FOLDER=$1

    if [[ -z $FOLDER ]] ; then
        echo "FOLDER empty"
        exit 33
    fi

    local CI_OPTIONS=""
    if $CI_BUILD ; then
        # Set no-cache and pull before build
        # https://newbedev.com/what-s-the-purpose-of-docker-build-pull
        CI_OPTIONS="--no-cache --pull"
    fi

    # Extract Libraries info
    local DPKG_STATUS=$(get_dpkg_status cuda-cudart-10-2)$(get_dpkg_status libcufft-10-2)

    # Extract tag name
    local TAG=${FOLDER#*-}
    echo "build docker $REPO_NAME:$TAG"

    # Check folders
    cd $FOLDER
    ls

    #docker build $CI_OPTIONS -t $REPO_NAME:$TAG --build-arg "DPKG_STATUS=$DPKG_STATUS" .
}

main $@
# EOF
