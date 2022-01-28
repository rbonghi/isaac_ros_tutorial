#!/bin/bash
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

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`

# Get the entry in the dpkg status file corresponding to the provied package name
# Prepend two newlines so it can be safely added to the end of any existing
# dpkg/status file.
get_dpkg_status() {
    echo -e "\n"
    awk '/Package: '"$1"'/,/^$/' /var/lib/dpkg/status
}

usage()
{
    if [ "$1" != "" ]; then
        echo "${red}$1${reset}" >&2
    fi
    
    local name=$(basename ${0})
    echo "Isaac_ros docker builder" >&2
    echo "" >&2
    echo "Commands:" >&2
    echo "  -v                      |  Verbose. Schow extra info " >&2
    echo "  -ci                     |  Build docker without cache " >&2
    echo "  --push                  |  Push docker. Need to be logged in " >&2
    echo "  --latest                |  Tag and push latest release" >&2
    echo "  --repo REPO_NAME        |  Set repository to push " >&2
    echo "  --pull-base-image       |  Pull the base image " >&2
    echo "  --branch BRANCH_DISTRO  |  Set tag from branch " >&2
    echo "  --base-image BASE_IMAGE |  Change base image to build. Default=${bold}$BASE_IMAGE_DEFAULT${reset}" >&2
}

main()
{
    local PLATFORM="$(uname -m)"
    # Check if is running on NVIDIA Jetson platform
    if [[ $PLATFORM != "aarch64" ]]; then
        echo "${red}Run this script only on ${bold}${green}NVIDIA${reset}${red} Jetson platform${reset}"
        exit 33
    fi
    
    local REPO_NAME="rbonghi/isaac_ros_tutorial"
    local FOLDER=$1
    
    local PUSH=false
    local VERBOSE=false
    local CI_BUILD=false
    local PULL_IMAGE=false
    local BRANCH_DISTRO=""
    local LATEST=false
    # Base image
    local BASE_IMAGE=""
    # Decode all information from startup
    while [ -n "$2" ]; do
        case "$2" in
            -h|--help) # Load help
                usage
                exit 0
            ;;
            -v)
                VERBOSE=true
            ;;
            -ci)
                CI_BUILD=true
            ;;
            --repo)
                REPO_NAME=$3
                shift 1
            ;;
            --branch)
                BRANCH_DISTRO=$2
                shift 1
            ;;
            --latest)
                LATEST=true
                shift 1
            ;;
            --pull-base-image)
                PULL_IMAGE=true
            ;;
            --push)
                PUSH=true
            ;;
            --base-image)
                BASE_IMAGE=$3
                shift 1
            ;;
            *)
                usage "[ERROR] Unknown option: $2" >&2
                exit 1
            ;;
        esac
        shift 1
    done
    
    if [[ -z $FOLDER ]] ; then
        echo "${red}Please write one of the tutorial folder you want build${reset}"
        exit 33
    fi
    
    # replace all blanks
    REPO_NAME=${REPO_NAME//_/-}
    
    # Extract tag name
    local TAG=${FOLDER#*-}
    # Strip "/"
    TAG=${TAG%"/"}
    # replace all blanks
    TAG=${TAG//_/-}
    
    if ! $PUSH ; then
        # Extract Libraries info
        local DPKG_STATUS=$(get_dpkg_status cuda-cudart-10-2)$(get_dpkg_status libcufft-10-2)
        if $VERBOSE ; then
            echo "${yellow} Libraries ${reset}"
            echo "$DPKG_STATUS"
        fi
        
        local CI_OPTIONS=""
        if $CI_BUILD ; then
            # Set no-cache and pull before build
            # https://newbedev.com/what-s-the-purpose-of-docker-build-pull
            CI_OPTIONS="--no-cache"
        fi
        
        local PULL_OPTION=""
        if $PULL_IMAGE ; then
            PULL_OPTION="--pull"
        fi
        
        # move to folder
        cd $FOLDER
        if $VERBOSE ; then
            echo "${yellow}- Change folder: $FOLDER ${reset}"
            ls
        fi
        
        echo "- Build repo ${green}$REPO_NAME:$TAG${reset}"
        docker build $CI_OPTIONS $PULL_OPTION -t $REPO_NAME:$TAG --build-arg "DPKG_STATUS=$DPKG_STATUS" $BASE_IMAGE_ARG . || { echo "${red}docker build failure!${reset}"; exit 1; }
        
        if $CI_BUILD ; then
            echo "- ${bold}Prune${reset} old docker images"
            docker image prune -f
        fi
    else
        echo "- Push repo ${green}$REPO_NAME:$TAG${reset}"
        docker image push $REPO_NAME:$TAG
    fi
}

main $@
# EOF
