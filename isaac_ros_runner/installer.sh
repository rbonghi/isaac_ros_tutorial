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

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`

usage()
{
    if [ "$1" != "" ]; then
        echo "${red}$1${reset}" >&2
    fi

    local name=$(basename ${0})
    echo "nanosaur-jetson-runner installer." >&2
    echo "" >&2
    echo "Options:" >&2
    echo "   -h|--help            | This help" >&2
    echo "   -y                   | Run this script silent" >&2
}

main()
{
    local PLATFORM="$(uname -m)"
    # Check if is running on NVIDIA Jetson platform
    if [[ $PLATFORM != "aarch64" ]]; then
        echo "${red}Run this script only on ${bold}${green}NVIDIA${reset}${red} Jetson platform${reset}"
        exit 33
    fi

    local SILENT=false
	# Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            -y)
                SILENT=true
                ;;
            *)
                usage "[ERROR] Unknown option: $1" >&2
                exit 1
                ;;
        esac
            shift 1
    done

    while ! $SILENT; do
        read -p "Do you wish to install isaac-ros-jetson-runner? [Y/n] " yn
            case $yn in
                [Yy]* ) # Break and install jetson_stats 
                        break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Setup enviroment
    if [ ! -f .env ] ; then
        touch .env
        echo "GITHUB_ACTIONS_RUNNER_NAME=$HOSTNAME" >> .env
        read -p "Enter GitHub Action token: " TOKEN
        echo "GITHUB_ACTIONS_ACCESS_TOKEN=$TOKEN" >> .env
    fi

    if ! getent group docker | grep -q "\b$USER\b" ; then
        echo " - Add docker permissions to ${bold}${green}user=$USER${reset}"
        sudo usermod -aG docker $USER
    fi

    # Check if is installed docker-compose
    if ! command -v docker-compose &> /dev/null ; then
        echo " - ${bold}${green}Install docker-compose${reset}"
        sudo apt-get install -y libffi-dev python-openssl libssl-dev
        sudo -H pip3 install -U pip
        sudo pip3 install -U docker-compose
    fi
}

main $@
#EOF