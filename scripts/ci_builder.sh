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


main()
{
    local local_folder=$(pwd)
    local json_file="matrix.json"
    # Build a list of folder available
    declare -a dirs
    local i=1
    for d in $local_folder/*/
    do
        # Exlude scripts folder
        if [[ $d != *"scripts"* ]] ; then
            dirs[i++]="${d%/}"
        fi
    done

    echo "There are ${green}${#dirs[@]} tutorials${reset} in the current path"

    # https://github.blog/changelog/2020-04-15-github-actions-new-workflow-features/
    # https://docs.github.com/en/actions/learn-github-actions/expressions#fromjson
    echo "{\"include\":[" > $json_file
    # Worker to build all docker file
    for((i=1;i<=${#dirs[@]};i++))
    do
        local folder=${dirs[i]}
        echo "- ${bold}$i${reset} tutorial: ${green}$folder${reset}"
        # Run build docker
        # bash $local_folder/scripts/build_docker.sh $folder

        local separator=","
        if [ $i = ${#dirs[@]} ] ; then
            separator="]"
        fi
        echo "{\"project\": \"$(basename $folder)\"}$separator" >> $json_file
    done
    echo "}" >> $json_file

}

main $@
# EOF