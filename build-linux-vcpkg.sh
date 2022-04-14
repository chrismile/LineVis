#!/bin/bash

# BSD 2-Clause License
#
# Copyright (c) 2021-2022, Christoph Neuhauser, Felix Brendel
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set -euo pipefail

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
PROJECTPATH="$SCRIPTPATH"
pushd $SCRIPTPATH > /dev/null

debug=false
build_dir_debug=".build_debug"
build_dir_release=".build_release"
destination_dir="Shipping"

is_installed_apt() {
    local pkg_name="$1"
    if [ "$(dpkg -l | awk '/'"$pkg_name"'/ {print }'|wc -l)" -ge 1 ]; then
        return 0
    else
        return 1
    fi
}

is_installed_pacman() {
    local pkg_name="$1"
    if pacman -Qs $pkg_name > /dev/null; then
        return 0
    else
        return 1
    fi
}

if command -v apt &> /dev/null; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null; then
        sudo apt install cmake git curl pkg-config build-essential
    fi

    # Dependencies of vcpkg GLEW port.
    if ! is_installed_apt "libxmu-dev" || ! is_installed_apt "libxi-dev" || ! is_installed_apt "libgl-dev"; then
        sudo apt install libxmu-dev libxi-dev libgl-dev
    fi
elif command -v pacman &> /dev/null; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null; then
        sudo pacman -S cmake git curl pkgconf base-devel
    fi

    # Dependencies of vcpkg GLEW port.
    if ! is_installed_pacman "libgl" || ! is_installed_pacman "vulkan-devel" || ! is_installed_pacman "shaderc"; then
        sudo pacman -S libgl vulkan-devel shaderc
    fi
else
    echo "Warning: Unsupported system package manager detected." >&2
fi

if ! command -v cmake &> /dev/null; then
    echo "CMake was not found, but is required to build the program."
    exit 1
fi
if ! command -v git &> /dev/null; then
    echo "git was not found, but is required to build the program."
    exit 1
fi
if ! command -v curl &> /dev/null; then
    echo "curl was not found, but is required to build the program."
    exit 1
fi
if ! command -v pkg-config &> /dev/null; then
    echo "pkg-config was not found, but is required to build the program."
    exit 1
fi

if [ ! -d "submodules/IsosurfaceCpp/src" ]; then
    echo "------------------------"
    echo "initializing submodules "
    echo "------------------------"
    git submodule init
    git submodule update
fi

[ -d "./third_party/" ] || mkdir "./third_party/"
pushd third_party > /dev/null

if [[ ! -v VULKAN_SDK ]]; then
    echo "------------------------"
    echo "searching for Vulkan SDK"
    echo "------------------------"

    found_vulkan=false

    if lsb_release -a 2> /dev/null | grep -q 'Ubuntu'; then
        if ! compgen -G "/etc/apt/sources.list.d/lunarg-vulkan-*" > /dev/null; then
            distro_code_name=$(lsb_release -c | grep -oP "\:\s+\K\S+")
            echo "Setting up Vulkan SDK for Ubuntu $(lsb_release -r | grep -oP "\:\s+\K\S+")..."
            wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo apt-key add -
            sudo curl --silent --show-error --fail \
            https://packages.lunarg.com/vulkan/1.2.198/lunarg-vulkan-1.2.198-${distro_code_name}.list \
            --output /etc/apt/sources.list.d/lunarg-vulkan-1.2.198-${distro_code_name}.list
            sudo apt update
            sudo apt install vulkan-sdk shaderc
        fi
    fi

    if [ -d "/usr/include/vulkan" ]; then
        if ! grep -q VULKAN_SDK ~/.bashrc; then
            echo 'export VULKAN_SDK="/usr"' >> ~/.bashrc
        fi
        VULKAN_SDK="/usr"
        found_vulkan=true
    fi

    if ! $found_vulkan; then
        echo "The environment variable VULKAN_SDK is not set but is required in the installation process."
        echo "Please refer to https://vulkan.lunarg.com/sdk/home#linux for instructions on how to install the Vulkan SDK."
        exit 1
    fi
fi

if [ ! -d "./vcpkg" ]; then
    echo "------------------------"
    echo "   fetching vcpkg       "
    echo "------------------------"
    if [[ ! -v VULKAN_SDK ]]; then
        echo "The environment variable VULKAN_SDK is not set but is required in the installation process."
        exit 1
    fi
    git clone --depth 1 https://github.com/Microsoft/vcpkg.git
    vcpkg/bootstrap-vcpkg.sh -disableMetrics
    vcpkg/vcpkg install
fi

if [ ! -d "./sgl" ]; then
    echo "------------------------"
    echo "     fetching sgl       "
    echo "------------------------"
    git clone --depth 1 https://github.com/chrismile/sgl.git
fi

if [ ! -d "./sgl/install" ]; then
    echo "------------------------"
    echo "     building sgl       "
    echo "------------------------"

    pushd "./sgl" >/dev/null
    mkdir -p .build_debug
    mkdir -p .build_release

    pushd "$build_dir_debug" >/dev/null
    cmake .. \
         -DCMAKE_BUILD_TYPE=Debug \
         -DCMAKE_TOOLCHAIN_FILE="../../vcpkg/scripts/buildsystems/vcpkg.cmake" \
         -DCMAKE_INSTALL_PREFIX="../install"
    popd >/dev/null

    pushd $build_dir_release >/dev/null
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="../../vcpkg/scripts/buildsystems/vcpkg.cmake" \
        -DCMAKE_INSTALL_PREFIX="../install"
    popd >/dev/null

    cmake --build $build_dir_debug --parallel
    cmake --build $build_dir_debug --target install

    cmake --build $build_dir_release --parallel
    cmake --build $build_dir_release --target install

    popd >/dev/null
fi

popd >/dev/null # back to project root

if [ $debug = true ] ; then
    echo "------------------------"
    echo "  building in debug     "
    echo "------------------------"

    cmake_config="Debug"
    build_dir=$build_dir_debug
else
    echo "------------------------"
    echo "  building in release   "
    echo "------------------------"

    cmake_config="Release"
    build_dir=$build_dir_release
fi
mkdir -p $build_dir

echo "------------------------"
echo "      generating        "
echo "------------------------"
pushd $build_dir >/dev/null
cmake -DCMAKE_TOOLCHAIN_FILE="$PROJECTPATH/third_party/vcpkg/scripts/buildsystems/vcpkg.cmake" \
      -DPYTHONHOME="./python3" \
      -DCMAKE_BUILD_TYPE=$cmake_config \
      -Dsgl_DIR="$PROJECTPATH/third_party/sgl/install/lib/cmake/sgl/" ..
Python3_VERSION=$(cat pythonversion.txt)
popd >/dev/null

echo "------------------------"
echo "      compiling         "
echo "------------------------"
cmake --build $build_dir --parallel

echo "------------------------"
echo "   copying new files    "
echo "------------------------"

[ -d $destination_dir ]             || mkdir $destination_dir
[ -d $destination_dir/python3 ]     || mkdir $destination_dir/python3
[ -d $destination_dir/python3/lib ] || mkdir $destination_dir/python3/lib

rsync -a "vcpkg_installed/$(ls --ignore=vcpkg vcpkg_installed)/lib/$Python3_VERSION" $destination_dir/python3/lib
#rsync -a "$(eval echo "vcpkg_installed/$(ls --ignore=vcpkg vcpkg_installed)/lib/python*")" $destination_dir/python3/lib
rsync -a $build_dir/LineVis $destination_dir

echo ""
echo "All done!"


pushd $build_dir >/dev/null

if [[ -z "${LD_LIBRARY_PATH+x}" ]]; then
    export LD_LIBRARY_PATH="${PROJECTPATH}/third_party/sgl/install/lib"
elif [[ ! "${LD_LIBRARY_PATH}" == *"${PROJECTPATH}/third_party/sgl/install/lib"* ]]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${PROJECTPATH}/third_party/sgl/install/lib"
fi
./LineVis
