#!/bin/bash

# BSD 2-Clause License
#
# Copyright (c) 2021-2023, Christoph Neuhauser
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

# Conda crashes with "set -euo pipefail".
set -eo pipefail

scriptpath="$( cd "$(dirname "$0")" ; pwd -P )"
projectpath="$scriptpath"
pushd $scriptpath > /dev/null

if [[ "$(uname -s)" =~ ^MSYS_NT.* ]] || [[ "$(uname -s)" =~ ^MINGW.* ]]; then
    use_msys=true
else
    use_msys=false
fi
if [[ "$(uname -s)" =~ ^Darwin.* ]]; then
    use_macos=true
else
    use_macos=false
fi
os_arch="$(uname -m)"

run_program=true
debug=false
glibcxx_debug=false
build_dir_debug=".build_debug"
build_dir_release=".build_release"
use_vcpkg=false
use_conda=false
conda_env_name="linevis"
link_dynamic=false
use_custom_vcpkg_triplet=false
custom_glslang=false
if [ $use_msys = false ] && command -v pacman &> /dev/null; then
    is_embree_installed=true
    is_ospray_installed=true
else
    is_embree_installed=false
    is_ospray_installed=false
fi

# Process command line arguments.
for ((i=1;i<=$#;i++));
do
    if [ ${!i} = "--do-not-run" ]; then
        run_program=false
    elif [ ${!i} = "--debug" ] || [ ${!i} = "debug" ]; then
        debug=true
    elif [ ${!i} = "--glibcxx-debug" ]; then
        glibcxx_debug=true
    elif [ ${!i} = "--vcpkg" ] || [ ${!i} = "--use-vcpkg" ]; then
        use_vcpkg=true
    elif [ ${!i} = "--conda" ] || [ ${!i} = "--use-conda" ]; then
        use_conda=true
    elif [ ${!i} = "--conda-env-name" ]; then
        ((i++))
        conda_env_name=${!i}
    elif [ ${!i} = "--link-static" ]; then
        link_dynamic=false
    elif [ ${!i} = "--link-dynamic" ]; then
        link_dynamic=true
    elif [ ${!i} = "--vcpkg-triplet" ]; then
        ((i++))
        vcpkg_triplet=${!i}
        use_custom_vcpkg_triplet=true
    elif [ ${!i} = "--custom-glslang" ]; then
        custom_glslang=true
    fi
done

if [ $debug = true ]; then
    cmake_config="Debug"
    build_dir=$build_dir_debug
else
    cmake_config="Release"
    build_dir=$build_dir_release
fi
destination_dir="Shipping"
if $use_macos; then
    binaries_dest_dir="$destination_dir/LineVis.app/Contents/MacOS"
    if ! command -v brew &> /dev/null; then
        if [ ! -d "/opt/homebrew/bin" ]; then
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        fi
        if [ -d "/opt/homebrew/bin" ]; then
            #echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> /Users/$USER/.zprofile
            eval "$(/opt/homebrew/bin/brew shellenv)"
        fi
    fi
fi

params_link=()
params_vcpkg=()
if [ $use_custom_vcpkg_triplet = true ]; then
    params_link+=(-DVCPKG_TARGET_TRIPLET=$vcpkg_triplet)
elif [ $use_vcpkg = true ] && [ $use_macos = false ] && [ $link_dynamic = true ]; then
    params_link+=(-DVCPKG_TARGET_TRIPLET=x64-linux-dynamic)
fi
if [ $use_vcpkg = true ] && [ $use_macos = false ]; then
    params_vcpkg+=(-DUSE_STATIC_STD_LIBRARIES=On)
fi
if [ $use_vcpkg = true ]; then
    params_vcpkg+=(-DCMAKE_TOOLCHAIN_FILE="$projectpath/third_party/vcpkg/scripts/buildsystems/vcpkg.cmake")
fi

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

is_installed_yay() {
    local pkg_name="$1"
    if yay -Ss $pkg_name > /dev/null | grep -q 'instal'; then
        return 1
    else
        return 0
    fi
}

is_installed_yum() {
    local pkg_name="$1"
    if yum list installed "$pkg_name" > /dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

is_installed_rpm() {
    local pkg_name="$1"
    if rpm -q "$pkg_name" > /dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

is_installed_brew() {
    local pkg_name="$1"
    if brew list $pkg_name > /dev/null; then
        return 0
    else
        return 1
    fi
}

# https://stackoverflow.com/questions/8063228/check-if-a-variable-exists-in-a-list-in-bash
list_contains() {
    if [[ "$1" =~ (^|[[:space:]])"$2"($|[[:space:]]) ]]; then
        return 0
    else
        return 1
    fi
}

if $use_msys && command -v pacman &> /dev/null && [ ! -d $build_dir_debug ] && [ ! -d $build_dir_release ]; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v rsync &> /dev/null \
            || ! command -v curl &> /dev/null || ! command -v wget &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        pacman --noconfirm -S --needed make git rsync curl wget mingw64/mingw-w64-x86_64-cmake \
        mingw64/mingw-w64-x86_64-gcc mingw64/mingw-w64-x86_64-gdb
    fi

    # Dependencies of sgl and the application.
    if ! is_installed_pacman "mingw-w64-x86_64-boost" || ! is_installed_pacman "mingw-w64-x86_64-glm" \
            || ! is_installed_pacman "mingw-w64-x86_64-libarchive" || ! is_installed_pacman "mingw-w64-x86_64-tinyxml2" \
            || ! is_installed_pacman "mingw-w64-x86_64-libpng" || ! is_installed_pacman "mingw-w64-x86_64-SDL2" \
            || ! is_installed_pacman "mingw-w64-x86_64-SDL2_image" || ! is_installed_pacman "mingw-w64-x86_64-glew" \
            || ! is_installed_pacman "mingw-w64-x86_64-vulkan-headers" \
            || ! is_installed_pacman "mingw-w64-x86_64-vulkan-loader" \
            || ! is_installed_pacman "mingw-w64-x86_64-vulkan-validation-layers" \
            || ! is_installed_pacman "mingw-w64-x86_64-shaderc" \
            || ! is_installed_pacman "mingw-w64-x86_64-opencl-headers" \
            || ! is_installed_pacman "mingw-w64-x86_64-opencl-icd" || ! is_installed_pacman "mingw-w64-x86_64-jsoncpp" \
            || ! is_installed_pacman "mingw-w64-x86_64-eigen3" || ! is_installed_pacman "mingw-w64-x86_64-python" \
            || ! is_installed_pacman "mingw-w64-x86_64-zeromq" || ! is_installed_pacman "mingw-w64-x86_64-cppzmq" \
            || ! is_installed_pacman "mingw-w64-x86_64-netcdf" || ! is_installed_pacman "mingw-w64-x86_64-openexr" \
            || ! is_installed_pacman "mingw-w64-x86_64-eccodes"; then
        echo "------------------------"
        echo "installing dependencies "
        echo "------------------------"
        pacman --noconfirm -S --needed mingw64/mingw-w64-x86_64-boost mingw64/mingw-w64-x86_64-glm \
        mingw64/mingw-w64-x86_64-libarchive mingw64/mingw-w64-x86_64-tinyxml2 mingw64/mingw-w64-x86_64-libpng \
        mingw64/mingw-w64-x86_64-SDL2 mingw64/mingw-w64-x86_64-SDL2_image mingw64/mingw-w64-x86_64-glew \
        mingw64/mingw-w64-x86_64-vulkan-headers mingw64/mingw-w64-x86_64-vulkan-loader \
        mingw64/mingw-w64-x86_64-vulkan-validation-layers mingw64/mingw-w64-x86_64-shaderc \
        mingw64/mingw-w64-x86_64-opencl-headers mingw64/mingw-w64-x86_64-opencl-icd mingw64/mingw-w64-x86_64-jsoncpp \
        mingw64/mingw-w64-x86_64-eigen3 mingw64/mingw-w64-x86_64-python mingw64/mingw-w64-x86_64-zeromq \
        mingw64/mingw-w64-x86_64-cppzmq mingw64/mingw-w64-x86_64-netcdf mingw64/mingw-w64-x86_64-openexr \
        mingw64/mingw-w64-x86_64-eccodes
    fi
elif $use_msys && command -v pacman &> /dev/null; then
    :
elif $use_macos && command -v brew &> /dev/null && [ ! -d $build_dir_debug ] && [ ! -d $build_dir_release ]; then
    if ! is_installed_brew "git"; then
        brew install git
    fi
    if ! is_installed_brew "cmake"; then
        brew install cmake
    fi
    if ! is_installed_brew "curl"; then
        brew install curl
    fi
    if ! is_installed_brew "pkg-config"; then
        brew install pkg-config
    fi
    if ! is_installed_brew "llvm"; then
        brew install llvm
    fi
    if ! is_installed_brew "libomp"; then
        brew install libomp
    fi
    if ! is_installed_brew "autoconf"; then
        brew install autoconf
    fi
    if ! is_installed_brew "automake"; then
        brew install automake
    fi
    if ! is_installed_brew "autoconf-archive"; then
        brew install autoconf-archive
    fi

    # Homebrew MoltenVK does not contain script for setting environment variables, unfortunately.
    #if ! is_installed_brew "molten-vk"; then
    #    brew install molten-vk
    #fi

    # Dependencies of sgl and the application.
    if [ $use_vcpkg = false ]; then
        if ! is_installed_brew "boost"; then
            brew install boost
        fi
        if ! is_installed_brew "glm"; then
            brew install glm
        fi
        if ! is_installed_brew "libarchive"; then
            brew install libarchive
        fi
        if ! is_installed_brew "tinyxml2"; then
            brew install tinyxml2
        fi
        if ! is_installed_brew "zlib"; then
            brew install zlib
        fi
        if ! is_installed_brew "libpng"; then
            brew install libpng
        fi
        if ! is_installed_brew "sdl2"; then
            brew install sdl2
        fi
        if ! is_installed_brew "sdl2_image"; then
            brew install sdl2_image
        fi
        if ! is_installed_brew "glew"; then
            brew install glew
        fi
        if ! is_installed_brew "opencl-headers"; then
            brew install opencl-headers
        fi
        if ! is_installed_brew "jsoncpp"; then
            brew install jsoncpp
        fi
        if ! is_installed_brew "eigen"; then
            brew install eigen
        fi
        if ! is_installed_brew "python@3.12"; then
            brew install python@3.12
        fi
        if ! is_installed_brew "zeromq"; then
            brew install zeromq
        fi
        if ! is_installed_brew "cppzmq"; then
            brew install cppzmq
        fi
        if ! is_installed_brew "netcdf"; then
            brew install netcdf
        fi
        if ! is_installed_brew "openexr"; then
            brew install openexr
        fi
    fi
elif $use_macos && command -v brew &> /dev/null; then
    :
elif command -v apt &> /dev/null && ! $use_conda; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null \
            || ! command -v patchelf &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        sudo apt install -y cmake git curl pkg-config build-essential patchelf
    fi

    # Dependencies of sgl and the application.
    if $use_vcpkg; then
        if ! is_installed_apt "libgl-dev" || ! is_installed_apt "libxmu-dev" || ! is_installed_apt "libxi-dev" \
                || ! is_installed_apt "libx11-dev" || ! is_installed_apt "libxft-dev" \
                || ! is_installed_apt "libxext-dev" || ! is_installed_apt "libxrandr-dev" \
                || ! is_installed_apt "libwayland-dev" || ! is_installed_apt "libxkbcommon-dev" \
                || ! is_installed_apt "libegl1-mesa-dev" || ! is_installed_apt "libibus-1.0-dev" \
                || ! is_installed_apt "autoconf" || ! is_installed_apt "automake" \
                || ! is_installed_apt "autoconf-archive"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo apt install -y libgl-dev libxmu-dev libxi-dev libx11-dev libxft-dev libxext-dev libxrandr-dev \
            libwayland-dev libxkbcommon-dev libegl1-mesa-dev libibus-1.0-dev autoconf automake autoconf-archive
        fi
    else
        if ! is_installed_apt "libboost-filesystem-dev" || ! is_installed_apt "libglm-dev" \
                || ! is_installed_apt "libarchive-dev" || ! is_installed_apt "libtinyxml2-dev" \
                || ! is_installed_apt "libpng-dev" || ! is_installed_apt "libsdl2-dev" \
                || ! is_installed_apt "libsdl2-image-dev" || ! is_installed_apt "libglew-dev" \
                || ! is_installed_apt "opencl-c-headers" || ! is_installed_apt "ocl-icd-opencl-dev" \
                || ! is_installed_apt "libjsoncpp-dev" || ! is_installed_apt "libeigen3-dev" \
                || ! is_installed_apt "python3-dev" || ! is_installed_apt "libzmq3-dev" \
                || ! is_installed_apt "libnetcdf-dev" || ! is_installed_apt "libopenexr-dev" \
                || ! is_installed_apt "libeccodes-dev" || ! is_installed_apt "libeccodes-tools" \
                || ! is_installed_apt "libopenjp2-7-dev"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo apt install -y libboost-filesystem-dev libglm-dev libarchive-dev libtinyxml2-dev libpng-dev libsdl2-dev \
            libsdl2-image-dev libglew-dev opencl-c-headers ocl-icd-opencl-dev libjsoncpp-dev libeigen3-dev python3-dev \
            libzmq3-dev libnetcdf-dev libopenexr-dev libeccodes-dev libeccodes-tools libopenjp2-7-dev
        fi
    fi
elif command -v pacman &> /dev/null && ! $use_conda; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null \
            || ! command -v patchelf &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        sudo pacman -S cmake git curl pkgconf base-devel patchelf
    fi

    # Dependencies of sgl and the application.
    if $use_vcpkg; then
        if ! is_installed_pacman "libgl" || ! is_installed_pacman "vulkan-devel" || ! is_installed_pacman "shaderc" \
                || ! is_installed_pacman "openssl" || ! is_installed_pacman "autoconf" \
                || ! is_installed_pacman "automake" || ! is_installed_pacman "autoconf-archive"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo pacman -S libgl vulkan-devel shaderc openssl autoconf automake autoconf-archive
        fi
    else
        if ! is_installed_pacman "boost" || ! is_installed_pacman "glm" || ! is_installed_pacman "libarchive" \
                || ! is_installed_pacman "tinyxml2" || ! is_installed_pacman "libpng" || ! is_installed_pacman "sdl2" \
                || ! is_installed_pacman "sdl2_image" || ! is_installed_pacman "glew" \
                || ! is_installed_pacman "vulkan-devel" || ! is_installed_pacman "shaderc" \
                || ! is_installed_pacman "opencl-headers" || ! is_installed_pacman "ocl-icd" \
                || ! is_installed_pacman "jsoncpp" || ! is_installed_pacman "eigen" || ! is_installed_pacman "python3" \
                || ! is_installed_pacman "zeromq" || ! is_installed_pacman "cppzmq" || ! is_installed_pacman "netcdf" \
                || ! is_installed_pacman "openexr" || ! is_installed_pacman "ospray"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo pacman -S boost glm libarchive tinyxml2 libpng sdl2 sdl2_image glew vulkan-devel shaderc opencl-headers \
            ocl-icd jsoncpp eigen python3 zeromq cppzmq netcdf openexr ospray
        fi
        if ! command -v yay &> /dev/null && ! is_installed_yay "eccodes"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            yay -S eccodes
        fi
    fi
elif command -v yum &> /dev/null && ! $use_conda; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null || ! command -v curl &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null \
            || ! command -v patchelf &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        sudo yum install -y cmake git curl pkgconf gcc gcc-c++ patchelf
    fi

    # Dependencies of sgl and the application.
    if $use_vcpkg; then
        if ! is_installed_rpm "perl" || ! is_installed_rpm "libstdc++-devel" || ! is_installed_rpm "libstdc++-static" \
                || ! is_installed_rpm "autoconf" || ! is_installed_rpm "automake" \
                || ! is_installed_rpm "autoconf-archive" || ! is_installed_rpm "glew-devel" \
                || ! is_installed_rpm "libXext-devel" || ! is_installed_rpm "vulkan-headers" \
                || ! is_installed_rpm "vulkan-loader" || ! is_installed_rpm "vulkan-tools" \
                || ! is_installed_rpm "vulkan-validation-layers" || ! is_installed_rpm "libshaderc-devel"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo yum install -y perl libstdc++-devel libstdc++-static autoconf automake autoconf-archive glew-devel \
            libXext-devel vulkan-headers vulkan-loader vulkan-tools vulkan-validation-layers libshaderc-devel
        fi
    else
        if ! is_installed_rpm "boost-devel" || ! is_installed_rpm "glm-devel" || ! is_installed_rpm "libarchive-devel" \
                || ! is_installed_rpm "tinyxml2-devel" || ! is_installed_rpm "libpng-devel" \
                || ! is_installed_rpm "SDL2-devel" || ! is_installed_rpm "SDL2_image-devel" \
                || ! is_installed_rpm "glew-devel" || ! is_installed_rpm "vulkan-headers" \
                || ! is_installed_rpm "libshaderc-devel" || ! is_installed_rpm "opencl-headers" \
                || ! is_installed_rpm "ocl-icd" || ! is_installed_rpm "jsoncpp-devel" \
                || ! is_installed_rpm "eigen3-devel" || ! is_installed_rpm "python3-devel" \
                || ! is_installed_rpm "zeromq-devel" || ! is_installed_rpm "cppzmq-devel" \
                || ! is_installed_rpm "netcdf-devel" || ! is_installed_rpm "openexr-devel" \
                || ! is_installed_rpm "eccodes-devel"; then
            echo "------------------------"
            echo "installing dependencies "
            echo "------------------------"
            sudo yum install -y boost-devel glm-devel libarchive-devel tinyxml2-devel libpng-devel SDL2-devel \
            SDL2_image-devel glew-devel vulkan-headers libshaderc-devel opencl-headers ocl-icd jsoncpp-devel \
            eigen3-devel python3-devel zeromq-devel cppzmq-devel netcdf-devel openexr-devel eccodes-devel
        fi
    fi
elif $use_conda && ! $use_macos; then
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/miniconda3/etc/profile.d/conda.sh" shell.bash hook
    elif [ -f "/opt/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/opt/anaconda3/etc/profile.d/conda.sh" shell.bash hook
    elif [ ! -z "${CONDA_PREFIX+x}" ]; then
        . "$CONDA_PREFIX/etc/profile.d/conda.sh" shell.bash hook
    fi

    if ! command -v conda &> /dev/null; then
        echo "------------------------"
        echo "  installing Miniconda  "
        echo "------------------------"
        if [ "$os_arch" = "x86_64" ]; then
            wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
            chmod +x Miniconda3-latest-Linux-x86_64.sh
            bash ./Miniconda3-latest-Linux-x86_64.sh
            . "$HOME/miniconda3/etc/profile.d/conda.sh" shell.bash hook
            rm ./Miniconda3-latest-Linux-x86_64.sh
        else
            wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
            chmod +x Miniconda3-latest-Linux-aarch64.sh
            bash ./Miniconda3-latest-Linux-aarch64.sh
            . "$HOME/miniconda3/etc/profile.d/conda.sh" shell.bash hook
            rm ./Miniconda3-latest-Linux-aarch64.sh
        fi
    fi

    if ! conda env list | grep ".*${conda_env_name}.*" >/dev/null 2>&1; then
        echo "------------------------"
        echo "creating conda environment"
        echo "------------------------"
        conda create -n "${conda_env_name}" -y
        conda init bash
        conda activate "${conda_env_name}"
    elif [ "${var+CONDA_DEFAULT_ENV}" != "${conda_env_name}" ]; then
        conda activate "${conda_env_name}"
    fi

    conda_pkg_list="$(conda list)"
    if ! list_contains "$conda_pkg_list" "boost" || ! list_contains "$conda_pkg_list" "glm" \
            || ! list_contains "$conda_pkg_list" "libarchive" || ! list_contains "$conda_pkg_list" "tinyxml2" \
            || ! list_contains "$conda_pkg_list" "libpng" || ! list_contains "$conda_pkg_list" "sdl2" \
            || ! list_contains "$conda_pkg_list" "sdl2" || ! list_contains "$conda_pkg_list" "glew" \
            || ! list_contains "$conda_pkg_list" "cxx-compiler" || ! list_contains "$conda_pkg_list" "make" \
            || ! list_contains "$conda_pkg_list" "cmake" || ! list_contains "$conda_pkg_list" "pkg-config" \
            || ! list_contains "$conda_pkg_list" "gdb" || ! list_contains "$conda_pkg_list" "git" \
            || ! list_contains "$conda_pkg_list" "mesa-libgl-devel-cos7-x86_64" \
            || ! list_contains "$conda_pkg_list" "libglvnd-glx-cos7-x86_64" \
            || ! list_contains "$conda_pkg_list" "mesa-dri-drivers-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxau-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libselinux-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxdamage-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxxf86vm-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "libxext-devel-cos7-aarch64" \
            || ! list_contains "$conda_pkg_list" "xorg-libxfixes" || ! list_contains "$conda_pkg_list" "xorg-libxau" \
            || ! list_contains "$conda_pkg_list" "xorg-libxrandr" || ! list_contains "$conda_pkg_list" "patchelf" \
            || ! list_contains "$conda_pkg_list" "libvulkan-headers" || ! list_contains "$conda_pkg_list" "shaderc" \
            || ! list_contains "$conda_pkg_list" "jsoncpp" || ! list_contains "$conda_pkg_list" "eigen" \
            || ! list_contains "$conda_pkg_list" "zeromq" || ! list_contains "$conda_pkg_list" "cppzmq" \
            || ! list_contains "$conda_pkg_list" "netcdf4" || ! list_contains "$conda_pkg_list" "openexr" \
            || ! list_contains "$conda_pkg_list" "eccodes"; then
        echo "------------------------"
        echo "installing dependencies "
        echo "------------------------"
        conda install -y -c conda-forge boost glm libarchive tinyxml2 libpng sdl2 sdl2 glew cxx-compiler make cmake \
        pkg-config gdb git mesa-libgl-devel-cos7-x86_64 libglvnd-glx-cos7-x86_64 mesa-dri-drivers-cos7-aarch64 \
        libxau-devel-cos7-aarch64 libselinux-devel-cos7-aarch64 libxdamage-devel-cos7-aarch64 \
        libxxf86vm-devel-cos7-aarch64 libxext-devel-cos7-aarch64 xorg-libxfixes xorg-libxau xorg-libxrandr patchelf \
        libvulkan-headers shaderc jsoncpp eigen zeromq cppzmq netcdf4 openexr eccodes
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
if [ $use_macos = false ] && ! command -v pkg-config &> /dev/null; then
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


params_sgl=()
params=()
params_run=()
params_gen=()

if [ $use_msys = true ]; then
    params_gen+=(-G "MSYS Makefiles")
    params_sgl+=(-G "MSYS Makefiles")
    params+=(-G "MSYS Makefiles")
fi

if [ $use_vcpkg = false ] && [ $use_macos = true ]; then
    params_gen+=(-DCMAKE_FIND_USE_CMAKE_SYSTEM_PATH=False)
    params_gen+=(-DCMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH=False)
    params_gen+=(-DCMAKE_FIND_FRAMEWORK=LAST)
    params_gen+=(-DCMAKE_FIND_APPBUNDLE=NEVER)
    params_gen+=(-DCMAKE_PREFIX_PATH="$(brew --prefix)")
    params_sgl+=(-DCMAKE_INSTALL_PREFIX="../install")
    params_sgl+=(-DZLIB_ROOT="$(brew --prefix)/opt/zlib")
    params+=(-DZLIB_ROOT="$(brew --prefix)/opt/zlib")
fi

if $glibcxx_debug; then
    params_sgl+=(-DUSE_GLIBCXX_DEBUG=On)
    params+=(-DUSE_GLIBCXX_DEBUG=On)
fi

use_vulkan=false
vulkan_sdk_env_set=true
use_vulkan=true

search_for_vulkan_sdk=false
if [ $use_msys = false ] && [ -z "${VULKAN_SDK+1}" ]; then
    search_for_vulkan_sdk=true
fi

if [ $search_for_vulkan_sdk = true ]; then
    echo "------------------------"
    echo "searching for Vulkan SDK"
    echo "------------------------"

    found_vulkan=false
    use_local_vulkan_sdk=false

    if [ $use_macos = false ]; then
        if [ -d "VulkanSDK" ]; then
            VK_LAYER_PATH=""
            source "VulkanSDK/$(ls VulkanSDK)/setup-env.sh"
            use_local_vulkan_sdk=true
            pkgconfig_dir="$(realpath "VulkanSDK/$(ls VulkanSDK)/$os_arch/lib/pkgconfig")"
            if [ -d "$pkgconfig_dir" ]; then
                export PKG_CONFIG_PATH="$pkgconfig_dir"
            fi
            found_vulkan=true
        fi

        if ! $found_vulkan && (lsb_release -a 2> /dev/null | grep -q 'Ubuntu' || lsb_release -a 2> /dev/null | grep -q 'Mint') && ! $use_conda; then
            if lsb_release -a 2> /dev/null | grep -q 'Ubuntu'; then
                distro_code_name=$(lsb_release -cs)
                distro_release=$(lsb_release -rs)
            else
                distro_code_name=$(cat /etc/upstream-release/lsb-release | grep "DISTRIB_CODENAME=" | sed 's/^.*=//')
                distro_release=$(cat /etc/upstream-release/lsb-release | grep "DISTRIB_RELEASE=" | sed 's/^.*=//')
            fi
            if ! compgen -G "/etc/apt/sources.list.d/lunarg-vulkan-*" > /dev/null \
                  && ! curl -s -I "https://packages.lunarg.com/vulkan/dists/${distro_code_name}/" | grep "2 404" > /dev/null; then
                echo "Setting up Vulkan SDK for $(lsb_release -ds)..."
                wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo apt-key add -
                sudo curl --silent --show-error --fail \
                https://packages.lunarg.com/vulkan/lunarg-vulkan-${distro_code_name}.list \
                --output /etc/apt/sources.list.d/lunarg-vulkan-${distro_code_name}.list
                sudo apt update
                sudo apt install -y vulkan-sdk shaderc glslang-dev
            elif dpkg --compare-versions "$distro_release" "ge" "24.04"; then
                if ! is_installed_apt "libvulkan-dev" && ! is_installed_apt "libshaderc-dev" && ! is_installed_apt "glslang-dev"; then
                    sudo apt install -y libvulkan-dev libshaderc-dev glslang-dev
                fi
            fi
        fi

        if [ -d "/usr/include/vulkan" ] && [ -d "/usr/include/shaderc" ]; then
            if ! grep -q VULKAN_SDK ~/.bashrc; then
                echo 'export VULKAN_SDK="/usr"' >> ~/.bashrc
            fi
            export VULKAN_SDK="/usr"
            found_vulkan=true
        fi

        if ! $found_vulkan; then
            curl --silent --show-error --fail -O https://sdk.lunarg.com/sdk/download/latest/linux/vulkan-sdk.tar.gz
            mkdir -p VulkanSDK
            tar -xf vulkan-sdk.tar.gz -C VulkanSDK
            if [ "$os_arch" != "x86_64" ]; then
                pushd "VulkanSDK/$(ls VulkanSDK)" >/dev/null
                ./vulkansdk -j $(nproc) vulkan-loader glslang shaderc
                popd >/dev/null
            fi
            VK_LAYER_PATH=""
            source "VulkanSDK/$(ls VulkanSDK)/setup-env.sh"
            use_local_vulkan_sdk=true

            # Fix pkgconfig file.
            shaderc_pkgconfig_file="VulkanSDK/$(ls VulkanSDK)/$os_arch/lib/pkgconfig/shaderc.pc"
            if [ -f "$shaderc_pkgconfig_file" ]; then
                prefix_path=$(realpath "VulkanSDK/$(ls VulkanSDK)/$os_arch")
                sed -i '3s;.*;prefix=\"'$prefix_path'\";' "$shaderc_pkgconfig_file"
                sed -i '5s;.*;libdir=${prefix}/lib;' "$shaderc_pkgconfig_file"
                export PKG_CONFIG_PATH="$(realpath "VulkanSDK/$(ls VulkanSDK)/$os_arch/lib/pkgconfig")"
            fi
            found_vulkan=true
        fi
    else
        if [ -d "$HOME/VulkanSDK" ] && [ ! -z "$(ls -A "$HOME/VulkanSDK")" ]; then
            source "$HOME/VulkanSDK/$(ls $HOME/VulkanSDK)/setup-env.sh"
            found_vulkan=true
        else
            vulkansdk_filename=$(curl -sIkL https://sdk.lunarg.com/sdk/download/latest/mac/vulkan-sdk.dmg | sed -r '/filename=/!d;s/.*filename=(.*)$/\1/')
            VULKAN_SDK_VERSION=$(echo $vulkansdk_filename | sed -r 's/^.*vulkansdk-macos-(.*)\.dmg.*$/\1/')
            curl -O https://sdk.lunarg.com/sdk/download/latest/mac/vulkan-sdk.dmg
            sudo hdiutil attach vulkan-sdk.dmg
            # The directory was changed from '/Volumes/VulkanSDK' to, e.g., 'vulkansdk-macos-1.3.261.0'.
            vulkan_dir=$(find /Volumes -maxdepth 1 -name '[Vv]ulkan*' -not -path "/Volumes/VMware*" || true)
            sudo "${vulkan_dir}/InstallVulkan.app/Contents/MacOS/InstallVulkan" \
            --root ~/VulkanSDK/$VULKAN_SDK_VERSION --accept-licenses --default-answer --confirm-command install
            pushd ~/VulkanSDK/$VULKAN_SDK_VERSION
            sudo python3 ./install_vulkan.py || true
            popd
            sudo hdiutil unmount "${vulkan_dir}"
            source "$HOME/VulkanSDK/$(ls $HOME/VulkanSDK)/setup-env.sh"
            found_vulkan=true
        fi
    fi

    if ! $found_vulkan; then
        if [ $use_macos = false ]; then
            os_name="linux"
        else
            os_name="mac"
        fi
        echo "The environment variable VULKAN_SDK is not set but is required in the installation process."
        echo "Please refer to https://vulkan.lunarg.com/sdk/home#${os_name} for instructions on how to install the Vulkan SDK."
        exit 1
    fi

    # On RHEL 8.6, I got the following errors when using the libvulkan.so provided by the SDK:
    # dlopen failed: /lib64/libm.so.6: version `GLIBC_2.29' not found (required by /home/u12458/Correrender/third_party/VulkanSDK/1.3.275.0/x86_64/lib/libvulkan.so)
    # Thus, we should remove it from the path if necessary.
    if $use_local_vulkan_sdk; then
        pushd VulkanSDK/$(ls VulkanSDK)/$os_arch/lib >/dev/null
        if ldd -r libvulkan.so | grep "undefined symbol"; then
            echo "Removing Vulkan SDK libvulkan.so from path..."
            export LD_LIBRARY_PATH=$(echo ${LD_LIBRARY_PATH} | awk -v RS=: -v ORS=: '/VulkanSDK/ {next} {print}' | sed 's/:*$//') && echo $OUTPATH
        fi
        popd >/dev/null
    fi
fi

if $custom_glslang; then
    if [ ! -d "./glslang" ]; then
        echo "------------------------"
        echo "  downloading glslang   "
        echo "------------------------"
        # Make sure we have no leftovers from a failed build attempt.
        if [ -d "./glslang-src" ]; then
            rm -rf "./glslang-src"
        fi
        git clone https://github.com/KhronosGroup/glslang.git glslang-src
        pushd glslang-src >/dev/null
        ./update_glslang_sources.py
        mkdir build
        pushd build >/dev/null
        cmake ${params_gen[@]+"${params_gen[@]}"} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${projectpath}/third_party/glslang" ..
        make -j $(nproc)
        make install
        popd >/dev/null
        popd >/dev/null
    fi
    params_sgl+=(-Dglslang_DIR="${projectpath}/third_party/glslang" -DUSE_SHADERC=Off)
fi

if [ $use_msys = false ] && [ -z "${VULKAN_SDK+1}" ]; then
    vulkan_sdk_env_set=true
fi

if [ $use_vcpkg = true ] && [ ! -d "./vcpkg" ]; then
    echo "------------------------"
    echo "    fetching vcpkg      "
    echo "------------------------"
    if $use_vulkan && [ $vulkan_sdk_env_set = false ]; then
        echo "The environment variable VULKAN_SDK is not set but is required in the installation process."
        exit 1
    fi
    git clone --depth 1 -b fix-libarchive-rpath https://github.com/chrismile/vcpkg.git
    vcpkg/bootstrap-vcpkg.sh -disableMetrics
    vcpkg/vcpkg install
fi

if [ $use_vcpkg = true ] && [ $use_macos = false ] && [ $link_dynamic = false ]; then
    params_sgl+=(-DBUILD_STATIC_LIBRARY=On)
fi

if [ ! -d "./sgl" ]; then
    echo "------------------------"
    echo "     fetching sgl       "
    echo "------------------------"
    git clone --depth 1 https://github.com/chrismile/sgl.git
fi

if [ -f "./sgl/$build_dir/CMakeCache.txt" ]; then
    if grep -q vcpkg_installed "./sgl/$build_dir/CMakeCache.txt"; then
        cache_uses_vcpkg=true
    else
        cache_uses_vcpkg=false
    fi
    if ([ $use_vcpkg = true ] && [ $cache_uses_vcpkg = false ]) || ([ $use_vcpkg = false ] && [ $cache_uses_vcpkg = true ]); then
        echo "Removing old sgl build cache..."
        if [ -d "./sgl/$build_dir_debug" ]; then
            rm -rf "./sgl/$build_dir_debug"
        fi
        if [ -d "./sgl/$build_dir_release" ]; then
            rm -rf "./sgl/$build_dir_release"
        fi
        if [ -d "./sgl/install" ]; then
            rm -rf "./sgl/install"
        fi
    fi
fi

if [ ! -d "./sgl/install" ]; then
    echo "------------------------"
    echo "     building sgl       "
    echo "------------------------"

    pushd "./sgl" >/dev/null
    mkdir -p $build_dir_debug
    mkdir -p $build_dir_release

    pushd "$build_dir_debug" >/dev/null
    cmake .. \
         -DCMAKE_BUILD_TYPE=Debug \
         -DCMAKE_INSTALL_PREFIX="../install" \
         ${params_gen[@]+"${params_gen[@]}"} ${params_link[@]+"${params_link[@]}"} \
         ${params_vcpkg[@]+"${params_vcpkg[@]}"} ${params_sgl[@]+"${params_sgl[@]}"}
    if [ $use_vcpkg = false ] && [ $use_macos = false ]; then
        make -j $(nproc)
        make install
    fi
    popd >/dev/null

    pushd $build_dir_release >/dev/null
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="../install" \
         ${params_gen[@]+"${params_gen[@]}"} ${params_link[@]+"${params_link[@]}"} \
         ${params_vcpkg[@]+"${params_vcpkg[@]}"} ${params_sgl[@]+"${params_sgl[@]}"}
    if [ $use_vcpkg = false ] && [ $use_macos = false ]; then
        make -j $(nproc)
        make install
    fi
    popd >/dev/null

    if [ $use_macos = true ]; then
        cmake --build $build_dir_debug --parallel $(sysctl -n hw.ncpu)
        cmake --build $build_dir_debug --target install

        cmake --build $build_dir_release --parallel $(sysctl -n hw.ncpu)
        cmake --build $build_dir_release --target install
    elif [ $use_vcpkg = true ]; then
        cmake --build $build_dir_debug --parallel $(nproc)
        cmake --build $build_dir_debug --target install
        if [ $link_dynamic = true ]; then
            cp $build_dir_debug/libsgld.so install/lib/libsgld.so
        fi

        cmake --build $build_dir_release --parallel $(nproc)
        cmake --build $build_dir_release --target install
        if [ $link_dynamic = true ]; then
            cp $build_dir_release/libsgl.so install/lib/libsgl.so
        fi
    fi

    popd >/dev/null
fi

if [ $use_vcpkg = true ]; then
    params+=(-DPYTHONHOME="./python3")
fi

if $use_msys; then
    Python3_VERSION="$(find "$MSYSTEM_PREFIX/lib/" -maxdepth 1 -type d -name 'python*' -printf "%f" -quit)"
    params+=(-DPython3_FIND_REGISTRY=NEVER -DPYTHONHOME="./python3" -DPYTHONPATH="./python3/lib/$Python3_VERSION")
fi
embree_version="3.13.3"
ospray_version="2.9.0"

if [ $use_macos = false ] && [ $use_msys = false ]; then
    if ! $is_embree_installed && [ $os_arch = "x86_64" ]; then
        if [ ! -d "./embree-${embree_version}.x86_64.linux" ]; then
            echo "------------------------"
            echo "   downloading Embree   "
            echo "------------------------"
            wget "https://github.com/embree/embree/releases/download/v${embree_version}/embree-${embree_version}.x86_64.linux.tar.gz"
            tar -xvzf "embree-${embree_version}.x86_64.linux.tar.gz"
        fi
        params+=(-Dembree_DIR="${projectpath}/third_party/embree-${embree_version}.x86_64.linux/lib/cmake/embree-${embree_version}")
    fi

    if ! $is_ospray_installed && [ $os_arch = "x86_64" ]; then
        if [ ! -d "./ospray-${ospray_version}.x86_64.linux" ]; then
            echo "------------------------"
            echo "   downloading OSPRay   "
            echo "------------------------"
            wget "https://github.com/ospray/OSPRay/releases/download/v${ospray_version}/ospray-${ospray_version}.x86_64.linux.tar.gz"
            tar -xvzf "ospray-${ospray_version}.x86_64.linux.tar.gz"
        fi
        params+=(-Dospray_DIR="${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/cmake/ospray-${ospray_version}")
    fi
elif [ $use_macos = true ]; then
    if [ ! -d "./ospray/ospray/lib/cmake" ]; then
        # Make sure we have no leftovers from a failed build attempt.
        if [ -d "./ospray-repo" ]; then
            rm -rf "./ospray-repo"
        fi
        if [ -d "./ospray-build" ]; then
            rm -rf "./ospray-build"
        fi
        if [ -d "./ospray" ]; then
            rm -rf "./ospray"
        fi

        params_ospray=()
        if [[ $(uname -m) == 'arm64' ]]; then
            params_ospray+=(-DBUILD_TBB_FROM_SOURCE=On)
        fi

        # Build OSPRay and its dependencies.
        git clone https://github.com/ospray/ospray.git ospray-repo
        mkdir ospray-build
        pushd "./ospray-build" >/dev/null
        cmake ../ospray-repo/scripts/superbuild -DCMAKE_INSTALL_PREFIX="$projectpath/third_party/ospray" \
        -DBUILD_JOBS=$(sysctl -n hw.ncpu) -DBUILD_OSPRAY_APPS=Off ${params_ospray[@]+"${params_ospray[@]}"}
        cmake --build . --parallel $(sysctl -n hw.ncpu)
        cmake --build . --parallel $(sysctl -n hw.ncpu)
        popd >/dev/null
    fi

    params+=(-Dembree_DIR="${projectpath}/third_party/ospray/embree/lib/cmake/$(ls "${projectpath}/third_party/ospray/embree/lib/cmake")")
    params+=(-Dospray_DIR="${projectpath}/third_party/ospray/ospray/lib/cmake/$(ls "${projectpath}/third_party/ospray/ospray/lib/cmake")")
fi

#if [ -d "./ospray/ospray/lib/cmake" ]; then
#    is_ospray_installed=true
#else
#    is_ospray_installed=false
#
#    # Make sure we have no leftovers from a failed build attempt.
#    if [ -d "./ospray-repo" ]; then
#        rm -rf "./ospray-repo"
#    fi
#    if [ -d "./ospray-build" ]; then
#        rm -rf "./ospray-build"
#    fi
#    if [ -d "./ospray" ]; then
#        rm -rf "./ospray"
#    fi
#
#    # Build OSPRay and its dependencies.
#    git clone https://github.com/ospray/ospray.git ospray-repo
#    mkdir ospray-build
#    pushd "./ospray-build" >/dev/null
#    cmake ../ospray-repo/scripts/superbuild -G "MSYS Makefiles" \
#    -DCMAKE_INSTALL_PREFIX="$projectpath/third_party/ospray" \
#    -DBUILD_JOBS=$(nproc) -DBUILD_OSPRAY_APPS=Off
#    cmake --build . --parallel $(nproc)
#    cmake --build . --parallel $(nproc)
#    popd >/dev/null
#
#    is_ospray_installed=true
#fi
#
#if $is_ospray_installed; then
#    params+=(-Dembree_DIR="${projectpath}/third_party/ospray/embree/lib/cmake/$(ls "${projectpath}/third_party/ospray/embree/lib/cmake")")
#    params+=(-Dospray_DIR="${projectpath}/third_party/ospray/ospray/lib/cmake/$(ls "${projectpath}/third_party/ospray/ospray/lib/cmake")")
#fi

popd >/dev/null # back to project root

if [ $debug = true ]; then
    echo "------------------------"
    echo "  building in debug     "
    echo "------------------------"
else
    echo "------------------------"
    echo "  building in release   "
    echo "------------------------"
fi

if [ -f "./$build_dir/CMakeCache.txt" ]; then
    if grep -q vcpkg_installed "./$build_dir/CMakeCache.txt"; then
        cache_uses_vcpkg=true
    else
        cache_uses_vcpkg=false
    fi
    if ([ $use_vcpkg = true ] && [ $cache_uses_vcpkg = false ]) || ([ $use_vcpkg = false ] && [ $cache_uses_vcpkg = true ]); then
        echo "Removing old application build cache..."
        if [ -d "./$build_dir_debug" ]; then
            rm -rf "./$build_dir_debug"
        fi
        if [ -d "./$build_dir_release" ]; then
            rm -rf "./$build_dir_release"
        fi
        if [ -d "./$destination_dir" ]; then
            rm -rf "./$destination_dir"
        fi
    fi
fi

mkdir -p $build_dir

echo "------------------------"
echo "      generating        "
echo "------------------------"
pushd $build_dir >/dev/null
cmake .. \
    -DCMAKE_BUILD_TYPE=$cmake_config \
    -Dsgl_DIR="$projectpath/third_party/sgl/install/lib/cmake/sgl/" \
    ${params_gen[@]+"${params_gen[@]}"} ${params_link[@]+"${params_link[@]}"} \
    ${params_vcpkg[@]+"${params_vcpkg[@]}"} ${params[@]+"${params[@]}"}
if [ $use_vcpkg = true ] || [ $use_msys = true ] || [ $use_macos = true ]; then
    Python3_VERSION=$(cat pythonversion.txt)
fi
popd >/dev/null

echo "------------------------"
echo "      compiling         "
echo "------------------------"
if [ $use_macos = true ]; then
    cmake --build $build_dir --parallel $(sysctl -n hw.ncpu)
elif [ $use_vcpkg = true ]; then
    cmake --build $build_dir --parallel $(nproc)
else
    pushd "$build_dir" >/dev/null
    make -j $(nproc)
    popd >/dev/null
fi

echo "------------------------"
echo "   copying new files    "
echo "------------------------"

# https://stackoverflow.com/questions/2829613/how-do-you-tell-if-a-string-contains-another-string-in-posix-sh
contains() {
    string="$1"
    substring="$2"
    if test "${string#*$substring}" != "$string"
    then
        return 0
    else
        return 1
    fi
}
startswith() {
    string="$1"
    prefix="$2"
    if test "${string#$prefix}" != "$string"
    then
        return 0
    else
        return 1
    fi
}

if $use_msys; then
    mkdir -p $destination_dir/bin

    # Copy sgl to the destination directory.
    if [ $debug = true ] ; then
        cp "./third_party/sgl/install/bin/libsgld.dll" "$destination_dir/bin"
    else
        cp "./third_party/sgl/install/bin/libsgl.dll" "$destination_dir/bin"
    fi

    # Copy the application to the destination directory.
    cp "$build_dir/LineVis.exe" "$destination_dir/bin"

    # Copy all dependencies of the application to the destination directory.
    ldd_output="$(ldd $destination_dir/bin/LineVis.exe)"
    for library in $ldd_output
    do
        if [[ $library == "$MSYSTEM_PREFIX"* ]] ;
        then
            cp "$library" "$destination_dir/bin"
        fi
        if [[ $library == libpython* ]] ;
        then
            tmp=${library#*lib}
            Python3_VERSION=${tmp%.dll}
        fi
    done
elif [ $use_macos = true ] && [ $use_vcpkg = true ]; then
    [ -d $destination_dir ] || mkdir $destination_dir
    rsync -a "$build_dir/LineVis.app/Contents/MacOS/LineVis" $destination_dir
elif [ $use_macos = true ] && [ $use_vcpkg = false ]; then
    brew_prefix="$(brew --prefix)"
    mkdir -p $destination_dir

    if [ -d "$destination_dir/LineVis.app" ]; then
        rm -rf "$destination_dir/LineVis.app"
    fi

    # Copy the application to the destination directory.
    cp -a "$build_dir/LineVis.app" "$destination_dir"

    # Copy sgl to the destination directory.
    if [ $debug = true ] ; then
        cp "./third_party/sgl/install/lib/libsgld.dylib" "$binaries_dest_dir"
    else
        cp "./third_party/sgl/install/lib/libsgl.dylib" "$binaries_dest_dir"
    fi

    # Copy all dependencies of the application and sgl to the destination directory.
    rsync -a "$VULKAN_SDK/lib/libMoltenVK.dylib" "$binaries_dest_dir"
    copy_dependencies_recursive() {
        local binary_path="$1"
        local binary_source_folder=$(dirname "$binary_path")
        local binary_name=$(basename "$binary_path")
        local binary_target_path="$binaries_dest_dir/$binary_name"
        if contains "$(file "$binary_target_path")" "dynamically linked shared library"; then
            install_name_tool -id "@executable_path/$binary_name" "$binary_target_path" &> /dev/null
        fi
        local otool_output="$(otool -L "$binary_path")"
        local otool_output=${otool_output#*$'\n'}
        while read -r line
        do
            local stringarray=($line)
            local library=${stringarray[0]}
            local library_name=$(basename "$library")
            local library_target_path="$binaries_dest_dir/$library_name"
            if ! startswith "$library" "@rpath/" \
                && ! startswith "$library" "@loader_path/" \
                && ! startswith "$library" "/System/Library/Frameworks/" \
                && ! startswith "$library" "/usr/lib/"
            then
                install_name_tool -change "$library" "@executable_path/$library_name" "$binary_target_path" &> /dev/null

                if [ ! -f "$library_target_path" ]; then
                    cp "$library" "$binaries_dest_dir"
                    copy_dependencies_recursive "$library"
                fi
            elif startswith "$library" "@rpath/"; then
                install_name_tool -change "$library" "@executable_path/$library_name" "$binary_target_path" &> /dev/null

                local rpath_grep_string="$(otool -l "$binary_target_path" | grep RPATH -A2)"
                local counter=0
                while read -r grep_rpath_line
                do
                    if [ $(( counter % 4 )) -eq 2 ]; then
                        local stringarray_grep_rpath_line=($grep_rpath_line)
                        local rpath=${stringarray_grep_rpath_line[1]}
                        if startswith "$rpath" "@loader_path"; then
                            rpath="${rpath/@loader_path/$binary_source_folder}"
                        fi
                        local library_rpath="${rpath}${library#"@rpath"}"

                        if [ -f "$library_rpath" ]; then
                            if [ ! -f "$library_target_path" ]; then
                                cp "$library_rpath" "$binaries_dest_dir"
                                copy_dependencies_recursive "$library_rpath"
                            fi
                            break
                        fi
                    fi
                    counter=$((counter + 1))
                done < <(echo "$rpath_grep_string")
            fi
        done < <(echo "$otool_output")
    }
    copy_dependencies_recursive "$build_dir/LineVis.app/Contents/MacOS/LineVis"
    if [ $debug = true ]; then
        copy_dependencies_recursive "./third_party/sgl/install/lib/libsgld.dylib"
    else
        copy_dependencies_recursive "./third_party/sgl/install/lib/libsgl.dylib"
    fi
    copy_ospray_lib_symlinked() {
        local lib_name="$1"
        local lib_path_1="./third_party/ospray/ospray/lib/$lib_name"
        local lib_path_2="./third_party/ospray/ospray/lib/$(readlink $lib_path_1)"
        local lib_path_3="./third_party/ospray/ospray/lib/$(readlink $lib_path_2)"
        cp "$lib_path_1" "$binaries_dest_dir"
        cp "$lib_path_2" "$binaries_dest_dir"
        cp "$lib_path_3" "$binaries_dest_dir"
        copy_dependencies_recursive "$lib_path_1"
        copy_dependencies_recursive "$lib_path_2"
        copy_dependencies_recursive "$lib_path_3"
    }
    copy_ospray_lib_symlinked "libopenvkl.dylib"
    copy_ospray_lib_symlinked "libopenvkl_module_cpu_device.dylib"
    copy_ospray_lib_symlinked "libopenvkl_module_cpu_device_4.dylib"
    copy_ospray_lib_symlinked "libospray_module_cpu.dylib"

    # Fix code signing for arm64.
    for filename in $binaries_dest_dir/*
    do
        if contains "$(file "$filename")" "arm64"; then
            codesign --force -s - "$filename" &> /dev/null
        fi
    done
${copy_dependencies_macos_post}
else
    mkdir -p $destination_dir/bin

    # Copy the application to the destination directory.
    rsync -a "$build_dir/LineVis" "$destination_dir/bin"

    # Copy all dependencies of the application to the destination directory.
    ldd_output="$(ldd $build_dir/LineVis)"

    if ! $is_ospray_installed && [ $os_arch = "x86_64" ]; then
        libembree3_so="$(readlink -f "${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/libembree3.so")"
        libospray_module_cpu_so="$(readlink -f "${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/libospray_module_cpu.so")"
        libopenvkl_so="$(readlink -f "${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/libopenvkl.so")"
        libopenvkl_module_cpu_device_so="$(readlink -f "${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/libopenvkl_module_cpu_device.so")"
        libopenvkl_module_cpu_device_4_so="$(readlink -f "${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/libopenvkl_module_cpu_device_4.so")"
        libopenvkl_module_cpu_device_8_so="$(readlink -f "${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/libopenvkl_module_cpu_device_8.so")"
        libopenvkl_module_cpu_device_16_so="$(readlink -f "${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib/libopenvkl_module_cpu_device_16.so")"
        ldd_output="$ldd_output $libembree3_so $libospray_module_cpu_so $libopenvkl_so $libopenvkl_module_cpu_device_so"
        ldd_output="$ldd_output $libopenvkl_module_cpu_device_4_so $libopenvkl_module_cpu_device_8_so $libopenvkl_module_cpu_device_16_so"
    fi
    library_blacklist=(
        "libOpenGL" "libGLdispatch" "libGL.so" "libGLX.so"
        "libwayland" "libffi." "libX" "libxcb" "libxkbcommon"
        "ld-linux" "libdl." "libutil." "libm." "libc." "libpthread." "libbsd." "librt."
    )
    if [ $use_vcpkg = true ]; then
        # We build with libstdc++.so and libgcc_s.so statically. If we were to ship them, libraries opened with dlopen will
        # use our, potentially older, versions. Then, we will get errors like "version `GLIBCXX_3.4.29' not found" when
        # the Vulkan loader attempts to load a Vulkan driver that was built with a never version of libstdc++.so.
        # I tried to solve this by using "patchelf --replace-needed" to directly link to the patch version of libstdc++.so,
        # but that made no difference whatsoever for dlopen.
        library_blacklist+=("libstdc++.so")
        library_blacklist+=("libgcc_s.so")
    fi
    for library in $ldd_output
    do
        if [[ $library != "/"* ]]; then
            continue
        fi
        is_blacklisted=false
        for blacklisted_library in ${library_blacklist[@]+"${library_blacklist[@]}"}; do
            if [[ "$library" == *"$blacklisted_library"* ]]; then
                is_blacklisted=true
                break
            fi
        done
        if [ $is_blacklisted = true ]; then
            continue
        fi
        # TODO: Add blacklist entries for pulseaudio and dependencies when not using vcpkg.
        #cp "$library" "$destination_dir/bin"
        #patchelf --set-rpath '$ORIGIN' "$destination_dir/bin/$(basename "$library")"
        if [ $use_vcpkg = true ]; then
            cp "$library" "$destination_dir/bin"
            patchelf --set-rpath '$ORIGIN' "$destination_dir/bin/$(basename "$library")"
        fi
    done
    patchelf --set-rpath '$ORIGIN' "$destination_dir/bin/LineVis"
    if ! $is_ospray_installed; then
        ln -sf "./$(basename "$libembree3_so")" "$destination_dir/bin/libembree3.so"
        ln -sf "./$(basename "$libospray_module_cpu_so")" "$destination_dir/bin/libospray_module_cpu.so"
        ln -sf "./$(basename "$libopenvkl_so")" "$destination_dir/bin/libopenvkl.so"
        ln -sf "./$(basename "$libopenvkl_so")" "$destination_dir/bin/libopenvkl.so.1"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_so")" "$destination_dir/bin/libopenvkl_module_cpu_device.so"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_so")" "$destination_dir/bin/libopenvkl_module_cpu_device.so.1"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_4_so")" "$destination_dir/bin/libopenvkl_module_cpu_device_4.so"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_4_so")" "$destination_dir/bin/libopenvkl_module_cpu_device_4.so.1"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_8_so")" "$destination_dir/bin/libopenvkl_module_cpu_device_8.so"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_8_so")" "$destination_dir/bin/libopenvkl_module_cpu_device_8.so.1"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_16_so")" "$destination_dir/bin/libopenvkl_module_cpu_device_16.so"
        ln -sf "./$(basename "$libopenvkl_module_cpu_device_16_so")" "$destination_dir/bin/libopenvkl_module_cpu_device_16.so.1"
    fi
fi

# 2023-11-11: It seems like for LineVis, vcpkg_installed is in the root directory, but for HexVolumeRenderer
# it is in the build folder.
if [ $use_vcpkg = true ]; then
    if [ -d "vcpkg_installed" ]; then
        vcpkg_installed_dir="vcpkg_installed"
    elif [ -d "$build_dir/vcpkg_installed" ]; then
        vcpkg_installed_dir="$build_dir/vcpkg_installed"
    fi
fi
# Copy python3 to the destination directory.
if $use_msys; then
    if [ ! -d "$destination_dir/bin/python3" ]; then
        mkdir -p "$destination_dir/bin/python3/lib"
        #cp -r "$MSYSTEM_PREFIX/lib/$Python3_VERSION" "$destination_dir/bin/python3/lib"
        rsync -qav "$MSYSTEM_PREFIX/lib/$Python3_VERSION" "$destination_dir/bin/python3/lib" --exclude site-packages --exclude dist-packages
    fi
elif [ $use_macos = true ] && [ $use_vcpkg = true ]; then
    [ -d $destination_dir/python3 ]     || mkdir $destination_dir/python3
    [ -d $destination_dir/python3/lib ] || mkdir $destination_dir/python3/lib
    rsync -a "vcpkg_installed/$(ls $vcpkg_installed_dir | grep -Ewv 'vcpkg')/lib/$Python3_VERSION" $destination_dir/python3/lib
    #rsync -a "$(eval echo "vcpkg_installed/$(ls $vcpkg_installed_dir | grep -Ewv 'vcpkg')/lib/python*")" $destination_dir/python3/lib
elif [ $use_macos = true ] && [ $use_vcpkg = false ]; then
    python_version=${Python3_VERSION#python}
    python_subdir="$brew_prefix/Cellar/python@${Python3_VERSION#python}"
    PYTHONHOME_global="$python_subdir/$(ls "$python_subdir")/Frameworks/Python.framework/Versions/$python_version"
    if [ ! -d "$binaries_dest_dir/python3" ]; then
        mkdir -p "$binaries_dest_dir/python3/lib"
        rsync -a "$PYTHONHOME_global/lib/$Python3_VERSION" "$binaries_dest_dir/python3/lib"
        rsync -a "$brew_prefix/lib/$Python3_VERSION" "$binaries_dest_dir/python3/lib"
        #rsync -a "$(eval echo "$brew_prefix/lib/python*")" $binaries_dest_dir/python3/lib
    fi
elif [ $use_vcpkg = true ]; then
    [ -d $destination_dir/bin/python3 ]     || mkdir $destination_dir/bin/python3
    [ -d $destination_dir/bin/python3/lib ] || mkdir $destination_dir/bin/python3/lib
    python_lib_dir="$vcpkg_installed_dir/$(ls --ignore=vcpkg $vcpkg_installed_dir)/lib/$Python3_VERSION"
    rsync -a "$python_lib_dir" $destination_dir/bin/python3/lib
    #rsync -a "$(eval echo "$vcpkg_installed_dir/$(ls --ignore=vcpkg $vcpkg_installed_dir)/lib/python*")" $destination_dir/python3/lib
fi

# Copy the docs to the destination directory.
cp "README.md" "$destination_dir"
if [ ! -d "$destination_dir/LICENSE" ]; then
    mkdir -p "$destination_dir/LICENSE"
    cp -r "docs/license-libraries/." "$destination_dir/LICENSE/"
    cp -r "LICENSE" "$destination_dir/LICENSE/LICENSE-linevis.txt"
    cp -r "submodules/IsosurfaceCpp/LICENSE" "$destination_dir/LICENSE/graphics/LICENSE-isosurfacecpp.txt"
fi
if [ ! -d "$destination_dir/docs" ]; then
    cp -r "docs" "$destination_dir"
fi

# Create a run script.
if $use_msys; then
    printf "@echo off\npushd %%~dp0\npushd bin\nstart \"\" LineVis.exe\n" > "$destination_dir/run.bat"
elif $use_macos; then
    printf "#!/bin/sh\npushd \"\$(dirname \"\$0\")\" >/dev/null\n./LineVis.app/Contents/MacOS/LineVis\npopd\n" > "$destination_dir/run.sh"
    chmod +x "$destination_dir/run.sh"
else
    printf "#!/bin/bash\npushd \"\$(dirname \"\$0\")/bin\" >/dev/null\n./LineVis\npopd\n" > "$destination_dir/run.sh"
    chmod +x "$destination_dir/run.sh"
fi



# Run the program as the last step.
echo ""
echo "All done!"
pushd $build_dir >/dev/null

if $use_msys; then
    if [[ -z "${PATH+x}" ]]; then
        export PATH="${projectpath}/third_party/sgl/install/bin"
    elif [[ ! "${PATH}" == *"${projectpath}/third_party/sgl/install/bin"* ]]; then
        export PATH="${projectpath}/third_party/sgl/install/bin:$PATH"
    fi
elif $use_macos; then
    if [ -z "${DYLD_LIBRARY_PATH+x}" ]; then
        export DYLD_LIBRARY_PATH="${projectpath}/third_party/sgl/install/lib"
    elif contains "${DYLD_LIBRARY_PATH}" "${projectpath}/third_party/sgl/install/lib"; then
        export DYLD_LIBRARY_PATH="DYLD_LIBRARY_PATH:${projectpath}/third_party/sgl/install/lib"
    fi
else
  if [[ -z "${LD_LIBRARY_PATH+x}" ]]; then
      export LD_LIBRARY_PATH="${projectpath}/third_party/sgl/install/lib"
  elif [[ ! "${LD_LIBRARY_PATH}" == *"${projectpath}/third_party/sgl/install/lib"* ]]; then
      export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${projectpath}/third_party/sgl/install/lib"
  fi
fi
if [ $use_macos = true ] && [ $use_vcpkg = true ]; then
    export PYTHONHOME="$PYTHONHOME_global"
elif [ $use_macos = true ] && [ $use_vcpkg = false ]; then
    export PYTHONHOME="../$destination_dir/python3"
elif $use_msys; then
    export PYTHONHOME="/mingw64"
else
    if [ $use_vcpkg = true ]; then
        export PYTHONHOME="../Shipping/bin/python3"
    fi
fi
if [ $use_macos = false ] && [ $use_msys = false ]; then
    if ! $is_ospray_installed && [ $os_arch = "x86_64" ]; then
        export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${projectpath}/third_party/ospray-${ospray_version}.x86_64.linux/lib"
    fi
fi

if [ $run_program = true ] && [ $use_macos = false ]; then
    ./LineVis ${params_run[@]+"${params_run[@]}"}
elif [ $run_program = true ] && [ $use_macos = true ]; then
    #open ./LineVis.app
    #open ./LineVis.app --args --perf
    ./LineVis.app/Contents/MacOS/LineVis ${params_run[@]+"${params_run[@]}"}
fi
