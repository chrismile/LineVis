# BSD 2-Clause License
#
# Copyright (c) 2021, Felix Brendel, Christoph Neuhauser
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
pushd $SCRIPTPATH > /dev/null

debug=true
build_dir_debug=".build_debug"
build_dir_release=".build_release"
destination_dir="Shipping"

if ! which cmake >/dev/null 2>&1; then
   echo "cmake was not found but is required to build the program"
   exit 1
fi

[ -d "./third_party/" ] || mkdir "./third_party/"
pushd third_party > /dev/null

if [ ! -d "./submodules/IsosurfaceCpp/src" ]; then
    echo "------------------------"
    echo "initializing submodules "
    echo "------------------------"
    git submodule init
    git submodule update
fi

if [ ! -d "./vcpkg" ]; then
    echo "------------------------"
    echo "   fetching vkpkg       "
    echo "------------------------"
    if [[ ! -v VULKAN_SDK ]]; then
        echo "The environment variable VULKAN_SDK is not set but is required in the installation process"
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
    cmake -S . -B .build_debug                                                 \
         -DCMAKE_BUILD_TYPE=Debug                                              \
         -DCMAKE_TOOLCHAIN_FILE="../../vcpkg/scripts/buildsystems/vcpkg.cmake" \
         -DCMAKE_INSTALL_PREFIX="./install"
    cmake -S . -B .build_release                                              \
        -DCMAKE_BUILD_TYPE=Release                                            \
        -DCMAKE_TOOLCHAIN_FILE="../../vcpkg/scripts/buildsystems/vcpkg.cmake" \
        -DCMAKE_INSTALL_PREFIX="./install"

    cmake --build .build_debug --parallel
    cmake --build .build_debug --target install

    cmake --build .build_release --parallel
    cmake --build .build_release --target install

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

echo "------------------------"
echo "      generating        "
echo "------------------------"
cmake -DCMAKE_TOOLCHAIN_FILE="third_party/vcpkg/scripts/buildsystems/vcpkg.cmake" \
      -DPYTHONHOME="./python3"                                                    \
      -DCMAKE_BUILD_TYPE=$cmake_config                                            \
      -Dsgl_DIR="third_party/sgl/install/lib/cmake/sgl/" -S . -B $build_dir

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

rsync -a ".build/vcpkg_installed/x64-linux/lib/python3.9"  $destination_dir/python3/lib
rsync -a $build_dir/LineVis $destination_dir

echo ""
echo "All done!"
