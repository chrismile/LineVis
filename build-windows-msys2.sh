#!/bin/bash

# BSD 2-Clause License
#
# Copyright (c) 2021, Christoph Neuhauser, Felix Brendel
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

is_installed_pacman() {
    local pkg_name="$1"
    if pacman -Qs $pkg_name > /dev/null; then
        return 0
    else
        return 1
    fi
}

if command -v pacman &> /dev/null && [ ! -d $build_dir_debug ] && [ ! -d $build_dir_release ]; then
    if ! command -v cmake &> /dev/null || ! command -v git &> /dev/null \
            || ! command -v curl &> /dev/null || ! command -v wget &> /dev/null \
            || ! command -v pkg-config &> /dev/null || ! command -v g++ &> /dev/null; then
        echo "------------------------"
        echo "installing build essentials"
        echo "------------------------"
        pacman --noconfirm -S --needed make git curl wget mingw64/mingw-w64-x86_64-cmake \
        mingw64/mingw-w64-x86_64-gcc mingw64/mingw-w64-x86_64-gdb
    fi

    if ! is_installed_pacman "mingw-w64-x86_64-glm" \
            || ! is_installed_pacman "mingw-w64-x86_64-libpng" \
            || ! is_installed_pacman "mingw-w64-x86_64-tinyxml2" \
            || ! is_installed_pacman "mingw-w64-x86_64-boost" \
            || ! is_installed_pacman "mingw-w64-x86_64-libarchive" \
            || ! is_installed_pacman "mingw-w64-x86_64-SDL2" \
            || ! is_installed_pacman "mingw-w64-x86_64-SDL2_image" \
            || ! is_installed_pacman "mingw-w64-x86_64-glew" \
            || ! is_installed_pacman "mingw-w64-x86_64-vulkan-headers" \
            || ! is_installed_pacman "mingw-w64-x86_64-vulkan-loader" \
            || ! is_installed_pacman "mingw-w64-x86_64-vulkan-validation-layers" \
            || ! is_installed_pacman "mingw-w64-x86_64-shaderc" \
            || ! is_installed_pacman "mingw-w64-x86_64-jsoncpp" \
            || ! is_installed_pacman "mingw-w64-x86_64-netcdf" \
            || ! is_installed_pacman "mingw-w64-x86_64-zeromq" \
            || ! is_installed_pacman "mingw-w64-x86_64-eigen3" \
            || ! is_installed_pacman "mingw-w64-x86_64-python" \
            || ! is_installed_pacman "mingw-w64-x86_64-python-numpy"; then
        echo "------------------------"
        echo "installing dependencies "
        echo "------------------------"
        pacman --noconfirm -S --needed \
        mingw64/mingw-w64-x86_64-glm mingw64/mingw-w64-x86_64-libpng mingw64/mingw-w64-x86_64-tinyxml2 \
        mingw64/mingw-w64-x86_64-boost mingw64/mingw-w64-x86_64-libarchive \
        mingw64/mingw-w64-x86_64-SDL2 mingw64/mingw-w64-x86_64-SDL2_image mingw64/mingw-w64-x86_64-glew \
        mingw64/mingw-w64-x86_64-vulkan-headers mingw64/mingw-w64-x86_64-vulkan-loader \
        mingw64/mingw-w64-x86_64-vulkan-validation-layers mingw64/mingw-w64-x86_64-shaderc \
        mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-netcdf mingw64/mingw-w64-x86_64-zeromq \
        mingw64/mingw-w64-x86_64-eigen3 mingw64/mingw-w64-x86_64-python mingw64/mingw-w64-x86_64-python-numpy	
    fi
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
         -G "MSYS Makefiles" \
         -DCMAKE_BUILD_TYPE=Debug \
         -DCMAKE_INSTALL_PREFIX="../install"
    make -j $(nproc)
    make install
    popd >/dev/null

    pushd $build_dir_release >/dev/null
    cmake .. \
         -G "MSYS Makefiles" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="../install"
    make -j $(nproc)
    make install
    popd >/dev/null

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
ls "$build_dir"

echo "------------------------"
echo "      generating        "
echo "------------------------"
python_name="$(find "$MSYSTEM_PREFIX/lib/" -maxdepth 1 -type d -name 'python*' -printf "%f" -quit)"
pushd $build_dir >/dev/null
cmake .. \
    -G "MSYS Makefiles" \
    -DPython3_FIND_REGISTRY=NEVER \
    -DCMAKE_BUILD_TYPE=$cmake_config \
    -Dsgl_DIR="$PROJECTPATH/third_party/sgl/install/lib/cmake/sgl/" \
    -DPYTHONHOME="./python3" \
    -DPYTHONPATH="./python3/lib/$python_name"
popd >/dev/null

echo "------------------------"
echo "      compiling         "
echo "------------------------"
pushd "$build_dir" >/dev/null
make -j $(nproc)
popd >/dev/null

echo ""


echo "------------------------"
echo "   copying new files    "
echo "------------------------"
mkdir -p $destination_dir/bin

# Copy sgl to the destination directory.
if [ $debug = true ] ; then
    cp "./third_party/sgl/install/bin/libsgld.dll" "$destination_dir/bin"
else
    cp "./third_party/sgl/install/bin/libsgl.dll" "$destination_dir/bin"
fi

# Copy LineVis to the destination directory.
cp "$build_dir/LineVis.exe" "$destination_dir/bin"
cp "README.md" "$destination_dir"

# Copy all dependencies of LineVis to the destination directory.
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
	    python_name=${tmp%.dll}
    fi
done

# Copy libopenblas (needed by numpy) and its dependencies to the destination directory.
cp "$MSYSTEM_PREFIX/bin/libopenblas.dll" "$destination_dir/bin"
ldd_output="$(ldd "$MSYSTEM_PREFIX/bin/libopenblas.dll")"
for library in $ldd_output
do
    if [[ $library == "$MSYSTEM_PREFIX"* ]] ;
    then
		cp "$library" "$destination_dir/bin"
    fi
done

# Copy python3 to the destination directory.
if [ ! -d "$destination_dir/bin/python3" ]; then
    mkdir -p "$destination_dir/bin/python3/lib"
    cp -r "$MSYSTEM_PREFIX/lib/$python_name" "$destination_dir/bin/python3/lib"
fi
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
printf "@echo off\npushd %%~dp0\npushd bin\nstart \"\" LineVis.exe\n" > "$destination_dir/run.bat"




# Run the program as the last step.
echo "All done!"
pushd $build_dir >/dev/null

if [[ -z "${PATH+x}" ]]; then
    export PATH="${PROJECTPATH}/third_party/sgl/install/bin"
elif [[ ! "${PATH}" == *"${PROJECTPATH}/third_party/sgl/install/bin"* ]]; then
    export PATH="${PROJECTPATH}/third_party/sgl/install/bin:$PATH"
fi
export PYTHONHOME="/mingw64"
./LineVis

