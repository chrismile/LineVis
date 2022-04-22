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

is_installed_brew() {
    local pkg_name="$1"
    if brew list $pkg_name > /dev/null; then
        return 0
    else
        return 1
    fi
}

if ! command -v brew &> /dev/null; then
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

if command -v brew &> /dev/null; then
    if ! is_installed_brew "git"; then
        brew install git
    fi
    if ! is_installed_brew "cmake"; then
        brew install cmake
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

    # Homebrew MoltenVK does not contain script for setting environment variables, unfortunately.
    #if ! is_installed_brew "molten-vk"; then
    #    brew install molten-vk
    #fi
    if ! is_installed_brew "zlib"; then
        brew install zlib
    fi
    if ! is_installed_brew "libpng"; then
        brew install libpng
    fi
    if ! is_installed_brew "glm"; then
        brew install glm
    fi
    if ! is_installed_brew "sdl2"; then
        brew install sdl2
    fi
    if ! is_installed_brew "sdl2_image"; then
        brew install sdl2_image
    fi
    if ! is_installed_brew "libarchive"; then
        brew install libarchive
    fi
    if ! is_installed_brew "boost"; then
        brew install boost
    fi
    if ! is_installed_brew "tinyxml2"; then
        brew install tinyxml2
    fi

    if ! is_installed_brew "jsoncpp"; then
        brew install jsoncpp
    fi
    if ! is_installed_brew "eigen"; then
        brew install eigen
    fi
    if ! is_installed_brew "openexr"; then
        brew install openexr
    fi
    if ! is_installed_brew "netcdf"; then
        brew install netcdf
    fi
    if ! is_installed_brew "zeromq"; then
        brew install zeromq
    fi
    if ! is_installed_brew "cppzmq"; then
        brew install cppzmq
    fi
    if ! is_installed_brew "python@3.10"; then
        brew install python@3.10
    fi
    if ! is_installed_brew "numpy"; then
        brew install numpy
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

if [ ! -d "submodules/IsosurfaceCpp/src" ]; then
    echo "------------------------"
    echo "initializing submodules "
    echo "------------------------"
    git submodule init
    git submodule update
fi

[ -d "./third_party/" ] || mkdir "./third_party/"
pushd third_party > /dev/null

if [ -z "${VULKAN_SDK+1}" ]; then
    echo "------------------------"
    echo "searching for Vulkan SDK"
    echo "------------------------"

    found_vulkan=false

    if [ -d "$HOME/VulkanSDK" ]; then
        source "$HOME/VulkanSDK/$(ls $HOME/VulkanSDK)/setup-env.sh"
        found_vulkan=true
    else
      VULKAN_SDK_VERSION=1.3.204.1
      curl -O https://sdk.lunarg.com/sdk/download/$VULKAN_SDK_VERSION/mac/vulkansdk-macos-$VULKAN_SDK_VERSION.dmg
      sudo hdiutil attach vulkansdk-macos-$VULKAN_SDK_VERSION.dmg
      sudo /Volumes/vulkansdk-macos-$VULKAN_SDK_VERSION/InstallVulkan.app/Contents/MacOS/InstallVulkan \
      --root ~/VulkanSDK/$VULKAN_SDK_VERSION --accept-licenses --default-answer --confirm-command install
      pushd ~/VulkanSDK/$VULKAN_SDK_VERSION
      sudo python3 ./install_vulkan.py
      popd
      sudo hdiutil unmount /Volumes/vulkansdk-macos-$VULKAN_SDK_VERSION
      source "$HOME/VulkanSDK/$(ls $HOME/VulkanSDK)/setup-env.sh"
      found_vulkan=true
    fi

    if ! $found_vulkan; then
        echo "The environment variable VULKAN_SDK is not set but is required in the installation process."
        echo "Please refer to https://vulkan.lunarg.com/sdk/home#mac for instructions on how to install the Vulkan SDK."
        exit 1
    fi
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
         -DCMAKE_FIND_USE_CMAKE_SYSTEM_PATH=False -DCMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH=False \
         -DCMAKE_FIND_FRAMEWORK=LAST -DCMAKE_FIND_APPBUNDLE=NEVER -DZLIB_ROOT="/usr/local/opt/zlib" \
         -DCMAKE_PREFIX_PATH="$(brew --prefix)" -DCMAKE_INSTALL_PREFIX="../install"
    popd >/dev/null

    pushd $build_dir_release >/dev/null
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_FIND_USE_CMAKE_SYSTEM_PATH=False -DCMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH=False \
        -DCMAKE_FIND_FRAMEWORK=LAST -DCMAKE_FIND_APPBUNDLE=NEVER -DZLIB_ROOT="/usr/local/opt/zlib" \
        -DCMAKE_PREFIX_PATH="$(brew --prefix)" -DCMAKE_INSTALL_PREFIX="../install"
    popd >/dev/null

    cmake --build $build_dir_debug --parallel $(sysctl -n hw.ncpu)
    cmake --build $build_dir_debug --target install

    cmake --build $build_dir_release --parallel $(sysctl -n hw.ncpu)
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
cmake -DCMAKE_FIND_USE_CMAKE_SYSTEM_PATH=False -DCMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH=False \
      -DCMAKE_FIND_FRAMEWORK=LAST -DCMAKE_FIND_APPBUNDLE=NEVER -DZLIB_ROOT="/usr/local/opt/zlib" \
      -DCMAKE_PREFIX_PATH="$(brew --prefix)" \
      -DPYTHONHOME="./python3" \
      -DCMAKE_BUILD_TYPE=$cmake_config \
      -Dsgl_DIR="$PROJECTPATH/third_party/sgl/install/lib/cmake/sgl/" ..
Python3_VERSION=$(cat pythonversion.txt)
popd >/dev/null

echo "------------------------"
echo "      compiling         "
echo "------------------------"
cmake --build $build_dir --parallel $(sysctl -n hw.ncpu)


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

brew_prefix="$(brew --prefix)"
mkdir -p $destination_dir/bin

# Copy sgl to the destination directory.
if [ $debug = true ] ; then
    cp "./third_party/sgl/install/lib/libsgld.dylib" "$destination_dir/bin"
else
    cp "./third_party/sgl/install/lib/libsgl.dylib" "$destination_dir/bin"
fi

# Copy LineVis to the destination directory.
cp "$build_dir/LineVis" "$destination_dir/bin"
cp "README.md" "$destination_dir"

# Copy all dependencies of LineVis and sgl to the destination directory.
rsync -a "$VULKAN_SDK/lib/libMoltenVK.dylib" "$destination_dir/bin"
copy_dependencies_recursive() {
    local binary_path="$1"
    local binary_name=$(basename "$binary_path")
    local binary_target_path="$destination_dir/bin/$binary_name"
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
        local library_target_path="$destination_dir/bin/$library_name"
        if ! startswith "$library" "@rpath/" \
            && ! startswith "$library" "@loader_path/" \
            && ! startswith "$library" "/System/Library/Frameworks/" \
            && ! startswith "$library" "/usr/lib/"
        then
            install_name_tool -change "$library" "@executable_path/$library_name" "$binary_target_path"

            if [ ! -f "$library_target_path" ]; then
                cp "$library" "$destination_dir/bin"
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
                    local library_rpath="${rpath}${library#"@rpath"}"

                    if [ -f "$library_rpath" ]; then
                        if [ ! -f "$library_target_path" ]; then
                            cp "$library_rpath" "$destination_dir/bin"
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
copy_dependencies_recursive "$build_dir/LineVis"
if [ $debug = true ]; then
    copy_dependencies_recursive "./third_party/sgl/install/lib/libsgld.dylib"
else
    copy_dependencies_recursive "./third_party/sgl/install/lib/libsgl.dylib"
fi

# Fix code signing for arm64.
for filename in $destination_dir/bin/*
do
    if contains "$(file "$filename")" "arm64"; then
        codesign --force -s - "$filename" &> /dev/null
    fi
done

# Copy python3 to the destination directory.
if [ ! -d "$destination_dir/bin/python3" ]; then
    mkdir -p "$destination_dir/bin/python3/lib"
    rsync -a "$brew_prefix/lib/$Python3_VERSION" "$destination_dir/bin/python3/lib"
    #rsync -a "$(eval echo "$brew_prefix/lib/python*")" $destination_dir/python3/lib
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
printf "#!/bin/sh\npushd bin >/dev/null\n./LineVis\npopd\n" > "$destination_dir/run.sh"
chmod +x "$destination_dir/run.sh"

echo ""
echo "All done!"


pushd $build_dir >/dev/null

if [ -z "${DYLD_LIBRARY_PATH+x}" ]; then
    export DYLD_LIBRARY_PATH="${PROJECTPATH}/third_party/sgl/install/lib"
elif contains "${DYLD_LIBRARY_PATH}" "${PROJECTPATH}/third_party/sgl/install/lib"; then
    export DYLD_LIBRARY_PATH="DYLD_LIBRARY_PATH:${PROJECTPATH}/third_party/sgl/install/lib"
fi
./LineVis
