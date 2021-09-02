:: BSD 2-Clause License
::
:: Copyright (c) 2021, Felix Brendel, Christoph Neuhauser
:: All rights reserved.
::
:: Redistribution and use in source and binary forms, with or without
:: modification, are permitted provided that the following conditions are met:
::
:: 1. Redistributions of source code must retain the above copyright notice, this
::    list of conditions and the following disclaimer.
::
:: 2. Redistributions in binary form must reproduce the above copyright notice,
::    this list of conditions and the following disclaimer in the documentation
::    and/or other materials provided with the distribution.
::
:: THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
:: AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
:: IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
:: DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
:: FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
:: DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
:: SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
:: CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
:: OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
:: OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@echo off
pushd %~dp0

set debug=true
set destination_dir=Shipping

where cmake >NUL 2>&1 || echo cmake was not found but is required to build the program && exit /b 1

set dependencies="boost-algorithm"                      ^
                 "boost-core"                           ^
                 "boost-filesystem"                     ^
                 "boost-interprocess"                   ^
                 "boost-locale"                         ^
                 "cppzmq"                               ^
                 "glew"                                 ^
                 "glm"                                  ^
                 "jsoncpp"                              ^
                 "libarchive[bzip2,core,lz4,lzma,zstd]" ^
                 "libpng"                               ^
                 "netcdf-c"                             ^
                 "python3"                              ^
                 "sdl2-image"                           ^
                 "sdl2[vulkan]"                         ^
                 "shaderc"                              ^
                 "tinyxml2"                             ^
                 "vulkan"                               ^
                 "vulkan-headers"

if not exist .\third_party\ (
   echo ------------------------
   echo    getting packages
   echo ------------------------

   where git >NUL 2>&1 || echo git was not in the path but is required for installing the packages && exit /b 1

   mkdir .\third_party\
   pushd third_party

   git clone --depth 1 https://github.com/chrismile/sgl.git   || exit /b 1
   git clone --depth 1 https://github.com/Microsoft/vcpkg.git || exit /b 1
   call vcpkg\bootstrap-vcpkg.bat -disableMetrics             || exit /b 1
   vcpkg\vcpkg install %dependencies% --triplet=x64-windows   || exit /b 1

   echo ------------------------
   echo      building sgl
   echo ------------------------
   mkdir sgl\.build
   pushd sgl\.build

   cmake .. -DCMAKE_TOOLCHAIN_FILE="../../vcpkg/scripts/buildsystems/vcpkg.cmake" ^
            -DCMAKE_INSTALL_PREFIX=../install  || exit /b 1

   :: build and prepare debug sgl
   cmake --build . --config Debug --parallel        || exit /b 1
   cmake --build . --config Debug --target install  || exit /b 1

   :: build and prepare release sgl
   cmake --build . --config Release --parallel        || exit /b 1
   cmake --build . --config Release --target install  || exit /b 1

   :: back to project dir
   popd
   popd
) else (
   echo Packages were found
)

if %debug% == true (
   echo ------------------------
   echo   building in debug
   echo ------------------------

   set build_dir=".build\debug"
   set cmake_config="Debug"
   set cmake_sgl_dir="third_party/sgl/install/debug/lib/cmake/sgl/"
) else (
   echo ------------------------
   echo   building in release
   echo ------------------------

   set build_dir=".build\release"
   set cmake_config="Release"
   set cmake_sgl_dir="third_party/sgl/install/release/lib/cmake/sgl/"
)

echo ------------------------
echo       generating
echo ------------------------
cmake -DCMAKE_TOOLCHAIN_FILE="third_party/vcpkg/scripts/buildsystems/vcpkg.cmake" -DPYTHONHOME="./python3" -Dsgl_DIR="third_party/sgl/install/lib/cmake/sgl/" -S . -B %build_dir%

echo ------------------------
echo       compiling
echo ------------------------
cmake --build %build_dir% --config %cmake_config% --parallel || exit /b 1

echo ------------------------
echo    copying new files
echo ------------------------
robocopy third_party\vcpkg\installed\x64-windows\tools\python3 ^
         %destination_dir%\python3 /e >NUL

if %debug% == true (
   robocopy %build_dir%\Debug\                                 %destination_dir% >NUL
   robocopy third_party\vcpkg\installed\x64-windows\debug\bin\ %destination_dir% >NUL
   robocopy third_party\sgl\install\bin                        %destination_dir% sgld.dll sgld.pdb >NUL
) else (
   robocopy %build_dir%\Release\                         %destination_dir% >NUL
   robocopy third_party\vcpkg\installed\x64-windows\bin\ %destination_dir% >NUL
   robocopy third_party\sgl\install\bin                  %destination_dir% sgl.dll >NUL
   del %destination_dir%\*.pdb
)

echo.
echo All done!
