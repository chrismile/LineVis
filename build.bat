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
cmake -S . -B %build_dir%

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
