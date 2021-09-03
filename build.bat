@echo off
pushd %~dp0

set debug=false
set build_dir=".build"
set destination_dir="Shipping"

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

if not exist .\third_party\ mkdir .\third_party\
pushd third_party

if not exist .\vcpkg (
   echo ------------------------
   echo    fetching vkpkg
   echo ------------------------
   git clone --depth 1 https://github.com/Microsoft/vcpkg.git || exit /b 1
   call vcpkg\bootstrap-vcpkg.bat -disableMetrics             || exit /b 1
   vcpkg\vcpkg install %dependencies% --triplet=x64-windows   || exit /b 1
)

if not exist .\sgl (
   echo ------------------------
   echo      fetching sgl
   echo ------------------------
   git clone --depth 1 https://github.com/chrismile/sgl.git   || exit /b 1
)

if not exist .\sgl\install (
   echo ------------------------
   echo      building sgl
   echo ------------------------
   mkdir sgl\.build 2> NUL
   pushd sgl\.build

   cmake .. -DCMAKE_TOOLCHAIN_FILE=../../vcpkg/scripts/buildsystems/vcpkg.cmake ^
            -DCMAKE_INSTALL_PREFIX=../install         || exit /b 1
   cmake --build . --config Debug   --parallel        || exit /b 1
   cmake --build . --config Debug   --target install  || exit /b 1
   cmake --build . --config Release --parallel        || exit /b 1
   cmake --build . --config Release --target install  || exit /b 1

   popd
)

popd

if %debug% == true (
   echo ------------------------
   echo   building in debug
   echo ------------------------

   set cmake_config="Debug"
) else (
   echo ------------------------
   echo   building in release
   echo ------------------------

   set cmake_config="Release"
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
   if not exist %destination_dir%\*.pdb (
      del %destination_dir%\*.dll
   )
   robocopy %build_dir%\Debug\             %destination_dir%  >NUL
   robocopy third_party\sgl\.build\Debug   %destination_dir% *.dll *.pdb >NUL
) else (
   if exist %destination_dir%\*.pdb (
      del %destination_dir%\*.dll
      del %destination_dir%\*.pdb
   )
   robocopy %build_dir%\Release\           %destination_dir% >NUL
   robocopy third_party\sgl\.build\Release %destination_dir% *.dll >NUL
)

echo.
echo All done!
