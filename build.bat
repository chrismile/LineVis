@echo off
pushd %~dp0

set debug=true
set destination_dir=Shipping

where cmake >NUL 2>&1 || echo cmake was not found && exit /b 1

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

if not exist .\third_party\vcpkg_export\ (
   echo ------------------------
   echo    getting packages
   echo ------------------------

   where vcpkg >NUL 2>&1 || echo vcpkg was not found && exit /b 1

   vcpkg install %dependencies%       --triplet=x64-windows
   vcpkg export  %dependencies% --raw --triplet=x64-windows ^
                                      --output="%~dp0third_party\vcpkg_export" || exit /b 1
) else (
  echo Packages were found
)

if %debug% == true (
  echo ------------------------
  echo   building in debug
  echo ------------------------

  set build_dir=".build\debug"
  set cmake_config="Debug"
  set cmake_sgl_dir="third_party/sgl/Debug/lib/cmake/sgl/"

) else (
  echo ------------------------
  echo   building in release
  echo ------------------------

  set build_dir=".build\release"
  set cmake_config="Release"
  set cmake_sgl_dir="third_party/sgl/Release/lib/cmake/sgl/"
)

echo ------------------------
echo       generating
echo ------------------------
cmake -S . -B %build_dir% ^
  -DCMAKE_TOOLCHAIN_FILE="third_party/vcpkg_export/scripts/buildsystems/vcpkg.cmake" ^
  -Dsgl_DIR=%cmake_sgl_dir%

echo ------------------------
echo       compiling
echo ------------------------
cmake --build %build_dir% --config %cmake_config% --parallel || exit /b 1

echo ------------------------
echo    copying new files
echo ------------------------
robocopy third_party\vcpkg_export\installed\x64-windows\tools\python3 ^
         %destination_dir%\python3 /e >NUL

if %debug% == true (
  robocopy %build_dir%\Debug\                                        %destination_dir% >NUL
  robocopy third_party\vcpkg_export\installed\x64-windows\debug\bin\ %destination_dir% >NUL
  robocopy third_party\sgl\Debug\bin                                 %destination_dir% >NUL
) else (
  robocopy %build_dir%\Release\                                      %destination_dir% >NUL
  robocopy third_party\vcpkg_export\installed\x64-windows\bin\       %destination_dir% >NUL
  robocopy third_party\sgl\Release\bin                               %destination_dir% sgl.dll >NUL
  del %destination_dir%\*.pdb
)

echo.
echo All done!
