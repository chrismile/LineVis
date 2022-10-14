:: BSD 2-Clause License
::
:: Copyright (c) 2021-2022, Christoph Neuhauser, Felix Brendel
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
setlocal
pushd %~dp0

set debug=false
set build_dir=".build"
set destination_dir="Shipping"

:: Leave empty to let cmake try to find the correct paths
set optix_install_dir=""

where cmake >NUL 2>&1 || echo cmake was not found but is required to build the program && exit /b 1

:: Creates a string with, e.g., -G "Visual Studio 17 2022".
:: Needs to be run from a Visual Studio developer PowerShell or command prompt.
set VCINSTALLDIR_ESC=%VCINSTALLDIR:\=\\%
set "x=%VCINSTALLDIR_ESC:Microsoft Visual Studio\\=" & set "VsPathEnd=%"
set VsYear=%VsPathEnd:~0,4%
set cmake_generator=-G "Visual Studio %VisualStudioVersion:~0,2% %VsYear%"

if not exist .\third_party\ mkdir .\third_party\
pushd third_party

if not exist .\submodules\IsosurfaceCpp\src (
   echo ------------------------
   echo initializing submodules
   echo ------------------------
   git submodule init   || exit /b 1
   git submodule update || exit /b 1
)

if not exist .\vcpkg (
   echo ------------------------
   echo    fetching vcpkg
   echo ------------------------
   git clone --depth 1 https://github.com/Microsoft/vcpkg.git || exit /b 1
   call vcpkg\bootstrap-vcpkg.bat -disableMetrics             || exit /b 1
   vcpkg\vcpkg install --triplet=x64-windows                  || exit /b 1
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

   cmake .. %cmake_generator% ^
            -DCMAKE_TOOLCHAIN_FILE=../../vcpkg/scripts/buildsystems/vcpkg.cmake ^
            -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_CXX_FLAGS="/MP" || exit /b 1
   cmake --build . --config Debug   -- /m            || exit /b 1
   cmake --build . --config Debug   --target install || exit /b 1
   cmake --build . --config Release -- /m            || exit /b 1
   cmake --build . --config Release --target install || exit /b 1

   popd
)

set cmake_args=-DCMAKE_TOOLCHAIN_FILE="third_party/vcpkg/scripts/buildsystems/vcpkg.cmake" ^
               -DPYTHONHOME="./python3"                                                    ^
               -DCMAKE_CXX_FLAGS="/MP"                                                     ^
               -Dsgl_DIR="third_party/sgl/install/lib/cmake/sgl/"

set embree_version=3.13.3
if not exist ".\embree-%embree_version%.x64.vc14.windows" (
    echo ------------------------
    echo    downloading Embree
    echo ------------------------
    curl.exe -L "https://github.com/embree/embree/releases/download/v%embree_version%/embree-%embree_version%.x64.vc14.windows.zip" --output embree-%embree_version%.x64.vc14.windows.zip
    tar -xvzf "embree-%embree_version%.x64.vc14.windows.zip"
)
set cmake_args=%cmake_args% -Dembree_DIR="third_party/embree-%embree_version%.x64.vc14.windows/lib/cmake/embree-%embree_version%"

set ospray_version=2.9.0
if not exist ".\ospray-%ospray_version%.x86_64.windows" (
    echo ------------------------
    echo   downloading OSPRay
    echo ------------------------
    curl.exe -L "https://github.com/ospray/OSPRay/releases/download/v%ospray_version%/ospray-%ospray_version%.x86_64.windows.zip" --output ospray-%ospray_version%.x86_64.windows.zip
    tar -xvzf "ospray-%ospray_version%.x86_64.windows.zip"
)
set cmake_args=%cmake_args% -Dospray_DIR="third_party/ospray-%ospray_version%.x86_64.windows/lib/cmake/ospray-%ospray_version%"

set eccodes_version=2.26.0
if not exist ".\eccodes-%eccodes_version%-Source" (
    echo ------------------------
    echo   downloading ecCodes
    echo ------------------------
    curl.exe -L "https://confluence.ecmwf.int/download/attachments/45757960/eccodes-%eccodes_version%-Source.tar.gz?api=v2" --output eccodes-%eccodes_version%-Source.tar.gz
    tar -xvzf "eccodes-%eccodes_version%-Source.tar.gz"
)

:: ecCodes needs bash.exe, but it is just a dummy pointing the caller to WSL if not installed.
:: ecCodes is disabled for now due to build errors on Windows.
set use_eccodes=false
:: set bash_output=empty
:: for /f %%i in ('where bash') do set bash_location=%%i
:: IF "%bash_location%"=="" (
::     set bash_location=C:\Windows\System32\bash.exe
:: )
:: IF EXIST "%bash_location%" (
::     goto bash_exists
:: ) ELSE (
::     goto system_bash_not_exists
:: )
::
:: :: goto circumvents problems when not using EnableDelayedExpansion.
:: :bash_exists
:: set bash_output=empty
:: for /f %%i in ('"%bash_location%" --version') do set bash_output=%%i
:: IF "%bash_output%"=="" (
::     set bash_output=empty
:: )
:: :: Output is usually https://aka.ms/wslstore when WSL is not installed.
:: if not x%bash_output:wsl=%==x%bash_output% (
::     set use_eccodes=false
:: ) ELSE (
::     set use_eccodes=true
:: )
:: goto finished
::
:: :system_bash_not_exists
:: set bash_location="C:/Program Files/Git/bin/bash.exe"
:: IF EXIST "%bash_location%" (
::     set "PATH=C:\Program Files\Git\bin;%PATH%"
::     goto bash_exists
:: ) ELSE (
::     goto finished
:: )
::
:: :finished
:: echo bash_location: %bash_location%
:: echo use_eccodes: %use_eccodes%

if %use_eccodes% == true if not exist ".\eccodes-%eccodes_version%" (
    echo ------------------------
    echo    building ecCodes
    echo ------------------------
    pushd eccodes-%eccodes_version%-Source
    if not exist .\build\ mkdir .\build\
    pushd build
    cmake .. %cmake_generator% -DCMAKE_INSTALL_PREFIX=../../eccodes-%eccodes_version% -DCMAKE_CXX_FLAGS="/MP" || exit /b 1
    cmake --build . --config Debug   -- /m            || exit /b 1
    cmake --build . --config Debug   --target install || exit /b 1
    cmake --build . --config Release -- /m            || exit /b 1
    cmake --build . --config Release --target install || exit /b 1
    popd
    popd
)
set cmake_args=%cmake_args% -Deccodes_DIR="third_party/eccodes-%eccodes_version%/lib/cmake/eccodes-%eccodes_version%"

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

if not %optix_install_dir% == "" (
   echo Using custom OptiX path
   set cmake_args=%cmake_args% -DOptiX_INSTALL_DIR=%optix_install_dir%
)

cmake %cmake_generator% %cmake_args% -S . -B %build_dir%

echo ------------------------
echo       compiling
echo ------------------------
cmake --build %build_dir% --config %cmake_config% -- /m || exit /b 1

echo ------------------------
echo    copying new files
echo ------------------------
robocopy .build\vcpkg_installed\x64-windows\tools\python3 ^
         %destination_dir%\python3 /e >NUL

if %debug% == true (
   if not exist %destination_dir%\*.pdb (
      del %destination_dir%\*.dll
   )
   robocopy %build_dir%\Debug\  %destination_dir%  >NUL
   robocopy third_party\sgl\.build\Debug %destination_dir% *.dll *.pdb >NUL
   robocopy third_party\ospray-%ospray_version%.x86_64.windows\bin %destination_dir% *.dll >NUL
) else (
   if exist %destination_dir%\*.pdb (
      del %destination_dir%\*.dll
      del %destination_dir%\*.pdb
   )
   robocopy %build_dir%\Release\ %destination_dir% >NUL
   robocopy third_party\sgl\.build\Release %destination_dir% *.dll >NUL
   robocopy third_party\ospray-%ospray_version%.x86_64.windows\bin %destination_dir% *.dll >NUL
)

echo.
echo All done!

pushd %destination_dir%
LineVis.exe
