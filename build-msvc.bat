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

set VSLANG=1033
set run_program=true
set debug=false
set devel=false
set clean=false
set build_dir=.build
set destination_dir=Shipping
set vcpkg_triplet="x64-windows"
:: Leave empty to let cmake try to find the correct paths
set optix_install_dir=""
:: Optionally, the program can be built with Direct3D 12 support for some renderers.
set use_d3d=false

:loop
IF NOT "%1"=="" (
    IF "%1"=="--do-not-run" (
        SET run_program=false
    )
    IF "%1"=="--debug" (
        SET debug=true
    )
    IF "%1"=="--devel" (
        SET devel=true
    )
    IF "%1"=="--clean" (
        SET clean=true
    )
    IF "%1"=="--vcpkg-triplet" (
        SET vcpkg_triplet=%2
        SHIFT
    )
    IF "%1"=="--d3d" (
        SET use_d3d=true
    )
    SHIFT
    GOTO :loop
)

:: Build other configuration and copy sgl DLLs to the build directory.
if %clean% == true (
    echo ------------------------
    echo  cleaning up old files
    echo ------------------------
    rd /s /q "third_party\sgl"
    rd /s /q "third_party\vcpkg"
    for /d %%G in (".build*") do rd /s /q "%%~G"
    rd /s /q "Shipping"
    git submodule update --init --recursive
)

where cmake >NUL 2>&1 || echo cmake was not found but is required to build the program && exit /b 1

:: Creates a string with, e.g., -G "Visual Studio 17 2022".
:: Needs to be run from a Visual Studio developer PowerShell or command prompt.
if defined VCINSTALLDIR (
    set VCINSTALLDIR_ESC=%VCINSTALLDIR:\=\\%
)
if defined VCINSTALLDIR (
    set "x=%VCINSTALLDIR_ESC:Microsoft Visual Studio\\=" & set "VsPathEnd=%"
)
if defined VCINSTALLDIR (
    set cmake_generator=-G "Visual Studio %VisualStudioVersion:~0,2% %VsPathEnd:~0,4%"
)
if not defined VCINSTALLDIR (
    set cmake_generator=
)

if %debug% == true (
    set cmake_config="Debug"
    set cmake_config_opposite="Release"
) else (
    set cmake_config="Release"
    set cmake_config_opposite="Debug"
)

if not exist .\submodules\IsosurfaceCpp\src (
   echo ------------------------
   echo initializing submodules
   echo ------------------------
   git submodule init   || exit /b 1
   git submodule update || exit /b 1
)

if not exist .\third_party\ mkdir .\third_party\
set proj_dir=%~dp0
set third_party_dir=%proj_dir%third_party
pushd third_party

IF "%VULKAN_SDK%"=="" (
  for /D %%F in (C:\VulkanSDK\*) do (
    set VULKAN_SDK=%%F
    goto vulkan_finished
  )
)
:vulkan_finished
if %use_d3d% == true (
    set cmake_args_sgl=%cmake_args_sgl% -DSUPPORT_D3D12=ON -DVCPKG_MANIFEST_FEATURES=d3d12
)

IF "%toolchain_file%"=="" (
    SET use_vcpkg=true
) ELSE (
    SET use_vcpkg=false
)
IF "%toolchain_file%"=="" SET toolchain_file="vcpkg/scripts/buildsystems/vcpkg.cmake"

set cmake_args=%cmake_args% -DCMAKE_TOOLCHAIN_FILE="third_party/%toolchain_file%" ^
-DPYTHONHOME="./python3" ^
               -Dsgl_DIR="third_party/sgl/install/lib/cmake/sgl/"

set cmake_args_general=%cmake_args_general% -DCMAKE_TOOLCHAIN_FILE="%third_party_dir%/%toolchain_file%"

if %use_vcpkg% == true (
    set cmake_args=%cmake_args% -DVCPKG_TARGET_TRIPLET=%vcpkg_triplet% -DCMAKE_CXX_FLAGS="/MP"
    set cmake_args_sgl=-DCMAKE_CXX_FLAGS="/MP"
    set cmake_args_general=%cmake_args_general% -DVCPKG_TARGET_TRIPLET=%vcpkg_triplet%
    if not exist .\vcpkg (
        echo ------------------------
        echo    fetching vcpkg
        echo ------------------------
        git clone --depth 1 -b fix-libarchive-rpath https://github.com/chrismile/vcpkg.git || exit /b 1
        call vcpkg\bootstrap-vcpkg.bat -disableMetrics || exit /b 1
        vcpkg\vcpkg install --triplet=%vcpkg_triplet% || exit /b 1
    )
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
    mkdir sgl\%build_dir% 2> NUL
    pushd sgl\%build_dir%

    cmake .. %cmake_generator% %cmake_args_sgl% %cmake_args_general% ^
            -DCMAKE_INSTALL_PREFIX="%third_party_dir%/sgl/install" || exit /b 1
    if %use_vcpkg% == true (
        if x%vcpkg_triplet:release=%==x%vcpkg_triplet% (
           cmake --build . --config Debug   -- /m            || exit /b 1
           cmake --build . --config Debug   --target install || exit /b 1
        )
        if x%vcpkg_triplet:debug=%==x%vcpkg_triplet% (
           cmake --build . --config Release -- /m            || exit /b 1
           cmake --build . --config Release --target install || exit /b 1
        )
    ) else (
        cmake --build . --config %cmake_config%                  || exit /b 1
        cmake --build . --config %cmake_config% --target install || exit /b 1
    )

    popd
)

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

set oidn_version=2.3.0
if not exist ".\oidn-%oidn_version%.x64.windows" (
    echo ------------------------
    echo downloading OpenImageDenoise
    echo ------------------------
    curl.exe -L "https://github.com/OpenImageDenoise/oidn/releases/download/v%oidn_version%/oidn-%oidn_version%.x64.windows.zip" --output oidn-%oidn_version%.x64.windows.zip
    tar -xvzf "oidn-%oidn_version%.x64.windows.zip"
    del "oidn-%oidn_version%.x64.windows.zip"
)
set cmake_args=%cmake_args% -DOpenImageDenoise_DIR="third_party/oidn-%oidn_version%.x64.windows/lib/cmake/OpenImageDenoise-%oidn_version%"

popd

if %debug% == true (
    echo ------------------------
    echo   building in debug
    echo ------------------------
) else (
    echo ------------------------
    echo   building in release
    echo ------------------------
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
if %use_vcpkg% == true (
    cmake --build %build_dir% --config %cmake_config% -- /m || exit /b 1
) else (
    cmake --build %build_dir% --config %cmake_config%       || exit /b 1
)

echo ------------------------
echo    copying new files
echo ------------------------
robocopy .build\vcpkg_installed\x64-windows\tools\python3 ^
         %destination_dir%\python3 /e >NUL
robocopy third_party\ospray-%ospray_version%.x86_64.windows\bin %destination_dir% *.dll >NUL
if %debug% == true (
    if not exist %destination_dir%\*.pdb (
        del %destination_dir%\*.dll
    )
    robocopy %build_dir%\Debug\ %destination_dir% >NUL
    robocopy third_party\sgl\%build_dir%\Debug %destination_dir% *.dll *.pdb >NUL
) else (
    if exist %destination_dir%\*.pdb (
        del %destination_dir%\*.dll
        del %destination_dir%\*.pdb
    )
    robocopy %build_dir%\Release\ %destination_dir% >NUL
    robocopy third_party\sgl\%build_dir%\Release %destination_dir% *.dll >NUL
)

:: Build other configuration and copy sgl DLLs to the build directory.
if %devel% == true (
    echo ------------------------
    echo   setting up dev files
    echo ------------------------
    if %use_vcpkg% == true (
        cmake --build %build_dir% --config %cmake_config_opposite% -- /m || exit /b 1
    ) else (
        cmake --build %build_dir% --config %cmake_config_opposite%       || exit /b 1
    )
    robocopy third_party\sgl\%build_dir%\Debug %build_dir%\Debug\ *.dll *.pdb >NUL
    robocopy third_party\sgl\%build_dir%\Release %build_dir%\Release\ *.dll >NUL
)

echo.
echo All done!

pushd %destination_dir%

if %run_program% == true (
    LineVis.exe
) else (
    echo Build finished.
)
