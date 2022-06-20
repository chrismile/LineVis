## Compilation using MSYS2

The build process was tested on Windows 10 and Windows 11 64-bit using MSYS2 and Mingw-w64 (http://www.msys2.org/).

As the first step, the library [sgl](https://github.com/chrismile/sgl) (https://github.com/chrismile/sgl) needs to be
installed somewhere on the system. In the following steps, it is assumed you have set up sgl and installed all of its
dependencies. Please follow the instructions in the file `docs/compilation_vcpkg.md` of sgl for the necessary steps.

After that, all obligatory dependencies can be installed using the following command with MSYS2 and Pacman.

```shell
pacman -S make git wget mingw64/mingw-w64-x86_64-gcc mingw64/mingw-w64-x86_64-gdb mingw64/mingw-w64-x86_64-cmake \
mingw64/mingw-w64-x86_64-glm mingw64/mingw-w64-x86_64-libpng mingw64/mingw-w64-x86_64-tinyxml2 \
mingw64/mingw-w64-x86_64-boost mingw64/mingw-w64-x86_64-libarchive \
mingw64/mingw-w64-x86_64-SDL2 mingw64/mingw-w64-x86_64-SDL2_image mingw64/mingw-w64-x86_64-glew \
mingw64/mingw-w64-x86_64-vulkan-headers mingw64/mingw-w64-x86_64-vulkan-loader \
mingw64/mingw-w64-x86_64-vulkan-validation-layers mingw64/mingw-w64-x86_64-shaderc \
mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-netcdf mingw64/mingw-w64-x86_64-zeromq \
mingw64/mingw-w64-x86_64-eigen3 mingw64/mingw-w64-x86_64-python mingw64/mingw-w64-x86_64-python-numpy \
mingw64/mingw-w64-x86_64-openexr mingw64/mingw-w64-x86_64-eccodes
```

After all dependencies have been set up, the following commands can be used to build the program.

```shell
mkdir build
cd build
rm -rf *
cmake -G "MSYS Makefiles" -Dsgl_DIR=<path-to-sgl> -DPython3_FIND_REGISTRY=NEVER ..
make -j
make install
export PATH=$PATH:"/c/msys64/usr/local/bin"
```

To run the program, please execute the following commands.

```shell
export PYTHONHOME="/mingw64"
./LineVis
```

Please note that, when launching programs using sgl on Windows, either the library path of sgl
(e.g., C:/msys2/usr/local/bin) needs to be included in the PATH variable, or the DLL file needs to be copied to the
application directory containing the executable. When launching the program outside the MSYS2 shell, the MinGW/MSYS2
DLL directories also need to be included in the PATH variable. To permanently modify the MSYS2 PATH variable,
/etc/profile needs to be edited.

If the program was built out-of-source (i.e., the folder `build` does not lie in the source directory), the user must
either create a symbolic link to the directory `Data` in the build folder (this only works on Linux and not Windows),
or the CMake variable `DATA_PATH` must be set to the path pointing to the `Data` folder.


### How to run with CLion on Windows

In order to run the program with CLion on Windows in conjunction with MSYS2, adding a MinGW toolchain is necessary.
For this, select the following options when creating a new toolchain in the settings (assuming MSYS2 is installed in
the standard directory C:\msys64).
- Environment: C:\msys64\mingw64
- CMake: C:\msys64\mingw64\bin\cmake.exe
- Make: C:\msys64\usr\bin\make.exe
- C Compiler: C:\msys64\mingw64\bin\gcc.exe
- C++ Compiler: C:\msys64\mingw64\bin\g++.exe
- Debugger: C:\msys64\mingw64\bin\gdb.exe

Then, add the following CMake options to the used build profiles (assuming sgl was installed to `/usr/local`).

```shell
-G "MSYS Makefiles" -DPython3_FIND_REGISTRY=NEVER -Dsgl_DIR=/usr/local/lib/cmake/sgl
```

Finally, in order for Windows to find the .dll files at runtime, add the following environment variables to the run
configurations.

```shell
PYTHONHOME=C:/msys64/mingw64;PATH=C:/msys64/mingw64/bin\;C:/msys64/usr/local/bin
```

