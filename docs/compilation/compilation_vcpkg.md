## Compilation using vcpkg

As the first step, the library [sgl](https://github.com/chrismile/sgl) (https://github.com/chrismile/sgl) needs to be
installed somewhere on the system. In the following steps, it is assumed you have set up sgl using vcpkg and installed
all of its dependencies. Please follow the instructions in the file `docs/compilation_vcpkg.md` of sgl for that.


### Installing all Packages (All Systems)

All other necessary dependencies besides sgl can be installed using the following command.
On Windows `--triplet=x64-windows` needs to be added if the 64-bit version of the packages should be installed.

```
./vcpkg install boost-core boost-algorithm boost-filesystem sdl2[vulkan] glew glm jsoncpp python3 cppzmq netcdf-c
```


### Compilation on Linux

To invoke the build process using CMake, the following commands can be used.
Please adapt `sgl_DIR` depending on which path sgl was installed to.

```
mkdir build
cd build
rm -rf *
cmake -DCMAKE_TOOLCHAIN_FILE=$VCPKG_HOME/scripts/buildsystems/vcpkg.cmake -Dsgl_DIR=<path-to-sgl>/lib/cmake/sgl ..
make -j
make install
```

If the program was built out-of-source (i.e., the folder `build` does not lie in the source directory), the user must
either create a symbolic link to the directory `Data` in the build folder (this only works on Linux and not Windows),
or the CMake variable `DATA_PATH` must be set to the path pointing to the `Data` folder.

If sgl was not installed globally on the system, the library path might need to be adapted before launching the
application.

```
export LD_LIBRARY_PATH=<path-to-sgl>/lib
./LineVis
```


### Compilation on Windows

On Windows, installing the Boost.Interprocess package is necessary.
Please do not forget to add `--triplet=x64-windows` if the 64-bit version of the package should be installed.

```
./vcpkg install boost-interprocess
```

Then, the program can be built using the following commands. Please adapt the paths where necessary.

```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_HOME/scripts/buildsystems/vcpkg.cmake" -Dsgl_DIR=<path-to-sgl>lib/cmake/sgl ..
cmake --build . --parallel
```

Hint: To change the language of warnings and error messages to English even if your system uses another language,
consider setting the environment variable `set VSLANG=1033`.

To run the program, use the following commands on cmd.exe ...

```
set PATH=%PATH%;<path-to-sgl>/bin
set PYTHONHOME=$VCPKG_HOME/installed/x64-windows/tools/python3

# Debug
set PATH=%PATH%;$VCPKG_HOME/installed/x64-windows/debug/bin

# Release
set PATH=%PATH%;$VCPKG_HOME/installed/x64-windows/bin

LineVis.exe
```

... or the following commands on the PowerShell.

```
$env:Path += ";<path-to-sgl>/bin"
$env:PYTHONHOME = "$env:VCPKG_HOME/installed/x64-windows/tools/python3"

# Debug
$env:Path += ";$env:VCPKG_HOME/installed/x64-windows/debug/bin"

# Release
$env:Path += ";$env:VCPKG_HOME/installed/x64-windows/bin"

./LineVis.exe
```
