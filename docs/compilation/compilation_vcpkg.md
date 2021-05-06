## Compilation using vcpkg

As the first step, the library [sgl](https://github.com/chrismile/sgl) (https://github.com/chrismile/sgl) needs to be
installed somewhere on the system. In the following steps, it is assumed you have set up sgl using vcpkg and installed
all of its dependencies. Please follow the instructions in the file `docs/compilation_vcpkg.md` of sgl for that.


### Installing all Packages (All Systems)

All other necessary dependencies besides sgl can be installed using the following command.
On Windows `--triplet=x64-windows` needs to be added if the 64-bit version of the packages should be installed.

```
./vcpkg install boost-core boost-algorithm boost-filesystem boost-locale libpng sdl2[vulkan] sdl2-image \
tinyxml2 glew glm libarchive[bzip2,core,lz4,lzma,zstd] jsoncpp python3 zeromq netcdf-c netcdf-cxx4
```


### Compilation on Linux

To invoke the build process using CMake, the following commands can be used.
Please adapt `sgl_DIR` depending on which path sgl was installed to.

```
mkdir build
cd build
rm -rf *
cmake -DCMAKE_TOOLCHAIN_FILE=$VCPKG_HOME/scripts/buildsystems/vcpkg.cmake -Dsgl_DIR=<path-to-sgl> ..
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

Still WIP
