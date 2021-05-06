## Compilation on Linux

### Ubuntu

The build process was tested on Ubuntu 16.04 and Ubuntu 20.04.

As the first step, the library [sgl](https://github.com/chrismile/sgl) (https://github.com/chrismile/sgl) needs to be
installed somewhere on the system. In the following steps, it is assumed you have set up sgl and installed all of its
dependencies.

After that, all obligatory dependencies can be installed using the following command.

```
sudo apt-get install cmake libglm-dev libsdl2-dev libsdl2-image-dev libpng-dev libboost-filesystem-dev libtinyxml2-dev \
libarchive-dev libjsoncpp-dev python3-dev libzmq3-dev libnetcdf-dev
```

Python 3 is an optional dependency necessary for enabling replay script support.

After all dependencies have been set up, the following commands can be used to build the program.

```
mkdir build
cd build
cmake -Dsgl_DIR=<path-to-sgl> ..
make -j
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


### Arch Linux

The following command can be used to install all dependencies on Arch Linux (last tested in May 2021).

```
sudo apt-get install cmake libglm-dev libsdl2-dev libsdl2-image-dev libpng-dev libboost-filesystem-dev libtinyxml2-dev \
libarchive-dev libjsoncpp-dev python3-dev libzmq3-dev libnetcdf-dev

sudo pacman -S cmake glew boost libarchive glm tinyxml2 sdl2 sdl2_image python3 jsoncpp zeromq netcdf
```

All other build instructions are identical to the ones for Ubuntu provided above.
