## Compilation on Linux

### Ubuntu

The build process was tested on Ubuntu 18.04 and Ubuntu 20.04.

As the first step, the [Vulkan SDK](https://vulkan.lunarg.com/sdk/home#linux) needs to be installed on the system. Next,
the library [sgl](https://github.com/chrismile/sgl) (https://github.com/chrismile/sgl) needs to be installed somewhere
on the system. In the following steps, it is assumed you have set up sgl and installed all of its dependencies.
Please follow the instructions in the file `docs/compilation_vcpkg.md` of sgl for the necessary steps.

After that, all obligatory dependencies can be installed using the following command.

```shell
sudo apt-get install cmake libglm-dev libsdl2-dev libsdl2-image-dev libpng-dev libboost-filesystem-dev libtinyxml2-dev \
libarchive-dev opencl-c-headers ocl-icd-opencl-dev libjsoncpp-dev libeigen3-dev python3-dev libzmq3-dev libnetcdf-dev \
libopenexr-dev libeccodes-dev libeccodes-tools libopenjp2-7-dev
```

Python 3 is an optional dependency necessary for enabling replay script support.

After all dependencies have been set up, the following commands can be used to build the program.

```shell
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

```shell
export LD_LIBRARY_PATH=<path-to-sgl>/lib
./LineVis
```


### Arch Linux

The following command can be used to install all dependencies on Arch Linux (last tested in May 2021).

```shell
sudo pacman -S cmake boost libarchive glm tinyxml2 sdl2 sdl2_image glew vulkan-devel shaderc opencl-headers ocl-icd \
python3 eigen jsoncpp zeromq netcdf ospray openexr
```

The optional dependency `eccodes` can be installed from AUR using, e.g., yay.

```shell
yay -Ss eccodes
```

All other build instructions are identical to the ones for Ubuntu provided above.
