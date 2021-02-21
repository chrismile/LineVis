# Flow and Stress Line Visualization

LineVis is a visualization tool for rendering dense sets of 3D lines using OpenGL.
It supports loading both traditional flow lines as well as stress lines from multiple principle stress directions.

The following rendering modes are supported:

- Opaque line rendering with up to 32x MSAA (multisample anti-aliasing).

- Transparent line rendering with per-pixel fragment lists (sometimes also called per-pixel linked lists).

For more details see: Yang, J. C., Hensley, J., Grün, H. and Thibieroz, N., "Real-Time Concurrent Linked List
Construction on the GPU", Computer Graphics Forum, 29, 2010.

- Decoupled opacity optimization.

Implementation of opacity optimization as described in:
Tobias Günther, Holger Theisel, and Markus Gross. 2017. Decoupled Opacity Optimization for Points, Lines and Surfaces.
Comput. Graph. Forum 36, 2 (May 2017), 153–162. DOI:https://doi.org/10.1111/cgf.13115


## Building and running the programm

The program requires the library sgl (https://github.com/chrismile/sgl).
On Ubuntu 20.04 for example, you can install all other necessary dependencies with this command (additionally to the prerequisites required by sgl):

```
sudo apt-get install libjsoncpp-dev libnetcdf-dev netcdf-bin libzmq3-dev
```

After installing sgl (see above) execute in the repository directory:

```
mkdir build
cd build
cmake ..
make -j
ln -s ../Data .
```
(Alternatively, use 'cp -R ../Data .' to copy the Data directory instead of creating a soft link to it).

The build process was also tested on Windows 10 64-bit using MSYS2 and Mingw-w64 (http://www.msys2.org/). Using MSYS2 and Pacman, the following packages need to be installed additionally to the prerequisites required by sgl.

```
pacman -S mingw64/mingw-w64-x86_64-jsoncpp mingw64/mingw-w64-x86_64-netcdf mingw-w64-x86_64-zeromq
```

Furthermore, the graph library LEMON (http://lemon.cs.elte.hu/trac/lemon) needs to be built manually, as no msys2 package is available for it at the time of writing this README file.

On Windows, using MSYS2 and Mingw-w64 (http://www.msys2.org/), it is best to use the following CMake command to configure CMake:
```
cmake .. -G"MSYS Makefiles"
```

To run the program, execute:
```
export LD_LIBRARY_PATH=/usr/local/lib
./LineVis
```

## How to add new data sets

Under `Data/LineDataSets/datasets.json`, loadable data sets can be specified. Example:

```json
{
    "datasets": [
        { "type" : "flow", "name" : "Rings", "filenames": "flow/rings.obj", "linewidth": 0.003, "attributes": "Vorticity" },
        { "type" : "flow", "name" : "Tornado", "filenames": "flow/tornado.obj", "linewidth": 0.003, "attributes": "Vorticity" },
        { "type" : "flow", "name" : "Aneurysm", "filenames": "flow/aneurysm.obj", "attributes": "Vorticity" },
        { "type" : "flow", "name" : "Convection Rolls", "filenames": "flow/convection_rolls.obj", "attributes": "Line Curvature" },
        { "type" : "flow", "name" : "Turbulence", "filenames": "flow/turbulence.obj", "attributes": "Lambda_2 Vortex Measure" },
        { "type" : "stress", "name" : "Cantilever", "filenames": [
            "stress/dataset1_majorPSL.dat","stress/dataset1_mediumPSL.dat","stress/dataset1_minorPSL.dat"
        ], "attributes": "von Mises Stress", "transform": "rotate(270°, 1, 0, 0)" },
        { "type" : "stress", "name" : "Femur", "filenames": [
            "stress/dataset2_majorPSL.dat","stress/dataset2_mediumPSL.dat","stress/dataset2_minorPSL.dat"
        ], "attributes": "von Mises Stress", "transform": "rotate(270°, 1, 0, 0)" }
    ]
}
```

All file paths are relative to the folder `Data/LineDataSets/`.

Supported formats currently are:
- .obj, .ncf (NetCDF format), and the custom .binlines format for flow lines.
- .dat files for stress lines.
