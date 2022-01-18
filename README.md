# Flow and Stress Line Visualization

LineVis is a visualization tool for rendering dense sets of 3D lines using the graphics APIs OpenGL and Vulkan.
It supports loading both traditional flow lines as well as stress lines from multiple principal stress directions.

The following rendering modes are supported:

- Opaque line rendering with up to 32x MSAA (multisample anti-aliasing).

- Opaque and transparent line rendering using a ray tracer based on the
  [Vulkan Ray Tracing](https://www.khronos.org/blog/ray-tracing-in-vulkan) extension.
  Both ray-triangle and analytic ray-tube intersections can be used.

- Opaque line rendering with Voxel Ray Casting (VRC).
  For more details see: M. Kanzler, M. Rautenhaus, R. Westermann.
  A Voxel-based Rendering Pipeline for Large 3D Line Sets.
  IEEE Transactions on Visualization and Computer Graphics 2018.
  https://www.in.tum.de/cg/research/publications/2018/a-voxel-based-rendering-pipeline-for-large-3d-line-sets/

- Opaque and transparent line rendering using [Intel OSPRay](https://www.ospray.org/).

- Transparent line rendering with per-pixel fragment lists (sometimes also called per-pixel linked lists).
  For more details see: Yang, J. C., Hensley, J., Grün, H. and Thibieroz, N., "Real-Time Concurrent Linked List
  Construction on the GPU", Computer Graphics Forum, 29, 2010.

- Transparent line rendering with Moment-based Order-Independent Transparency (MBOIT).
  For more details see: C. Munstermann, S. Krumpen, R. Klein, and C. Peters, "Moment-based order-independent transparency,"
  Proceedings of the ACM on Computer Graphics and Interactive Techniques, vol. 1, no. 1, pp. 7:1–7:20, May 2018.

- Transparent line rendering with Multi-Layer Alpha Blending (MLAB).
  For more details see: Marco Salvi and Karthik Vaidyanathan. 2014. Multi-layer Alpha Blending. In Proceedings of the
  18th Meeting of the ACM SIGGRAPH Symposium on Interactive 3D Graphics and Games (San Francisco, California)
  (I3D ’14). ACM, New York, NY, USA, 151–158. https://doi.org/10.1145/2556700.2556705

- Transparent line rendering with Multi-Layer Alpha Blending using depth buckets (MLABDB).
  For more details see: M. Kern, C. Neuhauser, T. Maack, M. Han, W. Usher and R. Westermann, "A Comparison of Rendering
  Techniques for 3D Line Sets with Transparency," in IEEE Transactions on Visualization and Computer Graphics, 2020.
  doi: 10.1109/TVCG.2020.2975795 URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9007507&isnumber=4359476

- Transparent line rendering with Weighted Blended Order-Independent Transparency (WBOIT)
  For more details see: Morgan McGuire and Louis Bavoil. 2013. Weighted Blended Order-Independent Transparency.
  Journal of Computer Graphics Techniques (JCGT), vol. 2, no. 2, 122-141, 201

- Transparent line rendering with Depth Peeling (DP).
  For more details see: C. Everitt, "Interactive order-independent transparency", NVIDIA Corporation, vol. 2, 10 2001.

- Decoupled opacity optimization.
  Implementation of decoupled opacity optimization as described in:
  Tobias Günther, Holger Theisel, and Markus Gross. 2017. Decoupled Opacity Optimization for Points, Lines and Surfaces.
  Comput. Graph. Forum 36, 2 (May 2017), 153–162. DOI:https://doi.org/10.1111/cgf.13115


## Building and running the programm

### Linux

There are two ways to build the program on Linux systems.
- Using the system package manager to install all dependencies (tested: apt on Ubuntu, pacman on Arch Linux).
- Using [vcpkg](https://github.com/microsoft/vcpkg) to install all dependencies.

In the project root directory, two scripts `build-linux.sh` and `build-linux-vcpkg.sh` can be found. The former uses the
system package manager to install all dependencies, while the latter uses vcpkg. The build scripts will also launch the
program after successfully building it. If you wish to build the program manually, instructions can be found in the
directory `docs/compilation`.

Below, more information concerning different Linux distributions tested can be found.

#### Arch Linux

Arch Linux and its derivative Manjaro are fully supported using both build modes (package manager and vcpkg).

The Vulkan SDK (which is an optional dependency for different advanced rendering modes) will be automatically installed
using the package manager `pacman` when using the scripts.

#### Ubuntu 18.04 & 20.04

Ubuntu 20.04 is fully supported.

The Vulkan SDK (which is an optional dependency for different advanced rendering modes) will be automatically installed
using the official PPA.

Please note that Ubuntu 18.04 is only partially supported. It ships an old version of CMake, which causes the build
process using vcpkg to fail if not updating CMake manually beforehand. Also, an old version of GLEW in the package
sources causes the Vulkan support to be disabled regardless of whether the Vulkan SDK is installed if the system
packages are used.

#### Other Linux Distributions

If you are using a different Linux distribution and face difficulties when building the program, please feel free to
open a [bug report](https://github.com/Junpeng-Wang-TUM/3D-TSV/issues).

### Windows

There are two ways to build the program on Windows.
- Using [vcpkg](https://github.com/microsoft/vcpkg) to install all dependencies. The program can then be compiled using
  [Microsoft Visual Studio](https://visualstudio.microsoft.com/vs/).
- Using [MSYS2](https://www.msys2.org/) to install all dependencies and compile the program using MinGW
  (not tested regularly).

In the project folder, a script called `build-windows.bat` can be found automating this build process using vcpkg and
Visual Studio. It is recommended to run the script using the `Developer PowerShell for VS 2022`. The build script will
also launch the program after successfully building it.

Please note that the [Vulkan SDK](https://vulkan.lunarg.com/sdk/home#windows) needs to be installed beforehand to make
sure different advanced rendering modes will be enabled in the program.

Building the program is regularly tested using Windows 10 and 11 with Microsoft Visual Studio 2022.
Building the program using Microsoft Visual Studio 2019 should theoretically also work, but is not guaranteed.

If you wish to build the program manually (or using MSYS2/MinGW instead of Visual Studio), instructions can be found in
the directory `docs/compilation`.


### macOS

Unfortunately, macOS is not supported, and will probably also never be supported, due to not natively supporting Vulkan
and deprecating OpenGL. MoltenVK, a Vulkan wrapper based on Apple's Metal API, unfortunately neither supports geometry
shaders (which are used for the non-raytracing renderers) nor OpenGL interoperability, which would be necessary for
running the program.


## How to add new data sets

Under `Data/LineDataSets/datasets.json`, loadable data sets can be specified. Additionally, the user can also open
arbitrary data sets using a file explorer via "File > Open Dataset..." (or using Ctrl+O).

Below, an example for a `Data/LineDataSets/datasets.json` file can be found.

```json
{
    "datasets": [
        { "type" : "flow", "name" : "Rings", "filenames": "flow/rings.obj", "linewidth": 0.003, "attributes": "Vorticity" },
        { "type" : "flow", "name" : "Tornado", "filenames": "flow/tornado.obj", "linewidth": 0.003, "attributes": "Vorticity" },
        { "type" : "flow", "name" : "Aneurysm", "filenames": "flow/aneurysm.obj", "attributes": "Vorticity" },
        { "type" : "flow", "name" : "Convection Rolls", "filenames": "flow/convection_rolls.obj", "attributes": "Line Curvature" },
        { "type" : "flow", "name" : "Turbulence", "filenames": "flow/turbulence.obj", "attributes": "Lambda_2 Vortex Measure" },
        { "type": "stress", "name": "Bearing",
          "filenames": "stress/bearing_psl.dat", "transform": "rotate(270°, 1, 0, 0)", "version": 3 },
        { "type": "stress", "name": "Cantilever",
          "filenames": "stress/cantilever3D_psl.dat", "transform": "rotate(270°, 1, 0, 0)", "version": 3 }
    ]
}
```

These files then appear with their specified name in the menu "File > Datasets". All paths must be specified relative to
the folder `Data/LineDataSets/` (unless they are global, like `C:/path/file.dat` or `/path/file.dat`).

Supported formats currently are:
- .obj, .ncf (NetCDF format), and the custom .binlines format for flow lines and ribbons.
- .dat files for principal stress lines (PSLs), stress ribbons and hyperstreamlines.

The format of the .obj files is expected to be as follows.

```
v <x> <y> <z> # vertex at index 1
vt <attribute>
...
g line0
l <v_idx_1> <v_idx_2> ... <v_idx_n>
...
```


## Principal Stress Line (PSL) tracing

This program can be used as the frontend for 3D-TSV, the 3D Trajectory-based Stress Visualizer.
For this, select "Stress Line Tracer" in the menu "File > Datasets" to open the line tracing menu.
When the 3D-TSV script `zeromqReplier.m` is running in the background, this application will then communicate with
3D-TSV over TCP/IP using ZeroMQ.

The repository of the backend can be found here: https://github.com/Junpeng-Wang-TUM/3D-TSV

3D-TSV is a visual analysis tool for the exploration of the principal stress directions in 3D solids under load.
It was created for the paper "3D-TSV: The 3D Trajectory-based Stress Visualizer" by Junpeng Wang, Christoph Neuhauser,
Jun Wu, Xifeng Gao and Rüdiger Westermann (to be published).

Under `Data/LineDataSets/mesh.json`, available simulation meshes can be specified. For example, when using the sample
meshes of 3D-TSV, the following file content can be specified:

```json
{
  "meshes": [
    { "name": "Cantilever", "filename": "../../../data/stress/ADES-2022/cantilever3D.carti" },
    { "name": "Kitten", "filename": "../../../data/stress/ADES-2022/kitten.stress" }
  ]
}
```

Additionally, the user can also open arbitrary simulation meshes (in `.carti` or `.stress` format) and
principal stress line (PSL) data sets using a file explorer via "File > Open Dataset..." (or using Ctrl+O).
`.carti` and `.stress` files will then be opened in the stress line tracing dialog.
