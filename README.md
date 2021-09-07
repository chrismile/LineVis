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

Currently, there are three main ways to compile the program:
- Linux: Using the system package manager to install all dependencies (tested: apt on Ubuntu, pacman on Arch Linux).
- Linux & Windows: Installing all dependencies using [vcpkg](https://github.com/microsoft/vcpkg).
- Windows: Using MSYS2 to install all dependencies.

On Windows, we recommend to use vcpkg if you need to use Microsoft Visual Studio.
Guides for the different build types can be found in the directory `docs/compilation`.


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
- .dat files for principal stress lines (PSLs).

The format of the .obj files is expected to be as follows.

```
v <x> <y> <z> # vertex at index 1
vt <attribute>
...
g line0
l <v_idx_1> <v_idx_2> ... <v_idx_n>
```


## Principal Stress Line (PSL) tracing

This program can be used as the frontend for 3D-TSV, the 3D Trajectory-based Stress Visualizer.
For this, select "Stress Line Tracer" in the data set drop-down menu to open the line tracing menu.
When 3D-TSV is running in the background, this application will then communicate with 3D-TSV over TCP/IP using ZeroMQ.

https://github.com/Junpeng-Wang-TUM/3D-TSV

3D-TSV is a visual analysis tool for the exploration of the principal stress directions in 3D solids under load.

3D-TSV was created for the paper "The 3D Trajectory-based Stress Visualizer" by Junpeng Wang, Christoph Neuhauser,
Jun Wu, Xifeng Gao and Rüdiger Westermann, which was submitted to IEEE VIS 2021.
