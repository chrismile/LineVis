/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LINEVIS_DEFERREDMODES_HPP
#define LINEVIS_DEFERREDMODES_HPP

const char* const deferredRenderingModeNames[5] = {
        "Draw Indexed",
        "Draw Indirect",
        "BVH Draw Indirect",
        "BVH Mesh Shader",
        "Task/Mesh Shader"
};
enum class DeferredRenderingMode {
    DRAW_INDEXED,
    DRAW_INDIRECT,
    BVH_DRAW_INDIRECT,
    BVH_MESH_SHADER,
    TASK_MESH_SHADER
};

const char* const drawIndexedGeometryModeNames[2] = {
        "Precomputed Triangles",
        "Programmable Pulling",
};
enum class DrawIndexedGeometryMode {
    TRIANGLES, PROGRAMMABLE_PULLING
};

const char* const drawIndirectReductionModeNames[3] = {
        "No Reduction", ///< vkCmdDrawIndexedIndirect + count is the total number of meshlets.
        "Atomic Counter", ///< vkCmdDrawIndexedIndirectCount + count reduction with an atomic counter.
        "Prefix Sum Scan", ///< vkCmdDrawIndexedIndirectCount + count reduction with a parallel prefix sum scan.
};
enum class DrawIndirectReductionMode {
    NO_REDUCTION, ATOMIC_COUNTER, PREFIX_SUM_SCAN
};

/**
 * Valid combinations:
 * 1. Draw Indirect:
 * a) BvhBuildGeometryMode::TRIANGLES:
 * - BINNED_SAH_CPU, SWEEP_SAH_CPU: max_leaf_size, max_depth
 *   Standard: max_leaf_size = maxNumPrimitivesPerMeshlet, max_depth = maxHeight
 *   => UI: if (useMaxNumPrimitives): SliderInt. else: max_leaf_size, max_depth.
 * - LOCALLY_ORDERED_CLUSTERING_CPU, LINEAR_BVH_CPU: No control.
 * b) BvhBuildGeometryMode::MESHLETS:
 * - BINNED_SAH_CPU, SWEEP_SAH_CPU: max_leaf_size, max_depth
 *   Standard: max_leaf_size = 1, max_depth = 64 (library standard).
 *   => UI: max_leaf_size, max_depth.
 * - LOCALLY_ORDERED_CLUSTERING_CPU, LINEAR_BVH_CPU: No control.
 * 2. Task/Mesh Shaders: Like 1b).
 */
const char* const bvhBuildAlgorithmNames[4] = {
        "Binned SAH (CPU)",
        "Sweep SAH (CPU)",
        "Locally Ordered Clustering (CPU)",
        "Linear BVH (CPU)",
        //"Linear BVH (Parallel, CPU)",
        //"Linear BVH (Parallel, GPU)"
};
enum class BvhBuildAlgorithm {
    BINNED_SAH_CPU, SWEEP_SAH_CPU, LOCALLY_ORDERED_CLUSTERING_CPU, LINEAR_BVH_CPU,
    //LINEAR_BVH_PARALLEL_CPU, LINEAR_BVH_PARALLEL_GPU
};

const char* const bvhBuildGeometryModeNames[2] = {
        "Triangles",
        "Meshlets",
};
enum class BvhBuildGeometryMode {
    TRIANGLES, MESHLETS
};

const char* const bvhBuildPrimitiveCenterModeNames[2] = {
        "Centroid",
        "BB Center",
};
enum class BvhBuildPrimitiveCenterMode {
    PRIMITIVE_CENTROID, BOUNDING_BOX_CENTER
};

#endif //LINEVIS_DEFERREDMODES_HPP
