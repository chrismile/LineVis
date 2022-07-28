/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#ifndef LINEVIS_RENDERINGMODES_HPP
#define LINEVIS_RENDERINGMODES_HPP

enum RenderingMode : int32_t {
    RENDERING_MODE_NONE = -1,
    RENDERING_MODE_ALL_LINES_OPAQUE = 0,
    RENDERING_MODE_DEFERRED_SHADING = 1,
    RENDERING_MODE_PER_PIXEL_LINKED_LIST = 2,
    RENDERING_MODE_MLAB = 3,
    RENDERING_MODE_OPACITY_OPTIMIZATION = 4,
    RENDERING_MODE_DEPTH_COMPLEXITY = 5,
    RENDERING_MODE_MBOIT = 6,
    RENDERING_MODE_MLAB_BUCKETS = 7,
    RENDERING_MODE_WBOIT = 8,
    RENDERING_MODE_DEPTH_PEELING = 9,
    RENDERING_MODE_VULKAN_RAY_TRACER = 10,
    RENDERING_MODE_VOXEL_RAY_CASTING = 11,
    RENDERING_MODE_OSPRAY_RAY_TRACER = 12,

    // For LineDataScattering:
    RENDERING_MODE_LINE_DENSITY_MAP_RENDERER = 13,
    RENDERING_MODE_VOLUMETRIC_PATH_TRACER = 14,
    RENDERING_MODE_SPHERICAL_HEAT_MAP_RENDERER = 15
};
const char* const RENDERING_MODE_NAMES[] = {
        "Opaque",
        "Deferred Opaque",
        "Per-Pixel Linked Lists",
        "Multi-Layer Alpha Blending",
        "Opacity Optimization",
        "Depth Complexity",
        "Moment-Based OIT",
        "MLAB (Buckets)",
        "WBOIT",
        "Depth Peeling",
        "Vulkan Ray Tracer",
        "Voxel Ray Casting",
        "OSPRay Ray Tracer",

        // For LineDataScattering:
        "Line Density Map Renderer",
        "Volumetric Path Tracer",
        "Spherical Heat Map Renderer"
};
const int NUM_RENDERING_MODES = ((int)(sizeof(RENDERING_MODE_NAMES) / sizeof(*RENDERING_MODE_NAMES)));

const uint32_t ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT = 4052753091u;
const uint32_t ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT = 4052753092u;

#endif //LINEVIS_RENDERINGMODES_HPP
