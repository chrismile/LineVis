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

enum RenderingMode {
    RENDERING_MODE_ALL_LINES_OPAQUE, RENDERING_MODE_PER_PIXEL_LINKED_LIST, RENDERING_MODE_MLAB,
    RENDERING_MODE_OPACITY_OPTIMIZATION, RENDERING_MODE_DEPTH_COMPLEXITY, RENDERING_MODE_MBOIT,
    RENDERING_MODE_MLAB_BUCKETS, RENDERING_MODE_WBOIT, RENDERING_MODE_DEPTH_PEELING,
    RENDERING_MODE_VULKAN_RAY_TRACER, RENDERING_MODE_VOXEL_RAY_CASTING, RENDERING_MODE_VULKAN_TEST,
    RENDERING_MODE_OSPRAY_RAY_TRACER, RENDERING_MODE_SCATTERED_LINES_RENDERER
};
const char* const RENDERING_MODE_NAMES[] = {
        "Opaque", "Per-Pixel Linked Lists", "Multi-Layer Alpha Blending", "Opacity Optimization", "Depth Complexity",
        "Moment-Based OIT", "MLAB (Buckets)", "WBOIT", "Depth Peeling", "Vulkan Ray Tracer", "Voxel Ray Casting",
        "Vulkan Test", "OSPRay Ray Tracer", "Scattered Lines Renderer"
};
const int NUM_RENDERING_MODES = ((int)(sizeof(RENDERING_MODE_NAMES) / sizeof(*RENDERING_MODE_NAMES)));

#endif //LINEVIS_RENDERINGMODES_HPP
