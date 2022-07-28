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

-- Compute

#version 450 core

#define WORKGROUP_SIZE 256

layout(local_size_x = WORKGROUP_SIZE) in;

#include "VisibilityCulling.glsl"

struct MeshletData {
    vec3 worldSpaceAabbMin;
    uint numIndices;
    vec3 worldSpaceAabbMax;
    uint firstIndex;
};
layout(std430, binding = 0) readonly buffer MeshletBuffer {
    MeshletData meshlets[];
};

layout(std430, binding = 1) writeonly buffer MeshletVisibilityArrayBuffer {
    MeshletData meshletVisibilityArray[];
};

void main() {
    uint meshletIdx = gl_GlobalInvocationID.x;
    if (meshletIdx >= numMeshlets) {
        return;
    }

    MeshletData meshlet = meshlets[meshletIdx];
    uint isVisible = visibilityCulling(meshlet.worldSpaceAabbMin, meshlet.worldSpaceAabbMin) ? 1u : 0u;
    meshletVisibilityArray[meshletIdx] = isVisible;
}
