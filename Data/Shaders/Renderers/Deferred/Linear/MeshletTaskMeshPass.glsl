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

-- Task

#version 450 core

#extension GL_NV_mesh_shader : require
#extension GL_EXT_shader_8bit_storage : require
#extension GL_KHR_shader_subgroup_ballot : require

layout(local_size_x = WORKGROUP_SIZE) in;

taskNV out Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
} OUT;

struct MeshletData {
    vec3 worldSpaceAabbMin;
    float padding;
    vec3 worldSpaceAabbMax;
    uint meshletFirstPrimitiveIdx; ///< Value for gl_PrimitiveID.
    uint vertexStart; ///< Pointer into dedupVerticesBuffer and dedupVertexIndexToOrigIndexMapBuffer.
    uint vertexCount;
    uint primitiveStart; ///< Pointer into dedupTriangleIndicesBuffer.
    uint primitiveCount;
};
layout(std430, binding = 0) readonly buffer MeshletDataBuffer {
    MeshletData meshlets[];
};

layout(std430, binding = 1) buffer MeshletVisibilityArrayBuffer {
    uint meshletVisibilityArray[];
};

#include "VisibilityCulling.glsl"

void main() {
    uint meshletIdx = gl_GlobalInvocationID.x;

    bool isVisible = meshletIdx < numMeshlets;
    if (isVisible) {
        MeshletData meshlet = meshlets[meshletIdx];

        // Should we only re-check previously occluded meshlets and not re-render already rendered ones?
#ifdef RECHECK_OCCLUDED_ONLY
        isVisible = false;
        if (meshletVisibilityArray[meshletIdx] == 0u) {
            isVisible = visibilityCulling(meshlet.worldSpaceAabbMin, meshlet.worldSpaceAabbMin);
        }
#else
        isVisible = visibilityCulling(meshlet.worldSpaceAabbMin, meshlet.worldSpaceAabbMin);
#endif
    }
    meshletVisibilityArray[meshletIdx] = isVisible ? 1u : 0u;

    // See: https://developer.nvidia.com/blog/introduction-turing-mesh-shaders/
    uvec4 vote = subgroupBallot(isVisible);
    uint numTasks = subgroupBallotBitCount(vote);

    if (gl_LocalInvocationID.x == 0) {
        // Write how much mesh workgroups should be spawned.
        gl_TaskCountNV = numTasks;
        OUT.baseIndex = gl_WorkGroupID.x * WORKGROUP_SIZE;
    }

    // Write the meshlets that should be rendered into the compacted array.
    uint idxOffset = subgroupBallotExclusiveBitCount(vote);
    if (isVisible) {
        OUT.subIndices[idxOffset] = uint8_t(gl_LocalInvocationID.x);
    }
}


-- Mesh

#version 450 core

#extension GL_NV_mesh_shader : require
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_shader_8bit_storage : require

layout(local_size_x = WORKGROUP_SIZE) in;
layout(triangles, max_vertices = MESHLET_MAX_VERTICES, max_primitives = MESHLET_MAX_PRIMITIVES) out;

taskNV in Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
} IN;

struct MeshletData {
    vec3 worldSpaceAabbMin;
    float padding;
    vec3 worldSpaceAabbMax;
    uint meshletFirstPrimitiveIdx; ///< Value for gl_PrimitiveID.
    uint vertexStart; ///< Pointer into dedupVerticesBuffer and dedupVertexIndexToOrigIndexMapBuffer.
    uint vertexCount;
    uint primitiveStart; ///< Pointer into dedupTriangleIndicesBuffer.
    uint primitiveCount;
};
layout(std430, binding = 0) readonly buffer MeshletDataBuffer {
    MeshletData meshlets[];
};

layout(scalar, binding = 2) readonly buffer DedupVerticesBuffer {
    vec3 dedupVertices[];
};

layout(scalar, binding = 3) readonly buffer DedupTriangleIndicesBuffer {
    uint8_t dedupTriangleIndices[];
};

void main() {
    uint meshletIdx = IN.baseIndex + uint(IN.subIndices[gl_WorkGroupID.x]);
    uint threadIdx = gl_LocalInvocationID.x;

    MeshletData meshletData = meshlets[meshletIdx];

    gl_PrimitiveCountNV = meshletData.primitiveCount;

    for (uint vertexIdx = threadIdx; vertexIdx < meshletData.vertexCount; vertexIdx += WORKGROUP_SIZE) {
        vec3 vertexPosition = dedupVertices[meshletData.vertexStart + vertexIdx];
        gl_MeshVerticesNV[vertexIdx].gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
    }

    for (uint triangleIdx = threadIdx; triangleIdx < meshletData.primitiveCount; triangleIdx += WORKGROUP_SIZE) {
        uint writeIdx = 3u * triangleIdx;
        uint readIdx = writeIdx + meshletData.primitiveStart * 3u;

        gl_PrimitiveIndicesNV[writeIdx] = uint(dedupTriangleIndices[readIdx]);
        gl_PrimitiveIndicesNV[writeIdx + 1u] = uint(dedupTriangleIndices[readIdx + 1u]);
        gl_PrimitiveIndicesNV[writeIdx + 2u] = uint(dedupTriangleIndices[readIdx + 2u]);

        // Available in gl_MeshPrimitivesNV:
        // https://github.com/KhronosGroup/GLSL/blob/master/extensions/nv/GLSL_NV_mesh_shader.txt
        gl_MeshPrimitivesNV[triangleIdx].gl_PrimitiveID = int(meshletData.meshletFirstPrimitiveIdx + triangleIdx);
    }
}


-- Fragment

#version 450 core

// Primitive index attachment uses clear value 0xFFFFFFFFu.
layout(location = 0) out uint fragmentPrimitiveIndex;

void main() {
    fragmentPrimitiveIndex = uint(gl_PrimitiveID);
}
