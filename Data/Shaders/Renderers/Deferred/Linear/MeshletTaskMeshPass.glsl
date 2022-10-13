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

#extension GL_EXT_shader_8bit_storage : require
#extension GL_EXT_shader_explicit_arithmetic_types_int8 : require
#extension GL_KHR_shader_subgroup_ballot : require

layout(local_size_x = WORKGROUP_SIZE) in;

#ifdef VK_NV_mesh_shader
taskNV out Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
} OUT;
#else
struct Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
};
taskPayloadSharedEXT Task OUT;
#endif

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

#ifdef RECHECK_OCCLUDED_ONLY
layout(std430, binding = 1) buffer MeshletVisibilityArrayBuffer {
    uint meshletVisibilityArray[];
};
#else
layout(std430, binding = 1) writeonly buffer MeshletVisibilityArrayBuffer {
    uint meshletVisibilityArray[];
};
#endif

#include "VisibilityCulling.glsl"
#include "VisualizeBvhHierarchy.glsl"

void main() {
    uint meshletIdx = gl_GlobalInvocationID.x;

    bool isVisible = meshletIdx < numMeshlets;
    if (isVisible) {
        MeshletData meshlet = meshlets[meshletIdx];

        // Should we only re-check previously occluded meshlets and not re-render already rendered ones?
#ifdef RECHECK_OCCLUDED_ONLY
        isVisible = false;
        if (meshletVisibilityArray[meshletIdx] == 0u) {
            isVisible = visibilityCulling(meshlet.worldSpaceAabbMin, meshlet.worldSpaceAabbMax);
        }
#else
        isVisible = visibilityCulling(meshlet.worldSpaceAabbMin, meshlet.worldSpaceAabbMax);
#endif

#ifdef VISUALIZE_BVH_HIERARCHY
        uint writePositionNodeAabb = atomicAdd(numNodeAabbs, 1u);
        NodeAabb nodeAabb;
        nodeAabb.worldSpaceAabbMin = meshlet.worldSpaceAabbMin;
        nodeAabb.worldSpaceAabbMax = meshlet.worldSpaceAabbMax;
        nodeAabb.normalizedHierarchyLevel = 0;
#ifdef RECHECK_OCCLUDED_ONLY
        nodeAabb.passIdx = 1u;
#else
        nodeAabb.passIdx = 0u;
#endif
        nodeAabbs[writePositionNodeAabb] = nodeAabb;
#endif
    }
    meshletVisibilityArray[meshletIdx] = isVisible ? 1u : 0u;

    // See: https://developer.nvidia.com/blog/introduction-turing-mesh-shaders/
    uvec4 vote = subgroupBallot(isVisible);
    uint numTasks = subgroupBallotBitCount(vote);

    if (gl_LocalInvocationID.x == 0) {
        // Write how much mesh workgroups should be spawned.
#ifdef VK_NV_mesh_shader
        gl_TaskCountNV = numTasks;
#else
        EmitMeshTasksEXT(numTasks, 1, 1);
#endif
        OUT.baseIndex = gl_WorkGroupID.x * WORKGROUP_SIZE;
    }

    // Write the meshlets that should be rendered into the compacted array.
    uint idxOffset = subgroupBallotExclusiveBitCount(vote);
    if (isVisible) {
        OUT.subIndices[idxOffset] = uint8_t(gl_LocalInvocationID.x);
    }
}


-- Mesh

#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_shader_8bit_storage : require
#extension GL_EXT_shader_explicit_arithmetic_types_int8 : require

layout(local_size_x = WORKGROUP_SIZE) in;
layout(triangles, max_vertices = MESHLET_MAX_VERTICES, max_primitives = MESHLET_MAX_PRIMITIVES) out;

#ifndef BVH_MESHLETS
#ifdef VK_NV_mesh_shader
taskNV in Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
} IN;
#else
struct Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
};
taskPayloadSharedEXT Task IN;
#endif
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
#else //< if defined(BVH_MESHLETS)
struct MeshletData {
    uint meshletFirstPrimitiveIdx; ///< Value for gl_PrimitiveID.
    uint vertexStart; ///< Pointer into dedupVerticesBuffer and dedupVertexIndexToOrigIndexMapBuffer.
    uint primitiveStart; ///< Pointer into dedupTriangleIndicesBuffer.
    uint vertexAndPrimitiveCountCombined; ///< Bit 0-15: Vertex count. Bit 16-31: Primitive count.
};
layout(std430, binding = 1) readonly buffer VisibleMeshletIndexArrayBuffer {
    uint visibleMeshletIndexArray[];
};
#endif

layout(std430, binding = 0) readonly buffer MeshletDataBuffer {
    MeshletData meshlets[];
};

layout(scalar, binding = 2) readonly buffer DedupVerticesBuffer {
    vec3 dedupVertices[];
};

#ifdef WRITE_PACKED_PRIMITIVE_INDICES
layout(scalar, binding = 3) readonly buffer DedupTriangleIndicesBuffer {
    uint dedupTriangleIndices[];
};
#else
layout(scalar, binding = 3) readonly buffer DedupTriangleIndicesBuffer {
    uint8_t dedupTriangleIndices[];
};
#endif

void main() {
    uint threadIdx = gl_LocalInvocationIndex;

#ifndef BVH_MESHLETS
    uint meshletIdx = IN.baseIndex + uint(IN.subIndices[gl_WorkGroupID.x]);
    MeshletData meshletData = meshlets[meshletIdx];
    uint vertexCount = meshletData.vertexCount;
    uint primitiveCount = meshletData.primitiveCount;
#else
    uint meshletIdx = visibleMeshletIndexArray[gl_WorkGroupID.x];
    MeshletData meshletData = meshlets[meshletIdx];
    uint vertexCount = meshletData.vertexAndPrimitiveCountCombined & 0xFFFFu;
    uint primitiveCount = meshletData.vertexAndPrimitiveCountCombined >> 16u;
#endif

#ifdef VK_NV_mesh_shader
    gl_PrimitiveCountNV = primitiveCount;
#else
    SetMeshOutputsEXT(vertexCount, primitiveCount);
#endif

    /*for (uint i = 0; i < 0xFFFFFFFFu; i++) {
#ifdef VK_NV_mesh_shader
        gl_PrimitiveIndicesNV[0] = 0u;
#else
        gl_PrimitiveTriangleIndicesEXT[0] = uvec3(0u, 0u, 0u);
#endif
    }*/

    for (uint vertexIdx = threadIdx; vertexIdx < vertexCount; vertexIdx += WORKGROUP_SIZE) {
        vec3 vertexPosition = dedupVertices[meshletData.vertexStart + vertexIdx];
#ifdef VK_NV_mesh_shader
        gl_MeshVerticesNV[vertexIdx].gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
#else
        gl_MeshVerticesEXT[vertexIdx].gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
#endif
    }

#if defined(VK_NV_mesh_shader) && defined(WRITE_PACKED_PRIMITIVE_INDICES)
    //uint numPackedIndices = (primitiveCount * 3u + WORKGROUP_SIZE * 4u - 1u) / (WORKGROUP_SIZE * 4u);
    uint numPackedIndices = (primitiveCount * 3u + 3u) / 4u;
    uint readOffset = meshletData.primitiveStart * 3u / 4u; // Assure alignment!
    for (uint packedIdx = threadIdx; packedIdx < numPackedIndices; packedIdx += WORKGROUP_SIZE) {
        uint writeIdx = packedIdx * 4u;
        uint readIdx = packedIdx + readOffset;

        // Specification: "The write operations must not exceed the size of the gl_PrimitiveIndicesNV array."
        // => Ensure to only use WRITE_PACKED_PRIMITIVE_INDICES when this can be guaranteed!
        writePackedPrimitiveIndices4x8NV(writeIdx, dedupTriangleIndices[readIdx]);
    }
    for (uint triangleIdx = threadIdx; triangleIdx < primitiveCount; triangleIdx += WORKGROUP_SIZE) {
        // Available in gl_MeshPrimitivesNV:
        // https://github.com/KhronosGroup/GLSL/blob/master/extensions/nv/GLSL_NV_mesh_shader.txt
        gl_MeshPrimitivesNV[triangleIdx].gl_PrimitiveID = int(meshletData.meshletFirstPrimitiveIdx + triangleIdx);
    }
#else
    for (uint triangleIdx = threadIdx; triangleIdx < primitiveCount; triangleIdx += WORKGROUP_SIZE) {
        uint readIdx = (triangleIdx + meshletData.primitiveStart) * 3u;

#ifdef VK_NV_mesh_shader
        uint writeIdx = triangleIdx * 3u;
        gl_PrimitiveIndicesNV[writeIdx] = uint(dedupTriangleIndices[readIdx]);
        gl_PrimitiveIndicesNV[writeIdx + 1u] = uint(dedupTriangleIndices[readIdx + 1u]);
        gl_PrimitiveIndicesNV[writeIdx + 2u] = uint(dedupTriangleIndices[readIdx + 2u]);

        // Available in gl_MeshPrimitivesNV:
        // https://github.com/KhronosGroup/GLSL/blob/master/extensions/nv/GLSL_NV_mesh_shader.txt
        gl_MeshPrimitivesNV[triangleIdx].gl_PrimitiveID = int(meshletData.meshletFirstPrimitiveIdx + triangleIdx);
#else
        gl_PrimitiveTriangleIndicesEXT[triangleIdx] = uvec3(
                uint(dedupTriangleIndices[readIdx]),
                uint(dedupTriangleIndices[readIdx + 1u]),
                uint(dedupTriangleIndices[readIdx + 2u]));

        // Available in gl_MeshPrimitivesEXT:
        // https://github.com/KhronosGroup/GLSL/blob/master/extensions/ext/GLSL_EXT_mesh_shader.txt
        gl_MeshPrimitivesEXT[triangleIdx].gl_PrimitiveID = int(meshletData.meshletFirstPrimitiveIdx + triangleIdx);
#endif
    }
#endif
}


-- TaskNV

#version 450 core

#extension GL_NV_mesh_shader : require

#define VK_NV_mesh_shader
#import ".Task"


-- TaskEXT

#version 450 core

#extension GL_EXT_mesh_shader : require

#define VK_EXT_mesh_shader
#import ".Task"


-- MeshNV

#version 450 core

#extension GL_NV_mesh_shader : require

#define VK_NV_mesh_shader
#import ".Mesh"


-- MeshEXT

#version 450 core

#extension GL_EXT_mesh_shader : require

#define VK_EXT_mesh_shader
#import ".Mesh"


-- Fragment

#version 450 core

// Primitive index attachment uses clear value 0xFFFFFFFFu.
layout(location = 0) out uint fragmentPrimitiveIndex;

void main() {
    fragmentPrimitiveIndex = uint(gl_PrimitiveID);
}
