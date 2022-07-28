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

layout(local_size_x = WORKGROUP_SIZE) in;

taskNV out Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
} OUT;

void main() {
    uint meshletIdx = gl_GlobalInvocationID.x;

    bool isVisible = meshletIdx < numMeshlets;
    if (isVisible) {
        MeshletData meshlet = meshlets[meshletIdx];
        isVisible = visibilityCulling(meshlet.worldSpaceAabbMin, meshlet.worldSpaceAabbMin);
    }

    // See: https://developer.nvidia.com/blog/introduction-turing-mesh-shaders/
    uvec4 vote = subgroupBallot(isVisible);
    uint numTasks = subgroupBallotBitCount(isVisible);

    if (gl_LocalInvocationID.x == 0) {
        // Write how much mesh workgroups should be spawned.
        gl_TaskCountNV = numTasks;
        OUT.baseIndex = gl_WorkGroupID.x * WORKGROUP_SIZE;
    }

    // Write the meshlets that should be rendered into the compacted array.
    uint idxOffset = subgroupBallotExclusiveBitCount(vote);
    if (render) {
        OUT.subIndices[idxOffset] = uint8_t(gl_LocalInvocationID.x);
    }
}


-- Mesh

#version 450 core

#extension GL_NV_mesh_shader : require

#define MESHLET_MAX_VERTICES 64
#define MESHLET_MAX_PRIMITIVES (2 * MESHLET_MAX_VERTICES - 2 * NUM_TUBE_SUBDIVISIONS)

layout(local_size_x = WORKGROUP_SIZE) in;
layout(triangles, max_vertices = MESHLET_MAX_VERTICES, max_primitives = MESHLET_MAX_PRIMITIVES) out;

taskNV in Task {
    uint baseIndex;
    uint8_t subIndices[WORKGROUP_SIZE];
} IN;

struct MeshletData {
    vec3 worldSpaceAabbMin;
    uint numIndices;
    vec3 worldSpaceAabbMax;
    uint firstIndex;
};
layout(std430, binding = 0) readonly buffer NodeBuffer {
    MeshletData meshlets[];
};

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to LinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};
layout(std430, binding = 1) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

layout(scalar, binding = 2) readonly buffer TriangleIndexBuffer {
    uvec3 indexBuffer[];
};

void main() {
    uint meshletIdx = IN.baseIndex + IN.subIndices[gl_WorkGroupID.x];
    uint threadIdx = gl_LocalInvocationID.x;

    MeshletData meshletData = meshlets[meshletIdx];

    uint firstPrimitiveID = firstIndex / 3u;
    uint numTriangles = meshletData.numIndices / 3u;
    gl_PrimitiveCountNV = numTriangles;

    // TODO
    for (uint vertexIdx = threadIdx; vertexIdx < numVertices; vertexIdx += WORKGROUP_SIZE) {
        gl_MeshVerticesNV[vertexIdx].gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
    }

    for (uint triangleIdx = threadIdx; triangleIdx < numTriangles; triangleIdx += WORKGROUP_SIZE) {
        uint writeIdx = 3u * triangleIdx;
        uint readIdx = writeIdx + firstPrimitiveID * 3u;

        gl_PrimitiveIndicesNV[writeIdx] = indexBuffer[readIdx];
        gl_PrimitiveIndicesNV[writeIdx + 1u] = indexBuffer[readIdx + 1u];
        gl_PrimitiveIndicesNV[writeIdx + 2u] = indexBuffer[readIdx + 2u];

        // Available in gl_MeshPrimitivesNV:
        // https://github.com/KhronosGroup/GLSL/blob/master/extensions/nv/GLSL_NV_mesh_shader.txt
        gl_PrimitiveID = firstPrimitiveID + triangleIdx;
    }
}


-- Fragment

#version 450 core

// Primitive index attachment uses clear value 0xFFFFFFFFu.
layout(location = 0) out uint fragmentPrimitiveIndex;

void main() {
    fragmentPrimitiveIndex = uint(gl_PrimitiveID);
}
