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

-- Compute

#version 430

#define BLOCK_SIZE 256

layout (local_size_x = BLOCK_SIZE, local_size_y = 1, local_size_z = 1) in;

// Size: numVertices
layout (std430, binding = 12) readonly buffer VertexPositionBuffer {
    vec4 vertexPositions[];
};

// Size: iceil(numVertices, BLOCK_SIZE)
layout (std430, binding = 11) writeonly buffer DepthMinMaxOutBuffer {
    vec2 depthMinMaxOutBuffer[];
};

uniform uint numVertices; ///< Number of entries in VertexPositionBuffer.
uniform float nearDist; ///< The distance of the near plane.
uniform float farDist; ///< The distance of the far plane.
uniform mat4 cameraViewMatrix;
uniform mat4 cameraProjectionMatrix;

shared vec2 sharedMemoryMinMaxDepth[BLOCK_SIZE];

const float EPSILON = 1e-2;

void main() {
    uint localIdx = gl_LocalInvocationID.x;
    uint localDim = gl_WorkGroupSize.x; // == BLOCK_SIZE
    uint reductionIdx = gl_WorkGroupID.x;

    vec2 depthMinMax = vec2(farDist, nearDist);
    if (gl_GlobalInvocationID.x < numVertices) {
        vec4 vertexPosition = vertexPositions[gl_GlobalInvocationID.x];
        vec4 screenSpacePosition = cameraViewMatrix * vertexPosition;
        vec4 ndcPosition = cameraProjectionMatrix * screenSpacePosition;
        ndcPosition.xyz /= ndcPosition.w;
        // View frustum culling.
        if (all(greaterThanEqual(ndcPosition.xyz, vec3(-1.0))) && all(lessThanEqual(ndcPosition.xyz, vec3(1.0)))) {
            float depth = clamp(-screenSpacePosition.z, nearDist, farDist);
            depthMinMax = vec2(depth - EPSILON, depth + EPSILON);
        }
    }

    sharedMemoryMinMaxDepth[localIdx] = depthMinMax;
    memoryBarrierShared();
    barrier();

    for (uint stride = localDim / 2; stride > 0; stride >>= 1) {
        if (localIdx < stride) {
            vec2 minMaxDepth0 = sharedMemoryMinMaxDepth[localIdx];
            vec2 minMaxDepth1 = sharedMemoryMinMaxDepth[localIdx + stride];
            sharedMemoryMinMaxDepth[localIdx] = vec2(
                    min(minMaxDepth0.x, minMaxDepth1.x),
                    max(minMaxDepth0.y, minMaxDepth1.y)
            );
        }
        memoryBarrierShared();
        barrier();
    }

    if (localIdx == 0) {
        depthMinMaxOutBuffer[reductionIdx] = sharedMemoryMinMaxDepth[0];
    }
}
