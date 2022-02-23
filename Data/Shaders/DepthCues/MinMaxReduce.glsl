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

layout (local_size_x = BLOCK_SIZE) in;

layout(push_constant) uniform PushConstants {
    uint sizeOfInput; ///< Number of entries in MinMaxInBuffer.
};

layout (binding = 0) uniform UniformDataBuffer {
    float nearDist; ///< The distance of the near plane.
    float farDist; ///< The distance of the far plane.
    uint numVertices; ///< Number of entries in VertexPositionBuffer.
    uint padding;
    mat4 cameraViewMatrix;
    mat4 cameraProjectionMatrix;
};

// Size: sizeOfInput
layout (std430, binding = 1) readonly buffer MinMaxInBuffer {
    vec4 minMaxInBuffer[];
};

// Size: iceil(sizeOfInput, BLOCK_SIZE*2))
layout (std430, binding = 2) writeonly buffer MinMaxOutBuffer {
    vec4 minMaxOutBuffer[];
};

shared vec2 sharedMemoryMinMax[BLOCK_SIZE];

/**
 * Reference: Based on kernel 4 from https://developer.download.nvidia.com/assets/cuda/files/reduction.pdf
 */
void main() {
    uint threadId = gl_LocalInvocationID.x;
    uint i = gl_LocalInvocationID.x + gl_WorkGroupID.x * gl_WorkGroupSize.x * 2;

    // Copy the data to the shared memory and do the first reduction step.
    if (i + gl_WorkGroupSize.x < sizeOfInput){
        vec2 minMax0 = minMaxInBuffer[i].xy;
        vec2 minMax1 = minMaxInBuffer[i + gl_WorkGroupSize.x].xy;
        sharedMemoryMinMax[threadId] = vec2(
                min(minMax0.x, minMax1.x),
                max(minMax0.y, minMax1.y)
        );
    } else if (i < sizeOfInput){
        sharedMemoryMinMax[threadId] = minMaxInBuffer[i].xy;
    } else{
        sharedMemoryMinMax[threadId] = vec2(farDist, nearDist);
    }
    memoryBarrierShared();
    barrier();

    // Do the reduction in the shared memory.
    for (uint stride = gl_WorkGroupSize.x / 2; stride > 0; stride >>= 1) {
        if (threadId < stride) {
            vec2 minMax0 = sharedMemoryMinMax[threadId];
            vec2 minMax1 = sharedMemoryMinMax[threadId + stride];
            sharedMemoryMinMax[threadId] = vec2(
                    min(minMax0.x, minMax1.x),
                    max(minMax0.y, minMax1.y)
            );
        }
        memoryBarrierShared();
        barrier();
    }

    // Write the result for this block to global memory.
    if (threadId == 0) {
        minMaxOutBuffer[gl_WorkGroupID.x] = vec4(sharedMemoryMinMax[0], 0.0, 0.0);
    }
}
