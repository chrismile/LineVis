-- Compute

#version 430

#define BLOCK_SIZE 256

layout (local_size_x = BLOCK_SIZE) in;

// Size: sizeOfInput
layout (std430, binding = 11) readonly buffer DepthMinMaxInBuffer {
    vec2 depthMinMaxInBuffer[];
};

// Size: iceil(sizeOfInput, BLOCK_SIZE*2))
layout (std430, binding = 12) writeonly buffer DepthMinMaxOutBuffer {
    vec2 depthMinMaxOutBuffer[];
};

uniform uint sizeOfInput; ///< Number of entries in DepthMinMaxInBuffer.
uniform float nearDist; ///< The distance of the near plane.
uniform float farDist; ///< The distance of the far plane.

shared vec2 sharedMemoryMinMaxDepth[BLOCK_SIZE];

/**
 * Reference: Based on kernel 4 from https://developer.download.nvidia.com/assets/cuda/files/reduction.pdf
 */
void main() {
    uint threadId = gl_LocalInvocationID.x;
    uint i = gl_LocalInvocationID.x + gl_WorkGroupID.x * gl_WorkGroupSize.x * 2;

    // Copy the data to the shared memory and do the first reduction step.
    if (i + gl_WorkGroupSize.x < sizeOfInput){
        vec2 minMaxDepth0 = depthMinMaxInBuffer[i];
        vec2 minMaxDepth1 = depthMinMaxInBuffer[i + gl_WorkGroupSize.x];
        sharedMemoryMinMaxDepth[threadId] = vec2(
                min(minMaxDepth0.x, minMaxDepth1.x),
                max(minMaxDepth0.y, minMaxDepth1.y)
        );
    } else if (i < sizeOfInput){
        sharedMemoryMinMaxDepth[threadId] = depthMinMaxInBuffer[i];
    } else{
        sharedMemoryMinMaxDepth[threadId] = vec2(farDist, nearDist);
    }
    memoryBarrierShared();
    barrier();

    // Do the reduction in the shared memory.
    for (uint stride = gl_WorkGroupSize.x / 2; stride > 0; stride >>= 1) {
        if (threadId < stride) {
            vec2 minMaxDepth0 = sharedMemoryMinMaxDepth[threadId];
            vec2 minMaxDepth1 = sharedMemoryMinMaxDepth[threadId + stride];
            sharedMemoryMinMaxDepth[threadId] = vec2(
                    min(minMaxDepth0.x, minMaxDepth1.x),
                    max(minMaxDepth0.y, minMaxDepth1.y)
            );
        }
        memoryBarrierShared();
        barrier();
    }

    // Write the result for this block to global memory.
    if (threadId == 0) {
        depthMinMaxOutBuffer[gl_WorkGroupID.x] = sharedMemoryMinMaxDepth[0];
    }
}
