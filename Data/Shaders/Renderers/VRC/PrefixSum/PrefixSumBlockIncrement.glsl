-- Compute

#version 430

#define BLOCK_SIZE 256

layout(local_size_x = BLOCK_SIZE) in;

layout(std430, binding = 1) buffer DataOutBuffer {
    uint dataOut[];
};

layout(std430, binding = 2) readonly buffer SumInBuffer {
    uint sumIn[];
};

uniform uint N;

shared uint a;

/**
 * Adds the blockDim.x elements of sumIn to the block elements in dataOut.
 * @param N The number of elements in dataOut.
 * @param dataOut The data array of size N.
 * @param sumIn Array of block increments to be added to the elements in dataOut.
 */
void main() {
    if (gl_LocalInvocationID.x == 0) {
        a = sumIn[gl_WorkGroupID.x];
    }
    memoryBarrierShared();
    barrier();

    uint globalOffset = gl_WorkGroupSize.x * 2 * gl_WorkGroupID.x;
    uint idx0 = globalOffset + gl_LocalInvocationID.x * 2;
    uint idx1 = globalOffset + gl_LocalInvocationID.x * 2 + 1;
    if (idx0 < N) {
        dataOut[idx0] += a;
    }
    if (idx1 < N) {
        dataOut[idx1] += a;
    }
}
