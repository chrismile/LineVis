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

#version 450 core

#define BLOCK_SIZE 256

layout(local_size_x = BLOCK_SIZE) in;

layout(push_constant) uniform PushConstants {
    uint N;
};

layout(std430, binding = 1) buffer DataOutBuffer {
    uint dataOut[];
};

layout(std430, binding = 2) readonly buffer SumInBuffer {
    uint sumIn[];
};

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
