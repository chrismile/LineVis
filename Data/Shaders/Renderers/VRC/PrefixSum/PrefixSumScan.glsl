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

layout(local_size_x = BLOCK_SIZE) in;

#define AVOID_MEMORY_BANK_CONFLICTS
#define NUM_BANKS 32
#define LOG_NUM_BANKS 5
#define CONFLICT_FREE_OFFSET(n) ((n) >> LOG_NUM_BANKS)

layout(std430, binding = 0) readonly buffer DataInBuffer {
    uint dataIn[];
};

layout(std430, binding = 1) buffer DataOutBuffer {
    uint dataOut[];
};

layout(std430, binding = 2) buffer SumOutBuffer {
    uint sumOut[];
};

uniform uint N;

shared uint sdata[2 * BLOCK_SIZE + 2 * BLOCK_SIZE / NUM_BANKS];

/*
 * Based on the idea from:
 * https://developer.nvidia.com/gpugems/gpugems3/part-vi-gpu-computing/chapter-39-parallel-prefix-sum-scan-cuda
 */
/**
 * Based on the work-efficient parallel sum scan algorithm [Blelloch 1990] implementation from:
 * https://developer.nvidia.com/gpugems/gpugems3/part-vi-gpu-computing/chapter-39-parallel-prefix-sum-scan-cuda
 * @param N The number of values in the array.
 * @param dataIn An input array with N entries.
 * @param dataOut A (possibly uninitialized, but allocated) output array with N entries.
 * @param sumOut Array of block increments to be added to the elements in dataOut.
 */
#ifdef AVOID_MEMORY_BANK_CONFLICTS
void main() {
    uint blockDimTimes2 = gl_WorkGroupSize.x * 2;
    uint globalOffset = blockDimTimes2 * gl_WorkGroupID.x;
    uint localIdx = gl_LocalInvocationID.x;
    uint localIdxTimes2 = localIdx * 2;

    // Copy the data to the input array.
    uint idxA = localIdx;
    uint idxB = localIdx + gl_WorkGroupSize.x;
    uint bankOffsetA = CONFLICT_FREE_OFFSET(idxA);
    uint bankOffsetB = CONFLICT_FREE_OFFSET(idxB);
    if (globalOffset + idxA < N) {
        sdata[idxA + bankOffsetA] = dataIn[globalOffset + idxA];
    } else {
        sdata[idxA + bankOffsetA] = 0;
    }
    if (globalOffset + idxB < N) {
        sdata[idxB + bankOffsetB] = dataIn[globalOffset + idxB];
    } else {
        sdata[idxB + bankOffsetB] = 0;
    }

    // Up-sweep (reduce) phase.
    uint offset = 1;
    for (uint i = gl_WorkGroupSize.x; i > 0; i >>= 1) {
        memoryBarrierShared();
        barrier();

        if (localIdx < i) {
            uint idxA = offset * (localIdxTimes2 + 1) - 1;
            uint idxB = offset * (localIdxTimes2 + 2) - 1;
            idxA += CONFLICT_FREE_OFFSET(idxA);
            idxB += CONFLICT_FREE_OFFSET(idxB);
            sdata[idxB] += sdata[idxA];
        }
        offset <<= 1;
    }

    if (localIdx == 0) {
        uint lastElementIdx = blockDimTimes2 - 1;
        lastElementIdx += CONFLICT_FREE_OFFSET(lastElementIdx);
        sumOut[gl_WorkGroupID.x] = sdata[lastElementIdx];
        sdata[lastElementIdx] = 0;
    }

    // Down-sweep phase.
    for (uint i = 1; i < blockDimTimes2; i <<= 1) {
        offset >>= 1;
        memoryBarrierShared();
        barrier();

        if (localIdx < i) {
            uint idxA = offset * (localIdxTimes2 + 1) - 1;
            uint idxB = offset * (localIdxTimes2 + 2) - 1;
            idxA += CONFLICT_FREE_OFFSET(idxA);
            idxB += CONFLICT_FREE_OFFSET(idxB);
            uint temp = sdata[idxA];
            sdata[idxA] = sdata[idxB];
            sdata[idxB] += temp;
        }
    }
    memoryBarrierShared();
    barrier();

    // Copy partial sum scan to the output array.
    if (globalOffset + idxA < N) {
        dataOut[globalOffset + idxA] = sdata[idxA + bankOffsetA];
    }
    if (globalOffset + idxB < N) {
        dataOut[globalOffset + idxB] = sdata[idxB + bankOffsetB];
    }
}
#else
// No handling of shared memory bank conflicts:
void main() {
    shared uint sdata[2 * SHARED_MEMORY_SIZE];

    uint blockDimTimes2 = gl_WorkGroupSize.x * 2;
    uint globalOffset = blockDimTimes2 * gl_WorkGroupID.x;
    uint localIdx = gl_LocalInvocationID.x;
    uint localIdxTimes2 = localIdx * 2;

    // Copy the data to the input array.
    if (globalOffset + localIdxTimes2 < N) {
        sdata[localIdxTimes2] = dataIn[globalOffset + localIdxTimes2];
    } else {
        sdata[localIdxTimes2] = 0;
    }
    if (globalOffset + localIdxTimes2 + 1 < N) {
        sdata[localIdxTimes2 + 1] = dataIn[globalOffset + localIdxTimes2 + 1];
    } else {
        sdata[localIdxTimes2 + 1] = 0;
    }

    // Up-sweep (reduce) phase.
    uint offset = 1;
    for (uint i = gl_WorkGroupSize.x; i > 0; i >>= 1) {
        memoryBarrierShared();
        barrier();

        if (localIdx < i) {
            uint data0 = offset * (localIdxTimes2 + 1) - 1;
            uint data1 = offset * (localIdxTimes2 + 2) - 1;
            sdata[data1] += sdata[data0];
        }
        offset <<= 1;
    }

    if (localIdx == 0) {
        sumOut[gl_WorkGroupID.x] = sdata[blockDimTimes2 - 1];
        sdata[blockDimTimes2 - 1] = 0;
    }

    // Down-sweep phase.
    for (uint i = 1; i < blockDimTimes2; i <<= 1) {
        offset >>= 1;
        memoryBarrierShared();
        barrier();

        if (localIdx < i) {
            uint ai = offset * (localIdxTimes2 + 1) - 1;
            uint bi = offset * (localIdxTimes2 + 2) - 1;
            uint temp = sdata[ai];
            sdata[ai] = sdata[bi];
            sdata[bi] += temp;
        }
    }
    memoryBarrierShared();
    barrier();

    // Copy partial sum scan to the output array.
    if (globalOffset + localIdxTimes2 < N) {
        dataOut[globalOffset + localIdxTimes2] = sdata[localIdxTimes2];
    }
    if (globalOffset + localIdxTimes2 + 1 < N) {
        dataOut[globalOffset + localIdxTimes2 + 1] = sdata[localIdxTimes2 + 1];
    }
}
#endif
