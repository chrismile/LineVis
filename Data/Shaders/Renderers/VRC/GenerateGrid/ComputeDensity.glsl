/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2021, Christoph Neuhauser
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

layout (local_size_x = 64, local_size_y = 4, local_size_z = 1) in;

#include "VRCComputeHeaderVoxel.glsl"
#include "TransferFunction.glsl"

layout (binding = 0, r32f) uniform image3D densityImage;

void main() {
    ivec3 voxelIndex = ivec3(gl_GlobalInvocationID.xyz);
    if (any(greaterThanEqual(voxelIndex, gridResolution))) {
        return;
    }
    uint voxelIndex1D = getVoxelIndex1D(voxelIndex);
    uint numLinePoints = max(numSegments[voxelIndex1D], MAX_NUM_LINES_PER_VOXEL);

    float density = 0.0;
    LineSegment lineSegment;
    for (uint i = 0; i < numLinePoints; i++) {
        decompressLine(vec3(voxelIndex), lineSegments[voxelIndex1D*MAX_NUM_LINES_PER_VOXEL+i], lineSegment);
        float lineLength = length(lineSegment.v1 - lineSegment.v0);
        density += lineLength * (transferFunction(lineSegment.a0).a + transferFunction(lineSegment.a1).a) / 2.0f;
    }
    imageStore(densityImage, voxelIndex, vec4(density));
}
