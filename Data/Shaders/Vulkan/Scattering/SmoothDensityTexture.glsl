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

layout (local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout (binding = 0) uniform UniformBuffer {
    ivec3 gridResolution;
};

layout (binding = 1, std430) readonly buffer FilterWeightBuffer {
    float blurKernel[];
};

layout (binding = 2) uniform sampler3D inputTexture;

layout (binding = 3, r32f) uniform image3D outputImage;

float filterDensity(ivec3 voxelIndex) {
    const int FILTER_SIZE = 7;
    const int FILTER_EXTENT = (FILTER_SIZE - 1) / 2;

    // Filter the densities
    float kernelSum = 0.0f;
    float filteredDensitySum = 0.0f;
    kernelSum += 0.0001f;

    for (int offsetZ = -FILTER_EXTENT; offsetZ <= FILTER_EXTENT; offsetZ++) {
        for (int offsetY = -FILTER_EXTENT; offsetY <= FILTER_EXTENT; offsetY++) {
            for (int offsetX = -FILTER_EXTENT; offsetX <= FILTER_EXTENT; offsetX++) {
                ivec3 readIndex = voxelIndex + ivec3(offsetX, offsetY, offsetZ);
                if (readIndex.x >= 0 && readIndex.y >= 0 && readIndex.z >= 0 && readIndex.x < gridResolution.x
                        && readIndex.y < gridResolution.y && readIndex.z < gridResolution.z) {
                    int filterIdx = (offsetZ+FILTER_EXTENT)*FILTER_SIZE*FILTER_SIZE
                            + (offsetY+FILTER_EXTENT)*FILTER_SIZE + (offsetX+FILTER_EXTENT);
                    float kernelValue = blurKernel[filterIdx];
                    kernelSum += kernelValue;
                    filteredDensitySum += texelFetch(inputTexture, readIndex, 0).x * kernelValue;
                }
            }
        }
    }

    return filteredDensitySum / kernelSum;
}

void main() {
    ivec3 voxelIndex = ivec3(gl_GlobalInvocationID.xyz);
    if (any(greaterThanEqual(voxelIndex, gridResolution))) {
        return;
    }

    float filteredDensity = filterDensity(voxelIndex);
    imageStore(outputImage, voxelIndex, vec4(filteredDensity));
}
