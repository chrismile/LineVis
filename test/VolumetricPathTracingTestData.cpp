/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#include "LineData/Scattering/CloudData.hpp"
#include "VolumetricPathTracingTestData.hpp"

CloudDataPtr createCloudBlock(
        uint32_t xs, uint32_t ys, uint32_t zs, float constValue, bool useEmptyBoundaryLayer) {
    auto* gridData = new float[xs * ys * zs];
    if (useEmptyBoundaryLayer && xs >= 3 && ys >= 3 && zs >= 3) {
        for (uint32_t z = 0; z < zs; z++) {
            for (uint32_t y = 0; y < ys; y++) {
                for (uint32_t x = 0; x < xs; x++) {
                    float value = 0.0f;
                    if (x >= 1 && y >= 1 && z >= 1 && x <= xs - 2 && y <= ys - 2 && z <= zs - 2) {
                        value = 1.0f;
                    }
                    gridData[x + y * xs + z * xs * ys] = value;
                }
            }
        }
    } else {
        for (uint32_t z = 0; z < zs; z++) {
            for (uint32_t y = 0; y < ys; y++) {
                for (uint32_t x = 0; x < xs; x++) {
                    gridData[x + y * xs + z * xs * ys] = 1.0f;
                }
            }
        }
    }

    CloudDataPtr cloudData = std::make_shared<CloudData>();
    cloudData->setDensityField(xs, ys, zs, gridData);
    return cloudData;
}
