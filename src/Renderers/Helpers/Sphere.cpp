/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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

#include <Math/Math.hpp>
#include "Sphere.hpp"

void getSphereSurfaceRenderData(
        const glm::vec3& center, float radius, int sectorCount, int stackCount,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals,
        std::vector<uint32_t>& triangleIndices) {
    float phi, theta, sinPhi, cosPhi;
    float sectorStep = sgl::TWO_PI / float(sectorCount);
    float stackStep = sgl::PI / float(stackCount);

    // 1. Build the vertex buffers.
    for (int stackIdx = 0; stackIdx <= stackCount; ++stackIdx) {
        phi = sgl::HALF_PI - float(stackIdx) * stackStep;
        cosPhi = std::cos(phi);
        sinPhi = std::sin(phi);
        for (int sectorIdx = 0; sectorIdx <= sectorCount; ++sectorIdx) {
            theta = float(sectorIdx) * sectorStep;
            glm::vec3 normal(cosPhi * std::cos(theta), cosPhi * std::sin(theta), sinPhi);
            glm::vec3 position = center + radius * normal;
            vertexPositions.push_back(position);
            vertexNormals.push_back(normal);
        }
    }

    // 2. Build the index buffer.
    uint32_t k1, k2;
    for (int stackIdx = 0; stackIdx <= stackCount; ++stackIdx) {
        k1 = stackIdx * (sectorCount + 1);
        k2 = k1 + sectorCount + 1;
        for (int sectorIdx = 0; sectorIdx <= sectorCount; ++sectorIdx) {
            if(stackIdx != 0) {
                triangleIndices.push_back(k1);
                triangleIndices.push_back(k2);
                triangleIndices.push_back(k1 + 1);
            }
            if(stackIdx != (stackCount-1)) {
                triangleIndices.push_back(k1 + 1);
                triangleIndices.push_back(k2);
                triangleIndices.push_back(k2 + 1);
            }
            k1++;
            k2++;
        }
    }
}
