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

#include <glm/glm.hpp>
#include <tracy/Tracy.hpp>
#include "MeshSmoothing.hpp"

void createNeighborMap(
        const std::vector<uint32_t>& triangleIndices,
        std::unordered_map<uint32_t, std::unordered_set<uint32_t>>& neighborsMap) {
    ZoneScoped;

    for (size_t i = 0; i < triangleIndices.size(); i += 3) {
        uint32_t idx0 = triangleIndices.at(i+0);
        uint32_t idx1 = triangleIndices.at(i+1);
        uint32_t idx2 = triangleIndices.at(i+2);
        neighborsMap[idx0].insert(idx1);
        neighborsMap[idx0].insert(idx2);
        neighborsMap[idx1].insert(idx0);
        neighborsMap[idx1].insert(idx2);
        neighborsMap[idx2].insert(idx0);
        neighborsMap[idx2].insert(idx1);
    }
}

void laplacianSmoothing(
        const std::vector<glm::vec3>& pointsIn, std::vector<glm::vec3>& pointsOut,
        const std::unordered_map<uint32_t, std::unordered_set<uint32_t>>& neighborsMap, float lambda) {
    ZoneScoped;

    for (uint32_t i = 0; i < pointsIn.size(); i++) {
        float weightSum = 0.0f;
        glm::vec3 pointSum(0.0f);
        auto it = neighborsMap.find(i);
        if (it != neighborsMap.end()) {
            const std::unordered_set<uint32_t>& neighbors = it->second;
            for (uint32_t j : neighbors) {
                float w_ij = 1.0f / glm::length(pointsIn.at(i) - pointsIn.at(j));
                weightSum += w_ij;
                pointSum += w_ij * pointsIn.at(j);
            }
        }
        pointsOut.at(i) = pointsIn.at(i) + lambda * (1.0f / weightSum * pointSum - pointsIn.at(i));
    }
}

void laplacianSmoothing(
        const std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
        int numIterations, float lambda) {
    ZoneScoped;

    std::unordered_map<uint32_t, std::unordered_set<uint32_t>> neighborsMap;
    createNeighborMap(triangleIndices, neighborsMap);

    std::vector<glm::vec3> pointsTmp;
    pointsTmp.resize(vertexPositions.size());
    for (int i = 0; i < 4; i++) {
        std::vector<glm::vec3>* pointsIn;
        std::vector<glm::vec3>* pointsOut;
        if (i % 2 == 0) {
            pointsIn = &vertexPositions;
            pointsOut = &pointsTmp;
        } else {
            pointsIn = &pointsTmp;
            pointsOut = &vertexPositions;
        }
        laplacianSmoothing(*pointsIn, *pointsOut, neighborsMap, lambda);
    }

    if (numIterations % 2 == 1) {
        vertexPositions = pointsTmp;
    }
}
