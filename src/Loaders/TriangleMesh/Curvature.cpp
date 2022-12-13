/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2019-2022, Christoph Neuhauser, Michael Kern
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

#include <algorithm>

#include <glm/glm.hpp>

#include "Curvature.hpp"

void insertNeighborMap(std::vector<uint32_t>& neighbors, uint32_t newIdx) {
    if (std::find(neighbors.begin(), neighbors.end(), newIdx) == neighbors.end()) {
        neighbors.push_back(newIdx);
    }
}

/*
 * TODO: Add code for computing Gaussian curvature and mean curvature.
 * http://rodolphe-vaillant.fr/entry/33/curvature-of-a-triangle-mesh-definition-and-computation
 * Gaussian curvature: k_g = (2pi - sum_j theta_j) / A_i
 * Mean curvature: H = (k_1 + k_2) / 2 = ||Delta p_i|| / 2
 * Delta p_i = 1 / (2*A_i) sum_j (cot alpha a_ij + cot beta_ij)(p_j - p_i)
 */

void computeCurvature(
        const std::vector<uint32_t>& triangleIndices, const std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes) {
    std::vector<std::vector<uint32_t>> vertexNeighborMap;
    vertexNeighborMap.resize(vertexPositions.size());

    // TODO: Use data structure with which we can access the triangles forming the ring-1 neighbors.
    for (size_t i = 0; i < triangleIndices.size(); i += 3) {
        uint32_t vertexIndex0 = triangleIndices.at(i);
        uint32_t vertexIndex1 = triangleIndices.at(i+1);
        uint32_t vertexIndex2 = triangleIndices.at(i+2);
        auto& neighborMap0 = vertexNeighborMap.at(vertexIndex0);
        auto& neighborMap1 = vertexNeighborMap.at(vertexIndex1);
        auto& neighborMap2 = vertexNeighborMap.at(vertexIndex2);
        insertNeighborMap(neighborMap0, vertexIndex1);
        insertNeighborMap(neighborMap0, vertexIndex2);
        insertNeighborMap(neighborMap1, vertexIndex0);
        insertNeighborMap(neighborMap1, vertexIndex2);
        insertNeighborMap(neighborMap2, vertexIndex0);
        insertNeighborMap(neighborMap2, vertexIndex1);
    }

    vertexAttributes.resize(vertexPositions.size());
    for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
        const auto& neighborVertexIndices = vertexNeighborMap.at(vertexIdx);
        const glm::vec3& normalCenter = vertexNormals.at(vertexIdx);
        const glm::vec3& positionCenter = vertexPositions.at(vertexIdx);

        // Compute curvature for ring-1 vertices considering their normals.
        std::vector<float> ring1Curvatures(neighborVertexIndices.size(), 0.0f);
        for (size_t neighborIdx = 0; neighborIdx < neighborVertexIndices.size(); neighborIdx++) {
            uint32_t neighborVertexIdx = neighborVertexIndices.at(neighborIdx);
            const glm::vec3& normalNeighbor = vertexNormals.at(neighborVertexIdx);
            const glm::vec3& positionNeighbor = vertexPositions.at(neighborVertexIdx);
            const glm::vec3 n = normalNeighbor - normalCenter;
            const glm::vec3 p = positionNeighbor - positionCenter;
            const float l2 = glm::length(p) * glm::length(p);
            ring1Curvatures.at(neighborIdx) = glm::dot(n, p) / l2;
        }

        // Iterate over all edges and compute the edge curvatures.
        float totalCurvature = 0.0f;
        float totalAngle = 0.0f;
        for (size_t neighborIdx = 0; neighborIdx < neighborVertexIndices.size() - 1; neighborIdx++) {
            const glm::vec3& positionNeighbor0 = vertexPositions.at(neighborVertexIndices.at(neighborIdx));
            const glm::vec3& positionNeighbor1 = vertexPositions.at(neighborVertexIndices.at(neighborIdx + 1));

            // Compute angle between the two edges.
            glm::vec3 edge0 = glm::normalize(positionNeighbor0 - positionCenter);
            glm::vec3 edge1 = glm::normalize(positionNeighbor1 - positionCenter);
            float absoluteSineValue = glm::length(glm::cross(edge0, edge1));
            // absoluteSineValue may become slightly larger than zero due to numerical problems.
            float angle = glm::asin(std::min(1.0f, absoluteSineValue));

            totalAngle += angle;
            totalCurvature += angle * (ring1Curvatures.at(neighborIdx) + ring1Curvatures.at(neighborIdx + 1));
        }

        totalCurvature = 0.5f * totalCurvature / totalAngle;
        vertexAttributes.at(vertexIdx) = totalCurvature;

        ring1Curvatures.clear();
        ring1Curvatures.shrink_to_fit();
    }
}
