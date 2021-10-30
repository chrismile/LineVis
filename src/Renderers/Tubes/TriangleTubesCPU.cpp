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

#include <Utils/File/Logfile.hpp>
#include "Tubes.hpp"

void createTriangleTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<TubeTriangleVertexData>& vertexDataList,
        std::vector<LinePointReference>& linePointReferenceList,
        uint32_t linePointOffset,
        std::vector<glm::vec3>& lineTangents,
        std::vector<glm::vec3>& lineNormals) {
    if (size_t(numCircleSubdivisions) != globalCircleVertexPositions.size() || tubeRadius != globalTubeRadius) {
        initGlobalCircleVertexPositions(numCircleSubdivisions, tubeRadius);
    }

    for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
        const std::vector<glm::vec3> &lineCenters = lineCentersList.at(lineId);
        size_t n = lineCenters.size();
        size_t indexOffset = vertexDataList.size();

        if (n < 2) {
            continue;
        }

        glm::vec3 lastLineNormal(1.0f, 0.0f, 0.0f);
        int numValidLinePoints = 0;
        for (size_t i = 0; i < n; i++) {
            glm::vec3 tangent;
            if (i == 0) {
                tangent = lineCenters[i + 1] - lineCenters[i];
            } else if (i == n - 1) {
                tangent = lineCenters[i] - lineCenters[i - 1];
            } else {
                tangent = lineCenters[i + 1] - lineCenters[i - 1];
            }
            float lineSegmentLength = glm::length(tangent);

            if (lineSegmentLength < 0.0001f) {
                // In case the two vertices are almost identical, just skip this path line segment.
                continue;
            }
            tangent = glm::normalize(tangent);

            insertOrientedCirclePoints(
                    lineCenters.at(i), tangent, lastLineNormal,
                    linePointOffset + uint32_t(linePointReferenceList.size()),
                    vertexDataList);
            lineTangents.push_back(tangent);
            lineNormals.push_back(lastLineNormal);
            linePointReferenceList.emplace_back(lineId, i);
            numValidLinePoints++;
        }

        if (numValidLinePoints == 1) {
            // Only one vertex left -> output nothing (tube consisting only of one point).
            for (int subdivIdx = 0; subdivIdx < numCircleSubdivisions; subdivIdx++) {
                vertexDataList.pop_back();
            }
            linePointReferenceList.pop_back();
            lineTangents.pop_back();
            lineNormals.pop_back();
            continue;
        }

        for (int i = 0; i < numValidLinePoints-1; i++) {
            for (int j = 0; j < numCircleSubdivisions; j++) {
                // Build two CCW triangles (one quad) for each side.
                // Triangle 1
                triangleIndices.push_back(
                        indexOffset + i*numCircleSubdivisions+j);
                triangleIndices.push_back(
                        indexOffset + i*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
                triangleIndices.push_back(
                        indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+(j+1)%numCircleSubdivisions);

                // Triangle 2
                triangleIndices.push_back(
                        indexOffset + i*numCircleSubdivisions+j);
                triangleIndices.push_back(
                        indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
                triangleIndices.push_back(
                        indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+j);
            }
        }
    }
}

void createTriangleEllipticTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<glm::vec3>>& lineRightVectorsList,
        float tubeNormalRadius,
        float tubeBinormalRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<TubeTriangleVertexData>& vertexDataList,
        std::vector<LinePointReference>& linePointReferenceList,
        uint32_t linePointOffset,
        std::vector<glm::vec3>& lineTangents,
        std::vector<glm::vec3>& lineNormals) {
    if (size_t(numCircleSubdivisions) != globalEllipseVertexPositions.size()
            || tubeNormalRadius != globalTubeNormalRadius
            || tubeBinormalRadius != globalTubeBinormalRadius) {
        initGlobalEllipseVertexPositions(numCircleSubdivisions, tubeNormalRadius, tubeBinormalRadius);
    }

    for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
        const std::vector<glm::vec3> &lineCenters = lineCentersList.at(lineId);
        const std::vector<glm::vec3> &lineRightVectors = lineRightVectorsList.at(lineId);
        size_t n = lineCenters.size();
        size_t indexOffset = vertexDataList.size();

        if (n < 2) {
            continue;
        }

        int numValidLinePoints = 0;
        for (size_t i = 0; i < n; i++) {
            glm::vec3 tangent;
            if (i == 0) {
                tangent = lineCenters[i+1] - lineCenters[i];
            } else if (i == n - 1) {
                tangent = lineCenters[i] - lineCenters[i-1];
            } else {
                tangent = (lineCenters[i+1] - lineCenters[i-1]);
            }
            float lineSegmentLength = glm::length(tangent);

            if (lineSegmentLength < 0.0001f) {
                // In case the two vertices are almost identical, just skip this path line segment.
                continue;
            }
            tangent = glm::normalize(tangent);
            glm::vec3 normal = glm::cross(lineRightVectors.at(i), tangent);

            insertOrientedEllipsePoints(
                    lineCenters.at(i), tangent, normal,
                    linePointOffset + uint32_t(linePointReferenceList.size()),
                    vertexDataList);
            lineTangents.push_back(tangent);
            lineNormals.push_back(normal);
            linePointReferenceList.emplace_back(lineId, i);
            numValidLinePoints++;
        }

        if (numValidLinePoints == 1) {
            // Only one vertex left -> Output nothing (tube consisting only of one point).
            for (int subdivIdx = 0; subdivIdx < numCircleSubdivisions; subdivIdx++) {
                vertexDataList.pop_back();
            }
            linePointReferenceList.pop_back();
            lineTangents.pop_back();
            lineNormals.pop_back();
            continue;
        }

        for (int i = 0; i < numValidLinePoints-1; i++) {
            for (int j = 0; j < numCircleSubdivisions; j++) {
                // Build two CCW triangles (one quad) for each side.
                // Triangle 1
                triangleIndices.push_back(
                        indexOffset + i*numCircleSubdivisions+j);
                triangleIndices.push_back(
                        indexOffset + i*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
                triangleIndices.push_back(
                        indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+(j+1)%numCircleSubdivisions);

                // Triangle 2
                triangleIndices.push_back(
                        indexOffset + i*numCircleSubdivisions+j);
                triangleIndices.push_back(
                        indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
                triangleIndices.push_back(
                        indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+j);
            }
        }
    }
}
