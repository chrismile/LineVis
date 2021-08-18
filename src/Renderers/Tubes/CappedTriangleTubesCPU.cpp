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
#include <Utils/File/Logfile.hpp>
#include "Tubes.hpp"

template<typename T>
void addHemisphereToMesh(
        const glm::vec3& center, glm::vec3 tangent, glm::vec3 normal, size_t indexOffset,
        float tubeRadius, int numLongitudeSubdivisions, int numLatitudeSubdivisions, bool isStartHemisphere,
        std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals, std::vector<glm::vec3>& vertexTangents,
        std::vector<T>& vertexAttributes) {
    glm::vec3 binormal = glm::cross(normal, tangent);
    glm::vec3 scaledTangent = tubeRadius * tangent;
    glm::vec3 scaledNormal = tubeRadius * normal;
    glm::vec3 scaledBinormal = tubeRadius * binormal;
    T attributeValue;
    if (isStartHemisphere) {
        attributeValue = vertexAttributes.at(indexOffset);
    } else {
        attributeValue = vertexAttributes.back();
    }

    float theta; // azimuth;
    float phi; // zenith;

    size_t vertexIndexOffset = vertexPositions.size() - indexOffset - numLongitudeSubdivisions;
    for (int lat = 1; lat <= numLatitudeSubdivisions; lat++) {
        phi = sgl::HALF_PI * (1.0f - float(lat) / numLatitudeSubdivisions);
        for (int lon = 0; lon < numLongitudeSubdivisions; lon++) {
            theta = -sgl::TWO_PI * float(lon) / numLongitudeSubdivisions;

            glm::vec3 pt(
                    std::cos(theta) * std::sin(phi),
                    std::sin(theta) * std::sin(phi),
                    std::cos(phi)
            );

            glm::vec3 trafoPt(
                    pt.x * scaledNormal.x + pt.y * scaledBinormal.x + pt.z * scaledTangent.x + center.x,
                    pt.x * scaledNormal.y + pt.y * scaledBinormal.y + pt.z * scaledTangent.y + center.y,
                    pt.x * scaledNormal.z + pt.y * scaledBinormal.z + pt.z * scaledTangent.z + center.z
            );
            glm::vec3 normal = glm::normalize(glm::vec3(
                    pt.x * scaledNormal.x + pt.y * scaledBinormal.x + pt.z * scaledTangent.x,
                    pt.x * scaledNormal.y + pt.y * scaledBinormal.y + pt.z * scaledTangent.y,
                    pt.x * scaledNormal.z + pt.y * scaledBinormal.z + pt.z * scaledTangent.z
            ));

            vertexPositions.push_back(trafoPt);
            vertexNormals.push_back(normal);
            vertexTangents.push_back(tangent);
            vertexAttributes.push_back(attributeValue);

            if (lat == numLatitudeSubdivisions) {
                break;
            }
        }
    }
    for (int lat = 0; lat < numLatitudeSubdivisions; lat++) {
        for (int lon = 0; lon < numLongitudeSubdivisions; lon++) {
            if (isStartHemisphere && lat == 0) {
                triangleIndices.push_back(indexOffset +
                        (2*numLongitudeSubdivisions-lon)%numLongitudeSubdivisions
                        + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset +
                        (2*numLongitudeSubdivisions-lon-1)%numLongitudeSubdivisions
                        + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon)%numLongitudeSubdivisions
                             + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset +
                        (2*numLongitudeSubdivisions-lon-1)%numLongitudeSubdivisions
                        + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon+1)%numLongitudeSubdivisions
                             + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon)%numLongitudeSubdivisions
                             + (lat+1)*numLongitudeSubdivisions);
            } else if (lat < numLatitudeSubdivisions-1) {
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon)%numLongitudeSubdivisions
                             + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon+1)%numLongitudeSubdivisions
                             + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon)%numLongitudeSubdivisions
                             + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon+1)%numLongitudeSubdivisions
                             + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon+1)%numLongitudeSubdivisions
                             + (lat+1)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon)%numLongitudeSubdivisions
                             + (lat+1)*numLongitudeSubdivisions);
            } else {
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon)%numLongitudeSubdivisions
                             + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + (lon+1)%numLongitudeSubdivisions
                             + (lat)*numLongitudeSubdivisions);
                triangleIndices.push_back(indexOffset + vertexIndexOffset
                             + 0
                             + (lat+1)*numLongitudeSubdivisions);
            }
        }
    }
}

template<typename T>
void createCappedTriangleTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<T>>& lineAttributesList,
        float tubeRadius,
        bool tubeClosed,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<T>& vertexAttributes) {
    if (numCircleSubdivisions != globalCircleVertexPositions.size() || tubeRadius != globalTubeRadius) {
        initGlobalCircleVertexPositions(numCircleSubdivisions, tubeRadius);
    }

    assert(lineCentersList.size() == lineAttributesList.size());
    for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
        const std::vector<glm::vec3>& lineCenters = lineCentersList.at(lineId);
        const std::vector<T>& lineAttributes = lineAttributesList.at(lineId);
        assert(lineCenters.size() == lineAttributes.size());
        size_t n = lineCenters.size();
        size_t indexOffset = vertexPositions.size();

        // Assert that we have a valid input data range
        if (tubeClosed && n < 3) {
            sgl::Logfile::get()->writeError(
                    "ERROR in createCappedTriangleTubesRenderDataCPU: Closed tube too short.");
            return;
        }
        if (!tubeClosed && n < 2) {
            sgl::Logfile::get()->writeError(
                    "ERROR in createCappedTriangleTubesRenderDataCPU: Open tube too short.");
            return;
        }

        glm::vec3 lastLineNormal(1.0f, 0.0f, 0.0f);
        std::vector<glm::vec3> lineNormals;
        int numValidLinePoints = 0;
        for (size_t i = 0; i < n; i++) {
            glm::vec3 tangent;
            if (!tubeClosed && i == 0) {
                tangent = lineCenters[i+1] - lineCenters[i];
            } else if (!tubeClosed && i == n - 1) {
                tangent = lineCenters[i] - lineCenters[i-1];
            } else {
                tangent = (lineCenters[(i+1)%n] - lineCenters[(i-1+n)%n]);
            }

            float lineSegmentLength = glm::length(tangent);

            if (lineSegmentLength < 0.0001f) {
                // In case the two vertices are almost identical, just skip this path line segment
                continue;
            }
            tangent = glm::normalize(tangent);

            //insertOrientedCirclePoints(
            //        lineCenters.at(i), tangent, lastLineNormal, vertexPositions, vertexNormals);
            lineNormals.push_back(glm::vec3(lastLineNormal.x, lastLineNormal.y, lastLineNormal.z));
            for (int j = 0; j < numCircleSubdivisions; j++) {
                vertexTangents.push_back(tangent);
                vertexAttributes.push_back(lineAttributes.at(i));
            }
            numValidLinePoints++;
        }

        if (numValidLinePoints == 1) {
            // Only one vertex left -> Output nothing (tube consisting only of one point).
            vertexPositions.pop_back();
            vertexNormals.pop_back();
            vertexTangents.pop_back();
            vertexAttributes.pop_back();
            continue;
        }

        for (int i = 0; i < numValidLinePoints-1; i++) {
            for (int j = 0; j < numCircleSubdivisions; j++) {
                // Build two CCW triangles (one quad) for each side
                // Triangle 1
                triangleIndices.push_back(indexOffset + i*numCircleSubdivisions+j);
                triangleIndices.push_back(indexOffset + i*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
                triangleIndices.push_back(indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+(j+1)%numCircleSubdivisions);

                // Triangle 2
                triangleIndices.push_back(indexOffset + i*numCircleSubdivisions+j);
                triangleIndices.push_back(indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
                triangleIndices.push_back(indexOffset + ((i+1)%numValidLinePoints)*numCircleSubdivisions+j);
            }
        }

        if (tubeClosed) {
            /*
             * The tube is supposed to be closed. However, as we iteratively construct an artificial normal for
             * each line point perpendicular to the approximated line tangent, the normals at the begin and the
             * end of the tube do not match (i.e. the normal is not continuous).
             * Thus, the idea is to connect the begin and the end of the tube in such a way that the length of
             * the connecting edges is minimized. This is done by computing the angle between the two line
             * normals and shifting the edge indices by a necessary offset.
             */
            glm::vec3 normalA = lineNormals[numValidLinePoints-1];
            glm::vec3 normalB = lineNormals[0];
            float normalAngleDifference = std::atan2(
                    glm::length(glm::cross(normalA, normalB)), glm::dot(normalA, normalB));
            normalAngleDifference = std::fmod(normalAngleDifference + sgl::TWO_PI, sgl::TWO_PI);
            int jOffset = int(std::round(normalAngleDifference / (sgl::TWO_PI) * float(numCircleSubdivisions)));
            for (int j = 0; j < numCircleSubdivisions; j++) {
                // Build two CCW triangles (one quad) for each side
                // Triangle 1
                triangleIndices.push_back(indexOffset + (numValidLinePoints-1)*numCircleSubdivisions+(j)%numCircleSubdivisions);
                triangleIndices.push_back(indexOffset + (numValidLinePoints-1)*numCircleSubdivisions+(j+1)%numCircleSubdivisions);
                triangleIndices.push_back(indexOffset + 0*numCircleSubdivisions+(j+1+jOffset)%numCircleSubdivisions);

                // Triangle 2
                triangleIndices.push_back(indexOffset + (numValidLinePoints-1)*numCircleSubdivisions+(j)%numCircleSubdivisions);
                triangleIndices.push_back(indexOffset + 0*numCircleSubdivisions+(j+1+jOffset)%numCircleSubdivisions);
                triangleIndices.push_back(indexOffset + 0*numCircleSubdivisions+(j+jOffset)%numCircleSubdivisions);
            }
        } else {
            /*
             * If the tube is open, close it with two hemisphere caps at the ends.
             */
            int numLongitudeSubdivisions = numCircleSubdivisions; // azimuth
            int numLatitudeSubdivisions = std::ceil(numCircleSubdivisions/2); // zenith

            // Hemisphere at the start
            glm::vec3 center0 = lineCenters[0];
            glm::vec3 tangent0 = lineCenters[0] - lineCenters[1];
            tangent0 = glm::normalize(tangent0);
            glm::vec3 normal0 = lineNormals[0];

            // Hemisphere at the end
            glm::vec3 center1 = lineCenters[n-1];
            glm::vec3 tangent1 = lineCenters[n-1] - lineCenters[n-2];
            tangent1 = glm::normalize(tangent1);
            glm::vec3 normal1 = lineNormals[numValidLinePoints-1];


            addHemisphereToMesh(
                    center1, tangent1, normal1, indexOffset, tubeRadius,
                    numLongitudeSubdivisions, numLatitudeSubdivisions, false,
                    triangleIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);
            addHemisphereToMesh(
                    center0, tangent0, normal0, indexOffset, tubeRadius,
                    numLongitudeSubdivisions, numLatitudeSubdivisions, true,
                    triangleIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);
        }
    }
}

template
void createCappedTriangleTubesRenderDataCPU<float>(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<float>>& lineAttributesList,
        float tubeRadius,
        bool tubeClosed,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<float>& vertexAttributes);
