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
#include "Tubes.hpp"

float globalTubeRadius = 0.0f;
std::vector<glm::vec3> globalCircleVertexPositions;

void initGlobalCircleVertexPositions(int numCircleSubdivisions, float tubeRadius) {
    globalCircleVertexPositions.clear();
    globalTubeRadius = tubeRadius;

    const float theta = sgl::TWO_PI / numCircleSubdivisions;
    const float tangentialFactor = std::tan(theta); // opposite / adjacent
    const float radialFactor = std::cos(theta); // adjacent / hypotenuse
    glm::vec3 position(tubeRadius, 0, 0);

    for (int i = 0; i < numCircleSubdivisions; i++) {
        globalCircleVertexPositions.push_back(position);

        // Add the tangent vector and correct the position using the radial factor.
        glm::vec3 tangent(-position.y, position.x, 0);
        position += tangentialFactor * tangent;
        position *= radialFactor;
    }
}

void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& tangent, glm::vec3& lastNormal, uint32_t vertexLinePointIndex,
        std::vector<TubeTriangleVertexData>& vertexDataList) {
    glm::vec3 helperAxis = lastNormal;
    if (glm::length(glm::cross(helperAxis, tangent)) < 0.01f) {
        // If tangent == lastNormal
        helperAxis = glm::vec3(0.0f, 1.0f, 0.0f);
        if (glm::length(glm::cross(helperAxis, tangent)) < 0.01f) {
            // If tangent == helperAxis
            helperAxis = glm::vec3(0.0f, 0.0f, 1.0f);
        }
    }
    glm::vec3 normal = glm::normalize(helperAxis - glm::dot(helperAxis, tangent) * tangent); // Gram-Schmidt
    lastNormal = normal;
    glm::vec3 binormal = glm::cross(tangent, normal);

    for (size_t i = 0; i < globalCircleVertexPositions.size(); i++) {
        glm::vec3 pt = globalCircleVertexPositions.at(i);
        glm::vec3 transformedPoint(
                pt.x * normal.x + pt.y * binormal.x + pt.z * tangent.x + center.x,
                pt.x * normal.y + pt.y * binormal.y + pt.z * tangent.y + center.y,
                pt.x * normal.z + pt.y * binormal.z + pt.z * tangent.z + center.z
        );
        glm::vec3 vertexNormal = glm::normalize(transformedPoint - center);

        TubeTriangleVertexData tubeTriangleVertexData{};
        tubeTriangleVertexData.vertexPosition = transformedPoint;
        tubeTriangleVertexData.vertexLinePointIndex = vertexLinePointIndex;
        tubeTriangleVertexData.vertexNormal = vertexNormal;
        tubeTriangleVertexData.phi = float(i) / float(globalCircleVertexPositions.size()) * sgl::TWO_PI;
        vertexDataList.push_back(tubeTriangleVertexData);
    }
}

void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& tangent, glm::vec3& lastNormal,
        std::vector<glm::vec3>& vertexPositions) {
    glm::vec3 helperAxis = lastNormal;
    if (glm::length(glm::cross(helperAxis, tangent)) < 0.01f) {
        // If tangent == lastNormal
        helperAxis = glm::vec3(0.0f, 1.0f, 0.0f);
        if (glm::length(glm::cross(helperAxis, tangent)) < 0.01f) {
            // If tangent == helperAxis
            helperAxis = glm::vec3(0.0f, 0.0f, 1.0f);
        }
    }
    glm::vec3 normal = glm::normalize(helperAxis - glm::dot(helperAxis, tangent) * tangent); // Gram-Schmidt
    lastNormal = normal;
    glm::vec3 binormal = glm::cross(tangent, normal);

    for (size_t i = 0; i < globalCircleVertexPositions.size(); i++) {
        glm::vec3 pt = globalCircleVertexPositions.at(i);
        glm::vec3 transformedPoint(
                pt.x * normal.x + pt.y * binormal.x + pt.z * tangent.x + center.x,
                pt.x * normal.y + pt.y * binormal.y + pt.z * tangent.y + center.y,
                pt.x * normal.z + pt.y * binormal.z + pt.z * tangent.z + center.z
        );
        vertexPositions.push_back(transformedPoint);
    }
}


float globalTubeNormalRadius = 0.0f;
float globalTubeBinormalRadius = 0.0f;
std::vector<glm::vec3> globalEllipseVertexPositions;
std::vector<glm::vec3> globalEllipseVertexNormals;

void initGlobalEllipseVertexPositions(int numCircleSubdivisions, float tubeNormalRadius, float tubeBinormalRadius) {
    globalEllipseVertexPositions.clear();
    globalTubeNormalRadius = tubeNormalRadius;
    globalTubeBinormalRadius = tubeBinormalRadius;

    for (int i = 0; i < numCircleSubdivisions; i++) {
        float t = float(i) / float(numCircleSubdivisions) * 2.0f * float(M_PI);
        float cosAngle = std::cos(t);
        float sinAngle = std::sin(t);
        glm::vec3 localPosition = glm::vec3(tubeNormalRadius * cosAngle, tubeBinormalRadius * sinAngle, 0.0f);
        glm::vec3 localNormal = glm::normalize(glm::vec3(
                tubeBinormalRadius * cosAngle, tubeNormalRadius * sinAngle, 0.0f));

        globalEllipseVertexPositions.emplace_back(localPosition);
        globalEllipseVertexNormals.push_back(localNormal);
    }
}

void insertOrientedEllipsePoints(
        const glm::vec3& center, const glm::vec3& tangent, glm::vec3& normal, uint32_t vertexLinePointIndex,
        std::vector<TubeTriangleVertexData>& vertexDataList) {
    glm::vec3 binormal = glm::cross(tangent, normal);

    for (size_t i = 0; i < globalEllipseVertexPositions.size(); i++) {
        glm::vec3 pt = globalEllipseVertexPositions.at(i);
        glm::vec3 ptNormal = globalEllipseVertexNormals.at(i);
        glm::vec3 transformedPoint(
                pt.x * normal.x + pt.y * binormal.x + pt.z * tangent.x + center.x,
                pt.x * normal.y + pt.y * binormal.y + pt.z * tangent.y + center.y,
                pt.x * normal.z + pt.y * binormal.z + pt.z * tangent.z + center.z
        );
        glm::vec3 transformedNormal(
                ptNormal.x * normal.x + ptNormal.y * binormal.x + ptNormal.z * tangent.x,
                ptNormal.x * normal.y + ptNormal.y * binormal.y + ptNormal.z * tangent.y,
                ptNormal.x * normal.z + ptNormal.y * binormal.z + ptNormal.z * tangent.z
        );

        TubeTriangleVertexData tubeTriangleVertexData{};
        tubeTriangleVertexData.vertexPosition = transformedPoint;
        tubeTriangleVertexData.vertexLinePointIndex = vertexLinePointIndex;
        tubeTriangleVertexData.vertexNormal = transformedNormal;
        tubeTriangleVertexData.phi = float(i) / float(globalCircleVertexPositions.size()) * sgl::TWO_PI;
        vertexDataList.push_back(tubeTriangleVertexData);
    }
}
