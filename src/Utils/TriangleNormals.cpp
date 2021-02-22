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

#include "TriangleNormals.hpp"

void computeSmoothTriangleNormals(
        const std::vector<uint32_t>& triangleIndices, const std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals) {
    vertexNormals.resize(vertexPositions.size(), glm::vec3(0.0f));

    for (size_t i = 0; i < triangleIndices.size(); i += 3) {
        uint32_t vertexIndex0 = triangleIndices.at(i);
        uint32_t vertexIndex1 = triangleIndices.at(i+1);
        uint32_t vertexIndex2 = triangleIndices.at(i+2);

        const glm::vec3& vertexPosition0 = vertexPositions.at(vertexIndex0);
        const glm::vec3& vertexPosition1 = vertexPositions.at(vertexIndex1);
        const glm::vec3& vertexPosition2 = vertexPositions.at(vertexIndex2);

        glm::vec3 triangleNormal = glm::cross(vertexPosition2 - vertexPosition0, vertexPosition1 - vertexPosition0);
        vertexNormals.at(vertexIndex0) += triangleNormal;
        vertexNormals.at(vertexIndex1) += triangleNormal;
        vertexNormals.at(vertexIndex2) += triangleNormal;
    }

    for (size_t i = 0; i < vertexNormals.size(); i++) {
        glm::vec3& vertexNormal = vertexNormals.at(i);
        vertexNormal = glm::normalize(vertexNormal);
    }
}
