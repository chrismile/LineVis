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

#ifndef HEXVOLUMERENDERER_TUBES_HPP
#define HEXVOLUMERENDERER_TUBES_HPP

#include <vector>
#include <memory>
#include <glm/glm.hpp>

class HexMesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

template<typename T>
void createTriangleTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<T>>& lineAttributesList,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<T>& vertexAttributes);

void createTriangleTubesRenderDataGPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<float>>& lineAttributesList,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<float>& vertexAttributes);

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
        std::vector<T>& vertexAttributes);

void createCappedTriangleTubesUnionRenderDataCPU(
        HexMeshPtr hexMesh,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<float>& vertexAttributes);

template<typename T>
void createLineTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<T>>& lineAttributesList,
        std::vector<uint32_t>& lineIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<T>& vertexAttributes);

template<typename T>
void createLineTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<T>>& lineAttributesList,
        std::vector<uint32_t>& lineIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<T>& vertexAttributes,
        std::vector<uint32_t>& validLineIndices,
        std::vector<uint32_t>& numValidLineVertices);


/**
 * Globally accessible circle data. Can be shared
 */
extern float globalTubeRadius;
extern std::vector<glm::vec3> globalCircleVertexPositions;
/**
 * Computes the points lying on the specified circle and stores them in @see globalCircleVertexPositions.
 * @param numCircleSubdivisions The number of segments to use to approximate the circle.
 * @param tubeRadius The radius of the circle.
 */
extern void initGlobalCircleVertexPositions(int numCircleSubdivisions, float tubeRadius);
/**
 * Appends the vertex points of an oriented and shifted copy of a 2D circle in 3D space.
 * @param vertices The list to append the circle points to.
 * @param normals The list to append the normal vectors circles to.
 * @param center The center of the circle in 3D space.
 * @param normal The normal orthogonal to the circle plane.
 * @param lastTangent The tangent of the last circle.
 */
extern void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& normal, glm::vec3& lastTangent,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals);
extern void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& normal, glm::vec3& lastTangent,
        std::vector<glm::vec3>& vertexPositions);


/*
 * Template forward declarations, as code is in .cpp file.
 */

extern template
void createTriangleTubesRenderDataCPU<float>(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<float>>& lineAttributesList,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<float>& vertexAttributes);

extern template
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

extern template
void createLineTubesRenderDataCPU<float>(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<float>>& lineAttributesList,
        std::vector<uint32_t>& lineIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<float>& vertexAttributes);

extern template
void createLineTubesRenderDataCPU<std::vector<float>>(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<std::vector<float>>>& lineAttributesList,
        std::vector<uint32_t>& lineIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<std::vector<float>>& vertexAttributes);

extern template
void createLineTubesRenderDataCPU<float>(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<float>>& lineAttributesList,
        std::vector<uint32_t>& lineIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<float>& vertexAttributes,
        std::vector<uint32_t>& validLineIndices,
        std::vector<uint32_t>& numValidLineVertices);

extern template
void createLineTubesRenderDataCPU<std::vector<float>>(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<std::vector<float>>>& lineAttributesList,
        std::vector<uint32_t>& lineIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<std::vector<float>>& vertexAttributes,
        std::vector<uint32_t>& validLineIndices,
        std::vector<uint32_t>& numValidLineVertices);

#endif //HEXVOLUMERENDERER_TUBES_HPP
