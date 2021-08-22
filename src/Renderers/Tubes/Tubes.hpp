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
#include "../../LineData/LineRenderData.hpp"

class HexMesh;
typedef std::shared_ptr<HexMesh> HexMeshPtr;

void createTriangleTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<TubeTriangleVertexData>& vertexDataList,
        std::vector<LinePointReference>& linePointReferenceList,
        uint32_t linePointOffset,
        std::vector<glm::vec3>& lineTangents,
        std::vector<glm::vec3>& lineNormals);

void createTriangleEllipticTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<glm::vec3>>& lineNormalsList,
        float tubeNormalRadius,
        float tubeBinormalRadius,
        int numEllipseSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<TubeTriangleVertexData>& vertexDataList,
        std::vector<LinePointReference>& linePointReferenceList,
        uint32_t linePointOffset,
        std::vector<glm::vec3>& lineTangents,
        std::vector<glm::vec3>& lineNormals);

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

void createCappedTriangleTubesRenderDataCPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        float tubeRadius, int numCircleSubdivisions, bool tubeClosed,
        std::vector<uint32_t>& triangleIndices,
        std::vector<TubeTriangleVertexData>& vertexDataList,
        std::vector<LinePointReference>& linePointReferenceList,
        uint32_t linePointOffset,
        std::vector<glm::vec3>& lineTangents,
        std::vector<glm::vec3>& lineNormals);

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
extern float globalTubeNormalRadius;
extern float globalTubeBinormalRadius;
extern std::vector<glm::vec3> globalEllipseVertexPositions;
extern std::vector<glm::vec3> globalEllipseVertexNormals;
/**
 * Computes the points lying on the specified circle and stores them in @see globalCircleVertexPositions.
 * @param numCircleSubdivisions The number of segments to use to approximate the circle.
 * @param tubeRadius The radius of the circle.
 */
extern void initGlobalCircleVertexPositions(int numCircleSubdivisions, float tubeRadius);
/**
 * Appends the vertex points of an oriented and shifted copy of a 2D circle in 3D space.
 * @param center The center of the circle in 3D space.
 * @param tangent The tangent of the curve, i.e., the normal orthogonal to the circle plane.
 * @param lastNormal The last normal of the curve, i.e., a vector lying in the plane of the circle.
 * @param vertexLinePointIndex The line point index corresponding to this tube circle.
 * @param vertexDataList The list to append the circle points and normals to.
 */
extern void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& tangent, glm::vec3& lastNormal, uint32_t vertexLinePointIndex,
        std::vector<TubeTriangleVertexData>& vertexDataList);
extern void insertOrientedCirclePoints(
        const glm::vec3& center, const glm::vec3& tangent, glm::vec3& lastNormal,
        std::vector<glm::vec3>& vertexPositions);

/**
 * Computes the points lying on the specified ellipse and stores them in @see globalCircleVertexPositions.
 * @param numCircleSubdivisions The number of segments to use to approximate the circle.
 * @param tubeNormalRadius The radius of the ellipse in the normal direction.
 * @param tubeBinormalRadius The radius of the ellipse in the binormal direction.
 */
extern void initGlobalEllipseVertexPositions(
        int numCircleSubdivisions, float tubeNormalRadius, float tubeBinormalRadius);
/**
 * Appends the vertex points of an oriented and shifted copy of a 2D circle in 3D space.
 * @param center The center of the circle in 3D space.
 * @param tangent The tangent of the curve, i.e., the normal orthogonal to the circle plane.
 * @param normal The normal of the curve, i.e., a vector lying in the plane of the circle.
 * @param vertexLinePointIndex The line point index corresponding to this tube circle.
 * @param vertexDataList The list to append the circle points and normals to.
 */
extern void insertOrientedEllipsePoints(
        const glm::vec3& center, const glm::vec3& tangent, glm::vec3& normal, uint32_t vertexLinePointIndex,
        std::vector<TubeTriangleVertexData>& vertexDataList);


/*
 * Template forward declarations, as code is in .cpp file.
 */

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
