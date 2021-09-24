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

#include <chrono>
#include <Math/Math.hpp>
#include <Utils/File/Logfile.hpp>
#include "LineData/SearchStructures/KdTree.hpp"
#include "LineData/SearchStructures/HashedGrid.hpp"
#include "IndexMesh.hpp"

void computeSharedIndexRepresentation(
        const std::vector<glm::vec3>& vertexPositions, const std::vector<glm::vec3>& vertexNormals,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositionsShared, std::vector<glm::vec3>& vertexNormalsShared) {
    SearchStructure* searchStructure = new HashedGrid(
            std::max(vertexPositions.size() / 4, size_t(1)), 1.0f / sgl::PI);

    std::vector<IndexedPoint> indexedPoints;
    std::vector<IndexedPoint*> indexedPointsPointers;
    indexedPoints.resize(vertexPositions.size());
    indexedPointsPointers.reserve(vertexPositions.size());
    for (size_t i = 0; i < vertexPositions.size(); i++) {
        IndexedPoint* indexedPoint = &indexedPoints.at(i);
        indexedPoint->index = ptrdiff_t(i);
        indexedPoint->position = vertexPositions.at(i);
        indexedPointsPointers.push_back(indexedPoint);
    }
    //searchStructure->build(indexedPointsPointers);
    searchStructure->reserveDynamic(vertexPositions.size());

    const float EPSILON = 1e-5f;
    size_t uniqueVertexCounter = 0;
    for (size_t i = 0; i < vertexPositions.size(); i++) {
        const glm::vec3& vertexPosition = vertexPositions.at(i);
        const glm::vec3& vertexNormal = vertexNormals.at(i);
        IndexedPoint* closestPoint = searchStructure->findClosestPoint(vertexPosition, EPSILON);
        if (closestPoint) {
            triangleIndices.push_back(closestPoint->index);
        } else {
            IndexedPoint* indexedPoint = indexedPointsPointers.at(uniqueVertexCounter);
            indexedPoint->index = ptrdiff_t(uniqueVertexCounter);
            indexedPoint->position = vertexPosition;
            searchStructure->addPoint(indexedPoint);
            vertexPositionsShared.push_back(vertexPosition);
            vertexNormalsShared.push_back(vertexNormal);
            triangleIndices.push_back(uint32_t(uniqueVertexCounter));
            uniqueVertexCounter++;
        }
    }

    delete searchStructure;
}

void computeSharedIndexRepresentation(
        const std::vector<glm::vec3>& vertexPositions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositionsShared) {
    SearchStructure* searchStructure = new KdTree;

    std::vector<IndexedPoint> indexedPoints;
    std::vector<IndexedPoint*> indexedPointsPointers;
    indexedPoints.resize(vertexPositions.size());
    indexedPointsPointers.reserve(vertexPositions.size());
    for (size_t i = 0; i < vertexPositions.size(); i++) {
        IndexedPoint* indexedPoint = &indexedPoints.at(i);
        indexedPoint->index = ptrdiff_t(i);
        indexedPoint->position = vertexPositions.at(i);
        indexedPointsPointers.push_back(indexedPoint);
    }
    //searchStructure->build(indexedPointsPointers);

    const float EPSILON = 1e-5f;
    size_t uniqueVertexCounter = 0;
    for (size_t i = 0; i < vertexPositions.size(); i++) {
        const glm::vec3& vertexPosition = vertexPositions.at(i);
        IndexedPoint* closestPoint = searchStructure->findClosestPoint(vertexPosition, EPSILON);
        if (closestPoint) {
            triangleIndices.push_back(closestPoint->index);
        } else {
            IndexedPoint* indexedPoint = indexedPointsPointers.at(uniqueVertexCounter);
            indexedPoint->index = ptrdiff_t(uniqueVertexCounter);
            indexedPoint->position = vertexPosition;
            searchStructure->addPoint(indexedPoint);
            vertexPositionsShared.push_back(vertexPosition);
            triangleIndices.push_back(uint32_t(uniqueVertexCounter));
            uniqueVertexCounter++;
        }
    }

    delete searchStructure;
}
