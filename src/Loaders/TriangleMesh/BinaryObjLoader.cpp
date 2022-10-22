/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#include <fstream>

#include <Utils/File/Logfile.hpp>

#include "../TrajectoryFile.hpp"
#include "BinaryObjLoader.hpp"

void computeNormals(
        const std::vector<uint32_t>& triangleIndices, const std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals) {
    // TODO
    vertexNormals.resize(vertexPositions.size());
    for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
        vertexNormals.at(vertexIdx) = glm::vec3(1.0f);
    }
}

void computeCurvature(
        const std::vector<uint32_t>& triangleIndices, const std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes) {
    // TODO
    vertexAttributes.resize(vertexPositions.size());
    for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
        vertexAttributes.at(vertexIdx) = 0.0f;
    }
}

void loadBinaryObjTriangleMesh(
        const std::string &filename, std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals,
        std::vector<std::vector<float>>& vertexAttributesList, std::vector<std::string>& vertexAttributeNames,
        bool shallNormalizeVertexPositions, bool shallNormalizeAttributes,
        sgl::AABB3* oldAABB, const glm::mat4* vertexTransformationMatrixPtr) {
    std::ifstream fileObj(filename.c_str(), std::ios::binary);
    if (!fileObj.is_open()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in loadBinaryObjTriangleMesh: File \""
                + filename + "\" does not exist.");
        return;
    }

    // Read the header.
    uint64_t numVertices = 0, numTriangles = 0;
    fileObj.read(reinterpret_cast<char*>(&numVertices), sizeof(uint64_t));
    fileObj.read(reinterpret_cast<char*>(&numTriangles), sizeof(uint64_t));

    // Read the vertex and index data.
    std::vector<uint64_t> triangleIndices64;
    vertexPositions.resize(numVertices);
    triangleIndices64.resize(numTriangles * 3);
    fileObj.read(reinterpret_cast<char*>(
            vertexPositions.data()), std::streamsize(sizeof(glm::vec3) * numVertices));
    fileObj.read(reinterpret_cast<char*>(
            triangleIndices64.data()), std::streamsize(sizeof(uint64_t) * 3 * numTriangles));

    // Interchange the y and z axis and mirror the y axis.
#if _OPENMP >= 200805
    #pragma omp parallel for default(none) shared(vertexPositions)
#endif
    for (size_t i = 0; i < vertexPositions.size(); i++) {
        auto& vertexPosition = vertexPositions.at(i);
        float z = vertexPosition.z;
        vertexPosition.z = vertexPosition.y;
        vertexPosition.y = -z;
    }

    // Check if we can safely convert the 64-bit indices to 32-bit indices.
    if (vertexPositions.size() / 3 > std::numeric_limits<uint32_t>::max()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in loadBinaryObjTriangleMesh: File \"" + filename
                + "\" has more than 2^32-1 vertices. Mesh splitting for larger meshes is currently not supported");
        return;
    }
    triangleIndices.resize(numTriangles * 3);
#if _OPENMP >= 200805
    #pragma omp parallel for default(none) shared(triangleIndices, triangleIndices64)
#endif
    for (size_t i = 0; i < triangleIndices64.size(); i++) {
        triangleIndices.at(i) = static_cast<uint32_t>(triangleIndices64.at(i));
    }
    triangleIndices64 = {};

    // Compute the vertex normals for the triangle mesh.
    computeNormals(triangleIndices, vertexPositions, vertexNormals);

    // Compute the mesh curvature as one attribute.
    vertexAttributeNames.emplace_back("Curvature");
    vertexAttributesList.resize(1);
    computeCurvature(triangleIndices, vertexPositions, vertexNormals, vertexAttributesList.at(0));

    if (shallNormalizeAttributes) {
        normalizeVertexAttributes(vertexAttributesList);
    }
    if (shallNormalizeVertexPositions) {
        sgl::AABB3 aabb = computeVertexPositionsAABB3(vertexPositions);
        if (oldAABB) {
            *oldAABB = aabb;
        }
        normalizeVertexPositions(vertexPositions, aabb, vertexTransformationMatrixPtr);
    }
}
