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

#ifdef USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#endif

#include <Utils/File/Logfile.hpp>
#include <Utils/Parallel/Reduction.hpp>
#include <Utils/Mesh/TriangleNormals.hpp>

#include "../TrajectoryFile.hpp"
#include "Curvature.hpp"
#include "BinaryObjLoader.hpp"

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
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, vertexPositions.size()), [&](auto const& r) {
        for (size_t i = r.begin(); i != r.end(); i++) {
#else
#if _OPENMP >= 200805
    #pragma omp parallel for default(none) shared(vertexPositions)
#endif
    for (size_t i = 0; i < vertexPositions.size(); i++) {
#endif
        auto& vertexPosition = vertexPositions.at(i);
        float z = vertexPosition.z;
        vertexPosition.z = vertexPosition.y;
        vertexPosition.y = -z;
    }
#ifdef USE_TBB
    });
#endif

    // Check if we can safely convert the 64-bit indices to 32-bit indices.
    if (vertexPositions.size() / 3 > std::numeric_limits<uint32_t>::max()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in loadBinaryObjTriangleMesh: File \"" + filename
                + "\" has more than 2^32-1 vertices. Mesh splitting for larger meshes is currently not supported");
        return;
    }
    triangleIndices.resize(numTriangles * 3);
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, triangleIndices64.size()), [&](auto const& r) {
        for (size_t i = r.begin(); i != r.end(); i++) {
#else
#if _OPENMP >= 200805
    #pragma omp parallel for default(none) shared(triangleIndices, triangleIndices64)
#endif
    for (size_t i = 0; i < triangleIndices64.size(); i++) {
#endif
        triangleIndices.at(i) = static_cast<uint32_t>(triangleIndices64.at(i));
    }
#ifdef USE_TBB
    });
#endif
    triangleIndices64 = {};

    // Compute the vertex normals for the triangle mesh.
    sgl::computeSmoothTriangleNormals(triangleIndices, vertexPositions, vertexNormals);

    // Compute the mesh curvature as one attribute.
    vertexAttributeNames.emplace_back("Curvature");
    vertexAttributesList.resize(1);
    computeCurvature(triangleIndices, vertexPositions, vertexNormals, vertexAttributesList.at(0));

    if (shallNormalizeAttributes) {
        normalizeVertexAttributes(vertexAttributesList);
    }
    if (shallNormalizeVertexPositions) {
        sgl::AABB3 aabb = sgl::reduceVec3ArrayAabb(vertexPositions);
        if (oldAABB) {
            *oldAABB = aabb;
        }
        normalizeVertexPositions(vertexPositions, aabb, vertexTransformationMatrixPtr);
        normalizeVertexNormals(vertexNormals, aabb, vertexTransformationMatrixPtr);
    }
}
