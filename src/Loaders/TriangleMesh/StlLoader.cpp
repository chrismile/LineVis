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

#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Parallel/Reduction.hpp>
#include <Utils/Events/Stream/Stream.hpp>
#include <Utils/Mesh/TriangleNormals.hpp>
#include <Utils/Mesh/IndexMesh.hpp>

#include "../TrajectoryFile.hpp"
#include "Curvature.hpp"
#include "StlLoader.hpp"

void loadBinaryStlTriangleMesh(
        uint8_t* buffer, size_t length,
        const std::string &filename, std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals,
        std::vector<std::vector<float>>& vertexAttributesList, std::vector<std::string>& vertexAttributeNames,
        bool shallNormalizeVertexPositions, bool shallNormalizeAttributes,
        sgl::AABB3* oldAABB, const glm::mat4* vertexTransformationMatrixPtr) {
    sgl::BinaryReadStream stream(buffer, length);

    // Skip the header.
    stream.skip(80);

    // Read the number of triangles.
    uint32_t numTriangles = 0;
    stream.read<uint32_t>(numTriangles);
    triangleIndices.resize(numTriangles * 3);

    // Read all facets.
    for (uint32_t i = 0; i < numTriangles; i++) {
        // Compute the facet normal (ignore stored normal data).
        glm::vec3 v0, v1, v2, facetNormal;
        stream.read(facetNormal);
        // Reverse winding order.
        stream.read(v2);
        stream.read(v1);
        stream.read(v0);

        triangleIndices.push_back(i * 3);
        triangleIndices.push_back(i * 3 + 1);
        triangleIndices.push_back(i * 3 + 2);
        vertexPositions.push_back(v0);
        vertexPositions.push_back(v1);
        vertexPositions.push_back(v2);
        vertexNormals.push_back(facetNormal);
        vertexNormals.push_back(facetNormal);
        vertexNormals.push_back(facetNormal);

        // Skip the attribute byte count.
        uint16_t attributeByteCount = 0;
        stream.read(attributeByteCount);
        if (attributeByteCount != 0) {
            sgl::Logfile::get()->throwError(
                    "Error in loadBinaryStlTriangleMesh: Attribute byte count is expected to be 0.");
        }
    }
}

void loadAsciiStlTriangleMesh(
        char* fileBuffer, size_t length,
        const std::string &filename, std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals,
        std::vector<std::vector<float>>& vertexAttributesList, std::vector<std::string>& vertexAttributeNames,
        bool shallNormalizeVertexPositions, bool shallNormalizeAttributes,
        sgl::AABB3* oldAABB, const glm::mat4* vertexTransformationMatrixPtr) {
    std::string lineBuffer;
    std::string stringBuffer;
    std::vector<std::string> faceLineParts;
    std::vector<uint32_t> objIndicesSplit;
    glm::vec3 currentVector;

    for (size_t charPtr = 0; charPtr < length; ) {
        while (charPtr < length) {
            char currentChar = fileBuffer[charPtr];
            if (currentChar == '\n' || currentChar == '\r') {
                charPtr++;
                break;
            }
            lineBuffer.push_back(currentChar);
            charPtr++;
        }

        if (lineBuffer.empty()) {
            continue;
        }

        if (sgl::startsWith(lineBuffer, "vertex")) {
#ifdef _MSC_VER
            sscanf_s(lineBuffer.c_str() + 7, "%f %f %f", &currentVector.x, &currentVector.y, &currentVector.z);
#else
            sscanf(lineBuffer.c_str() + 7, "%f %f %f", &currentVector.x, &currentVector.y, &currentVector.z);
#endif
            triangleIndices.push_back(uint32_t(vertexPositions.size()));
            vertexPositions.push_back(currentVector);
        }
        if (sgl::startsWith(lineBuffer, "facet normal")) {
#ifdef _MSC_VER
            sscanf_s(lineBuffer.c_str() + 13, "%f %f %f", &currentVector.x, &currentVector.y, &currentVector.z);
#else
            sscanf(lineBuffer.c_str() + 13, "%f %f %f", &currentVector.x, &currentVector.y, &currentVector.z);
#endif
            vertexNormals.push_back(currentVector);
            vertexNormals.push_back(currentVector);
            vertexNormals.push_back(currentVector);
        }

        lineBuffer.clear();
    }
}

void loadStlTriangleMesh(
        const std::string &filename, std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals,
        std::vector<std::vector<float>>& vertexAttributesList, std::vector<std::string>& vertexAttributeNames,
        bool shallNormalizeVertexPositions, bool shallNormalizeAttributes,
        bool shallComputeSharedVertexRepresentation,
        sgl::AABB3* oldAABB, const glm::mat4* vertexTransformationMatrixPtr) {
    uint8_t* buffer = nullptr;
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(filename, buffer, length, true);
    if (!loaded) {
        sgl::Logfile::get()->writeError("Error in loadStlTriangleMesh: Could not open file \"" + filename + "\".");
        return;
    }
    char* fileBuffer = reinterpret_cast<char*>(buffer);

    const char* headerString = "solid";
    if (length < strlen(headerString)) {
        sgl::Logfile::get()->writeError(
                "Error in loadStlTriangleMesh: Too short file length for file \"" + filename + "\".");
        return;
    }
    if (strcmp(fileBuffer, headerString) == 0) {
        loadAsciiStlTriangleMesh(
                fileBuffer, length,
                filename, triangleIndices, vertexPositions, vertexNormals, vertexAttributesList, vertexAttributeNames,
                shallNormalizeVertexPositions, shallNormalizeAttributes, oldAABB, vertexTransformationMatrixPtr);
        delete[] buffer;
        buffer = nullptr;
    } else {
        // BinaryReadStream does the deallocation of the buffer.
        loadBinaryStlTriangleMesh(
                buffer, length,
                filename, triangleIndices, vertexPositions, vertexNormals, vertexAttributesList, vertexAttributeNames,
                shallNormalizeVertexPositions, shallNormalizeAttributes, oldAABB, vertexTransformationMatrixPtr);
    }

    if (shallComputeSharedVertexRepresentation) {
        std::vector<glm::vec3> vertexPositionsNotShared = vertexPositions;
        triangleIndices.clear();
        vertexPositions.clear();
        vertexNormals.clear();
        std::vector<glm::vec3> vertexNormalsNotShared;
        sgl::computeSharedIndexRepresentation(vertexPositionsNotShared, triangleIndices, vertexPositions);
        sgl::computeSmoothTriangleNormals(triangleIndices, vertexPositions, vertexNormals);
    }

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
