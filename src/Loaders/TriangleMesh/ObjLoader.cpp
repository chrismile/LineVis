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

#include <cstdint>
#include <unordered_map>

#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Parallel/Reduction.hpp>

#include "../TrajectoryFile.hpp"
#include "Curvature.hpp"
#include "ObjLoader.hpp"

//#define FTB_CORE_IMPL
//#define FTB_MESH_IMPL
//#define FTB_MATH_IMPL
//#define FTB_NO_INIT_PRINTER
//#include "../../../third_party/ftb/core.hpp"
//#include "../../../third_party/ftb/mesh.hpp"


/**
 * Triangulates the face indices of the polygon in tempFaceIndices.
 * This code connects all polygon vertices to their center point.
 * Please note that this is fast, but may fail for some types of concave polygons!
 * @param tempFaceIndices The face indices to triangulate.
 * @param triangleIndices The global triangle index list.
 * @param vertexPositions The global vertex position list.
 * @param vertexNormals The global vertex normal list.
 */
void triangulatePathByCenter(
        const std::vector<uint32_t>& tempFaceIndices, std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals) {
    auto numPoints = int(tempFaceIndices.size());
    auto invN = 1.0f / float(numPoints);
    auto centerPosition = glm::vec3(0.0f);
    auto centerNormal = glm::vec3(0.0f);
    for (int i = 0; i < numPoints; i++) {
        const glm::vec3& vertexPosition = vertexPositions.at(tempFaceIndices.at(i));
        const glm::vec3& vertexNormal = vertexNormals.at(tempFaceIndices.at(i));
        centerPosition += vertexPosition;
        centerNormal += vertexNormal;
    }
    centerPosition *= invN;
    centerNormal = glm::normalize(centerNormal);
    auto centerPointIndex = uint32_t(vertexPositions.size());
    vertexPositions.push_back(centerPosition);
    vertexNormals.push_back(centerNormal);

    for (int i = 0; i < numPoints; i++) {
        triangleIndices.push_back(tempFaceIndices.at(i));
        triangleIndices.push_back(tempFaceIndices.at((i + 1) % numPoints));
        triangleIndices.push_back(centerPointIndex);
        const glm::vec3& vertexPosition = vertexPositions.at(tempFaceIndices.at(i));
        const glm::vec3& vertexNormal = vertexNormals.at(tempFaceIndices.at(i));
        centerPosition += vertexPosition;
        centerNormal += vertexNormal;
    }
}

void loadObjTriangleMesh(
        const std::string &filename, std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec3>& vertexNormals,
        std::vector<std::vector<float>>& vertexAttributesList, std::vector<std::string>& vertexAttributeNames,
        bool shallNormalizeVertexPositions, bool shallNormalizeAttributes,
        sgl::AABB3* oldAABB, const glm::mat4* vertexTransformationMatrixPtr) {

    // Mesh_Data md = load_obj(filename.c_str());

    // vertexPositions.resize(md.vertices.count);
    // vertexNormals.resize(md.vertices.count);
    // triangleIndices.resize(md.faces.count*3);
    // uint32_t v_idx=0;
    // for (auto& v : md.vertices) {
    //     glm::vec3 pos = {v.position.x, v.position.y, v.position.z};
    //     glm::vec3 nor = {v.normal.x, v.normal.y, v.normal.z};
    //     vertexPositions[v_idx] = pos;
    //     vertexNormals[v_idx] = nor;
    //     v_idx++;
    // }

    // memcpy(triangleIndices.data(), md.faces.data, sizeof(md.faces.data[0])*md.faces.count);


        std::vector<glm::vec3> objVertices;
    std::vector<glm::vec3> objNormals;
    std::vector<uint32_t> tempFaceIndices;

    uint8_t* buffer = nullptr;
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(filename, buffer, length, false);
    if (!loaded) {
        sgl::Logfile::get()->writeError("Error in loadObjTriangleMesh: Could not open file \"" + filename + "\".");
        return;
    }
    char* fileBuffer = reinterpret_cast<char*>(buffer);

    std::string lineBuffer;
    std::string stringBuffer;
    std::vector<std::string> faceLineParts;
    std::vector<int32_t> objIndicesSplit;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> objIndicesMap;

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

        char command = lineBuffer.at(0);
        char command2 = ' ';
        if (lineBuffer.size() > 1) {
            command2 = lineBuffer.at(1);
        }

        if (command == 'v' && command2 == 't') {
            // Not supported so far.
        } else if (command == 'v' && command2 == 'n') {
            glm::vec3 normal;
#ifdef _MSC_VER
            sscanf_s(lineBuffer.c_str() + 3, "%f %f %f", &normal.x, &normal.y, &normal.z);
#else
            sscanf(lineBuffer.c_str() + 3, "%f %f %f", &normal.x, &normal.y, &normal.z);
#endif
            objNormals.push_back(normal);
        } else if (command == 'v') {
            // Path line vertex position
            glm::vec3 position;
#ifdef _MSC_VER
            sscanf_s(lineBuffer.c_str() + 2, "%f %f %f", &position.x, &position.y, &position.z);
#else
            sscanf(lineBuffer.c_str() + 2, "%f %f %f", &position.x, &position.y, &position.z);
#endif
            objVertices.push_back(position);
        } else if (command == 'f') {
            faceLineParts.clear();
            sgl::splitStringWhitespace(lineBuffer.c_str() + 2, faceLineParts);
            if (faceLineParts.size() < 3) {
                sgl::Logfile::get()->writeError(
                        "Error in loadObjTriangleMesh: Invalid face statement in file \"" + filename + "\".");
            } else if (faceLineParts.size() < 5) {
                for (size_t i = 0; i < faceLineParts.size(); i++) {
                    objIndicesSplit.clear();
                    sgl::splitStringTyped<int32_t>(faceLineParts.at(i), '/', objIndicesSplit);
                    int32_t vidx = objIndicesSplit.front();
                    int32_t nidx = objIndicesSplit.back();
                    if (vidx > 0) {
                        vidx -= 1;
                    } else {
                        vidx = int(objVertices.size()) + vidx;
                    }
                    if (nidx > 0) {
                        nidx -= 1;
                    } else {
                        nidx = int(objNormals.size()) + nidx;
                    }
                    auto it = objIndicesMap.find({vidx, nidx});
                    if (it != objIndicesMap.end()) {
                        triangleIndices.push_back(it->second);
                    } else {
                        auto idx = uint32_t(vertexPositions.size());
                        objIndicesMap.insert({ {vidx, nidx}, idx });
                        triangleIndices.push_back(idx);
                        vertexPositions.push_back(objVertices.at(vidx));
                        vertexNormals.push_back(objNormals.at(nidx));
                    }
                }
                if (faceLineParts.size() == 4) {
                    // Triangulate.
                    triangleIndices.push_back(triangleIndices.at(triangleIndices.size() - 4));
                    triangleIndices.push_back(triangleIndices.at(triangleIndices.size() - 3));
                }
            } else {
                tempFaceIndices.clear();
                for (size_t i = 0; i < faceLineParts.size(); i++) {
                    objIndicesSplit.clear();
                    sgl::splitStringTyped<uint32_t>(faceLineParts.at(i), '/', objIndicesSplit);
                    uint32_t vidx = objIndicesSplit.front() - 1;
                    uint32_t nidx = objIndicesSplit.back() - 1;
                    auto it = objIndicesMap.find({vidx, nidx});
                    if (it != objIndicesMap.end()) {
                        tempFaceIndices.push_back(it->second);
                    } else {
                        auto idx = uint32_t(vertexPositions.size());
                        objIndicesMap.insert({ {vidx, nidx}, idx });
                        tempFaceIndices.push_back(idx);
                        vertexPositions.push_back(objVertices.at(vidx));
                        vertexNormals.push_back(objNormals.at(nidx));
                    }
                }
                triangulatePathByCenter(tempFaceIndices, triangleIndices, vertexPositions, vertexNormals);
            }
        } else if (command == 'o' || command == 'g') {
            // Ignore objects and groups.
        } else if (command == '#') {
            // Ignore comments.
        } else {
            //Logfile::get()->writeError(
            //        std::string() + "Error in loadObjTriangleMesh: Unknown command \"" + command + "\".");
        }

        lineBuffer.clear();
    }

    delete[] buffer;
    buffer = nullptr;

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
