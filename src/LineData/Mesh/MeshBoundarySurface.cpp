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

#include <set>
#include <map>
#include <unordered_map>
#include <algorithm>

#include <Utils/File/Logfile.hpp>

#include "VtkLoader.hpp"
#include "MeshLoader.hpp"
#include "MeshBoundarySurface.hpp"

struct FaceSlim {
    uint32_t vs[4]; ///< vertex indices
};
struct VertexSlim {
    std::vector<uint32_t> cs; ///< cell indices
};

void buildVerticesSlim(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices,
        std::vector<VertexSlim>& verticesSlim) {
    verticesSlim.resize(vertices.size());
    const uint32_t numCells = cellIndices.size() / 8;
    for (uint32_t c_id = 0; c_id < numCells; c_id++) {
        for (uint32_t v_internal_id = 0; v_internal_id < 8; v_internal_id++) {
            uint32_t v_id = cellIndices.at(c_id * 8 + v_internal_id);
            verticesSlim.at(v_id).cs.push_back(c_id);
        }
    }
}

const int hexFaceTable[6][4] = {
        // Use consistent winding for faces at the boundary (normals pointing out of the cell - no arbitrary decisions).
        { 0,1,2,3 },
        { 5,4,7,6 },
        { 4,5,1,0 },
        { 4,0,3,7 },
        { 6,7,3,2 },
        { 1,5,6,2 },
};

void buildFacesSlim(
        const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& cellIndices,
        std::vector<FaceSlim>& facesSlim, std::vector<bool>& isBoundaryFace) {
    struct TempFace {
        uint32_t vertexId[4];
        uint32_t faceId;

        inline bool operator==(const TempFace& other) const {
            for (int i = 0; i < 4; i++) {
                if (vertexId[i] != other.vertexId[i]) {
                    return false;
                }
            }
            return true;
        }

        inline bool operator!=(const TempFace& other) const {
            for (int i = 0; i < 4; i++) {
                if (vertexId[i] != other.vertexId[i]) {
                    return true;
                }
            }
            return false;
        }

        inline bool operator<(const TempFace& other) const {
            for (int i = 0; i < 4; i++) {
                if (vertexId[i] < other.vertexId[i]) {
                    return true;
                } else if (vertexId[i] > other.vertexId[i]) {
                    return false;
                }
            }
            return faceId < other.faceId;
        }
    };

    const uint32_t numCells = cellIndices.size() / 8;
    std::vector<FaceSlim> totalFaces(numCells * 6);
    std::vector<TempFace> tempFaces(numCells * 6);

    FaceSlim face;
    for (uint32_t cellId = 0; cellId < numCells; ++cellId) {
        for (uint32_t faceIdx = 0; faceIdx < 6; faceIdx++){
            for (uint32_t vertexIdx = 0; vertexIdx < 4; vertexIdx++) {
                face.vs[vertexIdx] = cellIndices.at(cellId * 8 + hexFaceTable[faceIdx][vertexIdx]);
            }

            uint32_t faceId = 6 * cellId + faceIdx;
            totalFaces[faceId] = face;
            std::sort(face.vs, face.vs + 4);
            tempFaces[faceId] = TempFace{
                    face.vs[0], face.vs[1], face.vs[2], face.vs[3], faceId
            };
        }
    }
    std::sort(tempFaces.begin(), tempFaces.end());

    facesSlim.reserve(tempFaces.size() / 3);
    uint32_t numFaces = 0;
    for (uint32_t i = 0; i < tempFaces.size(); ++i) {
        if (i == 0 || tempFaces[i] != tempFaces[i - 1]) {
            face = totalFaces[tempFaces[i].faceId];
            facesSlim.push_back(face);
            isBoundaryFace.push_back(true);
            numFaces++;
        } else {
            isBoundaryFace[numFaces - 1] = false;
        }
    }
}

void loadMeshBoundarySurfaceFromFile(
        const std::string& meshFilename,
        std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) {
    std::map<std::string, HexahedralMeshLoader*> meshLoaderMap;
    meshLoaderMap.insert(std::make_pair("vtk", new VtkLoader));
    meshLoaderMap.insert(std::make_pair("mesh", new MeshLoader));

    size_t extensionPos = meshFilename.find_last_of('.');
    if (extensionPos == std::string::npos) {
        sgl::Logfile::get()->writeError("Error: Mesh file name has no extension.");
        return;
    }
    std::string extension = meshFilename.substr(extensionPos + 1);
    auto it = meshLoaderMap.find(extension);
    if (it == meshLoaderMap.end()) {
        sgl::Logfile::get()->writeError(
                "Error in loadMeshBoundarySurfaceFromFile: Unknown extension: \"" + extension + "\".");
        return;
    }

    std::vector<glm::vec3> vertices;
    std::vector<uint32_t> cellIndices;
    std::vector<glm::vec3> deformations;
    std::vector<float> anisotropyMetricList;
    bool loadingSuccessful = it->second->loadHexahedralMeshFromFile(
            meshFilename, vertices, cellIndices, deformations, anisotropyMetricList);

    if (!loadingSuccessful) {
        sgl::Logfile::get()->writeError(
                "Error in loadMeshBoundarySurfaceFromFile: Couldn't load file \"" + meshFilename + "\".");
        return;
    }

    std::vector<VertexSlim> verticesSlim;
    std::vector<FaceSlim> facesSlim;
    std::vector<bool> facesBoundarySlim;

    buildVerticesSlim(vertices, cellIndices, verticesSlim);
    buildFacesSlim(vertices, cellIndices, facesSlim, facesBoundarySlim);

    std::set<uint32_t> usedVertexSet;
    std::unordered_map<uint32_t, uint32_t> vertexIndexMap;

    for (size_t f_id = 0; f_id < facesSlim.size(); f_id++) {
        FaceSlim &f = facesSlim.at(f_id);
        if (!facesBoundarySlim.at(f_id)) {
            continue;
        }
        for (size_t i = 0; i < 4; i++) {
            usedVertexSet.insert(f.vs[i]);
        }
    }

    // Add the used hex-mesh vertices to the triangle mesh vertex data.
    uint32_t ctr = 0;
    for (uint32_t v_id : usedVertexSet) {
        vertexIndexMap.insert(std::make_pair(v_id, ctr));
        vertexPositions.push_back(vertices.at(v_id));
        ctr++;
    }

    // Add the triangle indices.
    for (size_t f_id = 0; f_id < facesSlim.size(); f_id++) {
        FaceSlim& f = facesSlim.at(f_id);
        if (!facesBoundarySlim.at(f_id)) {
            continue;
        }

        triangleIndices.push_back(vertexIndexMap[f.vs[2]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[1]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[0]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[3]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[2]]);
        triangleIndices.push_back(vertexIndexMap[f.vs[0]]);
    }


    for (auto& it : meshLoaderMap) {
        delete it.second;
    }
    meshLoaderMap.clear();
}
