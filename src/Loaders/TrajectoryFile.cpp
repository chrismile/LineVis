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

#define _FILE_OFFSET_BITS 64

#include <cstdio>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <Utils/File/Logfile.hpp>
#include <Math/Geometry/AABB3.hpp>
#include <Utils/Events/Stream/Stream.hpp>
#include "NetCdfConverter.hpp"
#include "StressTrajectoriesDatLoader.hpp"
#include "TrajectoryFile.hpp"
#include <iostream>

sgl::AABB3 computeTrajectoriesAABB3(const Trajectories& trajectories) {
    sgl::AABB3 aabb;
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
    #pragma omp parallel for shared(trajectories) default(none)\
    reduction(min: minX) reduction(min: minY) reduction(min: minZ) reduction(max: maxX) reduction(max: maxY) reduction(max: maxZ)
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        const Trajectory& trajectory = trajectories.at(trajectoryIdx);
        for (const glm::vec3& pt : trajectory.positions) {
            minX = std::min(minX, pt.x);
            minY = std::min(minY, pt.y);
            minZ = std::min(minZ, pt.z);
            maxX = std::max(maxX, pt.x);
            maxY = std::max(maxY, pt.y);
            maxZ = std::max(maxZ, pt.z);
        }
    }
    aabb.min = glm::vec3(minX, minY, minZ);
    aabb.max = glm::vec3(maxX, maxY, maxZ);
    return aabb;
}

void normalizeTrajectoriesVertexPositions(
        Trajectories& trajectories, const sgl::AABB3& aabb, const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    #pragma omp parallel for shared(trajectories, scale, translation) default(none)
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        for (glm::vec3& v : trajectory.positions) {
            v = (v + translation) * scale;
        }
    }

    if (vertexTransformationMatrixPtr != nullptr) {
        glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

        #pragma omp parallel for shared(trajectories, transformationMatrix) default(none)
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            for (glm::vec3& v : trajectory.positions) {
                v = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
            }
        }
    }
}

void normalizeTrajectoriesVertexPositions(Trajectories& trajectories, const glm::mat4* vertexTransformationMatrixPtr) {
    sgl::AABB3 aabb = computeTrajectoriesAABB3(trajectories);
    normalizeTrajectoriesVertexPositions(trajectories, aabb, vertexTransformationMatrixPtr);
}

void normalizeVertexPositions(
        std::vector<glm::vec3>& vertexPositions, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    #pragma omp parallel for shared(vertexPositions, translation, scale) default(none)
    for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
        glm::vec3& v = vertexPositions.at(vertexIdx);
        v = (v + translation) * scale;
    }

    if (vertexTransformationMatrixPtr != nullptr) {
        glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

        #pragma omp parallel for shared(vertexPositions, transformationMatrix) default(none)
        for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
            glm::vec3& v = vertexPositions.at(vertexIdx);
            v = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
        }
    }
}

void normalizeTrajectoriesVertexAttributes(Trajectories& trajectories) {
    const size_t numAttributes = trajectories.empty() ? 0 : trajectories.front().attributes.size();

    for (size_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
        float minVal = FLT_MAX, maxVal = -FLT_MAX;
        #pragma omp parallel for shared(trajectories, attributeIdx) default(none) reduction(min: minVal) reduction(max: maxVal)
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
            for (const float& attrVal : attributes) {
                minVal = std::min(minVal, attrVal);
                maxVal = std::max(maxVal, attrVal);
            }
        }

        #pragma omp parallel for shared(trajectories, attributeIdx, minVal, maxVal) default(none)
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
            for (float& attrVal : attributes) {
                attrVal = (attrVal - minVal) / (maxVal - minVal);
            }
        }
    }
}



sgl::AABB3 computeTrajectoriesPsAABB3(const std::vector<Trajectories>& trajectoriesPs) {
    sgl::AABB3 aabb;
    for (const Trajectories& trajectories : trajectoriesPs) {
        aabb.combine(computeTrajectoriesAABB3(trajectories));
    }
    return aabb;
}

void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    for (Trajectories& trajectories : trajectoriesPs) {
        #pragma omp parallel for shared(trajectories, scale, translation) default(none)
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            for (glm::vec3& v : trajectory.positions) {
                v = (v + translation) * scale;
            }
        }

        if (vertexTransformationMatrixPtr != nullptr) {
            glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

            #pragma omp parallel for shared(trajectories, transformationMatrix) default(none)
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                for (glm::vec3& v : trajectory.positions) {
                    v = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                }
            }
        }
    }
}

void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListRightPs,
        const sgl::AABB3& aabb, const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        Trajectories& trajectories = trajectoriesPs.at(psIdx);
        std::vector<std::vector<glm::vec3>>& bandPointsListLeft = bandPointsListLeftPs.at(psIdx);
        std::vector<std::vector<glm::vec3>>& bandPointsListRight = bandPointsListRightPs.at(psIdx);

        #pragma omp parallel for shared(trajectories, bandPointsListLeft, bandPointsListRight, scale, translation) default(none)
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            for (glm::vec3& v : trajectory.positions) {
                v = (v + translation) * scale;
            }

            std::vector<glm::vec3>& bandPointsLeft = bandPointsListLeft.at(trajectoryIdx);
            for (glm::vec3& v : bandPointsLeft) {
                v = (v + translation) * scale;
            }

            std::vector<glm::vec3>& bandPointsRight = bandPointsListRight.at(trajectoryIdx);
            for (glm::vec3& v : bandPointsRight) {
                v = (v + translation) * scale;
            }
        }

        if (vertexTransformationMatrixPtr != nullptr) {
            glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

            #pragma omp parallel for shared(trajectories, bandPointsListLeft, bandPointsListRight, transformationMatrix) default(none)
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                for (glm::vec3& v : trajectory.positions) {
                    v = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                }

                std::vector<glm::vec3>& bandPointsLeft = bandPointsListLeft.at(trajectoryIdx);
                for (glm::vec3& v : bandPointsLeft) {
                    v = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                }

                std::vector<glm::vec3>& bandPointsRight = bandPointsListRight.at(trajectoryIdx);
                for (glm::vec3& v : bandPointsRight) {
                    v = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                }
            }
        }
    }
}


void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const glm::mat4* vertexTransformationMatrixPtr) {
    sgl::AABB3 aabb = computeTrajectoriesPsAABB3(trajectoriesPs);
    normalizeTrajectoriesPsVertexPositions(trajectoriesPs, aabb, vertexTransformationMatrixPtr);
}

void normalizeTrajectoriesPsVertexAttributes_Total(std::vector<Trajectories>& trajectoriesPs) {
    size_t numAttributes = 0;
    if (!trajectoriesPs.empty() && !trajectoriesPs.front().empty()) {
        numAttributes = trajectoriesPs.front().front().attributes.size();
    }

    for (size_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
        float minVal = FLT_MAX, maxVal = -FLT_MAX;
        for (Trajectories& trajectories : trajectoriesPs) {
            #pragma omp parallel for shared(trajectories, attributeIdx) default(none) reduction(min: minVal) reduction(max: maxVal)
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (const float& attrVal : attributes) {
                    minVal = std::min(minVal, attrVal);
                    maxVal = std::max(maxVal, attrVal);
                }
            }
        }
        for (Trajectories& trajectories : trajectoriesPs) {
            #pragma omp parallel for shared(trajectories, attributeIdx, minVal, maxVal) default(none)
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (float& attrVal : attributes) {
                    attrVal = (attrVal - minVal) / (maxVal - minVal);
                }
            }
        }
    }
}

void normalizeTrajectoriesPsVertexAttributes_PerPs(std::vector<Trajectories>& trajectoriesPs) {
    size_t numAttributes = 0;
    if (!trajectoriesPs.empty() && !trajectoriesPs.front().empty()) {
        numAttributes = trajectoriesPs.front().front().attributes.size();
    }

    for (size_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
        for (Trajectories& trajectories : trajectoriesPs) {
            float minVal = FLT_MAX, maxVal = -FLT_MAX;
            #pragma omp parallel for shared(trajectories, attributeIdx) default(none) reduction(min: minVal) reduction(max: maxVal)
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (const float& attrVal : attributes) {
                    minVal = std::min(minVal, attrVal);
                    maxVal = std::max(maxVal, attrVal);
                }
            }

            #pragma omp parallel for shared(trajectories, attributeIdx, minVal, maxVal) default(none)
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (float& attrVal : attributes) {
                    attrVal = (attrVal - minVal) / (maxVal - minVal);
                }
            }
        }
    }
}



Trajectories loadFlowTrajectoriesFromFile(
        const std::string& filename, std::vector<std::string>& attributeNames,
        bool normalizeVertexPositions, bool normalizeAttributes, const glm::mat4* vertexTransformationMatrixPtr) {
    Trajectories trajectories;

    std::string lowerCaseFilename = boost::to_lower_copy(filename);
    if (boost::ends_with(lowerCaseFilename, ".obj")) {
        trajectories = loadTrajectoriesFromObj(filename, attributeNames);
    } else if (boost::ends_with(lowerCaseFilename, ".nc")) {
        trajectories = loadTrajectoriesFromNetCdf(filename);
    } else if (boost::ends_with(lowerCaseFilename, ".binlines")) {
        trajectories = loadTrajectoriesFromBinLines(filename);
    } else {
        sgl::Logfile::get()->writeError("ERROR in loadFlowTrajectoriesFromFile: Unknown file extension.");
    }

    if (normalizeVertexPositions) {
        normalizeTrajectoriesVertexPositions(trajectories, vertexTransformationMatrixPtr);
    }
    if (normalizeAttributes) {
        normalizeTrajectoriesVertexAttributes(trajectories);
    }

    return trajectories;
}

void loadStressTrajectoriesFromFile(
        const std::vector<std::string>& filenamesTrajectories,
        const std::vector<std::string>& filenamesHierarchy,
        std::vector<int>& loadedPsIndices,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListRightPs,
        bool containsBandData, bool normalizeVertexPositions, bool normalizeAttributes,
        sgl::AABB3* oldAABB, const glm::mat4* vertexTransformationMatrixPtr) {
    std::string lowerCaseFilename = boost::to_lower_copy(filenamesTrajectories.front());
    if (boost::ends_with(lowerCaseFilename, ".dat")) {
        if (containsBandData) {
            loadStressTrajectoriesFromDat_v2(
                    filenamesTrajectories, loadedPsIndices, trajectoriesPs, stressTrajectoriesDataPs,
                    bandPointsListLeftPs, bandPointsListRightPs);
        } else {
            loadStressTrajectoriesFromDat(
                    filenamesTrajectories, filenamesHierarchy, loadedPsIndices, trajectoriesPs,
                    stressTrajectoriesDataPs);
        }
    } else {
        sgl::Logfile::get()->writeError("ERROR in loadStressTrajectoriesFromFile: Unknown file extension.");
    }

    if (normalizeVertexPositions) {
        sgl::AABB3 aabb = computeTrajectoriesPsAABB3(trajectoriesPs);
        if (oldAABB) {
            *oldAABB = aabb;
        }
        if (containsBandData) {
            normalizeTrajectoriesPsVertexPositions(
                    trajectoriesPs, bandPointsListLeftPs, bandPointsListRightPs, aabb, vertexTransformationMatrixPtr);
        } else {
            normalizeTrajectoriesPsVertexPositions(trajectoriesPs, aabb, vertexTransformationMatrixPtr);
        }
    }
    if (normalizeAttributes) {
        normalizeTrajectoriesPsVertexAttributes_PerPs(trajectoriesPs);
    }
}

Trajectories loadTrajectoriesFromObj(const std::string& filename,  std::vector<std::string>& attributeNames) {
    Trajectories trajectories;

    std::vector<glm::vec3> globalLineVertices;
    std::vector<float> globalLineVertexAttributes;
    size_t numVertexAttributesGlobal = 0;

    FILE* file = fopen64(filename.c_str(), "r");
    if (!file) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in loadTrajectoriesFromObj: File \""
                + filename + "\" does not exist.");
        return trajectories;
    }
#if defined(_WIN32) && !defined(__MINGW32__)
    _fseeki64(file, 0, SEEK_END);
    size_t length = _ftelli64(file);
    _fseeki64(file, 0, SEEK_SET);
#else
    fseeko(file, 0, SEEK_END);
    size_t length = ftello(file);
    fseeko(file, 0, SEEK_SET);
#endif

    char* fileBuffer = new char[length];
    if (fileBuffer == nullptr) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in loadTrajectoriesFromObj: Couldn't reserve sufficient "
                + "memory for reading the file \"" + filename + "\".");
        return trajectories;
    }
    size_t result = fread(fileBuffer, 1, length, file);
    fclose(file);
    if (result != length) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in loadTrajectoriesFromObj: Invalid return value when "
                + "reading the file \"" + filename + "\".");
        delete[] fileBuffer;
        return trajectories;
    }

    std::string lineBuffer;
    std::string stringBuffer;

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

        if (lineBuffer.size() == 0) {
            continue;
        }

        char command = lineBuffer.at(0);
        char command2 = ' ';
        if (lineBuffer.size() > 1) {
            command2 = lineBuffer.at(1);
        }

        if (command == 'g') {
            // New path
            /*static int ctr = 0;
            if (ctr >= 999) {
                Logfile::get()->writeInfo(std::string() + "Parsing trajectory line group " + line.at(1) + "...");
            }
            ctr = (ctr + 1) % 1000;*/
        } else if (command == 'v' && command2 == 't') {
            // Path line vertex attribute
            //float attr = 0.0f;
            //sscanf(lineBuffer.c_str()+2, "%f", &attr);
            //globalLineVertexAttributes.push_back(attr);

            size_t numAttributes = 0;
            for (size_t linePtr = 2; linePtr < lineBuffer.size(); linePtr++) {
                char currentChar = lineBuffer.at(linePtr);
                bool isWhitespace = currentChar == ' ' || currentChar == '\t';
                if (isWhitespace && stringBuffer.size() != 0) {
                    globalLineVertexAttributes.push_back(atof(stringBuffer.c_str()));
                    numAttributes++;
                    stringBuffer.clear();
                } else if (!isWhitespace) {
                    stringBuffer.push_back(currentChar);
                }
            }
            if (stringBuffer.size() != 0) {
                globalLineVertexAttributes.push_back(atof(stringBuffer.c_str()));
                numAttributes++;
                stringBuffer.clear();
            }

            if (numVertexAttributesGlobal > 0 && numVertexAttributesGlobal != numAttributes) {
                sgl::Logfile::get()->writeError(
                        std::string() + "Error in loadTrajectoriesFromObj: Encountered inconsistent number of "
                        + "vertex attributes in file \"" + filename + "\".");
            }
            numVertexAttributesGlobal = numAttributes;
        } else if (command == 'v' && command2 == 'n') {
            // Not supported so far
        } else if (command == 'v') {
            // Path line vertex position
            glm::vec3 position;
            sscanf(lineBuffer.c_str()+2, "%f %f %f", &position.x, &position.y, &position.z);
            globalLineVertices.push_back(position);
        } else if (command == 'l') {
            // Get indices of current path line
            std::vector<uint32_t> currentLineIndices;
            for (size_t linePtr = 2; linePtr < lineBuffer.size(); linePtr++) {
                char currentChar = lineBuffer.at(linePtr);
                bool isWhitespace = currentChar == ' ' || currentChar == '\t';
                if (isWhitespace && stringBuffer.size() != 0) {
                    currentLineIndices.push_back(atoi(stringBuffer.c_str()) - 1);
                    stringBuffer.clear();
                } else if (!isWhitespace) {
                    stringBuffer.push_back(currentChar);
                }
            }
            if (stringBuffer.size() != 0) {
                currentLineIndices.push_back(atoi(stringBuffer.c_str()) - 1);
                stringBuffer.clear();
            }

            Trajectory trajectory;
            trajectory.positions.reserve(currentLineIndices.size());
            trajectory.attributes.resize(numVertexAttributesGlobal);
            for (size_t j = 0; j < numVertexAttributesGlobal; j++) {
                trajectory.attributes.at(j).reserve(currentLineIndices.size());
            }

            for (size_t i = 0; i < currentLineIndices.size(); i++) {
                glm::vec3 pos = globalLineVertices.at(currentLineIndices.at(i));

                // Remove invalid line points (used in many scientific datasets to indicate invalid lines).
                const float MAX_VAL = 1e10f;
                if (std::fabs(pos.x) > MAX_VAL || std::fabs(pos.y) > MAX_VAL || std::fabs(pos.z) > MAX_VAL) {
                    continue;
                }

                trajectory.positions.push_back(globalLineVertices.at(currentLineIndices.at(i)));
                for (size_t j = 0; j < numVertexAttributesGlobal; j++) {
                    trajectory.attributes.at(j).push_back(
                            globalLineVertexAttributes.at(currentLineIndices.at(i) * numVertexAttributesGlobal + j));
                }
            }

            trajectories.push_back(trajectory);
        }  else if (command == 'a') {
            if (attributeNames.empty()) {
                for (size_t linePtr = 2; linePtr < lineBuffer.size(); linePtr++) {
                    char currentChar = lineBuffer.at(linePtr);
                    bool isWhitespace = currentChar == ' ' || currentChar == '\t';
                    if (isWhitespace && stringBuffer.size() != 0) {
                        attributeNames.push_back(stringBuffer.c_str());
                        stringBuffer.clear();
                    } else if (!isWhitespace) {
                        stringBuffer.push_back(currentChar);
                    }
                }
                if (stringBuffer.size() != 0) {
                    attributeNames.push_back(stringBuffer.c_str());
                    stringBuffer.clear();
                }
            }
        } else if (command == '#') {
            // Ignore comments
        } else {
            //Logfile::get()->writeError(std::string() + "Error in parseObjMesh: Unknown command \"" + command + "\".");
        }

        lineBuffer.clear();
    }

    size_t geometryByteSize = 0;
    for (const auto& trajectory : trajectories) {
        geometryByteSize += trajectory.positions.size() * sizeof(float) * 3;
        for (const std::vector<float>& attributes : trajectory.attributes) {
            geometryByteSize += attributes.size() * sizeof(float);
        }
    }
    std::cout << "Size of line geometry data (MiB): " << (geometryByteSize / (1024.0 * 1024.0)) << std::endl;

    return trajectories;
}

Trajectories loadTrajectoriesFromNetCdf(const std::string& filename) {
    Trajectories trajectories = loadNetCdfFile(filename);
    return trajectories;
}

Trajectories loadTrajectoriesFromBinLines(const std::string& filename) {
    Trajectories trajectories;

    std::ifstream file(filename.c_str(), std::ifstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(std::string() + "Error in loadTrajectoriesFromBinLines: File \""
                + filename + "\" not found.");
        return trajectories;
    }

    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0);
    char *buffer = new char[size];
    file.read(buffer, size);

    // Read format version
    sgl::BinaryReadStream stream(buffer, size);
    const uint32_t LINE_FILE_FORMAT_VERSION = 1u;
    uint32_t versionNumber;
    stream.read(versionNumber);
    if (versionNumber != LINE_FILE_FORMAT_VERSION) {
        sgl::Logfile::get()->writeError(std::string()
                + "Error in loadTrajectoriesFromBinLines: Invalid magic number in file \"" + filename + "\".");
        return trajectories;
    }



    // Rest of header after format version
    uint32_t numTrajectories, numAttributes, trajectoryNumPoints;
    stream.read(numTrajectories);
    stream.read(numAttributes);
    trajectories.resize(numTrajectories);

    for (uint32_t trajectoryIndex = 0; trajectoryIndex < numTrajectories; trajectoryIndex++) {
        Trajectory &currentTrajectory = trajectories.at(trajectoryIndex);
        stream.read(trajectoryNumPoints);
        currentTrajectory.positions.resize(trajectoryNumPoints);
        stream.read((void*)&currentTrajectory.positions.front(), sizeof(glm::vec3)*trajectoryNumPoints);
        currentTrajectory.attributes.resize(numAttributes);
        for (uint32_t attributeIndex = 0; attributeIndex < numAttributes; attributeIndex++) {
            std::vector<float> &currentAttribute = currentTrajectory.attributes.at(attributeIndex);
            currentAttribute.resize(trajectoryNumPoints);
            stream.read((void*)&currentAttribute.front(), sizeof(float)*trajectoryNumPoints);
        }
    }

    return trajectories;
}
