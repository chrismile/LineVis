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

#include <iostream>

#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileLoader.hpp>

#include "ObjLoader.hpp"

Trajectories loadTrajectoriesFromObj(const std::string& filename, std::vector<std::string>& attributeNames) {
    Trajectories trajectories;

    std::vector<glm::vec3> globalLineVertices;
    std::vector<float> globalLineVertexAttributes;
    size_t numVertexAttributesGlobal = 0;

    uint8_t* buffer = nullptr;
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(filename, buffer, length, false);
    if (!loaded) {
        return trajectories;
    }
    char* fileBuffer = reinterpret_cast<char*>(buffer);

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

        if (lineBuffer.empty()) {
            continue;
        }

        char command = lineBuffer.at(0);
        char command2 = ' ';
        if (lineBuffer.size() > 1) {
            command2 = lineBuffer.at(1);
        }

        if (command == 'g') {
            // New path
        } else if (command == 'v' && command2 == 't') {
            size_t numAttributes = 0;
            for (size_t linePtr = 2; linePtr < lineBuffer.size(); linePtr++) {
                char currentChar = lineBuffer.at(linePtr);
                bool isWhitespace = currentChar == ' ' || currentChar == '\t';
                if (isWhitespace && !stringBuffer.empty()) {
                    globalLineVertexAttributes.push_back(float(std::atof(stringBuffer.c_str())));
                    numAttributes++;
                    stringBuffer.clear();
                } else if (!isWhitespace) {
                    stringBuffer.push_back(currentChar);
                }
            }
            if (!stringBuffer.empty()) {
                globalLineVertexAttributes.push_back(float(std::atof(stringBuffer.c_str())));
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
#ifdef _MSC_VER
            sscanf_s(lineBuffer.c_str() + 2, "%f %f %f", &position.x, &position.y, &position.z);
#else
            sscanf(lineBuffer.c_str() + 2, "%f %f %f", &position.x, &position.y, &position.z);
#endif
            globalLineVertices.push_back(position);
        } else if (command == 'l') {
            // Get indices of current path line
            std::vector<uint32_t> currentLineIndices;
            for (size_t linePtr = 2; linePtr < lineBuffer.size(); linePtr++) {
                char currentChar = lineBuffer.at(linePtr);
                bool isWhitespace = currentChar == ' ' || currentChar == '\t';
                if (isWhitespace && !stringBuffer.empty()) {
                    currentLineIndices.push_back(atoi(stringBuffer.c_str()) - 1);
                    stringBuffer.clear();
                } else if (!isWhitespace) {
                    stringBuffer.push_back(currentChar);
                }
            }
            if (!stringBuffer.empty()) {
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

                // Remove large values (often used in many scientific datasets to indicate invalid lines/line points).
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
                    if (isWhitespace && !stringBuffer.empty()) {
                        attributeNames.push_back(stringBuffer);
                        stringBuffer.clear();
                    } else if (!isWhitespace) {
                        stringBuffer.push_back(currentChar);
                    }
                }
                if (!stringBuffer.empty()) {
                    attributeNames.push_back(stringBuffer);
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

    delete[] buffer;
    buffer = nullptr;

    return trajectories;
}
