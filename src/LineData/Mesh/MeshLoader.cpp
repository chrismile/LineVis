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

#include <cstdio>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include <Utils/Convert.hpp>
#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>

#include "MeshLoader.hpp"

bool MeshLoader::loadHexahedralMeshFromFile(
        const std::string& filename,
        std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices,
        std::vector<glm::vec3>& deformations, std::vector<float>& anisotropyMetricList) {
    bool foundVersionHeader = false;

    bool foundDimensionHeader = false;
    bool lastLineWasDimensionHeader = false;
    bool foundVerticesHeader = false;
    bool lastLineWasVerticesHeader = false;
    bool isVerticesReadMode = false;
    int numVerticesLeft = 0;
    bool foundHexahedraHeader = false;
    bool lastLineWasHexahedraHeader = false;
    bool isHexahedraReadMode = false;
    int numHexahedraLeft = 0;
    bool foundQuadrilateralsHeader = false;
    bool lastLineWasQuadrilateralsHeader = false;
    bool isQuadrilateralsReadMode = false;
    int numQuadrilateralsLeft = 0;
    bool endReached = false;

    bool loadingSuccessful = readFileLineByLine(
            filename, [&](const std::string& lineString, const std::vector<std::string>& tokens) {
        if (endReached) {
            sgl::Logfile::get()->writeError("Error in MeshLoader: End was reached, but it was not the last token!");
            return false;
        }

        if (isVerticesReadMode) {
            if (tokens.size() != 4) {
                sgl::Logfile::get()->writeError("Error in MeshLoader: Invalid number of vertex coordinates!");
                return false;
            }
            vertices.push_back(glm::vec3(
                    sgl::fromString<float>(tokens.at(0)),
                    sgl::fromString<float>(tokens.at(1)),
                    sgl::fromString<float>(tokens.at(2))));
            numVerticesLeft--;
            if (numVerticesLeft <= 0) {
                isVerticesReadMode = false;
            }
            return true;
        }

        if (isHexahedraReadMode) {
            if (tokens.size() != 9) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid number of hexahedral cell indices!");
                return false;
            }
            for (int i = 0; i < 8; i++) {
                cellIndices.push_back(sgl::fromString<uint32_t>(tokens.at(i)) - 1);
            }
            numHexahedraLeft--;
            if (numHexahedraLeft <= 0) {
                isHexahedraReadMode = false;
            }
            return true;
        }

        // Quadrilateral data is not used.
        if (isQuadrilateralsReadMode) {
            numQuadrilateralsLeft--;
            if (numQuadrilateralsLeft <= 0) {
                isQuadrilateralsReadMode = false;
            }
            return true;
        }

        // The version header must the first non-empty line!
        if (!foundVersionHeader) {
            if (sgl::startsWith(tokens.at(0), "MeshVersionFormatted")) {
                foundVersionHeader = true;
                return true;
            } else {
                sgl::Logfile::get()->writeError(
                        "Error in MeshLoader: The version header must the first non-empty line!");
                return false;
            }
        }

        if (!foundDimensionHeader && tokens.at(0) == "Dimension") {
            foundDimensionHeader = true;
            if (tokens.size() == 1) {
                lastLineWasDimensionHeader = true;
            } else {
                if (tokens.at(1) != "3") {
                    sgl::Logfile::get()->writeError("Error in MeshLoader: Invalid dimension number!");
                    return false;
                }
            }
            return true;
        }

        if (lastLineWasDimensionHeader) {
            if (tokens.at(0) != "3") {
                sgl::Logfile::get()->writeError("Error in MeshLoader: Invalid dimension number!");
                return false;
            } else {
                lastLineWasDimensionHeader = false;
                return true;
            }
        }

        if (!foundVerticesHeader && tokens.at(0) == "Vertices") {
            foundVerticesHeader = true;
            if (tokens.size() == 1) {
                lastLineWasVerticesHeader = true;
            } else {
                numVerticesLeft = sgl::fromString<int>(tokens.at(1));
                isVerticesReadMode = numVerticesLeft > 0;
            }
            return true;
        }

        if (lastLineWasVerticesHeader) {
            numVerticesLeft = sgl::fromString<int>(tokens.at(0));
            lastLineWasVerticesHeader = false;
            isVerticesReadMode = numVerticesLeft > 0;
            return true;
        }

        if (!foundHexahedraHeader && tokens.at(0) == "Hexahedra") {
            foundHexahedraHeader = true;
            if (tokens.size() == 1) {
                lastLineWasHexahedraHeader = true;
            } else {
                numHexahedraLeft = sgl::fromString<int>(tokens.at(1));
                isHexahedraReadMode = numHexahedraLeft > 0;
            }
            return true;
        }

        if (lastLineWasHexahedraHeader) {
            numHexahedraLeft = sgl::fromString<int>(tokens.at(0));
            lastLineWasHexahedraHeader = false;
            isHexahedraReadMode = numHexahedraLeft > 0;
            return true;
        }

        if (!foundQuadrilateralsHeader && (tokens.at(0) == "Quadrilaterals" || tokens.at(0) == "Quads")) {
            foundQuadrilateralsHeader = true;
            if (tokens.size() == 1) {
                lastLineWasQuadrilateralsHeader = true;
            } else {
                numQuadrilateralsLeft = sgl::fromString<int>(tokens.at(0));
                isQuadrilateralsReadMode = numQuadrilateralsLeft > 0;
            }
            return true;
        }

        if (lastLineWasQuadrilateralsHeader) {
            numQuadrilateralsLeft = sgl::fromString<int>(tokens.at(0));
            lastLineWasQuadrilateralsHeader = false;
            isQuadrilateralsReadMode = numQuadrilateralsLeft > 0;
            return true;
        }

        if (!endReached && tokens.at(0) == "End") {
            endReached = true;
            return true;
        }

        // Something unexpected happened.
        return false;
    });

    return loadingSuccessful;
}
