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

#include <boost/algorithm/string.hpp>
#include <glm/vec3.hpp>

#include <Utils/File/Logfile.hpp>
#include <Utils/Convert.hpp>

#include "VtkLoader.hpp"

bool VtkLoader::loadHexahedralMeshFromFile(
        const std::string& filename,
        std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices,
        std::vector<glm::vec3>& deformations, std::vector<float>& anisotropyMetricList) {
    bool foundVersionHeader = false;
    bool foundTitle = false;
    bool foundType = false;
    bool foundDatasetType = false;
    bool foundPoints = false;
    bool foundCells = false;
    bool foundCellTypes = false;
    bool isPointsReadMode = false;
    int numPointsLeft = 0;
    bool isCellsReadMode = false;
    int numCellsLeft = 0;
    bool isCellTypesReadMode = false;
    int numCellTypesLeft = 0;
    bool isCellDataReadMode = false;
    int numCellDataLinesLeft = 0;
    bool isPointDataReadMode = false;
    int numPointDataLinesLeft = 0;
    bool isDeformationDataReadMode = false;
    int numDeformationDataLinesLeft = 0;
    bool isAnisotropyMetricReadMode = false;
    int numAnisotropyMetricLinesLeft = 0;

    bool loadingSuccessful = readFileLineByLine(
            filename, [&](const std::string& lineString, const std::vector<std::string>& tokens) {
        if (isPointsReadMode) {
            if (tokens.size() != 3) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid number of point coordinates!");
                return false;
            }
            vertices.push_back(glm::vec3(
                    sgl::fromString<float>(tokens.at(0)),
                    sgl::fromString<float>(tokens.at(1)),
                    sgl::fromString<float>(tokens.at(2))));
            numPointsLeft--;
            if (numPointsLeft <= 0) {
                isPointsReadMode = false;
            }
            return true;
        }

        if (isCellsReadMode) {
            if (tokens.size() != 9 || tokens.at(0) != "8") {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid number of cell indices!");
                return false;
            }
            for (int i = 0; i < 8; i++) {
                cellIndices.push_back(sgl::fromString<uint32_t>(tokens.at(i + 1)));
            }
            numCellsLeft--;
            if (numCellsLeft <= 0) {
                isCellsReadMode = false;
            }
            return true;
        }

        if (isCellTypesReadMode) {
            // Somehow, data sets from "2019 - Symmetric Moving Frames" have 10 / tetrahedron as cell type...
            //if (tokens.size() != 1 || tokens.at(0) != "12") {
            //    sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid cell type!");
            //    return false;
            //}
            numCellTypesLeft--;
            if (numCellTypesLeft <= 0) {
                isCellTypesReadMode = false;
            }
            return true;
        }

        if (isCellDataReadMode) {
            numCellDataLinesLeft--;
            if (numCellDataLinesLeft <= 0) {
                isCellDataReadMode = false;
            }
            return true;
        }

        if (isPointDataReadMode) {
            numPointDataLinesLeft--;
            if (numPointDataLinesLeft <= 0) {
                isPointDataReadMode = false;
            }
            return true;
        }

        if (isDeformationDataReadMode) {
            if (tokens.size() != 3) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid number of point coordinates!");
                return false;
            }
            deformations.push_back(glm::vec3(
                    sgl::fromString<float>(tokens.at(0)),
                    sgl::fromString<float>(tokens.at(1)),
                    sgl::fromString<float>(tokens.at(2))));
            numDeformationDataLinesLeft--;
            if (numDeformationDataLinesLeft <= 0) {
                isDeformationDataReadMode = false;
            }
            return true;
        }

        if (isAnisotropyMetricReadMode) {
            if (tokens.size() != 1) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Data is not scalar!");
                return false;
            }
            anisotropyMetricList.push_back(sgl::fromString<float>(tokens.at(0)));
            numAnisotropyMetricLinesLeft--;
            if (numAnisotropyMetricLinesLeft <= 0) {
                isAnisotropyMetricReadMode = false;
            }
            return true;
        }

                // The version header must the first non-empty line!
        if (!foundVersionHeader) {
            if (boost::starts_with(lineString, "# vtk DataFile Version")) {
                foundVersionHeader = true;
                return true;
            } else {
                sgl::Logfile::get()->writeError(
                        "Error in VtkLoader: The version header must the first non-empty line!");
                return false;

            }
        }

        // Next line after the header is the title.
        if (!foundTitle) {
            foundTitle = true;
            return true;
        }

        // Next line is the type of data (ASCII vs BINARY).
        if (!foundType) {
            if (tokens.at(0) == "ASCII") {
                foundType = true;
                return true;
            } else {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid or unsupported type!");
                return false;
            }
        }

        // Expecting: DATASET UNSTRUCTURED_GRID
        if (!foundDatasetType && tokens.at(0) == "DATASET") {
            if (tokens.size() != 2 || tokens.at(1) != "UNSTRUCTURED_GRID") {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid or unsupported dataset type!");
                return false;
            }
            foundDatasetType = true;
            return true;
        }

        // Expecting: POINTS <num_points> <data_type>
        if (!foundPoints && tokens.at(0) == "POINTS") {
            if (tokens.size() != 3) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Malformed POINTS declaration!");
                return false;
            }
            numPointsLeft = sgl::fromString<int>(tokens.at(1));
            foundPoints = true;
            isPointsReadMode = numPointsLeft > 0;
            return true;
        }

        // Expecting: CELLS <num_cells> <num_entries>
        if (!foundCells && tokens.at(0) == "CELLS") {
            if (tokens.size() != 3) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Malformed POINTS declaration!");
                return false;
            }
            numCellsLeft = sgl::fromString<int>(tokens.at(1));
            int numEntries = sgl::fromString<int>(tokens.at(2));
            if (numEntries != numCellsLeft * 9) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Invalid number of cell entries!");
                return false;
            }
            foundCells = true;
            isCellsReadMode = numCellsLeft > 0;
            return true;
        }

        // Expecting: CELL_TYPES <num_cells>
        if (!foundCellTypes && tokens.at(0) == "CELL_TYPES") {
            if (tokens.size() != 2) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Malformed CELL_TYPES declaration!");
                return false;
            }
            numCellTypesLeft = sgl::fromString<int>(tokens.at(1));
            foundCellTypes = true;
            isCellTypesReadMode = numCellTypesLeft > 0;
            return true;
        }

        // Ignore cell data
        if (tokens.at(0) == "CELL_DATA") {
            if (tokens.size() != 2) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Malformed CELL_DATA declaration!");
                return false;
            }
            // Number of entries + SCALARS + LOOKUP_TABLE information
            numCellDataLinesLeft = sgl::fromString<int>(tokens.at(1)) + 2;
            isCellDataReadMode = true;
            return true;
        }

        // Ignore point data
        if (tokens.at(0) == "POINT_DATA") {
            if (tokens.size() != 2) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Malformed POINT_DATA declaration!");
                return false;
            }
            // Number of entries + SCALARS + LOOKUP_TABLE information
            numPointDataLinesLeft = sgl::fromString<int>(tokens.at(1)) + 2;
            isPointDataReadMode = true;
            return true;
        }

        // Read deformation data
        if (tokens.at(0) == "Deformation") {
            if (tokens.size() != 2) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Malformed Deformation declaration!");
                return false;
            }
            numDeformationDataLinesLeft = sgl::fromString<int>(tokens.at(1));
            isDeformationDataReadMode = true;
            return true;
        }

        // Read anisotropy metric data
        if (tokens.at(0) == "AnisotropyMetric") {
            if (tokens.size() != 2) {
                sgl::Logfile::get()->writeError("Error in VtkLoader: Malformed AnisotropyMetric declaration!");
                return false;
            }
            numAnisotropyMetricLinesLeft = sgl::fromString<int>(tokens.at(1));
            isAnisotropyMetricReadMode = true;
            return true;
        }

        // Something unexpected happened.
        return false;
    });

    return loadingSuccessful;
}
