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

#ifdef USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#endif

#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/StringUtils.hpp>
#include <Utils/Convert.hpp>

#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
#include "StructuredGridVtkLoader.hpp"

void StructuredGridVtkLoader::_readLines(
        ReadMode readMode, int numObjects, float* fieldData,
        size_t& charPtr, size_t& length, const char* fileBuffer) {
    int lineNum = 0;
    std::string lineBuffer;
    std::vector<std::string> splitLineString;
    while (charPtr < length && lineNum < numObjects) {
        lineBuffer.clear();
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
            sgl::Logfile::get()->throwError(
                    "Error in StructuredGridVtkLoader::_readLines: Encountered an empty line.");
        }

        if (readMode == ReadMode::SCALAR) {
            fieldData[lineNum] = sgl::fromString<float>(lineBuffer);
        } else if (readMode == ReadMode::VECTOR) {
            splitLineString.clear();
            sgl::splitStringWhitespace(lineBuffer, splitLineString);
            if (splitLineString.size() != 3) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::_readLines: Vector mode, but line contains less than three "
                        "items.");
            }
            fieldData[lineNum * 3] = sgl::fromString<float>(splitLineString.at(0));
            fieldData[lineNum * 3 + 1] = sgl::fromString<float>(splitLineString.at(1));
            fieldData[lineNum * 3 + 2] = sgl::fromString<float>(splitLineString.at(2));
        }

        lineNum++;
    }

    if (lineNum < numObjects) {
        sgl::Logfile::get()->throwError(
                "Error in StructuredGridVtkLoader::_readLines: The file has ended before all values could be read.");
    }
}

void StructuredGridVtkLoader::_readFieldLine(
        std::string& arrayName, int& numComponents, int& numTuples, std::string& dataType,
        size_t& charPtr, size_t& length, const char* fileBuffer) {
    while (charPtr < length) {
        char currentChar = fileBuffer[charPtr];
        if (currentChar == '\n' || currentChar == '\r') {
            charPtr++;
        } else {
            break;
        }
    }

    std::string lineBuffer;
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
        sgl::Logfile::get()->throwError(
                "Error in StructuredGridVtkLoader::_readFieldLine: Encountered an empty line.");
    }

    std::vector<std::string> splitLineString;
    sgl::splitStringWhitespace(lineBuffer, splitLineString);
    if (splitLineString.size() != 4) {
        sgl::Logfile::get()->throwError(
                "Error in StructuredGridVtkLoader::_readFieldLine: The line does not contain four entries.");
    }

    arrayName = splitLineString.at(0);
    numComponents = sgl::fromString<int>(splitLineString.at(1));
    numTuples = sgl::fromString<int>(splitLineString.at(2));
    dataType = splitLineString.at(3);
}

void StructuredGridVtkLoader::_convertScalarFieldCellToPointMode(
        const float* scalarFieldCell, float* scalarFieldPoint, int xs, int ys, int zs) {
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, zs), [&](auto const& r) {
        for (auto z = r.begin(); z != r.end(); z++) {
#else
    #pragma omp parallel for shared(xs, ys, zs, scalarFieldCell, scalarFieldPoint) default(none)
    for (int z = 0; z < zs; z++) {
#endif
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                int numNeighboringCells = 0;
                float valueSum = 0.0f;
                for (int zc = z - 1; zc <= z; zc++) {
                    for (int yc = y - 1; yc <= y; yc++) {
                        for (int xc = x - 1; xc <= x; xc++) {
                            if (xc >= 0 && yc >= 0 && zc >= 0 && xc < xs - 1 && yc < ys - 1 && zc < zs - 1) {
                                numNeighboringCells++;
                                valueSum += scalarFieldCell[IDXS_C(xc, yc, zc)];
                            }
                        }
                    }
                }
                scalarFieldPoint[IDXS(x, y, z)] = valueSum / float(numNeighboringCells);
            }
        }
    }
#ifdef USE_TBB
    });
#endif
}

void StructuredGridVtkLoader::load(
        const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
        StreamlineTracingGrid* grid) {
    int xs = 0, ys = 0, zs = 0;

    uint8_t* buffer = nullptr;
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(dataSourceFilename, buffer, length, false);
    if (!loaded) {
        sgl::Logfile::get()->throwError(
                "Error in StructuredGridVtkLoader::load: Couldn't open file \"" + dataSourceFilename + "\".");
    }
    char* fileBuffer = reinterpret_cast<char*>(buffer);

    std::string lineBuffer;
    std::string stringBuffer;
    std::vector<std::string> splitLineString;
    bool isBinaryMode = false;
    bool pointDataMode = true; //< Point data or cell data.
    std::string scalarFieldName;
    bool ignoreNextScalarData = false; //< For ignoring everything that is not of type float.
    int nextScalarDataBytesPerElement = 4; //< For ignoring everything that is not of type float.
    int numPoints = 0;
    int numCells = 0;

    glm::vec3* gridPoints = nullptr;
    float* points = nullptr;
    std::map<std::string, float*> vectorFields;
    std::map<std::string, float*> scalarFields;

    for (size_t charPtr = 0; charPtr < length; ) {
        lineBuffer.clear();
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

        splitLineString.clear();
        sgl::splitStringWhitespace(lineBuffer, splitLineString);
        if (splitLineString.empty()) {
            continue;
        }

        if (splitLineString.front() == "BINARY") {
            isBinaryMode = true;
        } else if (splitLineString.front() == "ASCII") {
            isBinaryMode = false;
        } else if (splitLineString.front() == "DATASET") {
            if (splitLineString.size() != 2 || splitLineString.at(1) != "STRUCTURED_GRID") {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid DATASET mode in file \""
                        + dataSourceFilename + "\".");
            }
        } else if (splitLineString.front() == "DIMENSIONS") {
            if (splitLineString.size() != 4) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid DIMENSIONS string in file \""
                        + dataSourceFilename + "\".");
            }
            xs = sgl::fromString<int>(splitLineString.at(1));
            ys = sgl::fromString<int>(splitLineString.at(2));
            zs = sgl::fromString<int>(splitLineString.at(3));
            numPoints = xs * ys * zs;
            numCells = (xs - 1) * (ys - 1) * (zs - 1);
        } else if (splitLineString.front() == "POINTS") {
            if (splitLineString.size() != 3) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid POINTS string in file \""
                        + dataSourceFilename + "\".");
            }
            if (splitLineString.at(2) != "float") {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid POINTS string in file \""
                        + dataSourceFilename + "\". The loader only supports float data.");
            }
            int numPointsLocal = sgl::fromString<int>(splitLineString.at(1));
            if (numPointsLocal != numPoints) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid POINTS string in file \""
                        + dataSourceFilename + "\". The number of points does not match the dimensions.");
            }
            if (gridPoints) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: The file \""
                        + dataSourceFilename + "\" contains more than one POINTS statement.");
            }

            gridPoints = new glm::vec3[numPoints];
            points = reinterpret_cast<float*>(gridPoints);
            if (isBinaryMode) {
                size_t numBytes = sizeof(float) * numPoints * 3;
                if (charPtr + numBytes > length) {
                    sgl::Logfile::get()->throwError(
                            "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                            + "\" ended before all data from a POINTS statement could be read.");
                }
                memcpy(points, fileBuffer + charPtr, sizeof(float) * numPoints * 3);
                swapEndianness(points, numPoints * 3);
                charPtr += sizeof(float) * numPoints * 3;
            } else {
                _readLines(ReadMode::VECTOR, numPoints, points, charPtr, length, fileBuffer);
            }
        } else if (splitLineString.front() == "POINT_DATA") {
            if (splitLineString.size() != 2) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid POINT_DATA string in file \""
                        + dataSourceFilename + "\".");
            }
            int numPointsLocal = sgl::fromString<int>(splitLineString.at(1));
            if (numPointsLocal != numPoints) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid POINT_DATA string in file \""
                        + dataSourceFilename + "\". The number of points does not match the dimensions.");
            }

            pointDataMode = true;
        } else if (splitLineString.front() == "CELL_DATA") {
            if (splitLineString.size() != 2) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid CELL_DATA string in file \""
                        + dataSourceFilename + "\".");
            }
            int numCellsLocal = sgl::fromString<int>(splitLineString.at(1));
            if (numCellsLocal != numCells) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid CELL_DATA string in file \""
                        + dataSourceFilename + "\". The number of points does not match the dimensions.");
            }

            pointDataMode = false;
        } else if (splitLineString.front() == "VECTORS") {
            if (splitLineString.size() != 3) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid VECTORS string in file \""
                        + dataSourceFilename + "\".");
            }
            if (splitLineString.at(2) != "float") {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid VECTORS string in file \""
                        + dataSourceFilename + "\". The loader only supports float data.");
            }

            std::string vectorFieldName = splitLineString.at(1);

            int numVectors = pointDataMode ? numPoints : numCells;
            auto* vectorField = new float[numVectors * 3];
            if (isBinaryMode) {
                size_t numBytes = sizeof(float) * numVectors * 3;
                if (charPtr + numBytes > length) {
                    sgl::Logfile::get()->throwError(
                            "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                            + "\" ended before all data from a POINTS statement could be read.");
                }
                memcpy(vectorField, fileBuffer + charPtr, sizeof(float) * numVectors * 3);
                swapEndianness(vectorField, numVectors * 3);
                charPtr += sizeof(float) * numVectors * 3;
            } else {
                _readLines(
                        ReadMode::VECTOR, numVectors, vectorField,
                        charPtr, length, fileBuffer);
            }

            if (vectorFields.find(vectorFieldName) != vectorFields.end()) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: The vector field \"" + vectorFieldName
                        + "\" exists more than one time in the file \"" + dataSourceFilename + "\".");
            }

            if (pointDataMode) {
                vectorFields.insert(std::make_pair(vectorFieldName, vectorField));
            } else {
                // Ignoring cell data for now.
                delete[] vectorField;
            }
        } else if (splitLineString.front() == "SCALARS") {
            if (splitLineString.size() != 4) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid SCALARS string in file \""
                        + dataSourceFilename + "\".");
            }
            if (splitLineString.at(2) != "float" && splitLineString.at(2) != "unsigned_char") {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid SCALARS string in file \"" + dataSourceFilename +
                        "\". The loader only supports float data. Additionally, unsigned_char data can be ignored.");
            }
            if (splitLineString.at(3) != "1") {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid SCALARS string in file \""
                        + dataSourceFilename + "\". The loader only supports scalars with one value.");
            }
            if (!scalarFieldName.empty()) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Encountered another SCALARS string in file \""
                        + dataSourceFilename + "\" even though no LOOKUP_TABLE statement was given.");
            }

            scalarFieldName = splitLineString.at(1);
            std::string dataType = splitLineString.at(2);
            ignoreNextScalarData = dataType != "float";
            if (dataType == "float") {
                nextScalarDataBytesPerElement = 4;
            } else if (dataType == "char" || dataType == "unsigned_char") {
                nextScalarDataBytesPerElement = 1;
            } else {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Encountered unsupported data type \""
                        + dataType + "\" in a SCALARS statement in the file \"" + dataSourceFilename + "\".");
            }
        } else if (splitLineString.front() == "LOOKUP_TABLE") {
            if (scalarFieldName.empty()) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Encountered a LOOKUP_TABLE string in file \""
                        + dataSourceFilename + "\" before a SCALARS string.");
            }

            int numScalars = pointDataMode ? numPoints : numCells;

            if (!ignoreNextScalarData) {
                auto* scalarField = new float[numScalars];
                if (isBinaryMode) {
                    size_t numBytes = sizeof(float) * numScalars;
                    if (charPtr + numBytes > length) {
                        sgl::Logfile::get()->throwError(
                                "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                                + "\" ended before all data from a SCALARS statement could be read.");
                    }
                    memcpy(scalarField, fileBuffer + charPtr, sizeof(float) * numScalars);
                    swapEndianness(scalarField, numScalars);
                    charPtr += sizeof(float) * numScalars;
                } else {
                    _readLines(
                            ReadMode::SCALAR, numScalars, scalarField,
                            charPtr, length, fileBuffer);
                }

                if (scalarFields.find(scalarFieldName) != scalarFields.end()) {
                    sgl::Logfile::get()->throwError(
                            "Error in StructuredGridVtkLoader::load: The scalar field \"" + scalarFieldName
                            + "\" exists more than one time in the file \"" + dataSourceFilename + "\".");
                }
                if (pointDataMode) {
                    scalarFields.insert(std::make_pair(scalarFieldName, scalarField));
                } else {
                    // Convert cell data to point data.
                    auto* scalarFieldPoint = new float[numPoints];
                    _convertScalarFieldCellToPointMode(scalarField, scalarFieldPoint, xs, ys, zs);
                    scalarFields.insert(std::make_pair(scalarFieldName, scalarFieldPoint));
                    delete[] scalarField;
                }
            } else {
                if (isBinaryMode) {
                    size_t numBytes = nextScalarDataBytesPerElement * numScalars;
                    if (charPtr + numBytes > length) {
                        sgl::Logfile::get()->throwError(
                                "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                                + "\" ended before all data from a SCALARS statement could be read.");
                    }
                    charPtr += sizeof(float) * numScalars;
                } else {
                    _readLines(
                            ReadMode::SKIP, numScalars, nullptr,
                            charPtr, length, fileBuffer);
                }
            }
            scalarFieldName.clear();
        } else if (splitLineString.front() == "FIELD") {
            if (splitLineString.size() != 3) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: Invalid FIELD string in file \""
                        + dataSourceFilename + "\".");
            }
            int numFieldEntries = sgl::fromString<int>(splitLineString.at(2));

            for (int fieldEntryIdx = 0; fieldEntryIdx < numFieldEntries; fieldEntryIdx++) {
                std::string arrayName;
                int numComponents = 0;
                int numTuples = 0;
                std::string dataType;
                _readFieldLine(arrayName, numComponents, numTuples, dataType, charPtr, length, fileBuffer);

                bool ignoreNextData = dataType != "float";
                if (dataType == "float") {
                    nextScalarDataBytesPerElement = 4;
                } else if (dataType == "char" || dataType == "unsigned_char") {
                    nextScalarDataBytesPerElement = 1;
                } else {
                    sgl::Logfile::get()->throwError(
                            "Error in StructuredGridVtkLoader::load: Encountered unsupported data type \""
                            + dataType + "\" in a FIELD statement in the file \"" + dataSourceFilename + "\".");
                }
                int numArrayEntries = numTuples * numComponents;

                if ((pointDataMode && numTuples != numPoints) || (!pointDataMode && numTuples != numCells)) {
                    sgl::Logfile::get()->throwError(
                            "Error in StructuredGridVtkLoader::load: FIELD array entry in file \""
                            + dataSourceFilename + "\". has an invalid number of tuples.");
                }

                if (numComponents != 1 && numComponents != 3) {
                    sgl::Logfile::get()->throwError(
                            "Error in StructuredGridVtkLoader::load: FIELD array entry in file \""
                            + dataSourceFilename + "\". has an invalid number of tuples.");
                }

                if (!ignoreNextData) {
                    auto* arrayField = new float[numArrayEntries];
                    if (isBinaryMode) {
                        size_t numBytes = sizeof(float) * numArrayEntries;
                        if (charPtr + numBytes > length) {
                            sgl::Logfile::get()->throwError(
                                    "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                                    + "\" ended before all data from a FIELD statement could be read.");
                        }
                        memcpy(arrayField, fileBuffer + charPtr, sizeof(float) * numArrayEntries);
                        swapEndianness(arrayField, numArrayEntries);
                        charPtr += sizeof(float) * numArrayEntries;
                    } else {
                        _readLines(
                                ReadMode::SCALAR, numArrayEntries, arrayField,
                                charPtr, length, fileBuffer);
                    }

                    if (numComponents == 1) {
                        if (scalarFields.find(arrayName) != scalarFields.end()) {
                            sgl::Logfile::get()->throwError(
                                    "Error in StructuredGridVtkLoader::load: The scalar field \"" + scalarFieldName
                                    + "\" exists more than one time in the file \"" + dataSourceFilename + "\".");
                        }
                        if (pointDataMode) {
                            scalarFields.insert(std::make_pair(arrayName, arrayField));
                        } else {
                            // Convert cell data to point data.
                            auto* scalarFieldPoint = new float[numPoints];
                            _convertScalarFieldCellToPointMode(arrayField, scalarFieldPoint, xs, ys, zs);
                            scalarFields.insert(std::make_pair(arrayName, scalarFieldPoint));
                            delete[] arrayField;
                        }
                    } else if (numComponents == 3) {
                        if (vectorFields.find(arrayName) != vectorFields.end()) {
                            sgl::Logfile::get()->throwError(
                                    "Error in StructuredGridVtkLoader::load: The vector field \"" + arrayName
                                    + "\" exists more than one time in the file \"" + dataSourceFilename + "\".");
                        }
                        if (pointDataMode) {
                            vectorFields.insert(std::make_pair(arrayName, arrayField));
                        } else {
                            // Ignoring cell data for now.
                            delete[] arrayField;
                        }
                    }
                } else {
                    if (isBinaryMode) {
                        size_t numBytes = nextScalarDataBytesPerElement * numArrayEntries;
                        if (charPtr + numBytes > length) {
                            sgl::Logfile::get()->throwError(
                                    "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                                    + "\" ended before all data from a SCALARS statement could be read.");
                        }
                        charPtr += sizeof(float) * numArrayEntries;
                    } else {
                        _readLines(
                                ReadMode::SKIP, numArrayEntries, nullptr,
                                charPtr, length, fileBuffer);
                    }
                }
            }
        }
    }

    if (!gridPoints) {
        sgl::Logfile::get()->throwError(
                "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                + "\" does not contain a POINTS statement.");
    }

    sgl::AABB3 domainExtents;
    for (int i = 0; i < numPoints; i++) {
        domainExtents.combine(gridPoints[i]);
    }
    glm::vec3 bbDim = domainExtents.getDimensions();

    /*glm::vec3 dimensions = domainExtents.getDimensions();
    float maxDimension = std::max(dimensions.x, std::max(dimensions.y, dimensions.z));
    float cellStep = 1.0f / maxDimension;*/

    float maxDimension = float(std::max(xs - 1, std::max(ys - 1, zs - 1)));
    float cellStep = 1.0f / maxDimension;
    float maxBbDim = std::max(bbDim.x, std::max(bbDim.y, bbDim.z));
    float dx = cellStep * float(maxDimension) / float(xs) * bbDim.x / maxBbDim;
    float dy = cellStep * float(maxDimension) / float(ys) * bbDim.y / maxBbDim;
    float dz = cellStep * float(maxDimension) / float(zs) * bbDim.z / maxBbDim;
    grid->setGridExtent(xs, ys, zs, dx, dy, dz);

    std::map<std::string, float*>::iterator itVelocity;
    if (!gridDataSetMetaData.velocityFieldName.empty()) {
        itVelocity = vectorFields.find(gridDataSetMetaData.velocityFieldName);
        if (itVelocity == vectorFields.end()) {
            sgl::Logfile::get()->throwError(
                    "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                    + "\" does not contain a vector field called '" + gridDataSetMetaData.velocityFieldName + "'.");
        }
    } else {
        itVelocity = vectorFields.find("velocity");
        if (itVelocity == vectorFields.end()) {
            itVelocity = vectorFields.find("Velocity");
            if (itVelocity == vectorFields.end()) {
                sgl::Logfile::get()->throwError(
                        "Error in StructuredGridVtkLoader::load: The file \"" + dataSourceFilename
                        + "\" does not contain a vector field called 'velocity' or 'Velocity'.");
            }
        }
    }
    float* velocityField = itVelocity->second;

    if (scalarFields.find("velocityMagnitude") == scalarFields.end()) {
        auto* velocityMagnitudeField = new float[numPoints];
        computeVectorMagnitudeField(velocityField, velocityMagnitudeField, xs, ys, zs);
        scalarFields.insert(std::make_pair("Velocity Magnitude", velocityMagnitudeField));
    }

    if (vectorFields.find("vorticity") == vectorFields.end()
            || scalarFields.find("vorticityMagnitude") == scalarFields.end()
            || scalarFields.find("helicity") == scalarFields.end()) {
        float* vorticityField;
        auto it = vectorFields.find("vorticity");
        if (it == vectorFields.end()) {
            vorticityField = new float[numPoints * 3];
            computeVorticityField(velocityField, vorticityField, xs, ys, zs, cellStep, cellStep, cellStep);
            vectorFields.insert(std::make_pair("Vorticity", vorticityField));
        } else {
            vorticityField = it->second;
        }
        if (scalarFields.find("vorticityMagnitude") == scalarFields.end()) {
            auto* vorticityMagnitudeField = new float[numPoints];
            computeVectorMagnitudeField(vorticityField, vorticityMagnitudeField, xs, ys, zs);
            scalarFields.insert(std::make_pair("Vorticity Magnitude", vorticityMagnitudeField));
        }
        // Don't use cached helicity if normalized velocity and/or normalized vorticity should be used.
        if (gridDataSetMetaData.useNormalizedVelocity || gridDataSetMetaData.useNormalizedVorticity) {
            auto helicityIt = scalarFields.find("helicity");
            if (helicityIt != scalarFields.end()) {
                delete[] it->second;
                scalarFields.erase(helicityIt);
            }
        }
        if (scalarFields.find("helicity") == scalarFields.end()) {
            auto* helicityField = new float[numPoints];
            computeHelicityFieldNormalized(
                    velocityField, vorticityField, helicityField, xs, ys, zs,
                    gridDataSetMetaData.useNormalizedVelocity,
                    gridDataSetMetaData.useNormalizedVorticity);
            scalarFields.insert(std::make_pair("Helicity", helicityField));
        }
    }

    for (auto& it : vectorFields) {
        // Convert first letter to upper case.
        std::string vectorDisplayName;
        vectorDisplayName = sgl::toUpperCopy(it.first.substr(0, 1)) + it.first.substr(1);
        grid->addVectorField(it.second, vectorDisplayName);
    }

    for (auto& it : scalarFields) {
        // Convert first letter to upper case.
        std::string scalarDisplayName;
        scalarDisplayName = sgl::toUpperCopy(it.first.substr(0, 1)) + it.first.substr(1);
        std::string::size_type magnitudePos = scalarDisplayName.find("Magnitude");
        if (scalarDisplayName.find(" Magnitude") == std::string::npos
                && magnitudePos != std::string::npos && magnitudePos > 0) {
            scalarDisplayName =
                    scalarDisplayName.substr(0, magnitudePos) + " " + scalarDisplayName.substr(magnitudePos);
        }
        grid->addScalarField(it.second, scalarDisplayName);
    }

    delete[] gridPoints;
    delete[] buffer;
    buffer = nullptr;
}
