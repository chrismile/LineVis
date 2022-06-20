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

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <Utils/Convert.hpp>
#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
#include "DatRawFileLoader.hpp"

struct FieldFileHeader {
    glm::uvec3 resolution;
    uint32_t dimensions;
    uint32_t mipLevels;
    uint32_t fieldType;
};

void DatRawFileLoader::load(
        const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
        StreamlineTracingGrid* grid) {
    std::string datFilePath;
    std::string rawFilePath;

    if (boost::ends_with(dataSourceFilename, ".dat")) {
        datFilePath = dataSourceFilename;
    }
    if (boost::ends_with(dataSourceFilename, ".raw")) {
        rawFilePath = dataSourceFilename;

        // We need to find the corresponding .dat file.
        std::string rawFileDirectory = sgl::FileUtils::get()->getPathToFile(rawFilePath);
        std::vector<std::string> filesInDir = sgl::FileUtils::get()->getFilesInDirectoryVector(rawFileDirectory);
        for (const std::string& filePath : filesInDir) {
            if (boost::ends_with(filePath, ".dat")) {
                datFilePath = filePath;
                break;
            }
        }
        if (datFilePath.empty()) {
            sgl::Logfile::get()->throwError(
                    "Error in DatRawFileLoader::load: No .dat file found for \"" + rawFilePath + "\".");
        }
    }

    // Load the .dat metadata file.
    uint8_t* bufferDat = nullptr;
    size_t lengthDat = 0;
    bool loadedDat = sgl::loadFileFromSource(datFilePath, bufferDat, lengthDat, false);
    if (!loadedDat) {
        sgl::Logfile::get()->throwError(
                "Error in DatRawFileLoader::load: Couldn't open file \"" + datFilePath + "\".");
    }
    char* fileBuffer = reinterpret_cast<char*>(bufferDat);

    std::string lineBuffer;
    std::string stringBuffer;
    std::vector<std::string> splitLineString;
    std::map<std::string, std::string> datDict;
    for (size_t charPtr = 0; charPtr < lengthDat; ) {
        lineBuffer.clear();
        while (charPtr < lengthDat) {
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
        sgl::splitString(lineBuffer, ':', splitLineString);
        if (splitLineString.empty()) {
            continue;
        }
        if (splitLineString.size() != 2) {
            sgl::Logfile::get()->throwError(
                    "Error in DatRawFileLoader::load: Invalid entry in file \"" + datFilePath + "\".");
        }

        std::string datKey = splitLineString.at(0);
        std::string datValue = splitLineString.at(1);
        boost::trim(datKey);
        boost::to_lower(datKey);
        boost::trim(datValue);
        datDict.insert(std::make_pair(datKey, datValue));
    }

    // Next, process the metadata.
    if (rawFilePath.empty()) {
        auto it = datDict.find("objectfilename");
        if (it == datDict.end()) {
            sgl::Logfile::get()->throwError(
                    "Error in DatRawFileLoader::load: Entry 'ObjectFileName' missing in \""
                    + datFilePath + "\".");
        }
        if (datDict.find("objectindices") != datDict.end()) {
            sgl::Logfile::get()->throwError(
                    "Error in DatRawFileLoader::load: ObjectIndices found in file \"" + datFilePath
                    + "\" is not yet supported.");
        }
        rawFilePath = it->second;
        bool isAbsolutePath = sgl::FileUtils::get()->getIsPathAbsolute(rawFilePath);
        if (!isAbsolutePath) {
            rawFilePath = sgl::FileUtils::get()->getPathToFile(datFilePath) + rawFilePath;
        }
    }

    auto itResolution = datDict.find("resolution");
    if (itResolution == datDict.end()) {
        sgl::Logfile::get()->throwError(
                "Error in DatRawFileLoader::load: Entry 'Resolution' missing in \"" + datFilePath + "\".");
    }
    std::vector<std::string> resolutionSplit;
    sgl::splitStringWhitespace(itResolution->second, resolutionSplit);
    if (resolutionSplit.size() != 3) {
        sgl::Logfile::get()->throwError(
                "Error in DatRawFileLoader::load: Entry 'Resolution' in \"" + datFilePath
                + "\" does not have three values.");
    }
    int xs = int(sgl::fromString<int>(resolutionSplit.at(0)));
    int ys = int(sgl::fromString<int>(resolutionSplit.at(1)));
    int zs = int(sgl::fromString<int>(resolutionSplit.at(2)));
    float maxDimension = float(std::max(xs - 1, std::max(ys - 1, zs - 1)));
    float cellStep = 1.0f / maxDimension;

    auto itFormat = datDict.find("format");
    if (itFormat == datDict.end()) {
        sgl::Logfile::get()->throwError(
                "Error in DatRawFileLoader::load: Entry 'Format' missing in \"" + datFilePath + "\".");
    }
    std::string formatString = boost::to_lower_copy(itFormat->second);
    int numComponents = 0;
    if (formatString == "float3") {
        numComponents = 3;
    } else if (formatString == "float4") {
        numComponents = 4;
    } else {
        sgl::Logfile::get()->throwError(
                "Error in DatRawFileLoader::load: Unsupported format '" + formatString + "' in file \""
                + datFilePath + "\".");
    }

    // Finally, load the data from the .raw file.
    uint8_t* bufferRaw = nullptr;
    size_t lengthRaw = 0;
    bool loadedRaw = sgl::loadFileFromSource(rawFilePath, bufferRaw, lengthRaw, true);
    if (!loadedRaw) {
        sgl::Logfile::get()->throwError(
                "Error in DatRawFileLoader::load: Couldn't open file \"" + rawFilePath + "\".");
    }
    auto* dataField = reinterpret_cast<float*>(bufferRaw);

    size_t numBytesData = lengthRaw;
    size_t gridNumCellsTotal = size_t(xs) * size_t(ys) * size_t(zs);
    if (numBytesData != gridNumCellsTotal * numComponents * sizeof(float)) {
        sgl::Logfile::get()->throwError(
                "Error in DatRawFileLoader::load: Invalid number of entries for file \""
                + rawFilePath + "\".");
    }

    int vectorFieldNumEntries = xs * ys * zs * 3;
    int scalarFieldNumEntries = xs * ys * zs;

    auto* velocityField = new float[vectorFieldNumEntries];
    auto* velocityMagnitudeField = new float[scalarFieldNumEntries];
    auto* vorticityField = new float[vectorFieldNumEntries];
    auto* vorticityMagnitudeField = new float[scalarFieldNumEntries];
    auto* helicityField = new float[scalarFieldNumEntries];
    float* scalarAttributeField = nullptr;

    if (numComponents == 3) {
        for (int z = 0; z < zs; z++) {
            for (int y = 0; y < ys; y++) {
                for (int x = 0; x < xs; x++) {
                    velocityField[IDXV(x, y, z, 0)] = dataField[IDXV(x, y, z, 0)];
                    velocityField[IDXV(x, y, z, 1)] = dataField[IDXV(x, y, z, 1)];
                    velocityField[IDXV(x, y, z, 2)] = dataField[IDXV(x, y, z, 2)];
                }
            }
        }
    } else if (numComponents == 4) {
        scalarAttributeField = new float[scalarFieldNumEntries];
        for (int z = 0; z < zs; z++) {
            for (int y = 0; y < ys; y++) {
                for (int x = 0; x < xs; x++) {
                    velocityField[IDXV(x, y, z, 0)] = dataField[IDXV4(x, y, z, 0)];
                    velocityField[IDXV(x, y, z, 1)] = dataField[IDXV4(x, y, z, 1)];
                    velocityField[IDXV(x, y, z, 2)] = dataField[IDXV4(x, y, z, 2)];
                    scalarAttributeField[IDXS(x, y, z)] = dataField[IDXV4(x, y, z, 3)];
                }
            }
        }
    }

    computeVectorMagnitudeField(velocityField, velocityMagnitudeField, xs, ys, zs);
    computeVorticityField(velocityField, vorticityField, xs, ys, zs, cellStep, cellStep, cellStep);
    computeVectorMagnitudeField(vorticityField, vorticityMagnitudeField, xs, ys, zs);
    computeHelicityField(velocityField, vorticityField, helicityField, xs, ys, zs);

    grid->setGridExtent(xs, ys, zs, cellStep, cellStep, cellStep);
    grid->addVectorField(velocityField, "Velocity");
    grid->addVectorField(vorticityField, "Vorticity");
    grid->addScalarField(helicityField, "Helicity");
    grid->addScalarField(velocityMagnitudeField, "Velocity Magnitude");
    grid->addScalarField(vorticityMagnitudeField, "Vorticity Magnitude");
    if (scalarAttributeField) {
        // Make an educated guess about the type of the attribute.
        std::string filenameRawLower = sgl::FileUtils::get()->getPureFilename(dataSourceFilename);
        std::string scalarAttributeName;
        if (filenameRawLower.find("borromean") != std::string::npos
            || filenameRawLower.find("magnet") != std::string::npos) {
            scalarAttributeName = "Field Strength";
        } else {
            scalarAttributeName = "Scalar Attribute";
        }
        grid->addScalarField(scalarAttributeField, scalarAttributeName);
    }

    delete[] bufferDat;
    delete[] bufferRaw;
}
