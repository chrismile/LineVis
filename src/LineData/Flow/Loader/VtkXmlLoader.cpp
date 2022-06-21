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

#include <Math/Math.hpp>
#include <Utils/Convert.hpp>
#include <Utils/XML.hpp>
#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/Zlib.hpp>

#include "base64/base64.h"
#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
#include "VtkXmlLoader.hpp"

void VtkXmlLoader::load(
        const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
        StreamlineTracingGrid* grid) {
    XMLDocument doc;
    if (doc.LoadFile(dataSourceFilename.c_str()) != 0) {
        sgl::Logfile::get()->writeError(
                std::string() + "VtkXmlLoader::load: Couldn't open file \"" + dataSourceFilename + "\"!");
        return;
    }

    XMLElement* vtkFileNode = doc.FirstChildElement("VTKFile");
    const char* typeString = vtkFileNode->Attribute("type");
    if (typeString == nullptr || strcmp(typeString, "ImageData") != 0) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Invalid VTKFile type string in file \""
                + dataSourceFilename + "\".");
    }

    bool isLittleEndian = false;
    const char* byteOrderString = vtkFileNode->Attribute("byte_order");
    if (byteOrderString != nullptr) {
        if (strcmp(byteOrderString, "LittleEndian") == 0) {
            isLittleEndian = true;
        } else if (strcmp(byteOrderString, "BigEndian") == 0) {
            isLittleEndian = false;
        } else {
            sgl::Logfile::get()->throwError(
                    "Error in VtkXmlLoader::load: Invalid byte order string in file \""
                    + dataSourceFilename + "\".");
        }
    }

    const char* headerTypeString = vtkFileNode->Attribute("header_type");
    if (headerTypeString != nullptr && strcmp(headerTypeString, "UInt32") != 0) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Unsupported VTKFile header type in file \""
                + dataSourceFilename + "\".");
    }

    bool useZlib = false;
    const char* compressorString = vtkFileNode->Attribute("compressor");
    if (compressorString != nullptr) {
        if (strcmp(compressorString, "vtkZLibDataCompressor") == 0) {
            useZlib = true;
        } else {
            sgl::Logfile::get()->throwError(
                    std::string() + "Error in VtkXmlLoader::load: Invalid compressor \""
                    + compressorString + "\" in file \"" + dataSourceFilename + "\".");
        }
    }


    XMLElement* imageDataNode = vtkFileNode->FirstChildElement("ImageData");

    const char* wholeExtentString = imageDataNode->Attribute("WholeExtent");
    if (wholeExtentString == nullptr) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Missing WholeExtent for ImageData in file \""
                + dataSourceFilename + "\".");
    }
    std::vector<std::string> wholeExtentStringArray;
    sgl::splitStringWhitespace(wholeExtentString, wholeExtentStringArray);
    if (wholeExtentStringArray.size() != 6) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: ImageData WholeExtent attribute in file \""
                + dataSourceFilename + "\" does not have 6 entries as expected.");
    }
    std::vector<int> wholeExtentArray;
    for (auto& extentString : wholeExtentStringArray) {
        wholeExtentArray.push_back(sgl::fromString<int>(extentString));
    }
    int xs = wholeExtentArray.at(1) - wholeExtentArray.at(0) + 1;
    int ys = wholeExtentArray.at(3) - wholeExtentArray.at(2) + 1;
    int zs = wholeExtentArray.at(5) - wholeExtentArray.at(4) + 1;
    int numPoints = xs * ys * zs;

    const char* spacingString = imageDataNode->Attribute("Spacing");
    if (spacingString == nullptr) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Missing Spacing for ImageData in file \""
                + dataSourceFilename + "\".");
    }
    std::vector<std::string> spacingStringArray;
    sgl::splitStringWhitespace(spacingString, spacingStringArray);
    if (spacingStringArray.size() != 3) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: ImageData Spacing attribute in file \""
                + dataSourceFilename + "\" does not have 3 entries as expected.");
    }
    std::vector<float> spacingArray;
    for (auto& spacingStr : spacingStringArray) {
        spacingArray.push_back(sgl::fromString<float>(spacingStr));
    }

    XMLElement* pieceNode = imageDataNode->FirstChildElement("Piece");
    const char* pieceExtentString = pieceNode->Attribute("Extent");
    if (pieceExtentString == nullptr) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Piece without Extent data detected in file \""
                + dataSourceFilename + "\".");
    }
    if (strcmp(wholeExtentString, pieceExtentString) != 0) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Pieces with a different extent than the image data detected in \""
                + dataSourceFilename + "\". This is not yet supported.");
    }

    XMLElement* pointDataNode = pieceNode->FirstChildElement("PointData");

    bool velocityOffsetsSet[3] = { false, false, false };
    size_t velocityOffsets[3] = { 0, 0, 0 };

    for (sgl::XMLIterator it(pointDataNode, sgl::XMLNameFilter("DataArray")); it.isValid(); ++it) {
        XMLElement* dataArrayNode = *it;
        const char* dataArrayTypeString = dataArrayNode->Attribute("type");
        if (dataArrayTypeString == nullptr || strcmp(dataArrayTypeString, "Float32") != 0) {
            sgl::Logfile::get()->throwError(
                    "Error in VtkXmlLoader::load: Encountered data array with type not equal to Float32 in file \""
                    + dataSourceFilename + "\". Currently, only Float32 is supported.");
        }

        const char* dataArrayFormatString = dataArrayNode->Attribute("format");
        if (dataArrayFormatString == nullptr || strcmp(dataArrayFormatString, "appended") != 0) {
            sgl::Logfile::get()->throwError(
                    "Error in VtkXmlLoader::load: Encountered data array with format not equal to appended in file \""
                    + dataSourceFilename + "\". Currently, only appended data is supported.");
        }

        const char* dataArrayOffsetString = dataArrayNode->Attribute("offset");
        if (dataArrayOffsetString == nullptr) {
            sgl::Logfile::get()->throwError(
                    "Error in VtkXmlLoader::load: Missing offset for data array in file \""
                    + dataSourceFilename + "\".");
        }
        auto offset = sgl::fromString<size_t>(dataArrayOffsetString);

        const char* dataArrayNameString = dataArrayNode->Attribute("Name");
        if (dataArrayNameString == nullptr) {
            sgl::Logfile::get()->throwError(
                    "Error in VtkXmlLoader::load: Expected name for data array in file \""
                    + dataSourceFilename + "\".");
        }
        if (strcmp(dataArrayNameString, "u") == 0) {
            velocityOffsetsSet[0] = true;
            velocityOffsets[0] = offset;
        }
        if (strcmp(dataArrayNameString, "v") == 0) {
            velocityOffsetsSet[1] = true;
            velocityOffsets[1] = offset;
        }
        if (strcmp(dataArrayNameString, "w") == 0) {
            velocityOffsetsSet[2] = true;
            velocityOffsets[2] = offset;
        }
    }

    if (!velocityOffsetsSet[0] || !velocityOffsetsSet[1] || !velocityOffsetsSet[2]) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: A velocity vector field component was not found in file \""
                + dataSourceFilename + "\".");
    }

    XMLElement* appendedDataNode = vtkFileNode->FirstChildElement("AppendedData");
    const char* encodingString = appendedDataNode->Attribute("encoding");
    if (encodingString == nullptr || strcmp(encodingString, "base64") != 0) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Unsupported appended data encoding in file \""
                + dataSourceFilename + "\".");
    }
    const char* appendedDataEncoded = appendedDataNode->GetText();

    size_t totalStringLength = strlen(appendedDataEncoded);
    ptrdiff_t startPos = 0;
    ptrdiff_t endPos = ptrdiff_t(totalStringLength) - 1;
    while (startPos < ptrdiff_t(totalStringLength)) {
        char c = appendedDataEncoded[startPos];
        if (c == '_' || c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            startPos++;
        } else {
            break;
        }
    }
    while (endPos >= 0) {
        char c = appendedDataEncoded[endPos];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            endPos--;
        } else {
            break;
        }
    }

    auto* uField = new float[numPoints];
    auto* vField = new float[numPoints];
    auto* wField = new float[numPoints];

    float* scalarFields[3] = { uField, vField, wField };

    if (useZlib) {
        for (int i = 0; i < 3; i++) {
            const char* headerBase64String = appendedDataEncoded + startPos + velocityOffsets[i];

            uint32_t numBlocks = base64DecodeUint32(headerBase64String);
            size_t headerSizeBytes = sizeof(uint32_t) * size_t(numBlocks + 3);
            size_t headerSizeBase64 = (headerSizeBytes + 2) / 3 * 4;
            auto* encodedDataHeader = new char[base64GetNumBytesDecoded(int(headerSizeBase64))];
            size_t decodedHeaderDataSizeInBytes = base64DecodeSized(
                    encodedDataHeader, headerBase64String, int(headerSizeBase64));
            (void)decodedHeaderDataSizeInBytes;

            auto* header = reinterpret_cast<uint32_t *>(encodedDataHeader);
            //uint32_t numBlocks = header[0];
            uint32_t uncompressedBlockSize = header[1];
            uint32_t uncompressedLastBlockPartialSize = header[2];

            size_t bufferSize;
            if (uncompressedLastBlockPartialSize == 0) {
                bufferSize = size_t(numBlocks) * size_t(uncompressedBlockSize);
            } else {
                bufferSize =
                        size_t(numBlocks - 1) * size_t(uncompressedBlockSize) +
                        size_t(uncompressedLastBlockPartialSize);
            }

            if (bufferSize != sizeof(float) * numPoints) {
                sgl::Logfile::get()->throwError(
                        "Error in VtkXmlLoader::load: Invalid uncompressed buffer size in file \""
                        + dataSourceFilename + "\".");
            }

            size_t compressedTotalSize = 0;
            for (uint32_t blockIdx = 0; blockIdx < numBlocks; blockIdx++) {
                auto compressedSize = size_t(header[blockIdx + 3]);
                compressedTotalSize += compressedSize;
            }
            const char* dataBase64String = headerBase64String + headerSizeBase64;
            size_t dataSizeBase64 = (compressedTotalSize + 2) / 3 * 4;
            auto* encodedData = new char[base64GetNumBytesDecoded(int(dataSizeBase64))];
            size_t decodedDataSizeInBytes = base64DecodeSized(
                    encodedData, dataBase64String, int(dataSizeBase64));
            (void)decodedDataSizeInBytes;

            auto* compressedDataReadPtr = reinterpret_cast<uint8_t*>(encodedData);
            auto* uncompressedDataWritePtr = reinterpret_cast<uint8_t*>(scalarFields[i]);
            for (uint32_t blockIdx = 0; blockIdx < numBlocks; blockIdx++) {
                auto compressedSize = size_t(header[blockIdx + 3]);
                size_t uncompressedSize;
                if (blockIdx == numBlocks - 1 && uncompressedLastBlockPartialSize != 0) {
                    uncompressedSize = uncompressedLastBlockPartialSize;
                } else {
                    uncompressedSize = uncompressedBlockSize;
                }
                sgl::decompressZlibData(
                        compressedDataReadPtr, compressedSize,
                        uncompressedDataWritePtr, uncompressedSize);
                compressedDataReadPtr += compressedSize;
                uncompressedDataWritePtr += uncompressedSize;
            }

            delete[] encodedData;
            delete[] encodedDataHeader;
        }
    } else {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Uncompressed appended data as used in file \""
                + dataSourceFilename + "\" is currently not yet supported.");
    }

    if (!isLittleEndian) {
        sgl::Logfile::get()->throwError(
                "Error in VtkXmlLoader::load: Big endian encoding used in file \""
                + dataSourceFilename + "\" is not yet supported.");
    }
    auto* velocityField = new float[3 * numPoints];
    for (int ptIdx = 0; ptIdx < numPoints; ptIdx++) {
        velocityField[3 * ptIdx + 0] = uField[ptIdx];
        velocityField[3 * ptIdx + 1] = vField[ptIdx];
        velocityField[3 * ptIdx + 2] = wField[ptIdx];
    }


    // --- Filling grid data ---
    float maxDimension = float(std::max(xs - 1, std::max(ys - 1, zs - 1)));
    float cellStep = 1.0f / maxDimension;
    float maxSpacing = std::max(spacingArray.at(0), std::max(spacingArray.at(1), spacingArray.at(2)));
    float dx = cellStep * spacingArray.at(0) / maxSpacing;
    float dy = cellStep * spacingArray.at(1) / maxSpacing;
    float dz = cellStep * spacingArray.at(2) / maxSpacing;
    grid->setGridExtent(xs, ys, zs, dx, dy, dz);

    auto* velocityMagnitudeField = new float[numPoints];
    computeVectorMagnitudeField(velocityField, velocityMagnitudeField, xs, ys, zs);

    auto* vorticityField = new float[numPoints * 3];
    computeVorticityField(velocityField, vorticityField, xs, ys, zs, dx, dy, dz);

    auto* vorticityMagnitudeField = new float[numPoints];
    computeVectorMagnitudeField(vorticityField, vorticityMagnitudeField, xs, ys, zs);

    auto* helicityField = new float[numPoints];
    computeHelicityField(velocityField, vorticityField, helicityField, xs, ys, zs);

    grid->addVectorField(velocityField, "Velocity");
    grid->addScalarField(velocityMagnitudeField, "Velocity Magnitude");
    grid->addVectorField(vorticityField, "Vorticity");
    grid->addScalarField(vorticityMagnitudeField, "Vorticity Magnitude");
    grid->addScalarField(helicityField, "Helicity");

    grid->addScalarField(uField, "u");
    grid->addScalarField(vField, "v");
    grid->addScalarField(wField, "w");
}
