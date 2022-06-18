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

#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/StringUtils.hpp>
#include <Utils/Convert.hpp>

#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
#include "AmiraMeshLoader.hpp"

char* AmiraMeshLoader::skipLine(const std::string& dataSourceFilename, char* fileBuffer, char* fileBufferEnd) {
    char c;
    while (true) {
        if (fileBuffer == fileBufferEnd) {
            sgl::Logfile::get()->throwError(
                    "Error in AmiraMeshLoader::skipLine: Reached end of string in file  \""
                    + dataSourceFilename + "\".");
        }
        c = *fileBuffer;
        if (c == '\r' || c == '\n') {
            do {
                fileBuffer++;
                c = *fileBuffer;
            } while ((c == '\r' || c == '\n') && fileBuffer != fileBufferEnd);
            break;
        }
        fileBuffer++;
    }
    return fileBuffer;
}

void AmiraMeshLoader::load(const std::string& dataSourceFilename, StreamlineTracingGrid* grid) {
    uint8_t* buffer = nullptr;
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(dataSourceFilename, buffer, length, false);
    if (!loaded) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Couldn't open file \"" + dataSourceFilename + "\".");
    }
    char* fileBuffer = reinterpret_cast<char*>(buffer);

    if (!strstr(fileBuffer, "# AmiraMesh BINARY-LITTLE-ENDIAN 2.1")) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Missing AmiraMesh header in file \"" + dataSourceFilename + "\".");
    }

    char* defineLatticeLine = strstr(fileBuffer, "define Lattice ");
    defineLatticeLine += strlen("define Lattice ");
    std::vector<int> latticeDimensions;
    std::string stringBuffer;
    while (true) {
        char c = *defineLatticeLine;
        if (c == ' ' || c == '\t') {
            if (stringBuffer.length() > 0) {
                latticeDimensions.push_back(sgl::fromString<int>(stringBuffer));
                stringBuffer = "";
            }
        } else if (c == '\r' || c == '\n') {
            break;
        } else {
            stringBuffer += c;
        }
        defineLatticeLine++;
    }
    if (stringBuffer.length() > 0) {
        latticeDimensions.push_back(sgl::fromString<int>(stringBuffer));
        stringBuffer = "";
    }
    if (latticeDimensions.size() != 3) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Lattice definition in file \""
                + dataSourceFilename + "\" does not contain 3 entries.");
    }
    int xs = latticeDimensions.at(0);
    int ys = latticeDimensions.at(1);
    int zs = latticeDimensions.at(2);
    int numPoints = xs * ys * zs;

    char* parametersLine = strstr(fileBuffer, "Parameters {");
    if (!parametersLine) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Lattice definition in file \""
                + dataSourceFilename + "\" does not contain 3 entries.");
    }
    char* boundingBoxLine = strstr(parametersLine, "BoundingBox ");
    char* boundingBoxLineStart = boundingBoxLine + strlen("BoundingBox ");
    char* boundingBoxLineStop = strstr(boundingBoxLineStart, ",");
    if (!boundingBoxLineStop) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Misformed BoundingBox statement in file \""
                + dataSourceFilename + "\". misses a comma");
    }
    std::string boundingBoxString(boundingBoxLineStart, boundingBoxLineStop);
    std::vector<std::string> boundingBoxStringList;
    sgl::splitStringWhitespace(boundingBoxString, boundingBoxStringList);
    if (boundingBoxStringList.size() != 6) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Misformed BoundingBox statement in file \""
                + dataSourceFilename + "\" does not contain 6 entries.");
    }
    float xmin = sgl::fromString<float>(boundingBoxStringList.at(0));
    float xmax = sgl::fromString<float>(boundingBoxStringList.at(1));
    float ymin = sgl::fromString<float>(boundingBoxStringList.at(2));
    float ymax = sgl::fromString<float>(boundingBoxStringList.at(3));
    float zmin = sgl::fromString<float>(boundingBoxStringList.at(4));
    float zmax = sgl::fromString<float>(boundingBoxStringList.at(5));
    float bbDimX = xmax - xmin;
    float bbDimY = ymax - ymin;
    float bbDimZ = zmax - zmin;

    char* coordTypeLine = strstr(boundingBoxLine, "CoordType \"");
    char* coordTypeLineStart = coordTypeLine + strlen("CoordType \"");
    char* coordTypeLineEnd = strstr(coordTypeLineStart, "\"");
    std::string coordTypeString(coordTypeLineStart, coordTypeLineEnd);
    if (coordTypeString != "uniform") {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Unsupported coordinate type in file \""
                + dataSourceFilename + "\".");
    }

    char* latticeLine = strstr(fileBuffer, "Lattice { float[");
    if (!latticeLine) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Missing 3D lattice definition in file \""
                + dataSourceFilename + "\".");
    }
    char* latticeLineArrayStart = latticeLine + strlen("Lattice { float[");
    char* latticeLineArrayEnd = strstr(latticeLineArrayStart, "] Data");
     if (!latticeLineArrayEnd) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Misformed 3D lattice definition in file \""
                + dataSourceFilename + "\".");
    }
    std::string numDimensionsString(latticeLineArrayStart, latticeLineArrayEnd);
    if (sgl::fromString<int>(numDimensionsString) != 3) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Lattice dimension in file \""
                + dataSourceFilename + "\" is not equal to 3.");
    }

    char* dataSectionStart = strstr(fileBuffer, "# Data section follows");
    if (!dataSectionStart) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Missing data section in file \"" + dataSourceFilename + "\".");
    }
    dataSectionStart = skipLine(dataSourceFilename, dataSectionStart, fileBuffer + length);
    dataSectionStart = skipLine(dataSourceFilename, dataSectionStart, fileBuffer + length);

    if (sizeof(float) * 3 * numPoints > size_t(fileBuffer + length - dataSectionStart)) {
        sgl::Logfile::get()->throwError(
                "Error in AmiraMeshLoader::load: Invalid data section size in file \"" + dataSourceFilename + "\".");
    }

    autocd* velocityField = new float[3 * numPoints];
    memcpy(velocityField, dataSectionStart, sizeof(float) * 3 * numPoints);

    float maxDimension = float(std::max(xs - 1, std::max(ys - 1, zs - 1)));
    float cellStep = 1.0f / maxDimension;
    float maxBbDim = std::max(bbDimX, std::max(bbDimY, bbDimZ));
    float dx = cellStep * bbDimX / maxBbDim;
    float dy = cellStep * bbDimY / maxBbDim;
    float dz = cellStep * bbDimZ / maxBbDim;
    grid->setGridMetadata(xs, ys, zs, dx, dy, dz);

    auto* velocityMagnitudeField = new float[numPoints];
    auto* vorticityField = new float[numPoints * 3];
    auto* vorticityMagnitudeField = new float[numPoints];
    auto* helicityField = new float[numPoints];

    computeVectorMagnitudeField(velocityField, velocityMagnitudeField, xs, ys, zs);
    computeVorticityField(velocityField, vorticityField, xs, ys, zs, cellStep, cellStep, cellStep);
    computeVectorMagnitudeField(vorticityField, vorticityMagnitudeField, xs, ys, zs);
    computeHelicityField(velocityField, vorticityField, helicityField, xs, ys, zs);

    grid->setGridMetadata(xs, ys, zs, cellStep, cellStep, cellStep);
    grid->addVectorField(velocityField, "Velocity");
    grid->addVectorField(vorticityField, "Vorticity");
    grid->addScalarField(helicityField, "Helicity");
    grid->addScalarField(velocityMagnitudeField, "Velocity Magnitude");
    grid->addScalarField(vorticityMagnitudeField, "Vorticity Magnitude");

    delete[] buffer;
    buffer = nullptr;
}
