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
#include <Utils/Events/Stream/Stream.hpp>

#include "CloudData.hpp"

CloudData::~CloudData() {
    if (densityField) {
        delete[] densityField;
        densityField = nullptr;
    }
}

bool CloudData::loadFromFile(const std::string& filename) {
    gridFilename = filename;

    uint8_t* fileBuffer = nullptr;
    size_t bufferSize = 0;
    bool loaded = sgl::loadFileFromSource(filename, fileBuffer, bufferSize, true);
    if (!loaded) {
        sgl::Logfile::get()->writeError(
                "Error in CloudData::loadFromFile: Couldn't load data from grid data set file \""
                + filename + "\".");
        return false;
    }
    sgl::BinaryReadStream binaryReadStream(fileBuffer, bufferSize);

    double voxelSizeXDouble = 0.0, voxelSizeYDouble = 0.0, voxelSizeZDouble = 0.0;
    binaryReadStream.read(gridSizeX);
    binaryReadStream.read(gridSizeY);
    binaryReadStream.read(gridSizeZ);
    binaryReadStream.read(voxelSizeXDouble);
    binaryReadStream.read(voxelSizeYDouble);
    binaryReadStream.read(voxelSizeZDouble);

    voxelSizeX = float(voxelSizeXDouble);
    voxelSizeY = float(voxelSizeYDouble);
    voxelSizeZ = float(voxelSizeZDouble);

    if (densityField) {
        delete[] densityField;
        densityField = nullptr;
    }
    densityField = new float[gridSizeX * gridSizeY * gridSizeZ];
    auto* densityFieldTransposed = new float[gridSizeX * gridSizeY * gridSizeZ];
    binaryReadStream.read(densityFieldTransposed, gridSizeX * gridSizeY * gridSizeZ * sizeof(float));

    // Transpose.
#if _OPENMP >= 201107
    #pragma omp parallel for shared(densityField, densityFieldTransposed, gridSizeX, gridSizeY, gridSizeZ) \
    default(none)
#endif
    for (uint32_t z = 0; z < gridSizeZ; z++) {
        for (uint32_t y = 0; y < gridSizeY; y++) {
            for (uint32_t x = 0; x < gridSizeX; x++) {
                densityField[x + (y + z * gridSizeY) * gridSizeX] =
                        densityFieldTransposed[z + (y + x * gridSizeY) * gridSizeZ];
            }
        }
    }
    delete[] densityFieldTransposed;

    float minVal = 0.0f;//std::numeric_limits<float>::max();
    float maxVal = std::numeric_limits<float>::lowest();

    size_t totalSize = gridSizeX * gridSizeY * gridSizeZ;
#if _OPENMP >= 201107
    #pragma omp parallel for default(none) shared(densityField, totalSize) reduction(min: minVal) reduction(max: maxVal)
#endif
    for (size_t i = 0; i < totalSize; i++) {
        float val = densityField[i];
        minVal = std::min(minVal, val);
        maxVal = std::max(maxVal, val);
    }

#if _OPENMP >= 201107
    #pragma omp parallel for default(none) shared(densityField, totalSize, minVal, maxVal)
#endif
    for (size_t i = 0; i < totalSize; i++) {
        densityField[i] = (densityField[i] - minVal) / (maxVal - minVal);
    }

    return true;
}

void CloudData::setDensityField(uint32_t _gridSizeX, uint32_t _gridSizeY, uint32_t _gridSizeZ, float* _densityField) {
    if (densityField) {
        delete[] densityField;
        densityField = nullptr;
    }
    gridSizeX = _gridSizeX;
    gridSizeY = _gridSizeY;
    gridSizeZ = _gridSizeZ;
    densityField = _densityField;
}
