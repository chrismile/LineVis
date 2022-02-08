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

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Events/Stream/Stream.hpp>

#include "Renderers/Vulkan/Scattering/nanovdb/NanoVDB.h"
#include "Renderers/Vulkan/Scattering/nanovdb/util/GridBuilder.h"
#include "Renderers/Vulkan/Scattering/nanovdb/util/IO.h"

#include "CloudData.hpp"

CloudData::~CloudData() {
    if (densityField) {
        delete[] densityField;
        densityField = nullptr;
    }
    sparseGridHandle = {};
}

void CloudData::computeGridBounds() {
    uint32_t maxDim = std::max(gridSizeX, std::max(gridSizeY, gridSizeZ));
    boxMax = glm::vec3(gridSizeX, gridSizeY, gridSizeZ) * 0.25f / float(maxDim);
    boxMin = -boxMax;
}

void CloudData::setDensityField(uint32_t _gridSizeX, uint32_t _gridSizeY, uint32_t _gridSizeZ, float* _densityField) {
    if (densityField) {
        delete[] densityField;
        densityField = nullptr;
    }
    sparseGridHandle = {};

    gridSizeX = _gridSizeX;
    gridSizeY = _gridSizeY;
    gridSizeZ = _gridSizeZ;

    computeGridBounds();

    densityField = _densityField;
    gridFilename = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/clouds/tmp.xyz";
    gridName = "tmp";
}

void CloudData::setNanoVdbGridHandle(nanovdb::GridHandle<nanovdb::HostBuffer>&& handle) {
    sparseGridHandle = std::move(handle);
    computeSparseGridMetadata();
}

bool CloudData::loadFromFile(const std::string& filename) {
    if (!sgl::FileUtils::get()->exists(filename)) {
        sgl::Logfile::get()->writeError(
                "Error in CloudData::loadFromFile: The file \"" + filename + "\" does not exist!");
        return false;
    }

    gridFilename = filename;
    gridName = boost::to_lower_copy(sgl::FileUtils::get()->removeExtension(
            sgl::FileUtils::get()->getPureFilename(gridFilename)));

    if (densityField) {
        delete[] densityField;
        densityField = nullptr;
    }
    sparseGridHandle = {};

    if (sgl::FileUtils::get()->hasExtension(filename.c_str(), ".xyz")) {
        return loadFromXyzFile(filename);
    } else if (sgl::FileUtils::get()->hasExtension(filename.c_str(), ".nvdb")) {
        return loadFromNvdbFile(filename);
    } else {
        sgl::Logfile::get()->writeError(
                "Error in CloudData::loadFromFile: The file \"" + filename + "\" has an unknown extension!");
        return false;
    }
}

bool CloudData::loadFromXyzFile(const std::string& filename) {
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

    computeGridBounds();

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

float* CloudData::getDenseDensityField() {
    if (!hasDenseData()) {
        if (sparseGridHandle.empty()) {
            sgl::Logfile::get()->throwError(
                    "Fatal error in CloudData::getDenseDensityField: Neither a dense nor a sparse field are "
                    "loaded!");
            return nullptr;
        }

        auto* grid = sparseGridHandle.grid<float>();
        if (!grid) {
            sgl::Logfile::get()->throwError(
                    "Fatal error in CloudData::getDenseDensityField: The sparse grid data from \"" + gridFilename
                    + "\" does not contain floating point data!");
            return nullptr;
        }

        auto& tree = grid->tree();
        auto minGridVal = grid->indexBBox().min();
        densityField = new float[gridSizeX * gridSizeY * gridSizeZ];
        for (uint32_t z = 0; z < gridSizeZ; z++) {
            for (uint32_t y = 0; y < gridSizeY; y++) {
                for (uint32_t x = 0; x < gridSizeX; x++) {
                    densityField[x + y * gridSizeX + z * gridSizeX * gridSizeY] = tree.getValue(nanovdb::Coord(
                            minGridVal[0] + int(x), minGridVal[1] + int(y), minGridVal[2] + int(z)));
                }
            }
        }
    }

    return densityField;
}


void CloudData::printSparseGridMetadata() {
    double denseGridSizeMiB = gridSizeX * gridSizeY * gridSizeZ * 4 / (1024.0 * 1024.0);
    double sparseGridSizeMiB = sparseGridHandle.gridMetaData()->gridSize() / (1024.0 * 1024.0);
    double compressionRatio = denseGridSizeMiB / sparseGridSizeMiB;
    sgl::Logfile::get()->writeInfo("Dense grid memory (MiB): " + std::to_string(denseGridSizeMiB));
    sgl::Logfile::get()->writeInfo("Sparse grid memory (MiB): " + std::to_string(sparseGridSizeMiB));
    sgl::Logfile::get()->writeInfo("Compression ratio: " + std::to_string(compressionRatio));
    sgl::Logfile::get()->writeInfo(
            "Total number of voxels: " + std::to_string(gridSizeX * gridSizeY * gridSizeZ));
    sgl::Logfile::get()->writeInfo(
            "Number of active voxels: " + std::to_string(sparseGridHandle.gridMetaData()->activeVoxelCount()));
    for (int i = 0; i < 3; i++) {
        sgl::Logfile::get()->writeInfo(
                "Nodes at level " + std::to_string(i) + ": "
                + std::to_string(sparseGridHandle.gridMetaData()->nodeCount(i)));
    }
}

void CloudData::computeSparseGridMetadata() {
    const auto* grid = sparseGridHandle.grid<float>();

    if (!grid) {
        sgl::Logfile::get()->throwError(
                "Fatal error in CloudData::computeSparseGridMetadata: The grid handle does not store a grid "
                "with value type float.");
    }

    gridSizeX = uint32_t(grid->indexBBox().max()[0] - grid->indexBBox().min()[0] + 1);
    gridSizeY = uint32_t(grid->indexBBox().max()[1] - grid->indexBBox().min()[1] + 1);
    gridSizeZ = uint32_t(grid->indexBBox().max()[2] - grid->indexBBox().min()[2] + 1);
    voxelSizeX = float(grid->voxelSize()[0]);
    voxelSizeY = float(grid->voxelSize()[1]);
    voxelSizeZ = float(grid->voxelSize()[2]);

    auto nanoVdbBoundingBox = grid->worldBBox();
    boxMin = glm::vec3(
            float(nanoVdbBoundingBox.min()[0]),
            float(nanoVdbBoundingBox.min()[1]),
            float(nanoVdbBoundingBox.min()[2]));
    boxMax = glm::vec3(
            float(nanoVdbBoundingBox.max()[0]),
            float(nanoVdbBoundingBox.max()[1]),
            float(nanoVdbBoundingBox.max()[2]));

    printSparseGridMetadata();
}

bool CloudData::loadFromNvdbFile(const std::string& filename) {
    sparseGridHandle = nanovdb::io::readGrid<nanovdb::HostBuffer>(filename, gridName);
    computeSparseGridMetadata();
    return !sparseGridHandle.empty();
}

void CloudData::getSparseDensityField(uint8_t*& data, uint64_t& size) {
    if (!hasSparseData()) {
        if (!densityField) {
            sgl::Logfile::get()->throwError(
                    "Fatal error in CloudData::getSparseDensityField: Neither a dense nor a sparse field are "
                    "loaded!");
            return;
        }

        std::string filenameNvdb = sgl::FileUtils::get()->removeExtension(gridFilename) + ".nvdb";
        if (cacheSparseGrid && sgl::FileUtils::get()->exists(filenameNvdb)) {
            bool isLoaded = loadFromNvdbFile(filenameNvdb);
            if (!isLoaded) {
                sgl::Logfile::get()->throwError(
                        "Error in CloudData::getSparseDensityField: Couldn't load data from grid data set file \""
                        + filenameNvdb + "\".");
            }
        } else {
            nanovdb::GridBuilder builder(0.0f);
            auto gridSamplingOperation = [this](const nanovdb::Coord& ijk) -> float {
                auto x = uint32_t(ijk.x());
                auto y = uint32_t(ijk.y());
                auto z = uint32_t(ijk.z());
                return densityField[x + (y + z * gridSizeY) * gridSizeX];
            };
            auto maxIdxX = int32_t(gridSizeX - 1);
            auto maxIdxY = int32_t(gridSizeY - 1);
            auto maxIdxZ = int32_t(gridSizeZ - 1);
            builder(gridSamplingOperation, nanovdb::CoordBBox(
                    nanovdb::Coord(0), nanovdb::Coord(maxIdxX, maxIdxY, maxIdxZ)));
            double dx = double(boxMax.x - boxMin.x) / double(gridSizeX);
            sparseGridHandle = builder.getHandle<>(
                    dx, nanovdb::Vec3d(boxMin.x, boxMin.y, boxMin.z),
                    gridName, nanovdb::GridClass::FogVolume);
            printSparseGridMetadata();

            /*auto* gridData = sparseGridHandle.grid<float>();
            const auto& rootNode = gridData->tree().getFirstNode<2>();
            std::cout << "Root min: " << rootNode->minimum() << std::endl;
            std::cout << "Root max: " << rootNode->maximum() << std::endl;*/

            if (cacheSparseGrid) {
                auto* gridData = sparseGridHandle.grid<float>();
                if (!gridData) {
                    sgl::Logfile::get()->throwError(
                            "Fatal error in CloudData::getSparseDensityField: The grid handle does not store a grid "
                            "with value type float.");
                }

                try {
                    nanovdb::io::writeGrid<nanovdb::HostBuffer>(filenameNvdb, sparseGridHandle);
                } catch (const std::exception& e) {
                    sgl::Logfile::get()->throwError(e.what());
                }
            }
        }
    }

    auto& buffer = sparseGridHandle.buffer();
    data = buffer.data();
    size = buffer.size();
}
