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

#ifndef LINEVIS_CLOUDDATA_HPP
#define LINEVIS_CLOUDDATA_HPP

#include <memory>
#include <Math/Geometry/AABB3.hpp>
#include <Graphics/Color.hpp>

#include "Renderers/Vulkan/Scattering/nanovdb/util/GridHandle.h"

class CloudData {
public:
    ~CloudData();

    /**
     * @param filename The filename of the .xyz or .nvdb file to load.
     * @return Whether the file was loaded successfully.
     */
    bool loadFromFile(const std::string& filename);

    /**
     * @param _gridSizeX The number of voxels in x direction.
     * @param _gridSizeY The number of voxels in y direction.
     * @param _gridSizeZ The number of voxels in z direction.
     * @param _densityField A dense floating point density field of size gridSizeX*gridSizeY*gridSizeZ.
     */
    void setDensityField(uint32_t _gridSizeX, uint32_t _gridSizeY, uint32_t _gridSizeZ, float* _densityField);

    /**
     * Sets the passed grid handle.
     * @param handle
     */
    void setNanoVdbGridHandle(nanovdb::GridHandle<nanovdb::HostBuffer>&& handle);

    [[nodiscard]] inline const std::string& getFileName() const { return gridFilename; }
    [[nodiscard]] inline uint32_t getGridSizeX() const { return gridSizeX; }
    [[nodiscard]] inline uint32_t getGridSizeY() const { return gridSizeY; }
    [[nodiscard]] inline uint32_t getGridSizeZ() const { return gridSizeZ; }

    [[nodiscard]] inline const glm::vec3& getWorldSpaceBoxMin() const { return boxMin; }
    [[nodiscard]] inline const glm::vec3& getWorldSpaceBoxMax() const { return boxMax; }
    [[nodiscard]] inline sgl::AABB3 getWorldSpaceBoundingBox() const { return sgl::AABB3(boxMin, boxMax); }

    void setClearColor(const sgl::Color& clearColor) {}

    /**
     * @return An array of size gridSizeX * gridSizeY * gridSizeZ containing the dense data field.
     * If the object was loaded using a .nvdb file, the dense field is created when calling this function.
     */
    float* getDenseDensityField();
    [[nodiscard]] inline bool hasDenseData() const { return densityField != nullptr; }

    /**
     * @param data A pointer to the raw NanoVDB data.
     * @param size The size of the NanoVDB data buffer in bytes.
     * If the object was loaded using a dense .xyz grid file, the sparse field is created when calling this function.
     */
    void getSparseDensityField(uint8_t*& data, uint64_t& size);
    [[nodiscard]] inline bool hasSparseData() const { return !sparseGridHandle.empty(); }
    inline void setCacheSparseGrid(bool cache) { cacheSparseGrid = true; }

private:
    std::string gridFilename, gridName;
    uint32_t gridSizeX = 0, gridSizeY = 0, gridSizeZ = 0;
    float voxelSizeX = 0.0f, voxelSizeY = 0.0f, voxelSizeZ = 0.0f;
    glm::vec3 boxMin{}, boxMax{};
    void computeGridBounds();

    // --- Dense field. ---
    /**
     * @param filename The filename of the .xyz file to load.
     * @return Whether the file was loaded successfully.
     */
    bool loadFromXyzFile(const std::string& filename);
    float* densityField = nullptr;

    // --- Sparse field. ---
    /**
     * @param filename The filename of the .nvdb file to load using NanoVDB.
     * @return Whether the file was loaded successfully.
     */
    bool loadFromNvdbFile(const std::string& filename);
    void computeSparseGridMetadata();
    void printSparseGridMetadata();
    nanovdb::GridHandle<nanovdb::HostBuffer> sparseGridHandle;
    bool cacheSparseGrid = false;
};

typedef std::shared_ptr<CloudData> CloudDataPtr;

#endif //LINEVIS_CLOUDDATA_HPP
