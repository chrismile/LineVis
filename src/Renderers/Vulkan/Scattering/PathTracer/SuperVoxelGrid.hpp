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

#ifndef LINEVIS_SUPERVOXELGRID_HPP
#define LINEVIS_SUPERVOXELGRID_HPP

#include <glm/vec3.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include <Graphics/Vulkan/Image/Image.hpp>

/**
 * Super voxel used in super voxel grids for residual ratio tracking.
 *
 * Paper on residual ratio tracking:
 * J. Novák, A. Selle, and W. Jarosz. Residual ratio tracking for estimating attenuation in participating media.
 * ACM Transactions on Graphics (Proceedings of SIGGRAPH Asia) , 33(6), Nov. 2014.
 *
 * Super voxels were first introduced in:
 * L. Szirmay-Kalos, B. Tóth, and M. Magdics. Free path sampling in high resolution inhomogeneous participating media.
 * Computer Graphics Forum, 30, 2011.
 */
struct SuperVoxelResidualRatioTracking {
    float mu_c;
    float mu_r_bar;
};

class SuperVoxelGridResidualRatioTracking {
public:
    SuperVoxelGridResidualRatioTracking(
            sgl::vk::Device* device, int voxelGridSizeX, int voxelGridSizeY, int voxelGridSizeZ,
            const float* voxelGridData, int superVoxelSize1D, bool clampToZeroBorder);
    ~SuperVoxelGridResidualRatioTracking();

    [[nodiscard]] inline const glm::ivec3& getSuperVoxelSize() const { return superVoxelSize; }
    [[nodiscard]] inline glm::ivec3 getSuperVoxelGridSize() const {
        return {superVoxelGridSizeX, superVoxelGridSizeY, superVoxelGridSizeZ};
    }
    inline const sgl::vk::TexturePtr& getSuperVoxelGridTexture() { return superVoxelGridTexture; }
    inline const sgl::vk::TexturePtr& getSuperVoxelGridOccupancyTexture() { return superVoxelGridOccupancyTexture; }

    void setExtinction(float extinction);
    void recomputeSuperVoxels();

private:
    void computeSuperVoxels(const float* voxelGridData);

    glm::ivec3 superVoxelSize = glm::ivec3(8);

    int superVoxelGridSizeX = 0, superVoxelGridSizeY = 0, superVoxelGridSizeZ = 0;
    int voxelGridSizeX = 0, voxelGridSizeY = 0, voxelGridSizeZ = 0;
    bool clampToZeroBorder = true;

    float extinction = 1024.0f;
    float scatteringAlbedo = 1.0f;

    SuperVoxelResidualRatioTracking* superVoxelGrid;
    uint8_t* superVoxelGridOccupany;
    float* superVoxelGridMinDensity = nullptr;
    float* superVoxelGridMaxDensity = nullptr;
    float* superVoxelGridAvgDensity = nullptr;

    sgl::vk::TexturePtr superVoxelGridTexture;
    sgl::vk::TexturePtr superVoxelGridOccupancyTexture;
};

/**
 * Super voxel grid for decomposition tracking.
 *
 * For more details on decomposition tracking, please refer to:
 * P. Kutz, R. Habel, Y. K. Li, and J. Novák. Spectral and decomposition tracking for rendering heterogeneous volumes.
 * ACM Trans. Graph., 36(4), Jul. 2017.
 */
class SuperVoxelGridDecompositionTracking {
public:
    SuperVoxelGridDecompositionTracking(
            sgl::vk::Device* device, int voxelGridSizeX, int voxelGridSizeY, int voxelGridSizeZ,
            const float* voxelGridData, int superVoxelSize1D, bool clampToZeroBorder);
    ~SuperVoxelGridDecompositionTracking();

    [[nodiscard]] inline const glm::ivec3& getSuperVoxelSize() const { return superVoxelSize; }
    [[nodiscard]] inline glm::ivec3 getSuperVoxelGridSize() const {
        return {superVoxelGridSizeX, superVoxelGridSizeY, superVoxelGridSizeZ};
    }
    inline const sgl::vk::TexturePtr& getSuperVoxelGridTexture() { return superVoxelGridTexture; }
    inline const sgl::vk::TexturePtr& getSuperVoxelGridOccupancyTexture() { return superVoxelGridOccupancyTexture; }

private:
    glm::ivec3 superVoxelSize = glm::ivec3(8);

    int superVoxelGridSizeX = 0, superVoxelGridSizeY = 0, superVoxelGridSizeZ = 0;
    int voxelGridSizeX = 0, voxelGridSizeY = 0, voxelGridSizeZ = 0;
    bool clampToZeroBorder = true;

    uint8_t* superVoxelGridOccupany;
    glm::vec2* superVoxelGridMinMaxDensity = nullptr;

    sgl::vk::TexturePtr superVoxelGridTexture;
    sgl::vk::TexturePtr superVoxelGridOccupancyTexture;
};

#endif //LINEVIS_SUPERVOXELGRID_HPP
