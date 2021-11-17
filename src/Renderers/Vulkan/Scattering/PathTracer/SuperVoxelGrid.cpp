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

#include <glm/glm.hpp>
#include <Math/Math.hpp>
#include "SuperVoxelGrid.hpp"

SuperVoxelGrid::SuperVoxelGrid(
        sgl::vk::Device* device, int voxelGridSizeX, int voxelGridSizeY, int voxelGridSizeZ,
        const float* voxelGridData, float extinction, int superVoxelSize1D)
        : voxelGridSizeX(voxelGridSizeX), voxelGridSizeY(voxelGridSizeY), voxelGridSizeZ(voxelGridSizeZ) {
    superVoxelSize = glm::ivec3(superVoxelSize1D);

    superVoxelGridSizeX = sgl::iceil(voxelGridSizeX, superVoxelSize1D);
    superVoxelGridSizeY = sgl::iceil(voxelGridSizeY, superVoxelSize1D);
    superVoxelGridSizeZ = sgl::iceil(voxelGridSizeZ, superVoxelSize1D);
    int superVoxelGridSize = superVoxelGridSizeX * superVoxelGridSizeY * superVoxelGridSizeZ;

    superVoxelGrid = new SuperVoxel[superVoxelGridSize];
    superVoxelGridEmpty = new uint8_t[superVoxelGridSize];

    sgl::vk::ImageSettings imageSettings{};
    imageSettings.width = superVoxelGridSizeX;
    imageSettings.height = superVoxelGridSizeY;
    imageSettings.depth = superVoxelGridSizeZ;
    imageSettings.imageType = VK_IMAGE_TYPE_3D;
    imageSettings.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageSettings.format = VK_FORMAT_R32G32_SFLOAT;
    superVoxelGridTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings);
    imageSettings.format = VK_FORMAT_R8_UINT;
    superVoxelGridEmptyTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings);

    computeSuperVoxels(voxelGridData);
}

SuperVoxelGrid::~SuperVoxelGrid() {
    delete[] superVoxelGrid;
    delete[] superVoxelGridEmpty;
}

void SuperVoxelGrid::computeSuperVoxels(const float* voxelGridData) {
    int superVoxelGridSize = superVoxelGridSizeX * superVoxelGridSizeY * superVoxelGridSizeZ;

    const float gamma = 2.0f;
    const float D = std::sqrt(3.0f) * float(std::max(superVoxelSize.x, std::max(superVoxelSize.y, superVoxelSize.z)));

    #pragma omp parallel for shared(voxelGridData) firstprivate(superVoxelGridSize, D, gamma) default(none)
    for (int superVoxelIdx = 0; superVoxelIdx < superVoxelGridSize; superVoxelIdx++) {
        int superVoxelIdxX = superVoxelIdx % superVoxelGridSizeX;
        int superVoxelIdxY = (superVoxelIdx / superVoxelGridSizeX) % superVoxelGridSizeY;
        int superVoxelIdxZ = superVoxelIdx / (superVoxelGridSizeX * superVoxelGridSizeY);

        float densityMin = std::numeric_limits<float>::max();
        float densityMax = std::numeric_limits<float>::lowest();
        float densityAvg = 0.0f;
        int numValidVoxels = 0;

        for (int offsetZ = 0; offsetZ < superVoxelSize.z; offsetZ++) {
            for (int offsetY = 0; offsetY < superVoxelSize.y; offsetY++) {
                for (int offsetX = 0; offsetX < superVoxelSize.x; offsetX++) {
                    int voxelIdxX = superVoxelIdxX * superVoxelSize.x + offsetX;
                    int voxelIdxY = superVoxelIdxY * superVoxelSize.y + offsetY;
                    int voxelIdxZ = superVoxelIdxZ * superVoxelSize.z + offsetZ;

                    if (voxelIdxX >= 0 && voxelIdxY >= 0 && voxelIdxZ >= 0
                        && voxelIdxX < voxelGridSizeX && voxelIdxY < voxelGridSizeY && voxelIdxZ < voxelGridSizeZ) {
                        int voxelIdx = voxelIdxX + (voxelIdxY + voxelIdxZ * voxelGridSizeY) * voxelGridSizeX;
                        float value = voxelGridData[voxelIdx];
                        densityMin = std::min(densityMin, value);
                        densityMax = std::max(densityMax, value);
                        densityAvg += value;
                        numValidVoxels++;
                    }
                }
            }
        }
        densityAvg /= float(numValidVoxels);

        // TODO: Is this correct?
        float mu_min = densityMin;
        float mu_max = densityMax;
        float mu_avg = densityAvg;

        // Sec. 5.1 in paper by Novák et al. [2014].
        float mu_r_bar = std::max(mu_max - mu_min, 0.1f);
        float mu_c = mu_min + mu_r_bar * std::pow(gamma, (1.0f / (D * mu_r_bar)) - 1.0f);
        float mu_c_prime = glm::clamp(mu_c, mu_min, mu_avg);

        SuperVoxel& superVoxel = superVoxelGrid[superVoxelIdx];
        superVoxel.mu_c = mu_c_prime;
        superVoxel.mu_r_bar = mu_r_bar;

        bool isSuperVoxelEmpty = densityMax < 1e-5;
        superVoxelGridEmpty[superVoxelIdx] = isSuperVoxelEmpty ? 0 : 1;
    }

    superVoxelGridTexture->getImage()->uploadData(
            superVoxelGridSize * sizeof(SuperVoxel), superVoxelGrid);
    superVoxelGridEmptyTexture->getImage()->uploadData(
            superVoxelGridSize * sizeof(uint8_t), superVoxelGridEmpty);
}
