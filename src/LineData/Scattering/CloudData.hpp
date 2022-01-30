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
#include <Graphics/Color.hpp>

class CloudData {
public:
    ~CloudData();
    bool loadFromFile(const std::string& filename);
    void setDensityField(uint32_t _gridSizeX, uint32_t _gridSizeY, uint32_t _gridSizeZ, float* _densityField);

    [[nodiscard]] inline const std::string& getFileName() const { return gridFilename; }
    [[nodiscard]] inline float* getDensityField() const { return densityField; }
    [[nodiscard]] inline uint32_t getGridSizeX() const { return gridSizeX; }
    [[nodiscard]] inline uint32_t getGridSizeY() const { return gridSizeY; }
    [[nodiscard]] inline uint32_t getGridSizeZ() const { return gridSizeZ; }

    void setClearColor(const sgl::Color& clearColor) {}

private:
    std::string gridFilename;
    float* densityField = nullptr;
    uint32_t gridSizeX = 0, gridSizeY = 0, gridSizeZ = 0;
    float voxelSizeX = 1.0f, voxelSizeY = 1.0f, voxelSizeZ = 1.0f;
};

typedef std::shared_ptr<CloudData> CloudDataPtr;

#endif //LINEVIS_CLOUDDATA_HPP
