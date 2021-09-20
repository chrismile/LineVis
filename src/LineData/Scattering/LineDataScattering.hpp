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

#ifndef LINEVIS_LINEDATASCATTERING_HPP
#define LINEVIS_LINEDATASCATTERING_HPP

#include "../LineDataFlow.hpp"

#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Render/Passes/Pass.hpp>

namespace sgl { namespace vk {
class Renderer;
class Fence;
typedef std::shared_ptr<Fence> FencePtr;
}}

struct VulkanLineDataScatteringRenderData {
    sgl::vk::TexturePtr lineDensityFieldTexture;
    sgl::vk::TexturePtr scalarFieldTexture;
};

class LineDensityFieldImageComputeRenderPass;
class LineDensityFieldMinMaxReduceRenderPass;
class LineDensityFieldNormalizeRenderPass;
#endif

/**
 * The scattering line data object combines three types of data:
 * - The lines representing the paths of rays as they are crossing a volumetric domain and are scattered.
 * - An array (representing a regular grid) storing the densities of a volumetric object.
 * - An array storing the density of lines in each grid cell.
 */
class LineDataScattering : public LineDataFlow {
public:
    LineDataScattering(
            sgl::TransferFunctionWindow& transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
            , sgl::vk::Renderer* rendererVk
#endif
    );
    ~LineDataScattering() override;

    inline uint32_t getGridSizeX() const { return gridSizeX; }
    inline uint32_t getGridSizeY() const { return gridSizeY; }
    inline uint32_t getGridSizeZ() const { return gridSizeZ; }
    inline float getVoxelSizeX() const { return voxelSizeX; }
    inline float getVoxelSizeY() const { return voxelSizeY; }
    inline float getVoxelSizeZ() const { return voxelSizeZ; }

    void setDataSetInformation(const std::string& dataSetName, const std::vector<std::string>& attributeNames);
    void setGridData(
            float* scalarField, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ,
            float voxelSizeX, float voxelSizeY, float voxelSizeZ);

    void rebuildInternalRepresentationIfNecessary() override;

#ifdef USE_VULKAN_INTEROP
    VulkanLineDataScatteringRenderData getVulkanLineDataScatteringRenderData();
#endif

private:
    uint32_t gridSizeX = 0, gridSizeY = 0, gridSizeZ = 0;
    float voxelSizeX = 0.0f, voxelSizeY = 0.0f, voxelSizeZ = 0.0f;
    float* scalarField = nullptr;

#ifdef USE_VULKAN_INTEROP
    // Caches the rendering data when using Vulkan.
    VulkanLineDataScatteringRenderData vulkanScatteredLinesGridRenderData;
    std::shared_ptr<LineDensityFieldImageComputeRenderPass> lineDensityFieldImageComputeRenderPass;
    std::shared_ptr<LineDensityFieldMinMaxReduceRenderPass> lineDensityFieldMinMaxReduceRenderPass;
    std::shared_ptr<LineDensityFieldNormalizeRenderPass> lineDensityFieldNormalizeRenderPass;
    sgl::vk::BufferPtr lineDensityFieldBuffer;
    bool isLineDensityFieldDirty = false;

    sgl::vk::Renderer* rendererVk = nullptr;
#endif
};

#ifdef USE_VULKAN_INTEROP
class LineDensityFieldImageComputeRenderPass : public sgl::vk::ComputePass {
    friend class VulkanAmbientOcclusionBaker;
public:
    explicit LineDensityFieldImageComputeRenderPass(sgl::vk::Renderer* renderer);

    // Public interface.
    void setData(LineDataScattering* lineData, sgl::vk::TexturePtr& lineDensityFieldTexture);

private:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override {}
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    // Line data.
    LineDataScattering* lineData;
    uint32_t gridSizeX = 0, gridSizeY = 0, gridSizeZ = 0;
    std::vector<std::vector<glm::vec3>> lines;
    sgl::vk::TexturePtr lineDensityFieldTexture;
    sgl::vk::BufferPtr spinlockBuffer;

    struct LinePoint {
        LinePoint(glm::vec3 linePoint, float lineAttribute) : linePoint(linePoint), lineAttribute(lineAttribute) {}
        glm::vec3 linePoint;
        float lineAttribute;
    };

    // Uniform buffer object storing the line rendering settings.
    struct UniformData {
        glm::ivec3 gridResolution;
        int numLines;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformBuffer;
};

class LineDensityFieldMinMaxReduceRenderPass : public sgl::vk::ComputePass {
public:
    explicit LineDensityFieldMinMaxReduceRenderPass(sgl::vk::Renderer* renderer);

    // Public interface.
    void setLineDensityFieldImage(sgl::vk::ImagePtr& lineDensityFieldImage);
    inline sgl::vk::BufferPtr& getMinMaxBuffer() { return minMaxReductionBuffers[outputDepthMinMaxIndex]; }

private:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override {}
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    const int BLOCK_SIZE = 256;
    sgl::vk::ImagePtr lineDensityFieldImage;
    sgl::vk::BufferPtr minMaxReductionBuffers[2];
    sgl::vk::ComputeDataPtr computeDataPingPong[2];
    int outputDepthMinMaxIndex = 0;

    // Uniform buffer object storing the line rendering settings.
    struct UniformData {
        uint32_t sizeOfInput;
        uint32_t padding;
        float nearDist = std::numeric_limits<float>::lowest();
        float farDist = std::numeric_limits<float>::max();
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformBuffer;
};

class LineDensityFieldNormalizeRenderPass : public sgl::vk::ComputePass {
public:
    explicit LineDensityFieldNormalizeRenderPass(sgl::vk::Renderer* renderer);

    // Public interface.
    void setLineDensityFieldImageView(sgl::vk::ImageViewPtr& lineDensityFieldImageView);
    void setMinMaxBuffer(sgl::vk::BufferPtr& minMaxBuffer);

private:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override {}
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    uint32_t imageSize = 0;
    sgl::vk::ImageViewPtr lineDensityFieldImageView;
    sgl::vk::BufferPtr minMaxBuffer;

    // Uniform buffer object storing the line rendering settings.
    struct UniformData {
        glm::ivec3 gridResolution;
        int padding;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformBuffer;
};
#endif

#endif //LINEVIS_LINEDATASCATTERING_HPP
