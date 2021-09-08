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

#include <Math/Math.hpp>

#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include <Graphics/Vulkan/Utils/SyncObjects.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#endif

#include "LineDataScattering.hpp"

LineDataScattering::LineDataScattering(
        sgl::TransferFunctionWindow& transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
        , sgl::vk::Renderer* rendererVk
#endif
        ) : LineDataFlow(transferFunctionWindow)
#ifdef USE_VULKAN_INTEROP
        , rendererVk(rendererVk)
#endif
{
    dataSetType = DATA_SET_TYPE_SCATTERING_LINES;
    lineDataWindowName = "Line Data (Scattering)";

    lineDensityFieldImageComputeRenderPass = std::make_shared<LineDensityFieldImageComputeRenderPass>(rendererVk);
}

LineDataScattering::~LineDataScattering() {
    if (scalarField) {
        delete[] scalarField;
    }
}

void LineDataScattering::setDataSetInformation(
        const std::string& dataSetName, const std::vector<std::string>& attributeNames) {
    this->fileNames = { dataSetName };
    this->attributeNames = attributeNames;
}

void LineDataScattering::setGridData(
        float* scalarField, uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ,
        float voxelSizeX, float voxelSizeY, float voxelSizeZ) {
    this->scalarField = scalarField;
    this->gridSizeX = gridSizeX;
    this->gridSizeY = gridSizeY;
    this->gridSizeZ = gridSizeZ;
    this->voxelSizeX = voxelSizeX;
    this->voxelSizeY = voxelSizeY;
    this->voxelSizeZ = voxelSizeZ;

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = gridSizeX;
    imageSettings.height = gridSizeY;
    imageSettings.depth = gridSizeZ;
    imageSettings.imageType = VK_IMAGE_TYPE_3D;
    imageSettings.format = VK_FORMAT_R32_SFLOAT;

    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
    vulkanScatteredLinesGridRenderData.scalarFieldTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, samplerSettings);
    imageSettings.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    vulkanScatteredLinesGridRenderData.lineDensityFieldTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, samplerSettings);

    lineDensityFieldImageComputeRenderPass->setData(this, vulkanScatteredLinesGridRenderData.lineDensityFieldTexture);
}

void LineDataScattering::rebuildInternalRepresentationIfNecessary() {
    if (dirty || triangleRepresentationDirty) {
        isLineDensityFieldDirty = true;
    }
    LineData::rebuildInternalRepresentationIfNecessary();
}

#ifdef USE_VULKAN_INTEROP
VulkanLineDataScatteringRenderData LineDataScattering::getVulkanLineDataScatteringRenderData() {
    rebuildInternalRepresentationIfNecessary();
    if (!isLineDensityFieldDirty) {
        return vulkanScatteredLinesGridRenderData;
    }

    lineDensityFieldImageComputeRenderPass->setDataDirty();
    rendererVk->insertImageMemoryBarrier(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_NONE_KHR, VK_ACCESS_TRANSFER_WRITE_BIT);
    vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImageView()->clearColor(
            glm::vec4(0.0f), rendererVk->getVkCommandBuffer());
    rendererVk->insertImageMemoryBarrier(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage(),
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_WRITE_BIT);
    lineDensityFieldImageComputeRenderPass->render();
    rendererVk->insertImageMemoryBarrier(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage(),
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);

    isLineDensityFieldDirty = false;
    return vulkanScatteredLinesGridRenderData;
}



LineDensityFieldImageComputeRenderPass::LineDensityFieldImageComputeRenderPass(sgl::vk::Renderer* renderer)
        : ComputePass(renderer) {
    uniformBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void LineDensityFieldImageComputeRenderPass::setData(
        LineDataScattering* lineData, sgl::vk::TexturePtr& lineDensityFieldTexture) {
    this->lineData = lineData;
    this->lineDensityFieldTexture = lineDensityFieldTexture;

    gridSizeX = lineData->getGridSizeX();
    gridSizeY = lineData->getGridSizeY();
    gridSizeZ = lineData->getGridSizeZ();

    uniformData.gridResolution = glm::ivec3(gridSizeX, gridSizeY, gridSizeZ);

    spinlockBuffer = std::make_shared<sgl::vk::Buffer>(
            device, gridSizeX * gridSizeY * gridSizeZ * sizeof(float),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    VkCommandBuffer commandBuffer = device->beginSingleTimeCommands();
    spinlockBuffer->fill(0, commandBuffer);
    device->endSingleTimeCommands(commandBuffer);
}

void LineDensityFieldImageComputeRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"ComputeLineDensityField.Compute"});
}

void LineDensityFieldImageComputeRenderPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    Trajectories trajectories = lineData->filterTrajectoryData();
    int selectedAttributeIndex = lineData->getSelectedAttributeIndex();

    std::vector<LinePoint> linePoints;
    std::vector<uint32_t> lineOffsets;
    lineOffsets.push_back(0);
    uint32_t offsetCounter = 0;
    for (Trajectory& trajectory : trajectories) {
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            linePoints.emplace_back(trajectory.positions.at(i), trajectory.attributes.at(selectedAttributeIndex).at(i));
        }
        offsetCounter += trajectory.positions.size();
        lineOffsets.push_back(offsetCounter);
    }
    sgl::vk::BufferPtr linePointBuffer = std::make_shared<sgl::vk::Buffer>(
            device, linePoints.size() * sizeof(LinePoint), linePoints.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    sgl::vk::BufferPtr lineOffsetBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineOffsets.size() * sizeof(uint32_t), lineOffsets.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    uniformData.numLines = int(trajectories.size());

    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(uniformBuffer, "UniformBuffer");
    computeData->setStaticBuffer(linePointBuffer, "LinePointBuffer");
    computeData->setStaticBuffer(lineOffsetBuffer, "LineOffsetBuffer");
    computeData->setStaticBuffer(spinlockBuffer, "SpinlockBuffer");
    computeData->setStaticImageView(lineDensityFieldTexture->getImageView(), "lineDensityFieldImage");
    lineData->setVulkanRenderDataDescriptors(computeData);
}

void LineDensityFieldImageComputeRenderPass::_render() {
    uniformBuffer->updateData(sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    uint32_t numWorkGroupsLines = sgl::iceil(uniformData.numLines, 256);
    renderer->dispatch(computeData, numWorkGroupsLines, 1, 1);
}
#endif
