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

#include <iostream>
#include <Math/Math.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include <Graphics/Vulkan/Utils/SyncObjects.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#endif

#include "Renderers/LineRenderer.hpp"
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

#ifdef USE_VULKAN_INTEROP
    lineDensityFieldImageComputeRenderPass = std::make_shared<LineDensityFieldImageComputeRenderPass>(rendererVk);
    lineDensityFieldMinMaxReduceRenderPass = std::make_shared<LineDensityFieldMinMaxReduceRenderPass>(rendererVk);
    lineDensityFieldNormalizeRenderPass = std::make_shared<LineDensityFieldNormalizeRenderPass>(rendererVk);
#endif
}

LineDataScattering::~LineDataScattering() {
}

sgl::ShaderProgramPtr LineDataScattering::reloadGatherShader() {
    recomputeHistogram();
    return LineDataFlow::reloadGatherShader();
}

void LineDataScattering::setDataSetInformation(
        const std::string& dataSetName, const std::vector<std::string>& attributeNames) {
    this->fileNames = { dataSetName };
    this->attributeNames = attributeNames;
}

void LineDataScattering::setGridData(
#ifdef USE_VULKAN_INTEROP
        const sgl::vk::TexturePtr& scalarFieldTexture,
#endif
        const std::vector<uint32_t>& outlineTriangleIndices,
        const std::vector<glm::vec3>& outlineVertexPositions, const std::vector<glm::vec3>& outlineVertexNormals,
        uint32_t gridSizeX, uint32_t gridSizeY, uint32_t gridSizeZ,
        float voxelSizeX, float voxelSizeY, float voxelSizeZ) {
    this->gridSizeX = gridSizeX;
    this->gridSizeY = gridSizeY;
    this->gridSizeZ = gridSizeZ;
    this->voxelSizeX = voxelSizeX;
    this->voxelSizeY = voxelSizeY;
    this->voxelSizeZ = voxelSizeZ;

    uint32_t maxDimSize = std::max(gridSizeX, std::max(gridSizeY, gridSizeZ));
    gridAabb.max = glm::vec3(gridSizeX, gridSizeY, gridSizeZ) * 0.25f / float(maxDimSize);
    gridAabb.min = -gridAabb.max;
    focusBoundingBox = gridAabb;

#ifdef USE_VULKAN_INTEROP
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = gridSizeX;
    imageSettings.height = gridSizeY;
    imageSettings.depth = gridSizeZ;
    imageSettings.imageType = VK_IMAGE_TYPE_3D;
    imageSettings.format = VK_FORMAT_R32_SFLOAT;

    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    vulkanScatteredLinesGridRenderData.scalarFieldTexture = scalarFieldTexture;
    imageSettings.usage =
            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT
            | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    vulkanScatteredLinesGridRenderData.lineDensityFieldTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, samplerSettings);

    lineDensityFieldImageComputeRenderPass->setData(
            this, vulkanScatteredLinesGridRenderData.lineDensityFieldTexture);
    lineDensityFieldMinMaxReduceRenderPass->setLineDensityFieldImage(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage());
    lineDensityFieldNormalizeRenderPass->setLineDensityFieldImageView(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImageView());
    lineDensityFieldNormalizeRenderPass->setLineDensityFieldImageView(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImageView());

    colorLegendWidgets.resize(colorLegendWidgets.size() + 1);
    colorLegendWidgets.back().setPositionIndex(0, 1);
    colorLegendWidgets.back().setAttributeMinValue(0.0f);
    colorLegendWidgets.back().setAttributeMaxValue(1.0f);
    colorLegendWidgets.back().setAttributeDisplayName("Line Density");
#endif

    if (!outlineTriangleIndices.empty()) {
        simulationMeshOutlineTriangleIndices = outlineTriangleIndices;
        simulationMeshOutlineVertexPositions = outlineVertexPositions;
        simulationMeshOutlineVertexNormals = outlineVertexNormals;
        shallRenderSimulationMeshBoundary = true;
    }
}

void LineDataScattering::recomputeHistogram() {
    if (lineRenderer && lineRenderer->getRenderingMode() == RENDERING_MODE_LINE_DENSITY_MAP_RENDERER
            && (histogramNeverComputedBefore || !isVolumeRenderer)) {
        selectedAttributeIndex = int(attributeNames.size());
        isVolumeRenderer = true;

        // TODO
        //this->vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage()->get
        transferFunctionWindow.computeHistogram({0.0f, 1.0f}, 0.0f, 1.0f);

        recomputeColorLegend();
    }
    if ((!lineRenderer || lineRenderer->getRenderingMode() != RENDERING_MODE_LINE_DENSITY_MAP_RENDERER)
            && (histogramNeverComputedBefore || isVolumeRenderer)) {
        selectedAttributeIndex = 0;
        isVolumeRenderer = false;
        LineDataFlow::recomputeHistogram();
    }

    histogramNeverComputedBefore = false;
}

bool LineDataScattering::renderGuiRenderer(bool isRasterizer) {
    bool shallReloadGatherShader = LineData::renderGuiRenderer(isRasterizer);
#ifdef USE_VULKAN_INTEROP
    if (lineRenderer && lineRenderer->getRenderingMode() == RENDERING_MODE_LINE_DENSITY_MAP_RENDERER) {
        if (ImGui::Checkbox("Use Line Segment Length", &useLineSegmentLengthForDensityField)) {
            dirty = true;
            reRender = true;
            lineDensityFieldImageComputeRenderPass->setUseLineSegmentLength(useLineSegmentLengthForDensityField);
        }
    }
#endif
    return shallReloadGatherShader;
}

bool LineDataScattering::renderGuiPropertyEditorNodesRenderer(sgl::PropertyEditor& propertyEditor, bool isRasterizer) {
    bool shallReloadGatherShader = LineData::renderGuiPropertyEditorNodesRenderer(propertyEditor, isRasterizer);
#ifdef USE_VULKAN_INTEROP
    if (lineRenderer && lineRenderer->getRenderingMode() == RENDERING_MODE_LINE_DENSITY_MAP_RENDERER) {
        if (propertyEditor.addCheckbox("Use Line Segment Length", &useLineSegmentLengthForDensityField)) {
            dirty = true;
            reRender = true;
            lineDensityFieldImageComputeRenderPass->setUseLineSegmentLength(useLineSegmentLengthForDensityField);
        }
    }
#endif
    return shallReloadGatherShader;
}


void LineDataScattering::rebuildInternalRepresentationIfNecessary() {
#ifdef USE_VULKAN_INTEROP
    if (dirty || triangleRepresentationDirty) {
        isLineDensityFieldDirty = true;
    }
#endif

    LineData::rebuildInternalRepresentationIfNecessary();
}

#ifdef USE_VULKAN_INTEROP
VulkanLineDataScatteringRenderData LineDataScattering::getVulkanLineDataScatteringRenderData() {
    rebuildInternalRepresentationIfNecessary();
    recomputeHistogram();
    if (!isLineDensityFieldDirty) {
        return vulkanScatteredLinesGridRenderData;
    }

    VkCommandBuffer commandBuffer = rendererVk->getDevice()->beginSingleTimeCommands(
            0xFFFFFFFF, false);
    rendererVk->setCustomCommandBuffer(commandBuffer);
    rendererVk->beginCommandBuffer();

    // Clear the line density field image.
    lineDensityFieldImageComputeRenderPass->setDataDirty();
    rendererVk->insertImageMemoryBarrier(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_NONE_KHR, VK_ACCESS_TRANSFER_WRITE_BIT);
    vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImageView()->clearColor(
            glm::vec4(0.0f), rendererVk->getVkCommandBuffer());

    // Compute the line densities and store them in the line density image.
    rendererVk->insertImageMemoryBarrier(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage(),
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_WRITE_BIT);
    lineDensityFieldImageComputeRenderPass->render();

    // Compute the minimum/maximum line density.
    lineDensityFieldMinMaxReduceRenderPass->render();
    lineDensityFieldNormalizeRenderPass->setMinMaxBuffer(lineDensityFieldMinMaxReduceRenderPass->getMinMaxBuffer());

    // Normalize the line density image.
    rendererVk->insertImageMemoryBarrier(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage(),
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT);
    // lineDensityFieldNormalizeRenderPass->render();
    rendererVk->insertImageMemoryBarrier(
            vulkanScatteredLinesGridRenderData.lineDensityFieldTexture->getImage(),
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);

    rendererVk->endCommandBuffer();
    rendererVk->resetCustomCommandBuffer();
    rendererVk->getDevice()->endSingleTimeCommands(commandBuffer, 0xFFFFFFFF, false);

    isLineDensityFieldDirty = false;
    return vulkanScatteredLinesGridRenderData;
}

VulkanTubeTriangleRenderData LineDataScattering::getVulkanTubeTriangleRenderData(bool raytracing) {
    recomputeHistogram();
    return LineDataFlow::getVulkanTubeTriangleRenderData(raytracing);
}
VulkanTubeAabbRenderData LineDataScattering::getVulkanTubeAabbRenderData() {
    recomputeHistogram();
    return LineDataFlow::getVulkanTubeAabbRenderData();
}
VulkanHullTriangleRenderData LineDataScattering::getVulkanHullTriangleRenderData(bool raytracing) {
    recomputeHistogram();
    return LineDataFlow::getVulkanHullTriangleRenderData(raytracing);
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

    sgl::AABB3 gridBoundingBox = lineData->getGridBoundingBox();
    worldToVoxelGridMatrix =
            sgl::matrixScaling(1.0f / gridBoundingBox.getDimensions() * glm::vec3(gridSizeX, gridSizeY, gridSizeZ))
            * sgl::matrixTranslation(-gridBoundingBox.getMinimum());

    uniformData.gridResolution = glm::ivec3(gridSizeX, gridSizeY, gridSizeZ);
    uniformData.worldToVoxelGridMatrix = worldToVoxelGridMatrix;

    spinlockBuffer = std::make_shared<sgl::vk::Buffer>(
            device, gridSizeX * gridSizeY * gridSizeZ * sizeof(float),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    VkCommandBuffer commandBuffer = device->beginSingleTimeCommands();
    spinlockBuffer->fill(0, commandBuffer);
    device->endSingleTimeCommands(commandBuffer);
}

void LineDensityFieldImageComputeRenderPass::setUseLineSegmentLength(bool useLineSegmentLengthForDensityField) {
    if (useLineSegmentLength != useLineSegmentLengthForDensityField) {
        setShaderDirty();
        useLineSegmentLength = useLineSegmentLengthForDensityField;
    }
}

void LineDensityFieldImageComputeRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    if (useLineSegmentLength) {
        preprocessorDefines.insert({ "USE_LINE_SEGMENT_LENGTH", "" });
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            {"ComputeLineDensityField.Compute"}, preprocessorDefines);
}

void LineDensityFieldImageComputeRenderPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    Trajectories trajectories = lineData->filterTrajectoryData();
    int selectedAttributeIndex = std::min(
            lineData->getSelectedAttributeIndex(),
            trajectories.empty() ? 0 : int(trajectories.front().attributes.size() - 1));

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



LineDensityFieldMinMaxReduceRenderPass::LineDensityFieldMinMaxReduceRenderPass(sgl::vk::Renderer* renderer)
        : ComputePass(renderer) {
    uniformBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void LineDensityFieldMinMaxReduceRenderPass::setLineDensityFieldImage(sgl::vk::ImagePtr& lineDensityFieldImage) {
    this->lineDensityFieldImage = lineDensityFieldImage;
}

void LineDensityFieldMinMaxReduceRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"MinMaxReduce.Compute"});
}

void LineDensityFieldMinMaxReduceRenderPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    auto& imageSettings = lineDensityFieldImage->getImageSettings();
    size_t imageSize = imageSettings.width * imageSettings.height * imageSettings.depth;
    minMaxReductionBuffers[0] = std::make_shared<sgl::vk::Buffer>(
            device, imageSize * sizeof(glm::vec2),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
            | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    minMaxReductionBuffers[1] = std::make_shared<sgl::vk::Buffer>(
            device, sgl::iceil(int(imageSize), BLOCK_SIZE * 2) * sizeof(glm::vec2),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    for (int i = 0; i < 2; i++) {
        computeDataPingPong[i] = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
        computeDataPingPong[i]->setStaticBuffer(uniformBuffer, "UniformBuffer");
        computeDataPingPong[i]->setStaticBuffer(minMaxReductionBuffers[i % 2], "MinMaxInBuffer");
        computeDataPingPong[i]->setStaticBuffer(minMaxReductionBuffers[(i + 1) % 2], "MinMaxOutBuffer");
    }
}

void LineDensityFieldMinMaxReduceRenderPass::_render() {
    renderer->insertImageMemoryBarrier(
            lineDensityFieldImage,
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
    lineDensityFieldImage->copyToBuffer(
            minMaxReductionBuffers[0], renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            minMaxReductionBuffers[0]);

    uint32_t numBlocks = sgl::iceil(int(minMaxReductionBuffers[0]->getSizeInBytes() / 8), BLOCK_SIZE);
    int iteration = 0;
    uint32_t inputSize;
    while (numBlocks > 1) {
        computeData = computeDataPingPong[iteration % 2];
        sgl::vk::BufferPtr readBuffer = minMaxReductionBuffers[iteration % 2];
        sgl::vk::BufferPtr writeBuffer = minMaxReductionBuffers[(iteration + 1) % 2];

        inputSize = numBlocks;
        numBlocks = sgl::iceil(int(numBlocks), BLOCK_SIZE*2);
        uniformData.sizeOfInput = inputSize;
        uniformBuffer->updateData(sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
        renderer->dispatch(computeData, int(numBlocks), 1, 1);
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                readBuffer);
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                writeBuffer);
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                uniformBuffer);
        iteration++;
    }
    outputDepthMinMaxIndex = iteration % 2;
}



LineDensityFieldNormalizeRenderPass::LineDensityFieldNormalizeRenderPass(sgl::vk::Renderer* renderer)
        : ComputePass(renderer) {
    uniformBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void LineDensityFieldNormalizeRenderPass::setLineDensityFieldImageView(
        sgl::vk::ImageViewPtr& lineDensityFieldImageView) {
    this->lineDensityFieldImageView = lineDensityFieldImageView;

    auto& imageSettings = lineDensityFieldImageView->getImage()->getImageSettings();
    uniformData.gridResolution = glm::ivec3(imageSettings.width, imageSettings.height, imageSettings.depth);
}

void LineDensityFieldNormalizeRenderPass::setMinMaxBuffer(sgl::vk::BufferPtr& minMaxBuffer) {
    this->minMaxBuffer = minMaxBuffer;
}

void LineDensityFieldNormalizeRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"NormalizeLineDensityField.Compute"});
}

void LineDensityFieldNormalizeRenderPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(uniformBuffer, "UniformBuffer");
    computeData->setStaticBuffer(minMaxBuffer, "MinMaxBuffer");
    computeData->setStaticImageView(lineDensityFieldImageView, "lineDensityFieldImage");
}

void LineDensityFieldNormalizeRenderPass::_render() {
    uniformBuffer->updateData(sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    auto& imageSettings = lineDensityFieldImageView->getImage()->getImageSettings();
    renderer->dispatch(
            computeData,
            sgl::iceil(int(imageSettings.width), 8),
            sgl::iceil(int(imageSettings.height), 8),
            sgl::iceil(int(imageSettings.depth), 8));
}



LineDensityFieldSmoothingPass::LineDensityFieldSmoothingPass(sgl::vk::Renderer* renderer)
        : ComputePass(renderer) {
    smoothingUniformBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(SmoothingUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Compute the Gaussian filter kernel weights.
    constexpr int FILTER_SIZE = 7;
    float kernel[FILTER_SIZE * FILTER_SIZE];
    float sigma = 1.0f;
    float c = 1.0f / (sgl::TWO_PI * sigma * sigma);
    for (int iy = 0; iy < FILTER_SIZE; iy++) {
        for (int ix = 0; ix < FILTER_SIZE; ix++) {
            int x = ix - (FILTER_SIZE / 2);
            int y = iy - (FILTER_SIZE / 2);
            kernel[ix + iy * FILTER_SIZE] = c * std::exp(-float(x * x + y * y) / (2.0f * sigma * sigma));
        }
    }
    smoothingKernelBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(float) * FILTER_SIZE * FILTER_SIZE, kernel,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void LineDensityFieldSmoothingPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"SmoothDensityTexture.Compute"});
}

void LineDensityFieldSmoothingPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(smoothingUniformBuffer, "UniformBuffer");
    computeData->setStaticBuffer(smoothingKernelBuffer, "FilterWeightBuffer");
    computeData->setStaticTexture(inputTexture, "inputTexture");
    computeData->setStaticImageView(outputImageView, "outputImage");
}

void LineDensityFieldSmoothingPass::_render() {
    smoothingUniformBuffer->updateData(
            sizeof(SmoothingUniformData), &smoothingUniformData, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            smoothingUniformBuffer);
    auto& imageSettings = computeData->getImageView("outputImage")->getImage()->getImageSettings();
    renderer->dispatch(
            computeData,
            sgl::iceil(int(imageSettings.width), 8),
            sgl::iceil(int(imageSettings.height), 8),
            sgl::iceil(int(imageSettings.depth), 8));
}

sgl::vk::ImageViewPtr LineDensityFieldSmoothingPass::smoothScalarField(
        const sgl::vk::TexturePtr& densityFieldTexture, int padding) {
    VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands(
            0xFFFFFFFF, false);
    renderer->setCustomCommandBuffer(commandBuffer);
    renderer->beginCommandBuffer();

    sgl::vk::ImageSettings imageSettings = densityFieldTexture->getImage()->getImageSettings();
    imageSettings.width += 2 * padding;
    imageSettings.height += 2 * padding;
    imageSettings.depth += 2 * padding;
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    outputImageView = std::make_shared<sgl::vk::ImageView>(
            std::make_shared<sgl::vk::Image>(device, imageSettings));
    renderer->insertImageMemoryBarrier(
            outputImageView->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT);

    inputTexture = densityFieldTexture;
    smoothingUniformData.gridResolution = glm::ivec3(imageSettings.width, imageSettings.height, imageSettings.depth);
    smoothingUniformData.padding = padding;
    this->setDataDirty();
    this->render();

    renderer->endCommandBuffer();
    renderer->resetCustomCommandBuffer();
    renderer->getDevice()->endSingleTimeCommands(commandBuffer, 0xFFFFFFFF, false);

    return outputImageView;
}

float* LineDensityFieldSmoothingPass::smoothScalarFieldCpu(sgl::vk::TexturePtr& densityFieldTexture, int padding) {
    sgl::vk::ImageViewPtr smoothedImageView = smoothScalarField(densityFieldTexture, padding);
    const auto& imageSettings = densityFieldTexture->getImage()->getImageSettings();

    VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands();
    uint32_t paddedSizeX = imageSettings.width + 2 * padding;
    uint32_t paddedSizeY = imageSettings.height + 2 * padding;
    uint32_t paddedSizeZ = imageSettings.depth + 2 * padding;
    size_t bufferSize = paddedSizeX * paddedSizeY * paddedSizeZ * sizeof(float);
    sgl::vk::BufferPtr stagingBuffer = std::make_shared<sgl::vk::Buffer>(
            device, bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_CPU_ONLY);
    smoothedImageView->getImage()->insertMemoryBarrier(
            commandBuffer,
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
    smoothedImageView->getImage()->copyToBuffer(stagingBuffer, commandBuffer);
    renderer->getDevice()->endSingleTimeCommands(commandBuffer);

    float* dataOutput = new float[paddedSizeX * paddedSizeY * paddedSizeZ];
    float* data = static_cast<float*>(stagingBuffer->mapMemory());
    memcpy(dataOutput, data, bufferSize);
    stagingBuffer->unmapMemory();
    return dataOutput;
}
#endif
