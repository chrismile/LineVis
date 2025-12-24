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

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "VoxelRayCastingRenderer.hpp"

VoxelRayCastingRenderer::VoxelRayCastingRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Voxel Ray Casting Renderer", sceneData, transferFunctionWindow),
          voxelCurveDiscretizer((*sceneData->renderer)->getDevice()){
    isRasterizer = false;
}

void VoxelRayCastingRenderer::initialize() {
    LineRenderer::initialize();

    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    voxelRayCastingPass = std::make_shared<ResolvePass>(
            this,
            std::vector<std::string>{ "VoxelRayCasting.Vertex", "VoxelRayCasting.Fragment" });
    voxelRayCastingPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    for (int i = 0; i < 2; i++) {
        lineHullRasterPasses[i] = std::make_shared<LineHullRasterPass>(
                this, &voxelCurveDiscretizer);
    }
    lineHullRasterPasses[0]->setDepthCompareOp(VK_COMPARE_OP_GREATER);
    lineHullRasterPasses[1]->setDepthCompareOp(VK_COMPARE_OP_LESS);

    onClearColorChanged();
}

void VoxelRayCastingRenderer::reloadGatherShader() {
    voxelRayCastingPass->setShaderDirty();
}

void VoxelRayCastingRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    if (!gridResolutionSetManually) {
        double avgLineSegsPerAxis = std::cbrt(lineData->getNumLineSegments());
        gridResolution1D = glm::clamp(sgl::nextPowerOfTwo(int(avgLineSegsPerAxis)) * 2, 64, 512);
    }

    voxelCurveDiscretizer.loadLineData(lineData, gridResolution1D, quantizationResolution1D);
    if (useGpuForVoxelization) {
        voxelCurveDiscretizer.createVoxelGridGpu();
    } else {
        voxelCurveDiscretizer.createVoxelGridCpu();
    }
    voxelCurveDiscretizer.createLineHullMesh();
    voxelGridLineSegmentOffsetsBuffer = voxelCurveDiscretizer.getVoxelGridLineSegmentOffsetsBuffer();
    voxelGridNumLineSegmentsBuffer = voxelCurveDiscretizer.getVoxelGridNumLineSegmentsBuffer();
    voxelGridLineSegmentsBuffer = voxelCurveDiscretizer.getVoxelGridLineSegmentsBuffer();

    for (int i = 0; i < 2; i++) {
        lineHullRasterPasses[i]->setLineData(lineData, isNewData);
    }

    worldToVoxelGridMatrix = voxelCurveDiscretizer.getWorldToVoxelGridMatrix();
    voxelGridToWorldMatrix = voxelCurveDiscretizer.getVoxelGridToWorldMatrix();
    gridResolution = voxelCurveDiscretizer.getGridResolution();
    quantizationResolution = voxelCurveDiscretizer.getQuantizationResolution();

    reloadGatherShader();

    dirty = false;
    reRender = true;
}

void VoxelRayCastingRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);

    preprocessorDefines.insert(std::make_pair(
            "gridResolution", ivec3ToString(gridResolution)));
    preprocessorDefines.insert(std::make_pair(
            "GRID_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(gridResolution.x)))));
    preprocessorDefines.insert(std::make_pair(
            "GRID_RESOLUTION", sgl::toString(gridResolution.x)));
    preprocessorDefines.insert(std::make_pair(
            "quantizationResolution", uvec3ToString(quantizationResolution)));
    preprocessorDefines.insert(std::make_pair(
            "QUANTIZATION_RESOLUTION", sgl::toString(quantizationResolution.x)));
    preprocessorDefines.insert(std::make_pair(
            "QUANTIZATION_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(quantizationResolution.x)))));
    preprocessorDefines.insert(std::make_pair(
            "MAX_NUM_HITS", sgl::toString(maxNumHits)));
    preprocessorDefines.insert(std::make_pair(
            "MAX_NUM_LINES_PER_VOXEL", sgl::toString(maxNumLinesPerVoxel)));
    if (computeNearestFurthestHitsUsingHull) {
        preprocessorDefines.insert(std::make_pair("COMPUTE_NEAREST_FURTHEST_HIT_USING_HULL", ""));
    }
}

void VoxelRayCastingRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
}

void VoxelRayCastingRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    lineData->setVulkanRenderDataDescriptors(renderData);
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
    renderData->setStaticBufferOptional(voxelGridLineSegmentOffsetsBuffer, "VoxelLineListOffsetBuffer");
    renderData->setStaticBufferOptional(voxelGridNumLineSegmentsBuffer, "NumLinesBuffer");
    renderData->setStaticBufferOptional(voxelGridLineSegmentsBuffer, "LineSegmentBuffer");
    if (computeNearestFurthestHitsUsingHull) {
        renderData->setStaticTexture(nearestLineHullHitDepthTexture, "nearestLineHullHitDepthTexture");
        renderData->setStaticTexture(furthestLineHullHitDepthTexture, "furthestLineHullHitDepthTexture");
    }
    //renderData->setStaticTextureOptional(densityTexture, "densityTexture");
    //renderData->setStaticTextureOptional(aoTexture, "aoTexture");
    if ((*sceneData->performanceMeasurer)) {
        if (renderData->getShaderStages()->getShaderModules().front()->getShaderModuleId() == "VoxelRayCasting.Vertex") {
            auto renderDataSize = renderData->getRenderDataSize();
            (*sceneData->performanceMeasurer)->setCurrentDataSetBufferSizeBytes(
                    renderDataSize.storageBufferSize);
        }
    }
}

void VoxelRayCastingRenderer::updateVulkanUniformBuffers() {
}

void VoxelRayCastingRenderer::setFramebufferAttachments(
        sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    framebuffer->setColorAttachment(
            (*sceneData->sceneTexture)->getImageView(), 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());
}

void VoxelRayCastingRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);

    sgl::vk::ImageSettings depthImageSettings{};
    depthImageSettings.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    depthImageSettings.format = VK_FORMAT_D32_SFLOAT;
    depthImageSettings.width = width;
    depthImageSettings.height = height;

    // Linear filter is not needed, and also not supported on some Vulkan implementations (e.g., software rasterizers).
    // On llvmpipe, for example, VK_FORMAT_D32_SFLOAT misses VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT.
    sgl::vk::ImageSamplerSettings depthImageSamplerSettings{};
    depthImageSamplerSettings.minFilter = VK_FILTER_NEAREST;
    depthImageSamplerSettings.magFilter = VK_FILTER_NEAREST;
    depthImageSamplerSettings.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;

    furthestLineHullHitDepthTexture = std::make_shared<sgl::vk::Texture>(
            renderer->getDevice(), depthImageSettings, depthImageSamplerSettings,
            VK_IMAGE_ASPECT_DEPTH_BIT);
    nearestLineHullHitDepthTexture = std::make_shared<sgl::vk::Texture>(
            renderer->getDevice(), depthImageSettings, depthImageSamplerSettings,
            VK_IMAGE_ASPECT_DEPTH_BIT);

    voxelRayCastingPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    voxelRayCastingPass->recreateSwapchain(width, height);

    lineHullRasterPasses[0]->setOutputDepthImageView(furthestLineHullHitDepthTexture->getImageView());
    lineHullRasterPasses[1]->setOutputDepthImageView(nearestLineHullHitDepthTexture->getImageView());
    for (int i = 0; i < 2; i++) {
        lineHullRasterPasses[i]->recreateSwapchain(width, height);
    }
}

void VoxelRayCastingRenderer::onClearColorChanged() {
    voxelRayCastingPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void VoxelRayCastingRenderer::setUniformData() {
    lineData->updateVulkanUniformBuffers(this, renderer);
    glm::mat4 inverseViewMatrix = glm::inverse(sceneData->camera->getViewMatrix());
    glm::mat4 inverseProjectionMatrix = glm::inverse(sceneData->camera->getProjectionMatrixOpenGL());
    uniformData.cameraPositionVoxelGrid =
            sgl::transformPoint(worldToVoxelGridMatrix, sceneData->camera->getPosition());
    uniformData.aspectRatio = sceneData->camera->getAspectRatio();
    float voxelSpaceLineRadius = lineWidth * 0.5f * glm::length(worldToVoxelGridMatrix[0]);
    uniformData.lineRadius = voxelSpaceLineRadius;
    uniformData.worldSpaceToVoxelSpace = worldToVoxelGridMatrix;
    uniformData.voxelSpaceToWorldSpace = voxelGridToWorldMatrix;
    uniformData.ndcToVoxelSpace = worldToVoxelGridMatrix * inverseViewMatrix * inverseProjectionMatrix;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void VoxelRayCastingRenderer::render() {
    LineRenderer::renderBase();

    setUniformData();

    // 1. Compute the minimum and maximum projected line hull depth at every pixel.
    if (computeNearestFurthestHitsUsingHull) {
        lineHullRasterPasses[0]->render();
        lineHullRasterPasses[1]->render();
    }

    // 2. Execute the VRC rendering pass.
    if (voxelGridLineSegmentsBuffer) {
        voxelRayCastingPass->render();
    } else {
        renderer->transitionImageLayout(
                (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        (*sceneData->sceneTexture)->getImageView()->clearColor(
                sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
    }
}

void VoxelRayCastingRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    bool voxelGridDirty = false;

    if (propertyEditor.addCheckbox("Use GPU for Voxelization", &useGpuForVoxelization)) {
        voxelGridDirty = true;
    }

    if (propertyEditor.addCheckbox("Use Line Hull", &computeNearestFurthestHitsUsingHull)) {
        renderer->getDevice()->waitIdle();
        reloadGatherShader();
        internalReRender = true;
        reRender = true;
    }

    if (propertyEditor.addSliderInt("Grid Resolution", &gridResolution1D, 4, 256)) {
        voxelGridDirty = true;
        gridResolutionSetManually = true;
    }
    if (propertyEditor.addSliderIntPowerOfTwo(
            "Quantization Resolution", &quantizationResolution1D, 1, 64)) {
        voxelGridDirty = true;
        gridResolutionSetManually = true;
    }

    if (voxelGridDirty) {
        dirty = true;
        internalReRender = true;
        reRender = true;
    }
}

void VoxelRayCastingRenderer::setNewState(const InternalState& newState) {
    bool voxelGridDirty = false;

    if (newState.rendererSettings.getValueOpt("useGpuForVoxelization", useGpuForVoxelization)) {
        voxelGridDirty = true;
    }

    if (newState.rendererSettings.getValueOpt(
            "computeNearestFurthestHitsUsingHull", computeNearestFurthestHitsUsingHull)) {
        reloadGatherShader();
        internalReRender = true;
        reRender = true;
    }

    if (newState.rendererSettings.getValueOpt("gridResolution", gridResolution1D)) {
        voxelGridDirty = true;
        gridResolutionSetManually = true;
    }
    if (newState.rendererSettings.getValueOpt(
            "quantizationResolution", quantizationResolution1D)) {
        voxelGridDirty = true;
        gridResolutionSetManually = true;
    }

    if (voxelGridDirty) {
        dirty = true;
        internalReRender = true;
        reRender = true;
    }
}



LineHullRasterPass::LineHullRasterPass(LineRenderer* lineRenderer, VoxelCurveDiscretizer* voxelCurveDiscretizer)
        : RasterPass(*lineRenderer->getSceneData()->renderer), lineRenderer(lineRenderer),
          sceneData(lineRenderer->getSceneData()), camera(&sceneData->camera),
          voxelCurveDiscretizer(voxelCurveDiscretizer) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
}

void LineHullRasterPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    dataDirty = true;
}

void LineHullRasterPass::setDepthCompareOp(VkCompareOp compareOp) {
    depthCompareOp = compareOp;
}

void LineHullRasterPass::setOutputDepthImageView(const sgl::vk::ImageViewPtr& imageView) {
    outputDepthImageView = imageView;
}

void LineHullRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAttachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            outputDepthImageView, depthAttachmentState,
            depthCompareOp == VK_COMPARE_OP_LESS ? 1.0f : 0.0f);

    framebufferDirty = true;
    dataDirty = true;
}

void LineHullRasterPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "LineHull.Vertex", "LineHull.Fragment" }, preprocessorDefines);
}

void LineHullRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);
    pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    pipelineInfo.setDepthCompareOp(depthCompareOp);
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexPosition", sizeof(glm::vec3));
}

void LineHullRasterPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    rasterData->setStaticBuffer(uniformDataBuffer, "UniformDataBuffer");
    if (!voxelCurveDiscretizer->getLineHullIndexBuffer()) {
        return;
    }
    rasterData->setIndexBuffer(voxelCurveDiscretizer->getLineHullIndexBuffer());
    rasterData->setVertexBuffer(voxelCurveDiscretizer->getLineHullVertexBuffer(), 0);
}

void LineHullRasterPass::_render() {
    uniformData.voxelSpaceToWorldSpace = voxelCurveDiscretizer->getVoxelGridToWorldMatrix();
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT);
    RasterPass::_render();
}
