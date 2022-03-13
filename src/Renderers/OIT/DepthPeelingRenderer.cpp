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

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Utils/SyncObjects.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "DepthPeelingRenderer.hpp"

DepthPeelingRenderer::DepthPeelingRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Depth Peeling", sceneData, transferFunctionWindow) {
}

void DepthPeelingRenderer::initialize() {
    LineRenderer::initialize();

    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    depthComplexityRasterPass = std::make_shared<DepthComplexityRasterPass>(this);
    depthComplexityRasterPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
    depthComplexityRasterPass->setUpdateUniformData(false);

    for (auto& depthPeelingRasterPass : depthPeelingRasterPasses) {
        depthPeelingRasterPass = std::make_shared<LineRasterPass>(this);
        depthPeelingRasterPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_LOAD);
        depthPeelingRasterPass->setUpdateUniformData(false);
    }

    for (auto& depthPeelingBlitPass : depthPeelingBlitPasses) {
        depthPeelingBlitPass = std::make_shared<sgl::vk::BlitRenderPass>(renderer);
        depthPeelingBlitPass->setBlendMode(sgl::vk::BlendMode::FRONT_TO_BACK_PREMUL_ALPHA);
        depthPeelingBlitPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        depthPeelingBlitPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        depthPeelingBlitPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_LOAD);
        depthPeelingBlitPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_STORE);
    }

    blitRenderPass = std::make_shared<sgl::vk::BlitRenderPass>(renderer);
    blitRenderPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    blitRenderPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_PREMUL_ALPHA);
    blitRenderPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);

    // Disable render passes of parent class.
    lineRasterPass = {};
    hullRasterPass = {};

    sgl::vk::CommandPoolType commandPoolType;
    commandPoolType.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    commandPoolType.queueFamilyIndex = renderer->getDevice()->getGraphicsQueueIndex();
    depthComplexityCommandBuffer = renderer->getDevice()->allocateCommandBuffer(commandPoolType, &commandPool);

    fence = std::make_shared<sgl::vk::Fence>(renderer->getDevice());

    onClearColorChanged();
}

DepthPeelingRenderer::~DepthPeelingRenderer() {
    renderer->getDevice()->freeCommandBuffer(commandPool, depthComplexityCommandBuffer);
}

void DepthPeelingRenderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();

    depthPeelingRasterPasses[0]->setShaderDirty();
    depthPeelingRasterPasses[1]->setShaderDirty();
}

void DepthPeelingRenderer::setNewState(const InternalState& newState) {
}

void DepthPeelingRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    depthComplexityRasterPass->setLineData(lineData, isNewData);
    depthPeelingRasterPasses[0]->setLineData(lineData, isNewData);
    depthPeelingRasterPasses[1]->setLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void DepthPeelingRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"DepthPeelingGather.glsl\""));
}

void DepthPeelingRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::OVERWRITE);
}

void DepthPeelingRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    renderData->setStaticTextureOptional(depthRenderTextures[(currIdx + 1) % 2], "depthReadTexture");
    renderData->setStaticBufferOptional(fragmentCounterBuffer, "FragmentCounterBuffer");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
}

void DepthPeelingRenderer::updateVulkanUniformBuffers() {
}

void DepthPeelingRenderer::setFramebufferAttachments(
        sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    if (loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR) {
        // Depth complexity pass.
        sgl::vk::AttachmentState attachmentState;
        attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        framebuffer->setColorAttachment(
                (*sceneData->sceneTexture)->getImageView(), 0, attachmentState,
                sceneData->clearColor->getFloatColorRGBA());
    } else if (loadOp == VK_ATTACHMENT_LOAD_OP_LOAD) {
        // Depth peeling raster pass.
        sgl::vk::AttachmentState attachmentState;
        attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachmentState.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        framebuffer->setColorAttachment(
                colorRenderTextures[currIdx]->getImageView(), 0, attachmentState,
                { 0.0f, 0.0f, 0.0f, 0.0f });

        sgl::vk::AttachmentState depthAttachmentState;
        depthAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAttachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depthAttachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        framebuffer->setDepthStencilAttachment(
                depthRenderTextures[currIdx]->getImageView(), depthAttachmentState, 1.0f);
    }
}

void DepthPeelingRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    sgl::vk::ImageSettings imageSettingsColor;
    imageSettingsColor.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettingsColor.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    imageSettingsColor.width = width;
    imageSettingsColor.height = height;
    sgl::vk::ImageSettings imageSettingsDepth;
    imageSettingsDepth.format = VK_FORMAT_D32_SFLOAT;
    imageSettingsDepth.usage =
            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    imageSettingsDepth.width = width;
    imageSettingsDepth.height = height;

    for (int i = 0; i < 2; i++) {
        colorRenderTextures[i] = std::make_shared<sgl::vk::Texture>(renderer->getDevice(), imageSettingsColor);
        depthRenderTextures[i] = std::make_shared<sgl::vk::Texture>(
                renderer->getDevice(), imageSettingsDepth, VK_IMAGE_ASPECT_DEPTH_BIT);
    }

    imageSettingsColor.usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    colorAccumulatorTexture = std::make_shared<sgl::vk::Texture>(renderer->getDevice(), imageSettingsColor);

    depthComplexityRasterPass->recreateSwapchain(width, height);
    for (int i = 0; i < 2; i++) {
        currIdx = i;
        depthPeelingRasterPasses[i]->recreateSwapchain(width, height);
    }

    for (int i = 0; i < 2; i++) {
        depthPeelingBlitPasses[i]->setBlendMode(sgl::vk::BlendMode::FRONT_TO_BACK_PREMUL_ALPHA);
        depthPeelingBlitPasses[i]->setInputTexture(colorRenderTextures[i % 2]);
        depthPeelingBlitPasses[i]->setOutputImage(colorAccumulatorTexture->getImageView());
        depthPeelingBlitPasses[i]->recreateSwapchain(width, height);
    }

    blitRenderPass->setInputTexture(colorAccumulatorTexture);
    blitRenderPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    blitRenderPass->recreateSwapchain(width, height);

    // Buffer for determining the (maximum) depth complexity of the scene
    size_t numFragmentsBufferSizeBytes = sizeof(uint32_t) * width * height;
    fragmentCounterBuffer = {}; // Delete old data first (-> refcount 0)
    fragmentCounterBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numFragmentsBufferSizeBytes,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    stagingBuffers.reserve(swapchain->getNumImages());
    for (size_t i = 0; i < swapchain->getNumImages(); i++) {
        stagingBuffers.push_back(std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), numFragmentsBufferSizeBytes,
                VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU));
    }

    size_t bufferSizeColorAccumulators = 2 * sizeof(float) * 4 * width * height;
    size_t bufferSizeDepthTextures = 2 * sizeof(float) * width * height;
    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(
                numFragmentsBufferSizeBytes + bufferSizeColorAccumulators + bufferSizeDepthTextures);
    }
}

void DepthPeelingRenderer::onClearColorChanged() {
    blitRenderPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void DepthPeelingRenderer::setUniformData() {
    lineData->updateVulkanUniformBuffers(this, renderer);
    this->updateVulkanUniformBuffers();

    int width = int(*sceneData->viewportWidth);
    uniformData.viewportW = width;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT);
}

void DepthPeelingRenderer::render() {
    LineRenderer::renderBase();

    setUniformData();
    computeDepthComplexity();
    gather();
    resolve();
}

void DepthPeelingRenderer::gather() {
    renderer->insertImageMemoryBarrier(
            colorAccumulatorTexture->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_NONE_KHR, VK_ACCESS_TRANSFER_WRITE_BIT);
    colorAccumulatorTexture->getImageView()->clearColor(
            { 0.0f, 0.0f, 0.0f, 0.0f }, renderer->getVkCommandBuffer());
    renderer->transitionImageLayout(
            colorAccumulatorTexture->getImage(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    renderer->insertImageMemoryBarrier(
            depthRenderTextures[1]->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_NONE_KHR, VK_ACCESS_TRANSFER_WRITE_BIT);
    depthRenderTextures[1]->getImageView()->clearDepthStencil(
            1.0f, 0, renderer->getVkCommandBuffer());
    renderer->transitionImageLayout(
            depthRenderTextures[1]->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    int swapInterval = 512;
    if (renderer->getDevice()->getDeviceType() == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
        swapInterval = 256;
    }

    for (uint64_t i = 0; i < std::min(maxDepthComplexity, uint64_t(2000ul)); i++) {
        currIdx = int(i);

        // 1. Peel one layer of the scene.
        depthPeelingRasterPasses[i % 2]->buildIfNecessary();
        renderer->pushConstants(
                depthPeelingRasterPasses[i % 2]->getGraphicsPipeline(),
                VK_SHADER_STAGE_FRAGMENT_BIT, 0, int(i));
        depthPeelingRasterPasses[i % 2]->render();

        // 2. Store it in the accumulator.
        depthPeelingBlitPasses[i % 2]->render();

        // Avoid GPU driver TDR.
        if (i != 0 && i % swapInterval == 0) {
            renderer->syncWithCpu();
        }
    }
}

void DepthPeelingRenderer::resolve() {
    renderer->transitionImageLayout(
            colorAccumulatorTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    blitRenderPass->render();
}

void DepthPeelingRenderer::computeDepthComplexity() {
    //renderer->setCustomCommandBuffer(depthComplexityCommandBuffer);
    //renderer->beginCommandBuffer();

    // Clear numFragmentsBuffer.
    fragmentCounterBuffer->fill(0, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            fragmentCounterBuffer);

    // Render to numFragmentsBuffer to determine the depth complexity of the scene.
    //renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    //renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    //renderer->setModelMatrix(sgl::matrixIdentity());

    depthComplexityRasterPass->render();
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            fragmentCounterBuffer);

    // Compute the maximum depth complexity of the scene.
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int bufferSize = width * height;

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    auto& stagingBuffer = stagingBuffers.at(swapchain->getImageIndex());
    fragmentCounterBuffer->copyDataTo(stagingBuffer, renderer->getVkCommandBuffer());
    //renderer->endCommandBuffer();
    //renderer->submitToQueue({}, {}, fence, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT);
    //renderer->resetCustomCommandBuffer();
    //fence->wait();
    //fence->reset();
    renderer->syncWithCpu();

    auto *data = (uint32_t*)stagingBuffer->mapMemory();

    uint64_t maxDepthComplexity = 0;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(max:maxDepthComplexity) default(none) shared(data, bufferSize) schedule(static)
#endif
    for (int i = 0; i < bufferSize; i++) {
        maxDepthComplexity = std::max(maxDepthComplexity, (uint64_t)data[i]);
    }
    this->maxDepthComplexity = maxDepthComplexity;

    stagingBuffer->unmapMemory();
}

void DepthPeelingRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);
    propertyEditor.addText("Max. Depth Complexity", std::to_string(maxDepthComplexity));
}



DepthComplexityRasterPass::DepthComplexityRasterPass(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
}

void DepthComplexityRasterPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines["OIT_GATHER_HEADER"] = "\"DepthComplexityGatherInc.glsl\"";
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            lineData->getShaderModuleNames(), preprocessorDefines);
}

void DepthComplexityRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    LineRasterPass::setGraphicsPipelineInfo(pipelineInfo);
    pipelineInfo.setColorWriteEnabled(false);
    pipelineInfo.setDepthWriteEnabled(false);
}
