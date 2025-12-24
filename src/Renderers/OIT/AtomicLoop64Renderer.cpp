/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2023, Christoph Neuhauser
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

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "../HullRasterPass.hpp"
#include "AtomicLoop64Renderer.hpp"

AtomicLoop64Renderer::AtomicLoop64Renderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : AtomicLoop64Renderer("Atomic Loop 64 Renderer", sceneData, transferFunctionWindow) {
}

AtomicLoop64Renderer::AtomicLoop64Renderer(
        const std::string& windowName, SceneData* sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineRenderer("Atomic Loop 64 Renderer", sceneData, transferFunctionWindow) {
}

void AtomicLoop64Renderer::initialize() {
    LineRenderer::initialize();

    sgl::vk::Device* device = renderer->getDevice();
    maxStorageBufferSize = std::min(
            size_t(device->getMaxMemoryAllocationSize()), size_t(device->getMaxStorageBufferRange()));
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);
    size_t fragmentBufferSizeBytes =
            (sizeof(uint32_t) + sizeof(float)) * size_t(numLayers) * size_t(paddedWidth) * size_t(paddedHeight);
    if (fragmentBufferSizeBytes > maxStorageBufferSize) {
        numLayers = std::max(
                1, int(maxStorageBufferSize /
                       ((sizeof(uint32_t) + sizeof(float)) * size_t(paddedWidth) * size_t(paddedHeight))));
    }

    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    clearBitSet = true;

    resolveRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"AtomicLoop64Resolve.Vertex", "AtomicLoop64Resolve.Fragment"}));
    resolveRasterPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_LOAD);
    resolveRasterPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    resolveRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);

    onClearColorChanged();
}

void AtomicLoop64Renderer::updateLayerMode() {
    onResolutionChanged();
    reloadShaders();
}

void AtomicLoop64Renderer::reloadShaders() {
    reloadGatherShader();
    reloadResolveShader();
}

void AtomicLoop64Renderer::reloadResolveShader() {
    resolveRasterPass->setShaderDirty();
}

void AtomicLoop64Renderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();
}

void AtomicLoop64Renderer::setNewState(const InternalState& newState) {
    bool recompileGatherShader = false;
    recompileGatherShader |= newState.rendererSettings.getValueOpt("numLayers", numLayers);

    if (recompileGatherShader && lineData) {
        reloadGatherShader();
    }
}

void AtomicLoop64Renderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void AtomicLoop64Renderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"AtomicLoop64Gather.glsl\""));
    preprocessorDefines.insert(std::make_pair("MAX_NUM_LAYERS", sgl::toString(numLayers)));
}

void AtomicLoop64Renderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setDepthWriteEnabled(false);
    pipelineInfo.setDepthTestEnabled(false);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);
}

void AtomicLoop64Renderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    renderData->setStaticBufferOptional(fragmentBuffer, "FragmentNodes");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
}

void AtomicLoop64Renderer::updateVulkanUniformBuffers() {
}

void AtomicLoop64Renderer::setFramebufferAttachments(
        sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    framebuffer->setColorAttachment(
            (*sceneData->sceneTexture)->getImageView(), 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());
}

void AtomicLoop64Renderer::reallocateFragmentBuffer() {
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    size_t fragmentBufferSizeBytes =
            (sizeof(uint32_t) + sizeof(float)) * size_t(numLayers) * size_t(paddedWidth) * size_t(paddedHeight);
    if (fragmentBufferSizeBytes > maxStorageBufferSize) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than maximum allocation size ("
                + std::to_string(maxStorageBufferSize) + "). Clamping to maximum allocation size.",
                false);
        fragmentBufferSizeBytes = maxStorageBufferSize / 8ull - 8ull;
    } else {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Fragment buffer size GiB: "
                + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
    }

    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }

    fragmentBuffer = {}; // Delete old data first (-> refcount 0)
    fragmentBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), fragmentBufferSizeBytes,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    // Buffer has to be cleared again.
    clearBitSet = true;
}

void AtomicLoop64Renderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    windowWidth = width;
    windowHeight = height;
    paddedWindowWidth = windowWidth, paddedWindowHeight = windowHeight;
    getScreenSizeWithTiling(paddedWindowWidth, paddedWindowHeight);

    reallocateFragmentBuffer();

    lineRasterPass->recreateSwapchain(width, height);
    if (hullRasterPass) {
        hullRasterPass->recreateSwapchain(width, height);
    }

    resolveRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    resolveRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void AtomicLoop64Renderer::onClearColorChanged() {
    if (lineRasterPass && !lineRasterPass->getIsDataEmpty()) {
        lineRasterPass->updateFramebuffer();
    } else if (hullRasterPass && !hullRasterPass->getIsDataEmpty()) {
        hullRasterPass->updateFramebuffer();
    }
    resolveRasterPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void AtomicLoop64Renderer::render() {
    LineRenderer::renderBase();

    setUniformData();
    clear();
    gather();
    resolve();
}

void AtomicLoop64Renderer::setUniformData() {
    uniformData.viewportW = paddedWindowWidth;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void AtomicLoop64Renderer::clear() {
    if (clearBitSet) {
        fragmentBuffer->fill(0xFFFFFFFFu, renderer->getVkCommandBuffer());
        renderer->insertMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
        clearBitSet = false;
    }
}

void AtomicLoop64Renderer::gather() {
    //renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    //renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    //renderer->setModelMatrix(sgl::matrixIdentity());

    lineRasterPass->buildIfNecessary();
    if (!lineRasterPass->getIsDataEmpty()) {
        lineRasterPass->render();
    }
    renderHull();
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void AtomicLoop64Renderer::resolve() {
    resolveRasterPass->render();
}

void AtomicLoop64Renderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addSliderInt("Num Layers", &numLayers, 1, 64)) {
        renderer->getDevice()->waitIdle();
        updateLayerMode();
        reRender = true;
    }
    if (selectTilingModeUI(propertyEditor)) {
        renderer->getDevice()->waitIdle();
        reloadShaders();
        clearBitSet = true;
        reRender = true;
    }
    if (propertyEditor.addButton("Reload Gather Shader", "Reload")) {
        renderer->getDevice()->waitIdle();
        reloadGatherShader();
        reRender = true;
    }
}
