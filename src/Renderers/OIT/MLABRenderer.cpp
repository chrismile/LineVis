/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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
#include "MLABRenderer.hpp"

MLABRenderer::MLABRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : MLABRenderer("Multi-Layer Alpha Blending Renderer", sceneData, transferFunctionWindow) {
}

MLABRenderer::MLABRenderer(
        const std::string& windowName, SceneData* sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineRenderer("Multi-Layer Alpha Blending Renderer", sceneData, transferFunctionWindow) {
}

void MLABRenderer::initialize() {
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
    syncMode = getSupportedSyncMode(renderer->getDevice());

    resolveRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"MLABResolve.Vertex", "MLABResolve.Fragment"}));
    resolveRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);

    clearRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"MLABClear.Vertex", "MLABClear.Fragment"}));
    clearRasterPass->setColorWriteEnabled(false);
    clearRasterPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    clearRasterPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_DONT_CARE);
    clearRasterPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
    clearRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    onClearColorChanged();
}

void MLABRenderer::updateSyncMode() {
    checkSyncModeSupported(sceneData, renderer->getDevice(), syncMode);

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    spinlockViewportBuffer = {};
    if (syncMode == SYNC_SPINLOCK && *sceneData->viewportWidth > 0 && *sceneData->viewportHeight > 0) {
        spinlockViewportBuffer = std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), sizeof(uint32_t) * size_t(paddedWidth) * size_t(paddedHeight),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

        // Set all values in the buffer to zero.
        spinlockViewportBuffer->fill(0, renderer->getVkCommandBuffer());
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                spinlockViewportBuffer);
    }
}

void MLABRenderer::updateLayerMode() {
    onResolutionChanged();
    reloadShaders();
}

void MLABRenderer::reloadShaders() {
    reloadGatherShader();
    reloadResolveShader();
    clearRasterPass->setShaderDirty();
}

void MLABRenderer::reloadResolveShader() {
    resolveRasterPass->setShaderDirty();
}

void MLABRenderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();
}

void MLABRenderer::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    newState.rendererSettings.getValueOpt("numLayers", numLayers);

    bool recompileGatherShader = false;
    if (newState.rendererSettings.getValueOpt(
            "useOrderedFragmentShaderInterlock", useOrderedFragmentShaderInterlock)) {
        recompileGatherShader = true;
    }
    int syncModeInt = int(syncMode);
    if (newState.rendererSettings.getValueOpt("syncMode", syncModeInt)) {
        syncMode = SyncMode(syncModeInt);
        updateSyncMode();
        recompileGatherShader = true;
    }
    if (recompileGatherShader && lineData) {
        reloadGatherShader();
    }

    timerDataIsWritten = false;
    if ((*sceneData->performanceMeasurer) && !timerDataIsWritten) {
        timer = {};
        timer = std::make_shared<sgl::vk::Timer>(renderer);
        // TODO
        //(*sceneData->performanceMeasurer)->setMlabTimer(timer);
    }
}

void MLABRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void MLABRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"MLABGather.glsl\""));
    preprocessorDefines.insert(std::make_pair("MAX_NUM_LAYERS", sgl::toString(numLayers)));
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK) {
        preprocessorDefines.insert(std::make_pair("__extensions", "GL_ARB_fragment_shader_interlock"));
        preprocessorDefines.insert(std::make_pair("USE_SYNC_FRAGMENT_SHADER_INTERLOCK", ""));
        if (!useOrderedFragmentShaderInterlock) {
            preprocessorDefines.insert(std::make_pair("INTERLOCK_UNORDERED", ""));
        }
    } else if (syncMode == SYNC_SPINLOCK) {
        preprocessorDefines.insert(std::make_pair("USE_SYNC_SPINLOCK", ""));
        // Do not discard while keeping the spinlock locked.
        preprocessorDefines.insert(std::make_pair("GATHER_NO_DISCARD", ""));
    }
}

void MLABRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setColorWriteEnabled(false);
    pipelineInfo.setDepthWriteEnabled(false);
    pipelineInfo.setDepthTestEnabled(false);
}

void MLABRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    renderData->setStaticBufferOptional(fragmentBuffer, "FragmentNodes");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
    if (syncMode == SYNC_SPINLOCK) {
        renderData->setStaticBufferOptional(spinlockViewportBuffer, "SpinlockViewportBuffer");
    }
}

void MLABRenderer::updateVulkanUniformBuffers() {
}

void MLABRenderer::setFramebufferAttachments(
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

void MLABRenderer::reallocateFragmentBuffer() {
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
            renderer->getDevice(), fragmentBufferSizeBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    updateSyncMode();

    // Buffer has to be cleared again.
    clearBitSet = true;
}

void MLABRenderer::onResolutionChanged() {
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

    clearRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    clearRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void MLABRenderer::onClearColorChanged() {
    resolveRasterPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void MLABRenderer::render() {
    LineRenderer::renderBase();

    setUniformData();
    clear();
    gather();
    resolve();
}

void MLABRenderer::setUniformData() {
    uniformData.viewportW = paddedWindowWidth;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void MLABRenderer::clear() {
    if (clearBitSet) {
        clearRasterPass->render();
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
        clearBitSet = false;
    }
}

void MLABRenderer::gather() {
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

void MLABRenderer::resolve() {
    resolveRasterPass->render();
}

void MLABRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addSliderInt("Num Layers", &numLayers, 1, 64)) {
        renderer->getDevice()->waitIdle();
        updateLayerMode();
        reRender = true;
    }
    const char *syncModeNames[] = { "No Sync (Unsafe)", "Fragment Shader Interlock", "Spinlock" };
    if (propertyEditor.addCombo(
            "Sync Mode", (int*)&syncMode, syncModeNames, IM_ARRAYSIZE(syncModeNames))) {
        renderer->getDevice()->waitIdle();
        updateSyncMode();
        reloadGatherShader();
        reRender = true;
    }
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK && propertyEditor.addCheckbox(
            "Ordered Sync", &useOrderedFragmentShaderInterlock)) {
        renderer->getDevice()->waitIdle();
        reloadGatherShader();
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
