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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "../HullRasterPass.hpp"
#include "PerPixelLinkedListLineRenderer.hpp"

PerPixelLinkedListLineRenderer::PerPixelLinkedListLineRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Per-Pixel Linked List Renderer", sceneData, transferFunctionWindow) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    resolveRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"LinkedListResolve.Vertex", "LinkedListResolve.Fragment"}));
    resolveRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);

    clearRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"LinkedListClear.Vertex", "LinkedListClear.Fragment"}));
    clearRasterPass->setColorWriteEnabled(false);
    clearRasterPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    clearRasterPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_DONT_CARE);
    clearRasterPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
    clearRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    // SORTING_ALGORITHM_MODE_PRIORITY_QUEUE produces incorrect results on AMD GPUs for whatever reasons...
    if (device->getDeviceDriverId() == VK_DRIVER_ID_AMD_PROPRIETARY) {
        sortingAlgorithmMode = SORTING_ALGORITHM_MODE_SHELL_SORT;
    }

    onClearColorChanged();
}

PerPixelLinkedListLineRenderer::~PerPixelLinkedListLineRenderer() {
    if ((*sceneData->performanceMeasurer) && !timerDataIsWritten && timer) {
        timer = {};
        (*sceneData->performanceMeasurer)->setPpllTimer({});
    }
}

void PerPixelLinkedListLineRenderer::reloadResolveShader() {
    resolveRasterPass->setShaderDirty();
}

void PerPixelLinkedListLineRenderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();
}

void PerPixelLinkedListLineRenderer::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    timerDataIsWritten = false;
    if ((*sceneData->performanceMeasurer) && !timerDataIsWritten) {
        timer = {};
        timer = std::make_shared<sgl::vk::Timer>(renderer);
        timer->setStoreFrameTimeList(true);
        (*sceneData->performanceMeasurer)->setPpllTimer(timer);
    }
}

void PerPixelLinkedListLineRenderer::updateLargeMeshMode() {
    // More than one million cells?
    LargeMeshMode newMeshLargeMeshMode = MESH_SIZE_MEDIUM;
    if (lineData->getNumLineSegments() > size_t(1e6)) { // > 1m line segments
        newMeshLargeMeshMode = MESH_SIZE_LARGE;
    }
    if (newMeshLargeMeshMode != largeMeshMode) {
        renderer->getDevice()->waitIdle();
        largeMeshMode = newMeshLargeMeshMode;
        expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_PPLL[int(largeMeshMode)][0];
        expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_PPLL[int(largeMeshMode)][1];
        reallocateFragmentBuffer();
        reloadResolveShader();
    }
}

void PerPixelLinkedListLineRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);
    updateLargeMeshMode();

    dirty = false;
    reRender = true;
}

void PerPixelLinkedListLineRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\""));

    preprocessorDefines.insert(std::make_pair("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity)));
    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
            || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = int(std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4);
        preprocessorDefines.insert(std::make_pair("STACK_SIZE", sgl::toString(stackSize)));
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_PRIORITY_QUEUE) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "frontToBackPQ"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BUBBLE_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "bubbleSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_INSERTION_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "insertionSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_SHELL_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "shellSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_MAX_HEAP) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "heapSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BITONIC_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "bitonicSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "quicksort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "quicksortHybrid"));
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
            || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        preprocessorDefines.insert(std::make_pair("USE_QUICKSORT", ""));
    }
}

void PerPixelLinkedListLineRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setColorWriteEnabled(false);
    pipelineInfo.setDepthWriteEnabled(false);
    pipelineInfo.setDepthTestEnabled(false);
}

void PerPixelLinkedListLineRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    renderData->setStaticBufferOptional(fragmentBuffer, "FragmentBuffer");
    renderData->setStaticBuffer(startOffsetBuffer, "StartOffsetBuffer");
    renderData->setStaticBufferOptional(fragmentCounterBuffer, "FragCounterBuffer");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
}

void PerPixelLinkedListLineRenderer::updateVulkanUniformBuffers() {
}

void PerPixelLinkedListLineRenderer::setFramebufferAttachments(
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

void PerPixelLinkedListLineRenderer::reallocateFragmentBuffer() {
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    fragmentBufferSize = size_t(expectedAvgDepthComplexity) * size_t(paddedWidth) * size_t(paddedHeight);
    size_t fragmentBufferSizeBytes = 12ull * fragmentBufferSize;
    if (fragmentBufferSizeBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.",
                false);
        fragmentBufferSizeBytes = (1ull << 32ull) - 12ull;
        fragmentBufferSize = fragmentBufferSizeBytes / 12ull;
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
}

void PerPixelLinkedListLineRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    paddedWindowWidth = width, paddedWindowHeight = height;
    getScreenSizeWithTiling(paddedWindowWidth, paddedWindowHeight);

    reallocateFragmentBuffer();

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * paddedWindowWidth * paddedWindowHeight;
    startOffsetBuffer = {}; // Delete old data first (-> refcount 0)
    startOffsetBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), startOffsetBufferSizeBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    fragmentCounterBuffer = {}; // Delete old data first (-> refcount 0)
    fragmentCounterBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    lineRasterPass->recreateSwapchain(width, height);
    if (hullRasterPass) {
        hullRasterPass->recreateSwapchain(width, height);
    }

    resolveRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    resolveRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);

    clearRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    clearRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void PerPixelLinkedListLineRenderer::onClearColorChanged() {
    resolveRasterPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void PerPixelLinkedListLineRenderer::render() {
    LineRenderer::renderBase();

    uniformData.linkedListSize = static_cast<uint32_t>(fragmentBufferSize);
    uniformData.viewportW = paddedWindowWidth;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

    if ((*sceneData->performanceMeasurer)) {
        timer->startGPU("PPLLClear");
        clear();
        timer->endGPU("PPLLClear");
        timer->startGPU("FCGather");
        gather();
        timer->endGPU("FCGather");
        timer->startGPU("PPLLResolve");
        resolve();
        timer->endGPU("PPLLResolve");
    } else {
        clear();
        gather();
        resolve();
    }
    frameCounter++;
}

void PerPixelLinkedListLineRenderer::clear() {
    clearRasterPass->render();
    fragmentCounterBuffer->fill(0, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            startOffsetBuffer);
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            fragmentCounterBuffer);
}

void PerPixelLinkedListLineRenderer::gather() {
    //renderer->setProjectionMatrix((*sceneData->camera)->getProjectionMatrix());
    //renderer->setViewMatrix((*sceneData->camera)->getViewMatrix());
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

void PerPixelLinkedListLineRenderer::resolve() {
    resolveRasterPass->render();
}

void PerPixelLinkedListLineRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addCombo(
            "Sorting Mode", (int*)&sortingAlgorithmMode,
            SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
        reloadResolveShader();
        reRender = true;
    }
    if (propertyEditor.addButton("Reload Gather Shader", "Reload")) {
        reloadGatherShader();
        reRender = true;
    }
}
