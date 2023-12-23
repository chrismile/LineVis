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

#ifdef USE_TBB
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#endif

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/Timer.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include <ImGui/Widgets/NumberFormatting.hpp>

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
    maxStorageBufferSize = std::min(
            size_t(device->getMaxMemoryAllocationSize()), size_t(device->getMaxStorageBufferRange()));

    auto memoryHeapIndex = uint32_t(device->findMemoryHeapIndex(VK_MEMORY_HEAP_DEVICE_LOCAL_BIT));
    size_t availableVram = device->getMemoryHeapBudgetVma(memoryHeapIndex);
    double availableMemoryFactor = 28.0 / 32.0;
    maxDeviceMemoryBudget = size_t(double(availableVram) * availableMemoryFactor);

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
        if (*sceneData->viewportWidth > 0 && *sceneData->viewportHeight > 0) {
            reallocateFragmentBuffer();
        }
        reloadResolveShader();
    }
}

void PerPixelLinkedListLineRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);
    updateLargeMeshMode();

    statisticsUpToDate = false;
    counterPrintFrags = 0.0f;
    firstFrame = true;
    totalNumFragments = 0;
    usedLocations = 1;
    maxComplexity = 0;
    bufferSize = 1;

    dirty = false;
    reRender = true;
}

void PerPixelLinkedListLineRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\""));

    if (showDepthComplexity) {
        preprocessorDefines.insert(std::make_pair("SHOW_DEPTH_COMPLEXITY", ""));
    }

    if (fragmentBufferMode == FragmentBufferMode::BUFFER_REFERENCE_ARRAY) {
        preprocessorDefines.insert(std::make_pair("FRAGMENT_BUFFER_REFERENCE_ARRAY", ""));
    } else if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY) {
        preprocessorDefines.insert(std::make_pair("FRAGMENT_BUFFER_ARRAY", ""));
    }
    if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY
            || fragmentBufferMode == FragmentBufferMode::BUFFER_REFERENCE_ARRAY) {
        preprocessorDefines.insert(std::make_pair("NUM_FRAGMENT_BUFFERS", std::to_string(cachedNumFragmentBuffers)));
        preprocessorDefines.insert(std::make_pair("NUM_FRAGS_PER_BUFFER", std::to_string(maxStorageBufferSize / 12ull) + "u"));
        auto it = preprocessorDefines.find("__extensions");
        std::string extensionString;
        if (it != preprocessorDefines.end()) {
            extensionString = it->second + ";";
        }
        if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY) {
            extensionString += "GL_EXT_nonuniform_qualifier";
        } else if (fragmentBufferMode == FragmentBufferMode::BUFFER_REFERENCE_ARRAY) {
            extensionString += "GL_EXT_shader_explicit_arithmetic_types_int64;GL_EXT_buffer_reference";
        }
        preprocessorDefines["__extensions"] = extensionString;
    }

    preprocessorDefines.insert(std::make_pair("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity)));
    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
            || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = int(std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4);
        preprocessorDefines.insert(std::make_pair("STACK_SIZE", sgl::toString(stackSize)));
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_PRIORITY_QUEUE) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "frontToBackPQ"));
        if (renderer->getDevice()->getDeviceDriverId() == VK_DRIVER_ID_AMD_PROPRIETARY
                || renderer->getDevice()->getDeviceDriverId() == VK_DRIVER_ID_AMD_OPEN_SOURCE) {
            preprocessorDefines.insert(std::make_pair("INITIALIZE_ARRAY_POW2", ""));
        }
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
    if (fragmentBufferMode == FragmentBufferMode::BUFFER) {
        renderData->setStaticBufferOptional(fragmentBuffer, "FragmentBuffer");
    } else if (fragmentBufferMode == FragmentBufferMode::BUFFER_ARRAY) {
        renderData->setStaticBufferArrayOptional(fragmentBuffers, "FragmentBuffer");
    } else {
        renderData->setStaticBufferOptional(fragmentBufferReferenceBuffer, "FragmentBuffer");
    }
    renderData->setStaticBuffer(startOffsetBuffer, "StartOffsetBuffer");
    renderData->setStaticBufferOptional(fragmentCounterBuffer, "FragCounterBuffer");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");

    if (showDepthComplexity) {
        renderData->setStaticBuffer(depthComplexityCounterBuffer, "DepthComplexityCounterBuffer");
    }
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

    // Delete old data first (-> refcount 0)
    fragmentBuffers = {};
    fragmentBuffer = {};
    fragmentBufferReferenceBuffer = {};
    resolveRasterPass->clearFragmentBuffer();
    clearRasterPass->clearFragmentBuffer();
    if (lineRasterPass) {
        lineRasterPass->clearFragmentBuffer();
    }
    if (hullRasterPass) {
        hullRasterPass->clearFragmentBuffer();
    }

    // We only need buffer arrays when the maximum allocation is larger than our budget.
    if (maxDeviceMemoryBudget < maxStorageBufferSize) {
        fragmentBufferMode = FragmentBufferMode::BUFFER;
        resolveRasterPass->setShaderDirty();
        clearRasterPass->setShaderDirty();
        reloadGatherShader();
    }
    size_t maxSingleBufferAllocation = std::min(maxDeviceMemoryBudget, maxStorageBufferSize);

    if (fragmentBufferMode == FragmentBufferMode::BUFFER) {
        if (fragmentBufferSizeBytes > maxSingleBufferAllocation) {
            sgl::Logfile::get()->writeError(
                    std::string() + "Fragment buffer size was larger than maximum allocation size ("
                    + std::to_string(maxSingleBufferAllocation) + "). Clamping to maximum allocation size.",
                    false);
            fragmentBufferSize = maxSingleBufferAllocation / 12ull;
            fragmentBufferSizeBytes = fragmentBufferSize * 12ull;
        } else {
            sgl::Logfile::get()->writeInfo(
                    std::string() + "Fragment buffer size GiB: "
                    + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
        }

        numFragmentBuffers = 1;
        cachedNumFragmentBuffers = 1;
        fragmentBuffer = std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), fragmentBufferSizeBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    } else {
        if (fragmentBufferSizeBytes > maxDeviceMemoryBudget) {
            sgl::Logfile::get()->writeError(
                    std::string() + "Fragment buffer size was larger than maximum allocation size ("
                    + std::to_string(maxDeviceMemoryBudget) + "). Clamping to maximum allocation size.",
                    false);
            fragmentBufferSize = maxDeviceMemoryBudget / 12ull;
            fragmentBufferSizeBytes = fragmentBufferSize * 12ull;
        } else {
            sgl::Logfile::get()->writeInfo(
                    std::string() + "Fragment buffer size GiB: "
                    + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
        }

        numFragmentBuffers = sgl::sizeceil(fragmentBufferSizeBytes, maxStorageBufferSize);
        size_t fragmentBufferSizeBytesLeft = fragmentBufferSizeBytes;
        for (size_t i = 0; i < numFragmentBuffers; i++) {
            VkBufferUsageFlags flags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            if (fragmentBufferMode == FragmentBufferMode::BUFFER_REFERENCE_ARRAY) {
                flags |= VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
            }
            fragmentBuffers.emplace_back(std::make_shared<sgl::vk::Buffer>(
                    renderer->getDevice(), std::min(fragmentBufferSizeBytesLeft, maxStorageBufferSize),
                    flags, VMA_MEMORY_USAGE_GPU_ONLY));
            fragmentBufferSizeBytesLeft -= maxStorageBufferSize;
        }

        if (numFragmentBuffers != cachedNumFragmentBuffers) {
            cachedNumFragmentBuffers = numFragmentBuffers;
            resolveRasterPass->setShaderDirty();
            clearRasterPass->setShaderDirty();
            reloadGatherShader();
        }

        if (fragmentBufferMode == FragmentBufferMode::BUFFER_REFERENCE_ARRAY) {
            auto* bufferReferences = new uint64_t[numFragmentBuffers];
            for (size_t i = 0; i < numFragmentBuffers; i++) {
                bufferReferences[i] = fragmentBuffers.at(i)->getVkDeviceAddress();
            }
            fragmentBufferReferenceBuffer = std::make_shared<sgl::vk::Buffer>(
                    renderer->getDevice(), sizeof(uint64_t) * numFragmentBuffers,
                    VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                    VMA_MEMORY_USAGE_GPU_ONLY);
            fragmentBufferReferenceBuffer->updateData(
                    sizeof(uint64_t) * numFragmentBuffers, bufferReferences, renderer->getVkCommandBuffer());
            renderer->insertBufferMemoryBarrier(
                    VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                    fragmentBufferReferenceBuffer);
            delete[] bufferReferences;
        }
    }

    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }
}

void PerPixelLinkedListLineRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    windowWidth = width, windowHeight = height;
    paddedWindowWidth = width, paddedWindowHeight = height;
    getScreenSizeWithTiling(paddedWindowWidth, paddedWindowHeight);

    createDepthComplexityBuffers();
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
    uniformData.viewportLinearW = windowWidth;
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
    if (showDepthComplexity) {
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                depthComplexityCounterBuffer);
    }
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
    if (propertyEditor.addCombo(
            "Fragment Buffer Mode", (int*)&fragmentBufferMode,
            FRAGMENT_BUFFER_MODE_NAMES, IM_ARRAYSIZE(FRAGMENT_BUFFER_MODE_NAMES))) {
        renderer->syncWithCpu();
        renderer->getDevice()->waitIdle();
        resolveRasterPass->setShaderDirty();
        clearRasterPass->setShaderDirty();
        reloadGatherShader();
        reallocateFragmentBuffer();
        reRender = true;
    }
    bool depthComplexityJustChanged = false;
    if (propertyEditor.addCheckbox("Show Depth Complexity", &showDepthComplexity)) {
        resolveRasterPass->setShaderDirty();
        clearRasterPass->setShaderDirty();
        reloadGatherShader();
        reRender = true;
        depthComplexityJustChanged = true;
        createDepthComplexityBuffers();
    }
    if (showDepthComplexity && !depthComplexityJustChanged) {
        std::string totalNumFragmentsString = sgl::numberToCommaString(int64_t(totalNumFragments));
        propertyEditor.addText("#Fragments", totalNumFragmentsString);
        propertyEditor.addText(
                "Average Used",
                sgl::toString(double(totalNumFragments) / double(usedLocations), 2));
        propertyEditor.addText(
                "Average All",
                sgl::toString(double(totalNumFragments) / double(bufferSize), 2));
        propertyEditor.addText(
                "Max. Complexity", sgl::toString(maxComplexity) + " / " + sgl::toString(expectedMaxDepthComplexity));
        propertyEditor.addText(
                "Memory",
                sgl::getNiceMemoryString(totalNumFragments * 12ull, 2) + " / "
                + sgl::getNiceMemoryString(fragmentBufferSize * 12ull, 2));
    }
    if (propertyEditor.addButton("Reload Gather Shader", "Reload")) {
        reloadGatherShader();
        reRender = true;
    }
}

void PerPixelLinkedListLineRenderer::createDepthComplexityBuffers() {
    depthComplexityCounterBuffer = {};
    stagingBuffers.clear();
    if (!showDepthComplexity) {
        return;
    }

    int width = windowWidth;
    int height = windowHeight;

    size_t depthComplexityCounterBufferSizeBytes = sizeof(uint32_t) * width * height;
    depthComplexityCounterBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), depthComplexityCounterBufferSizeBytes,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    depthComplexityCounterBuffer->fill(0, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT,
            VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            depthComplexityCounterBuffer);

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    stagingBuffers.reserve(swapchain->getNumImages());
    for (size_t i = 0; i < swapchain->getNumImages(); i++) {
        stagingBuffers.push_back(std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), depthComplexityCounterBufferSizeBytes,
                VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU));
    }
}

bool PerPixelLinkedListLineRenderer::needsReRender() {
    bool reRender = LineRenderer::needsReRender();

    if (showDepthComplexity && !statisticsUpToDate) {
        // Update & print statistics if enough time has passed
        counterPrintFrags += sgl::Timer->getElapsedSeconds();
        if (lineData && (counterPrintFrags > 1.0f || firstFrame)) {
            if (!firstFrame) {
                statisticsUpToDate = true;
            }
            computeStatistics(true);
            counterPrintFrags = 0.0f;
            firstFrame = false;
        }
    }

    return reRender;
}

void PerPixelLinkedListLineRenderer::onHasMoved() {
    LineRenderer::onHasMoved();
    statisticsUpToDate = false;
    counterPrintFrags = 0.0f;
}

void PerPixelLinkedListLineRenderer::computeStatistics(bool isReRender) {
    int width = windowWidth;
    int height = windowHeight;
    bufferSize = width * height;

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    auto& stagingBuffer = stagingBuffers.at(swapchain->getImageIndex());
    depthComplexityCounterBuffer->copyDataTo(stagingBuffer, renderer->getVkCommandBuffer());
    renderer->syncWithCpu();

    auto *data = (uint32_t*)stagingBuffer->mapMemory();

    // Local reduction variables necessary for older OpenMP implementations
    uint64_t minComplexity = 0;
#ifdef USE_TBB
    using T = std::tuple<uint64_t, uint64_t, uint64_t, uint64_t>;
    std::tie(totalNumFragments, usedLocations, maxComplexity, minComplexity) = tbb::parallel_reduce(
            tbb::blocked_range<uint64_t>(0, bufferSize), T{},
            [&data](tbb::blocked_range<uint64_t> const& r, T init) {
                uint64_t& totalNumFragments = std::get<0>(init);
                uint64_t& usedLocations = std::get<1>(init);
                uint64_t& maxComplexity = std::get<2>(init);
                uint64_t& minComplexity = std::get<3>(init);
                for (auto i = r.begin(); i != r.end(); i++) {
#else
    uint64_t totalNumFragments = 0;
    uint64_t usedLocations = 0;
    uint64_t maxComplexity = 0;
#if _OPENMP >= 201107
#pragma omp parallel for reduction(+:totalNumFragments,usedLocations) reduction(max:maxComplexity) \
    reduction(min:minComplexity) schedule(static) default(none) shared(data)
#endif
    for (uint64_t i = 0; i < bufferSize; i++) {
#endif
        totalNumFragments += data[i];
        if (data[i] > 0) {
            usedLocations++;
        }
        maxComplexity = std::max(maxComplexity, uint64_t(data[i]));
        minComplexity = std::min(minComplexity, uint64_t(data[i]));
    }
#ifdef USE_TBB
    return init;
            }, [&](T lhs, T rhs) -> T {
                return {
                        std::get<0>(lhs) + std::get<0>(lhs),
                        std::get<1>(lhs) + std::get<1>(lhs),
                        std::max(std::get<2>(lhs), std::get<2>(lhs)),
                        std::min(std::get<3>(lhs), std::get<3>(lhs))
                };
            });
#endif
    // Avoid dividing by zero in code below
    if (totalNumFragments == 0) {
        usedLocations = 1;
    }
#ifndef USE_TBB
    this->totalNumFragments = totalNumFragments;
    this->usedLocations = usedLocations;
    this->maxComplexity = maxComplexity;
#endif

    stagingBuffer->unmapMemory();

    firstFrame = false;

    /*bool performanceMeasureMode = (*sceneData->performanceMeasurer) != nullptr;
    if ((performanceMeasureMode || (*sceneData->recordingMode)) || firstFrame || true) {
        if (!isReRender) {
            firstFrame = false;
        }
        numFragmentsMaxColor = uint32_t(std::max(maxComplexity, uint64_t(4ull)) / intensity);
    }

    if (performanceMeasureMode) {
        (*sceneData->performanceMeasurer)->pushDepthComplexityFrame(
                minComplexity, maxComplexity,
                (float)totalNumFragments / usedLocations,
                (float)totalNumFragments / bufferSize, totalNumFragments);
    }*/

    /*if (totalNumFragments == 0) usedLocations = 1; // Avoid dividing by zero in code below
    std::cout << "Depth complexity: avg used: " << ((float)totalNumFragments / usedLocations)
              << ", avg all: " << ((float)totalNumFragments / bufferSize) << ", max: " << maxComplexity
              << ", #fragments: " << totalNumFragments << std::endl;*/
}
