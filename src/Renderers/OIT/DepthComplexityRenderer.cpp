/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2022, Christoph Neuhauser
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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/Timer.hpp>
#include <Utils/AppSettings.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "../HullRasterPass.hpp"
#include "DepthComplexityRenderer.hpp"

DepthComplexityRenderer::DepthComplexityRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineRenderer("Depth Complexity Renderer", sceneData, transferFunctionWindow) {
}

void DepthComplexityRenderer::initialize() {
    LineRenderer::initialize();

    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    resolveRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"DepthComplexityResolve.Vertex", "DepthComplexityResolve.Fragment"}));
    resolveRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);

    clearRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"DepthComplexityClear.Vertex", "DepthComplexityClear.Fragment"}));
    clearRasterPass->setColorWriteEnabled(false);
    clearRasterPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    clearRasterPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_DONT_CARE);
    clearRasterPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
    clearRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    onClearColorChanged();
}

void DepthComplexityRenderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();
}

void DepthComplexityRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    firstFrame = true;
    totalNumFragments = 0;
    usedLocations = 1;
    maxComplexity = 0;
    bufferSize = 1;
    intensity = 1.5f;

    dirty = false;
    reRender = true;
}

void DepthComplexityRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"DepthComplexityGatherInc.glsl\""));
}

void DepthComplexityRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setColorWriteEnabled(false);
    pipelineInfo.setDepthWriteEnabled(false);
    pipelineInfo.setDepthTestEnabled(false);
}

void DepthComplexityRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    renderData->setStaticBufferOptional(fragmentCounterBuffer, "FragmentCounterBuffer");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
}

void DepthComplexityRenderer::updateVulkanUniformBuffers() {
}

void DepthComplexityRenderer::setFramebufferAttachments(
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

void DepthComplexityRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);

    size_t fragmentCounterBufferSizeBytes = sizeof(uint32_t) * width * height;
    fragmentCounterBuffer = {}; // Delete old data first (-> refcount 0)
    fragmentCounterBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), fragmentCounterBufferSizeBytes,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    fragmentCounterBuffer->fill(0, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT,
            VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            fragmentCounterBuffer);

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    stagingBuffers.reserve(swapchain->getNumImages());
    for (size_t i = 0; i < swapchain->getNumImages(); i++) {
        stagingBuffers.push_back(std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), fragmentCounterBufferSizeBytes,
                VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU));
    }

    lineRasterPass->recreateSwapchain(width, height);
    if (hullRasterPass) {
        hullRasterPass->recreateSwapchain(width, height);
    }

    resolveRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    resolveRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);

    clearRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    clearRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void DepthComplexityRenderer::onClearColorChanged() {
    resolveRasterPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void DepthComplexityRenderer::setUniformData() {
    int width = int(*sceneData->viewportWidth);
    uniformData.viewportW = width;
    uniformData.numFragmentsMaxColor = numFragmentsMaxColor;
    uniformData.color = renderColor.getFloatColorRGBA();
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void DepthComplexityRenderer::clear() {
    clearRasterPass->render();
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void DepthComplexityRenderer::gather() {
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

void DepthComplexityRenderer::resolve() {
    resolveRasterPass->render();
}

void DepthComplexityRenderer::render() {
    LineRenderer::renderBase();

    setUniformData();
    clear();
    gather();
    resolve();

    if ((*sceneData->performanceMeasurer) != nullptr || (*sceneData->recordingMode)) {
        computeStatistics(false);
    }
}

// Converts e.g. 123456789 to "123,456,789"
std::string numberToCommaString(int64_t number, bool attachLeadingZeroes = false) {
    if (number < 0) {
        return std::string() + "-" + numberToCommaString(-number, attachLeadingZeroes);
    } else if (number < 1000) {
        return sgl::toString(number);
    } else {
        std::string numberString = sgl::toString(number % 1000);
        while (attachLeadingZeroes && numberString.size() < 3) {
            numberString = "0" + numberString;
        }
        return std::string() + numberToCommaString(number / 1000, true) + "," + numberString;
    }
}

void DepthComplexityRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    std::string totalNumFragmentsString = numberToCommaString(int64_t(totalNumFragments));
    propertyEditor.addText("#Fragments", totalNumFragmentsString);
    propertyEditor.addText(
            "Average Used",
            sgl::toString(double(totalNumFragments) / double(usedLocations), 2));
    propertyEditor.addText(
            "Average All",
            sgl::toString(double(totalNumFragments) / double(bufferSize), 2));
    propertyEditor.addText("Max. Complexity", sgl::toString(maxComplexity));

    if (propertyEditor.addColorEdit4("Coloring", (float*)&colorSelection, 0)) {
        sgl::Color newColor = sgl::colorFromFloat(colorSelection.x, colorSelection.y, colorSelection.z, 1.0f);
        renderColor = newColor;
        intensity = 0.0001f + 3*colorSelection.w;
        numFragmentsMaxColor = uint32_t(float(std::max(maxComplexity, uint64_t(4ull))) / intensity);
        reRender = true;
    }
}

bool DepthComplexityRenderer::needsReRender() {
    bool reRender = LineRenderer::needsReRender();

    // Update & print statistics if enough time has passed
    static float counterPrintFrags = 0.0f;
    counterPrintFrags += sgl::Timer->getElapsedSeconds();
    if (lineData && (counterPrintFrags > 1.0f || firstFrame)) {
        computeStatistics(true);
        counterPrintFrags = 0.0f;
        firstFrame = false;
        reRender = true;
    }

    return reRender;
}

void DepthComplexityRenderer::computeStatistics(bool isReRender) {
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    bufferSize = width * height;

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    auto& stagingBuffer = stagingBuffers.at(swapchain->getImageIndex());
    fragmentCounterBuffer->copyDataTo(stagingBuffer, renderer->getVkCommandBuffer());
    renderer->syncWithCpu();

    auto *data = (uint32_t*)stagingBuffer->mapMemory();

    // Local reduction variables necessary for older OpenMP implementations
    uint64_t totalNumFragments = 0;
    uint64_t usedLocations = 0;
    uint64_t maxComplexity = 0;
    uint64_t minComplexity = 0;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(+:totalNumFragments,usedLocations) reduction(max:maxComplexity) \
    reduction(min:minComplexity) schedule(static) default(none) shared(data)
#endif
    for (uint64_t i = 0; i < bufferSize; i++) {
        totalNumFragments += data[i];
        if (data[i] > 0) {
            usedLocations++;
        }
        maxComplexity = std::max(maxComplexity, (uint64_t)data[i]);
        minComplexity = std::min(maxComplexity, (uint64_t)data[i]);
    }
    if (totalNumFragments == 0) usedLocations = 1; // Avoid dividing by zero in code below
    this->totalNumFragments = totalNumFragments;
    this->usedLocations = usedLocations;
    this->maxComplexity = maxComplexity;

    stagingBuffer->unmapMemory();

    bool performanceMeasureMode = (*sceneData->performanceMeasurer) != nullptr;
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
    }

    if (totalNumFragments == 0) usedLocations = 1; // Avoid dividing by zero in code below
    std::cout << "Depth complexity: avg used: " << ((float)totalNumFragments / usedLocations)
              << ", avg all: " << ((float)totalNumFragments / bufferSize) << ", max: " << maxComplexity
              << ", #fragments: " << totalNumFragments << std::endl;
}
