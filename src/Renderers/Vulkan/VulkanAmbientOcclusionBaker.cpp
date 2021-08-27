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

#include <Utils/AppSettings.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/AccelerationStructure.hpp>
#include <Graphics/Vulkan/Utils/Interop.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>

#include "LineData/LineData.hpp"
#include "Renderers/LineRenderer.hpp"
#include "VulkanAmbientOcclusionBaker.hpp"

VulkanAmbientOcclusionBaker::VulkanAmbientOcclusionBaker(
        sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk)
        : AmbientOcclusionBaker(transferFunctionWindow, rendererVk) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    renderReadySemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);
    renderFinishedSemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);

    aoComputeRenderPass = std::make_shared<AmbientOcclusionComputeRenderPass>(rendererVk, aoBufferVk, aoBufferGl);
}

void VulkanAmbientOcclusionBaker::startAmbientOcclusionBaking(LineDataPtr& lineData) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    if (lineData) {
        aoComputeRenderPass->setLineData(lineData);
    }

    if (aoComputeRenderPass->getNumLineVertices() == 0) {
        return;
    }

    VkCommandPool commandPool;
    sgl::vk::CommandPoolType commandPoolType;
    commandPoolType.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    commandPoolType.queueFamilyIndex = device->getComputeQueueIndex();
    commandBuffers = device->allocateCommandBuffers(commandPoolType, &commandPool, maxNumIterations);

    waitSemaphores.resize(maxNumIterations);
    signalSemaphores.resize(maxNumIterations);

    for (size_t i = 1; i < maxNumIterations; i++) {
        waitSemaphores.at(i) = std::make_shared<sgl::vk::Semaphore>(device);
        signalSemaphores.at(i - 1) = waitSemaphores.at(i);
    }
    waitSemaphores.front() = renderReadySemaphore;
    signalSemaphores.back() = renderFinishedSemaphore;

    renderReadySemaphore->signalSemaphoreGl(aoBufferGl);

    for (numIterations = 0; numIterations < maxNumIterations; numIterations++) {
        aoComputeRenderPass->setFrameNumber(numIterations);
        rendererVk->setCustomCommandBuffer(commandBuffers.at(numIterations), false);
        rendererVk->beginCommandBuffer();
        aoComputeRenderPass->render();
        rendererVk->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
        rendererVk->endCommandBuffer();

        // Submit the rendering operation in Vulkan.
        sgl::vk::FencePtr fence;
        //sgl::vk::SemaphorePtr renderReadySemaphoreVk =
        //        std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderReadySemaphore);
        //sgl::vk::SemaphorePtr renderFinishedSemaphoreVk =
        //        std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderFinishedSemaphore);
        rendererVk->submitToQueue(
                waitSemaphores.at(numIterations), signalSemaphores.at(numIterations), fence,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    }
    rendererVk->resetCustomCommandBuffer();

    // Wait for the rendering to finish on the Vulkan side.
    renderFinishedSemaphore->waitSemaphoreGl(aoBufferGl);
}

bool VulkanAmbientOcclusionBaker::getHasComputationFinished() {
    return aoBufferGl.get() != nullptr;
}

sgl::GeometryBufferPtr& VulkanAmbientOcclusionBaker::getAmbientOcclusionBuffer() {
    return aoComputeRenderPass->getAmbientOcclusionBuffer();
}

sgl::GeometryBufferPtr& VulkanAmbientOcclusionBaker::getBlendingWeightsBuffer() {
    return aoComputeRenderPass->getBlendingWeightsBuffer();
}

sgl::vk::BufferPtr& VulkanAmbientOcclusionBaker::getAmbientOcclusionBufferVulkan() {
    return aoComputeRenderPass->getAmbientOcclusionBufferVulkan();
}

sgl::vk::BufferPtr& VulkanAmbientOcclusionBaker::getBlendingWeightsBufferVulkan() {
    return aoComputeRenderPass->getBlendingWeightsBufferVulkan();
}

uint32_t VulkanAmbientOcclusionBaker::getNumTubeSubdivisions() {
    return aoComputeRenderPass->getNumTubeSubdivisions();
}

uint32_t VulkanAmbientOcclusionBaker::getNumLineVertices() {
    return aoComputeRenderPass->getNumLineVertices();
}

uint32_t VulkanAmbientOcclusionBaker::getNumParametrizationVertices() {
    return aoComputeRenderPass->getNumParametrizationVertices();
}

bool VulkanAmbientOcclusionBaker::renderGui() {
    bool dirty = false;

    if (ImGui::Begin("RTAO Baking", &showWindow)) {
        ImGui::Checkbox("Use Main Thread", &useMainThread);

        if (ImGui::SliderIntEdit(
                "#Iterations", &maxNumIterations, 1, 4096) == EditMode::INPUT_FINISHED) {
            dirty = true;
        }
        if (ImGui::SliderFloatEdit(
                "Line Resolution", &aoComputeRenderPass->expectedParamSegmentLength,
                0.0001f, 0.01f, "%.4f") == EditMode::INPUT_FINISHED) {
            dirty = true;
        }
        if (ImGui::SliderIntEdit(
                "#Subdivisions", reinterpret_cast<int*>(&aoComputeRenderPass->numTubeSubdivisions),
                3, 16) == EditMode::INPUT_FINISHED) {
            dirty = true;
        }
        if (ImGui::SliderIntEdit(
                "#Samples/Frame",
                reinterpret_cast<int*>(&aoComputeRenderPass->numAmbientOcclusionSamplesPerFrame),
                1, 4096) == EditMode::INPUT_FINISHED) {
            dirty = true;
        }
        if (ImGui::SliderFloatEdit(
                "AO Radius", &aoComputeRenderPass->ambientOcclusionRadius,
                0.01f, 0.2f) == EditMode::INPUT_FINISHED) {
            dirty = true;
        }
        if (ImGui::Checkbox("Use Distance-based AO", &aoComputeRenderPass->useDistance)) {
            dirty = true;
        }

        ImGui::End();
    }

    if (dirty) {
        aoComputeRenderPass->generateBlendingWeightParametrization();
        LineDataPtr lineData;
        startAmbientOcclusionBaking(lineData);
    }

    return dirty;
}


AmbientOcclusionComputeRenderPass::AmbientOcclusionComputeRenderPass(
        sgl::vk::Renderer* renderer, sgl::vk::BufferPtr& aoBufferVk, sgl::GeometryBufferPtr& aoBufferGl)
        : ComputePass(renderer), aoBufferVk(aoBufferVk), aoBufferGl(aoBufferGl) {
    lineRenderSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(LineRenderSettings),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void AmbientOcclusionComputeRenderPass::setLineData(LineDataPtr& lineData) {
    topLevelAS = lineData->getRayTracingTubeTriangleTopLevelAS();
    lines = lineData->getFilteredLines();

    if (this->lineData && this->lineData->getType() != lineData->getType()) {
        setShaderDirty();
    }
    this->lineData = lineData;

    linePointsBuffer = lineData->getVulkanTubeTriangleRenderData(true).linePointBuffer;

    numLineVertices = linePointsBuffer ? uint32_t(linePointsBuffer->getSizeInBytes() / sizeof(TubeLinePointData)) : 0;
    if (numLineVertices != 0) {
        generateBlendingWeightParametrization();
    } else {
        linesLengthSum = 0.0f;
        numPolylineSegments = 0;
        polylineLengths.clear();

        numParametrizationVertices = 0;

        blendingWeightParametrizationBuffer = sgl::vk::BufferPtr();
        blendingWeightParametrizationBufferGl = sgl::GeometryBufferPtr();

        lineSegmentVertexConnectivityBuffer = sgl::vk::BufferPtr();

        samplingLocationsBuffer = sgl::vk::BufferPtr();

        aoBufferVk = sgl::vk::BufferPtr();
        aoBufferGl = sgl::GeometryBufferPtr();
    }
}

void AmbientOcclusionComputeRenderPass::generateBlendingWeightParametrization() {
    // First, compute data necessary for parametrizing the polylines (number of segments, segment lengths).
    linesLengthSum = 0.0f;
    numPolylineSegments = 0;
    polylineLengths.clear();
    polylineLengths.shrink_to_fit();
    polylineLengths.resize(lines.size());

#if _OPENMP >= 201107
    #pragma omp parallel for reduction(+: linesLengthSum) reduction(+: numPolylineSegments) shared(polylineLengths) \
    default(none)
#endif
    for (size_t lineIdx = 0; lineIdx < lines.size(); lineIdx++) {
        std::vector<glm::vec3>& line = lines.at(lineIdx);
        const size_t n = line.size();
        float polylineLength = 0.0f;
        for (size_t i = 1; i < n; i++) {
            polylineLength += glm::length(line[i] - line[i-1]);
        }
        polylineLengths.at(lineIdx) = polylineLength;
        linesLengthSum += polylineLength;
        numPolylineSegments += n - 1;
    }

    recomputeStaticParametrization();
}

void AmbientOcclusionComputeRenderPass::recomputeStaticParametrization() {
    std::vector<float> blendingWeightParametrizationData(numLineVertices, 0);
    std::vector<glm::uvec2> lineSegmentVertexConnectivityData;
    std::vector<float> samplingLocations;

    const float EPSILON = 1e-5f;
    const int approximateLineSegmentsTotal = int(std::ceil(linesLengthSum / expectedParamSegmentLength));
    lineSegmentVertexConnectivityData.reserve(approximateLineSegmentsTotal);
    samplingLocations.reserve(approximateLineSegmentsTotal);

    size_t segmentVertexIdOffset = 0;
    size_t vertexIdx = 0;
    for (size_t lineIdx = 0; lineIdx < lines.size(); lineIdx++) {
        std::vector<glm::vec3>& line = lines.at(lineIdx);
        const size_t n = line.size();
        float polylineLength = polylineLengths.at(lineIdx);

        uint32_t numLineSubdivs = std::max(1u, uint32_t(std::ceil(polylineLength / expectedParamSegmentLength)));
        float lineSubdivLength = polylineLength / float(numLineSubdivs);
        uint32_t numSubdivVertices = numLineSubdivs + 1;

        // Set the first vertex manually (we can guarantee there is no segment before it).
        assert(line.size() >= 2);
        auto startVertexIdx = uint32_t(vertexIdx);
        blendingWeightParametrizationData.at(vertexIdx) = float(segmentVertexIdOffset);
        vertexIdx++;

        // Compute the per-vertex blending weight parametrization.
        float currentLength = 0.0f;
        for (size_t i = 1; i < n; i++) {
            currentLength += glm::length(line[i] - line[i-1]);
            float w = currentLength / lineSubdivLength;
            blendingWeightParametrizationData.at(vertexIdx) =
                    float(segmentVertexIdOffset) + glm::clamp(w, 0.0f, float(numLineSubdivs) - EPSILON);
            vertexIdx++;
        }

        float lastLength = 0.0f;
        currentLength = glm::length(line[1] - line[0]);
        size_t currVertexIdx = 1;
        samplingLocations.push_back(float(startVertexIdx));
        for (uint32_t i = 1; i < numSubdivVertices; i++) {
            auto parametrizationIdx = uint32_t(currentLength / lineSubdivLength);
            while (i > parametrizationIdx && currVertexIdx < n - 1) {
                float segLength = glm::length(line[currVertexIdx + 1] - line[currVertexIdx]);
                lastLength = currentLength;
                currentLength += segLength;
                parametrizationIdx = uint32_t(currentLength / lineSubdivLength);
                currVertexIdx++;
            }

            float samplingLocation =
                    float(currVertexIdx - 1)
                    + (float(i) * lineSubdivLength - lastLength) / (currentLength - lastLength);
            samplingLocation = float(startVertexIdx) + std::min(samplingLocation, float(uint32_t(n) - 1u) - EPSILON);
            samplingLocations.push_back(samplingLocation);
        }

        if (numSubdivVertices == 1) {
            lineSegmentVertexConnectivityData.emplace_back(segmentVertexIdOffset, segmentVertexIdOffset);
        } else {
            lineSegmentVertexConnectivityData.emplace_back(segmentVertexIdOffset, segmentVertexIdOffset + 1);
            for (size_t i = 1; i < numSubdivVertices - 1; i++) {
                lineSegmentVertexConnectivityData.emplace_back(
                        segmentVertexIdOffset + i - 1u, segmentVertexIdOffset + i + 1u);
            }
            lineSegmentVertexConnectivityData.emplace_back(
                    segmentVertexIdOffset + numSubdivVertices - 2u, segmentVertexIdOffset + numSubdivVertices - 1u);
        }

        segmentVertexIdOffset += numSubdivVertices;
    }
    numParametrizationVertices = samplingLocations.size();

    blendingWeightParametrizationBuffer = std::make_shared<sgl::vk::Buffer>(
            device, numLineVertices * sizeof(float), blendingWeightParametrizationData.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY, true, true);
    blendingWeightParametrizationBufferGl = sgl::GeometryBufferPtr(new sgl::GeometryBufferGLExternalMemoryVk(
            blendingWeightParametrizationBuffer, sgl::SHADER_STORAGE_BUFFER));

    lineSegmentVertexConnectivityBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineSegmentVertexConnectivityData.size() * sizeof(glm::uvec2),
            lineSegmentVertexConnectivityData.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    samplingLocationsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, samplingLocations.size() * sizeof(float), samplingLocations.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    aoBufferVk = std::make_shared<sgl::vk::Buffer>(
            device, samplingLocations.size() * numTubeSubdivisions * sizeof(float),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
            true, true);
    aoBufferGl = sgl::GeometryBufferPtr(new sgl::GeometryBufferGLExternalMemoryVk(
            aoBufferVk, sgl::SHADER_STORAGE_BUFFER));

    dataDirty = true;
}

void AmbientOcclusionComputeRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            {"VulkanAmbientOcclusionBaker.Compute"},
            lineData->getVulkanShaderPreprocessorDefines());
}

void AmbientOcclusionComputeRenderPass::setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) {
}

void AmbientOcclusionComputeRenderPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(lineRenderSettingsBuffer, "UniformsBuffer");
    computeData->setStaticBuffer(linePointsBuffer, "TubeLinePointDataBuffer");
    computeData->setStaticBuffer(samplingLocationsBuffer, "SamplingLocationsBuffer");
    computeData->setStaticBuffer(aoBufferVk, "AmbientOcclusionFactorsBuffer");
    computeData->setTopLevelAccelerationStructure(topLevelAS, "topLevelAS");
    lineData->setVulkanRenderDataDescriptors(computeData);
}

void AmbientOcclusionComputeRenderPass::_render() {
    lineData->updateVulkanUniformBuffers(renderer);

    lineRenderSettings.lineRadius = LineRenderer::getLineWidth() * 0.5f;
    lineRenderSettings.ambientOcclusionRadius = ambientOcclusionRadius;
    lineRenderSettings.numLinePoints = numLineVertices;
    lineRenderSettings.numParametrizationVertices = numParametrizationVertices;
    lineRenderSettings.numTubeSubdivisions = numTubeSubdivisions;
    lineRenderSettings.numAmbientOcclusionSamples = numAmbientOcclusionSamplesPerFrame;
    lineRenderSettings.useDistance = int(useDistance);
    lineRenderSettingsBuffer->updateData(
            sizeof(LineRenderSettings), &lineRenderSettings, renderer->getVkCommandBuffer());

    renderer->dispatch(
            computeData, samplingLocationsBuffer->getSizeInBytes() / sizeof(float),
            1, 1);
}
