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

#include "LineData/LineData.hpp"
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
    aoComputeRenderPass->setLineData(lineData);

    for (numIterations = 0; numIterations < maxNumIterations; numIterations++) {
        renderReadySemaphore->signalSemaphoreGl(aoBufferGl);

        rendererVk->beginCommandBuffer();
        aoComputeRenderPass->render();
        rendererVk->endCommandBuffer();

        // Submit the rendering operation in Vulkan.
        sgl::vk::FencePtr fence;
        sgl::vk::SemaphorePtr renderReadySemaphoreVk =
                std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderReadySemaphore);
        sgl::vk::SemaphorePtr renderFinishedSemaphoreVk =
                std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderFinishedSemaphore);
        rendererVk->submitToQueue(renderReadySemaphoreVk, renderFinishedSemaphoreVk, fence);

        // Wait for the rendering to finish on the Vulkan side.
        renderFinishedSemaphore->waitSemaphoreGl(aoBufferGl);
    }
}

bool VulkanAmbientOcclusionBaker::getHasComputationFinished() {
    return aoBufferGl.get() != nullptr;
}

sgl::GeometryBufferPtr VulkanAmbientOcclusionBaker::getAmbientOcclusionBuffer() {
    return aoBufferGl;
}

sgl::vk::BufferPtr VulkanAmbientOcclusionBaker::getAmbientOcclusionBufferVulkan() {
    return aoBufferVk;
}

void VulkanAmbientOcclusionBaker::renderGui() {
    ImGui::SliderInt("#Iterations", &maxNumIterations, 0, 4096);
    ImGui::SameLine();
    ImGui::Checkbox("Use Main Thread", &useMainThread);
}


AmbientOcclusionComputeRenderPass::AmbientOcclusionComputeRenderPass(
        sgl::vk::Renderer* renderer, sgl::vk::BufferPtr& aoBufferVk, sgl::GeometryBufferPtr& aoBufferGl)
        : ComputePass(renderer), aoBufferVk(aoBufferVk), aoBufferGl(aoBufferGl) {
}

void AmbientOcclusionComputeRenderPass::setLineData(LineDataPtr& lineData) {
    setTubeTriangleRenderData(lineData->getVulkanTubeTriangleRenderData(true));

    lineData->getFilteredLines();

    size_t sizeInBytes = 10;
    aoBufferVk = std::make_shared<sgl::vk::Buffer>(
            device, sizeInBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
            true, true);
    aoBufferGl = sgl::GeometryBufferPtr(new sgl::GeometryBufferGLExternalMemoryVk(
            aoBufferVk, sgl::SHADER_STORAGE_BUFFER));
}

void AmbientOcclusionComputeRenderPass::setTubeTriangleRenderData(
        const VulkanTubeTriangleRenderData& triangleRenderData) {
    tubeTriangleRenderData = triangleRenderData;

    auto asInput = new sgl::vk::TrianglesAccelerationStructureInput(device);
    asInput->setIndexBuffer(tubeTriangleRenderData.indexBuffer);
    asInput->setVertexBuffer(
            tubeTriangleRenderData.vertexBuffer, VK_FORMAT_R32G32B32_SFLOAT,
            sizeof(TubeTriangleVertexData));
    auto asInputPtr = sgl::vk::BottomLevelAccelerationStructureInputPtr(asInput);

    sgl::vk::BottomLevelAccelerationStructurePtr blas = buildBottomLevelAccelerationStructureFromInput(asInputPtr);

    topLevelAS = std::make_shared<sgl::vk::TopLevelAccelerationStructure>(device);
    topLevelAS->build({ blas }, { sgl::vk::BlasInstance() });
}


void AmbientOcclusionComputeRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"VulkanAmbientOcclusionBaker.Compute"});
}

void AmbientOcclusionComputeRenderPass::setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) {
}

void AmbientOcclusionComputeRenderPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(lineRenderSettingsBuffer, 0);
    computeData->setStaticBuffer(tubeTriangleRenderData.indexBuffer, 1);
    computeData->setStaticBuffer(tubeTriangleRenderData.vertexBuffer, 2);
    computeData->setStaticBuffer(tubeTriangleRenderData.linePointBuffer, 3);
    computeData->setStaticBuffer(aoBufferVk, 4);
    computeData->setTopLevelAccelerationStructure(topLevelAS, 5);
}

void AmbientOcclusionComputeRenderPass::_render() {
    lineRenderSettings.dummyData = 0.0f;
    lineRenderSettingsBuffer->updateData(
            sizeof(LineRenderSettings), &lineRenderSettings, renderer->getVkCommandBuffer());

    renderer->dispatch(computeData, 1024, 1, 1);
}
