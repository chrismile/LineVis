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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Texture.hpp>

#include <Graphics/Vulkan/Utils/Interop.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/RayTracingPipeline.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <utility>

#include "VulkanRayTracer.hpp"

using namespace sgl;

VulkanRayTracer::VulkanRayTracer(
        SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk)
        : LineRenderer("Vulkan Test Renderer", sceneData, transferFunctionWindow),
        rendererVk(rendererVk) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    renderReadySemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);
    renderFinishedSemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);

    rayTracingRenderPass = std::make_shared<RayTracingRenderPass>(rendererVk, sceneData.camera);
    rayTracingRenderPass->setBackgroundColor(sceneData.clearColor.getFloatColorRGBA());

    onResolutionChanged();
}

VulkanRayTracer::~VulkanRayTracer() {
    sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();
}

void VulkanRayTracer::reloadGatherShader(bool canCopyShaderAttributes) {
}

void VulkanRayTracer::setLineData(LineDataPtr& lineData, bool isNewData) {
    if (!this->lineData || lineData->getType() != this->lineData->getType()
            || lineData->settingsDiffer(this->lineData.get())) {
        this->lineData = lineData;
        //reloadGatherShader(false);
    }
    this->lineData = lineData;
    rayTracingRenderPass->setLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void VulkanRayTracer::onResolutionChanged() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    sgl::Window* window = sgl::AppSettings::get()->getMainWindow();
    auto width = uint32_t(window->getWidth());
    auto height = uint32_t(window->getHeight());

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageSettings.exportMemory = true;
    sgl::vk::ImageSamplerSettings samplerSettings;
    renderTextureVk = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    renderTextureGl = sgl::TexturePtr(new sgl::TextureGLExternalMemoryVk(renderTextureVk));

    rayTracingRenderPass->setOutputImage(renderTextureVk->getImageView());
    rayTracingRenderPass->recreateSwapchain(width, height);
}

void VulkanRayTracer::render() {
    renderReadySemaphore->signalSemaphoreGl(renderTextureGl, GL_NONE);

    rendererVk->beginCommandBuffer();
    rayTracingRenderPass->setBackgroundColor(sceneData.clearColor.getFloatColorRGBA());
    rayTracingRenderPass->render();
    rendererVk->endCommandBuffer();

    // Submit the rendering operation in Vulkan.
    sgl::vk::FencePtr fence;
    sgl::vk::SemaphorePtr renderReadySemaphoreVk =
            std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderReadySemaphore);
    sgl::vk::SemaphorePtr renderFinishedSemaphoreVk =
            std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderFinishedSemaphore);
    rendererVk->submitToQueue(renderReadySemaphoreVk, renderFinishedSemaphoreVk, fence);

    // Wait for the rendering to finish on the Vulkan side.
    renderFinishedSemaphore->waitSemaphoreGl(renderTextureGl, GL_LAYOUT_SHADER_READ_ONLY_EXT);

    // Now, blit the data using OpenGL to the scene texture.
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    sgl::Renderer->bindFBO(sceneData.framebuffer);
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->blitTexture(
            renderTextureGl, sgl::AABB2(glm::vec2(-1, -1), glm::vec2(1, 1)));
}

void VulkanRayTracer::renderGui() {
    if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
        reRender = true;
    }
    if (lineData) {
        lineData->renderGuiRenderingSettings();
    }
}


RayTracingRenderPass::RayTracingRenderPass(sgl::vk::Renderer* renderer, sgl::CameraPtr camera)
        : RayTracingPass(renderer), camera(std::move(camera)) {
    cameraSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(CameraSettings),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    lineRenderSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(LineRenderSettings),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void RayTracingRenderPass::setOutputImage(sgl::vk::ImageViewPtr& imageView) {
    sceneImageView = imageView;

    if (rayTracingData) {
        rayTracingData->setStaticImageView(sceneImageView, 1);
    }
}

void RayTracingRenderPass::setBackgroundColor(const glm::vec4& color) {
    lineRenderSettings.backgroundColor = color;
}

void RayTracingRenderPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    tubeTriangleRenderData = lineData->getVulkanTubeTriangleRenderData(true);
    topLevelAS = lineData->getRayTracingTriangleTopLevelAS();
    dataDirty = true;
}

void RayTracingRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            {"TubeRayTracing.RayGen", "TubeRayTracing.ClosestHit", "TubeRayTracing.Miss"});
}

void RayTracingRenderPass::createRayTracingData(
        sgl::vk::Renderer* renderer, sgl::vk::RayTracingPipelinePtr& rayTracingPipeline) {
    rayTracingData = std::make_shared<sgl::vk::RayTracingData>(renderer, rayTracingPipeline);
    rayTracingData->setStaticBuffer(cameraSettingsBuffer, 0);
    rayTracingData->setStaticBuffer(lineRenderSettingsBuffer, 1);
    rayTracingData->setStaticBuffer(tubeTriangleRenderData.indexBuffer, 2);
    rayTracingData->setStaticBuffer(tubeTriangleRenderData.vertexBuffer, 3);
    rayTracingData->setStaticBuffer(tubeTriangleRenderData.linePointBuffer, 4);
    rayTracingData->setStaticImageView(sceneImageView, 5);
    rayTracingData->setTopLevelAccelerationStructure(topLevelAS, 6);
}

void RayTracingRenderPass::updateLineRenderSettings() {
    glm::vec3 cameraPosition = glm::vec3(camera->getViewMatrix()[3]);
    lineRenderSettings.cameraPosition = cameraPosition;

    lineRenderSettingsBuffer->updateData(
            sizeof(LineRenderSettings), &lineRenderSettings, renderer->getVkCommandBuffer());
}

void RayTracingRenderPass::_render() {
    updateLineRenderSettings();

    cameraSettings.inverseViewMatrix = glm::inverse(camera->getViewMatrix());
    cameraSettings.inverseProjectionMatrix = glm::inverse(camera->getProjectionMatrixVulkan());
    cameraSettingsBuffer->updateData(
            sizeof(CameraSettings), &cameraSettings, renderer->getVkCommandBuffer());

    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_GENERAL);
    renderer->traceRays(rayTracingData, launchSizeX, launchSizeY, launchSizeZ);
    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

sgl::vk::RayTracingPipelinePtr RayTracingRenderPass::createRayTracingPipeline() {
    sgl::vk::ShaderBindingTable sbt = sgl::vk::ShaderBindingTable::generateSimpleShaderBindingTable(shaderStages);
    sgl::vk::RayTracingPipelineInfo rayTracingPipelineInfo(sbt);
    return std::make_shared<sgl::vk::RayTracingPipeline>(device, rayTracingPipelineInfo);
}
