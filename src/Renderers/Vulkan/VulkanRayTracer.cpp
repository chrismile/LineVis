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
    rayTracingRenderPass->setMaxNumFrames(maxNumAccumulatedFrames);

    isVulkanRenderer = true;
    isRasterizer = false;

    onResolutionChanged();
}

VulkanRayTracer::~VulkanRayTracer() {
    sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();
}

void VulkanRayTracer::reloadGatherShader(bool canCopyShaderAttributes) {
    rayTracingRenderPass->setShaderDirty();
    rayTracingRenderPass->setUseDepthCues(useDepthCues);
    rayTracingRenderPass->setUseAmbientOcclusion(useAmbientOcclusion);
    rayTracingRenderPass->setAmbientOcclusionBaker(ambientOcclusionBaker);
    accumulatedFramesCounter = 0;
}

bool VulkanRayTracer::getIsTriangleRepresentationUsed() const {
    // TODO: Adapt if using AABB representation?
    return LineRenderer::getIsTriangleRepresentationUsed() || true;
}

void VulkanRayTracer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    rayTracingRenderPass->setLineData(lineData, isNewData);

    accumulatedFramesCounter = 0;
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

    accumulatedFramesCounter = 0;
}

void VulkanRayTracer::render() {
    if (useDepthCues && lineData) {
        computeDepthRange();
        renderReadySemaphore->signalSemaphoreGl(
                { depthMinMaxBuffers[outputDepthMinMaxBufferIndex] },
                { renderTextureGl }, { GL_NONE });
    } else {
        renderReadySemaphore->signalSemaphoreGl(renderTextureGl, GL_NONE);
    }

    if (accumulatedFramesCounter >= maxNumAccumulatedFrames) {
        Logfile::get()->throwError(
                "Error in VulkanRayTracer::render: accumulatedFramesCounter >= maxNumAccumulatedFrames. "
                "This should never happen!");
        accumulatedFramesCounter = 0;
    }

    if (ambientOcclusionBuffersDirty) {
        rayTracingRenderPass->setDataDirty();
        ambientOcclusionBuffersDirty = false;
    }

    rendererVk->beginCommandBuffer();
    rayTracingRenderPass->setBackgroundColor(sceneData.clearColor.getFloatColorRGBA());
    rayTracingRenderPass->setFrameNumber(accumulatedFramesCounter);
    rayTracingRenderPass->setDepthMinMaxBuffer(depthMinMaxBuffersVk[outputDepthMinMaxBufferIndex]);
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
    if (useDepthCues && lineData) {
        renderFinishedSemaphore->waitSemaphoreGl(
                { depthMinMaxBuffers[outputDepthMinMaxBufferIndex] },
                { renderTextureGl }, { GL_LAYOUT_SHADER_READ_ONLY_EXT });
    } else {
        renderFinishedSemaphore->waitSemaphoreGl(renderTextureGl, GL_LAYOUT_SHADER_READ_ONLY_EXT);
    }

    // Now, blit the data using OpenGL to the scene texture.
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    sgl::Renderer->bindFBO(sceneData.framebuffer);
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->blitTexture(
            renderTextureGl, sgl::AABB2(glm::vec2(-1, -1), glm::vec2(1, 1)));

    if (accumulatedFramesCounter < maxNumAccumulatedFrames) {
        accumulatedFramesCounter++;
    }
}

void VulkanRayTracer::renderGui() {
    LineRenderer::renderGui();

    if (ImGui::SliderInt("#Accum. Frames", reinterpret_cast<int*>(&maxNumAccumulatedFrames), 1, 32)) {
        rayTracingRenderPass->setMaxNumFrames(maxNumAccumulatedFrames);
        accumulatedFramesCounter = 0;
    }
}

bool VulkanRayTracer::needsReRender() {
    if (accumulatedFramesCounter < maxNumAccumulatedFrames) {
        return true;
    }
    return false;
}

void VulkanRayTracer::notifyReRenderTriggeredExternally() {
    internalReRender = false;
    accumulatedFramesCounter = 0;
}

void VulkanRayTracer::onHasMoved() {
    accumulatedFramesCounter = 0;
}

void VulkanRayTracer::update(float dt) {
}



RayTracingRenderPass::RayTracingRenderPass(sgl::vk::Renderer* renderer, sgl::CameraPtr camera)
        : RayTracingPass(renderer), camera(std::move(camera)) {
    cameraSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(CameraSettings),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    rayTracerSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(RayTracerSettings),
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
    rayTracerSettings.backgroundColor = color;
    rayTracerSettings.foregroundColor.x = 1.0f - color.x;
    rayTracerSettings.foregroundColor.y = 1.0f - color.y;
    rayTracerSettings.foregroundColor.z = 1.0f - color.z;
    rayTracerSettings.foregroundColor.w = color.w;
}

void RayTracingRenderPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    tubeTriangleRenderData = lineData->getVulkanTubeTriangleRenderData(true);
    hullTriangleRenderData = lineData->getVulkanHullTriangleRenderData(true);
    topLevelAS = lineData->getRayTracingTubeAndHullTriangleTopLevelAS();
    dataDirty = true;
}

void RayTracingRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines = lineData->getVulkanShaderPreprocessorDefines();
    if (maxNumFrames > 1) {
        preprocessorDefines.insert(std::make_pair("USE_JITTERED_RAYS", ""));
    }
    if (useDepthCues) {
        preprocessorDefines.insert(std::make_pair("USE_DEPTH_CUES", ""));
        preprocessorDefines.insert(std::make_pair("COMPUTE_DEPTH_CUES_GPU", ""));
        preprocessorDefines.insert(std::make_pair("USE_SCREEN_SPACE_POSITION", ""));
    }
    if (useAmbientOcclusion) {
        preprocessorDefines.insert(std::make_pair("USE_AMBIENT_OCCLUSION", ""));
        preprocessorDefines.insert(std::make_pair("GEOMETRY_PASS_TUBE", ""));
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            {"TubeRayTracing.RayGen", "TubeRayTracing.Miss",
             "TubeRayTracing.ClosestHit", "TubeRayTracing.ClosestHitHull"},
             preprocessorDefines);
}

void RayTracingRenderPass::createRayTracingData(
        sgl::vk::Renderer* renderer, sgl::vk::RayTracingPipelinePtr& rayTracingPipeline) {
    rayTracingData = std::make_shared<sgl::vk::RayTracingData>(renderer, rayTracingPipeline);
    rayTracingData->setStaticBuffer(cameraSettingsBuffer, "CameraSettingsBuffer");
    rayTracingData->setStaticImageView(sceneImageView, "outputImage");
    rayTracingData->setTopLevelAccelerationStructure(topLevelAS, "topLevelAS");
    rayTracingData->setStaticBuffer(rayTracerSettingsBuffer, "RayTracerSettingsBuffer");
    if (tubeTriangleRenderData.indexBuffer) {
        rayTracingData->setStaticBuffer(tubeTriangleRenderData.indexBuffer, "TubeIndexBuffer");
        rayTracingData->setStaticBuffer(tubeTriangleRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
        rayTracingData->setStaticBuffer(
                tubeTriangleRenderData.linePointBuffer, "TubeLinePointDataBuffer");
    } else {
        // Just bind anything in order for sgl to not complain...
        rayTracingData->setStaticBuffer(hullTriangleRenderData.indexBuffer, "TubeIndexBuffer");
        rayTracingData->setStaticBuffer(hullTriangleRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
        rayTracingData->setStaticBuffer(
                hullTriangleRenderData.vertexBuffer, "TubeLinePointDataBuffer");
    }
    if (hullTriangleRenderData.indexBuffer) {
        rayTracingData->setStaticBuffer(hullTriangleRenderData.indexBuffer, "HullIndexBuffer");
        rayTracingData->setStaticBuffer(
                hullTriangleRenderData.vertexBuffer, "HullTriangleVertexDataBuffer");
    } else {
        // Just bind anything in order for sgl to not complain...
        rayTracingData->setStaticBuffer(tubeTriangleRenderData.indexBuffer, "HullIndexBuffer");
        rayTracingData->setStaticBuffer(
                tubeTriangleRenderData.vertexBuffer, "HullTriangleVertexDataBuffer");
    }
    if (useDepthCues) {
        rayTracingData->setStaticBuffer(depthMinMaxBuffer, "DepthMinMaxBuffer");
    }
    if (useAmbientOcclusion && ambientOcclusionBaker) {
        auto ambientOcclusionBuffer = ambientOcclusionBaker->getAmbientOcclusionBufferVulkan();
        auto blendingWeightsBuffer = ambientOcclusionBaker->getBlendingWeightsBufferVulkan();
        if (ambientOcclusionBuffer && blendingWeightsBuffer) {
            rayTracingData->setStaticBuffer(ambientOcclusionBuffer, "AmbientOcclusionFactors");
            rayTracingData->setStaticBuffer(blendingWeightsBuffer, "AmbientOcclusionBlendingWeights");
        } else {
            // Just bind anything in order for sgl to not complain...
            sgl::vk::BufferPtr buffer =
                    tubeTriangleRenderData.vertexBuffer
                    ? tubeTriangleRenderData.vertexBuffer
                    : hullTriangleRenderData.vertexBuffer;
            rayTracingData->setStaticBuffer(buffer, "AmbientOcclusionFactors");
            rayTracingData->setStaticBuffer(buffer, "AmbientOcclusionBlendingWeights");
        }
    }
    lineData->setVulkanRenderDataDescriptors(std::static_pointer_cast<vk::RenderData>(rayTracingData));
}

void RayTracingRenderPass::updateLineRenderSettings() {
    glm::vec3 cameraPosition = camera->getPosition();
    rayTracerSettings.cameraPosition = cameraPosition;

    rayTracerSettingsBuffer->updateData(
            sizeof(RayTracerSettings), &rayTracerSettings, renderer->getVkCommandBuffer());
}

void RayTracingRenderPass::_render() {
    updateLineRenderSettings();

    cameraSettings.viewMatrix = camera->getViewMatrix();
    cameraSettings.projectionMatrix = camera->getProjectionMatrixVulkan();
    cameraSettings.inverseViewMatrix = glm::inverse(camera->getViewMatrix());
    cameraSettings.inverseProjectionMatrix = glm::inverse(camera->getProjectionMatrixVulkan());
    cameraSettingsBuffer->updateData(
            sizeof(CameraSettings), &cameraSettings, renderer->getVkCommandBuffer());

    vk::ShaderGroupSettings shaderGroupSettings{};
    shaderGroupSettings.hitShaderGroupSize = lineData->getShallRenderSimulationMeshBoundary() ? 2 : 1;
    rayTracingData->setShaderGroupSettings(shaderGroupSettings);

    lineData->updateVulkanUniformBuffers(renderer);

    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_GENERAL);
    renderer->traceRays(rayTracingData, launchSizeX, launchSizeY, launchSizeZ);
    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

sgl::vk::RayTracingPipelinePtr RayTracingRenderPass::createRayTracingPipeline() {
    //sgl::vk::ShaderBindingTable sbt = sgl::vk::ShaderBindingTable::generateSimpleShaderBindingTable(shaderStages);
    sgl::vk::ShaderBindingTable sbt(shaderStages);
    sbt.addRayGenShaderGroup()->setRayGenShader(0);
    sbt.addMissShaderGroup()->setMissShader(1);
    sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setClosestHitShader(
            2);
    sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setClosestHitShader(
            3);

    sgl::vk::RayTracingPipelineInfo rayTracingPipelineInfo(sbt);
    return std::make_shared<sgl::vk::RayTracingPipeline>(device, rayTracingPipelineInfo);
}
