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

#include <Utils/File/Logfile.hpp>
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
#include <Graphics/Vulkan/Render/GraphicsPipeline.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "LineDensityMapRenderer.hpp"

using namespace sgl;

LineDensityMapRenderer::LineDensityMapRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk)
        : LineRenderer("Scattered Lines Renderer", sceneData, transferFunctionWindow),
          rendererVk(rendererVk) {
    isVulkanRenderer = true;
    isRasterizer = true;

    lineDensityFieldDvrPass = std::make_shared<LineDensityFieldDvrPass>(rendererVk, sceneData->camera);
    lineDensityFieldDvrPass->setBackgroundColor(sceneData->clearColor->getFloatColorRGBA());

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    renderReadySemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);
    renderFinishedSemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);

    onResolutionChanged();
}

LineDensityMapRenderer::~LineDensityMapRenderer() {
    sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();

    lineDensityFieldDvrPass = {};

    if (rendererVk) {
        delete rendererVk;
        rendererVk = nullptr;
    }
}

void LineDensityMapRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    dirty = false;
    reRender = true;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in LineDensityMapRenderer::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");
        return;
    }

    lineDensityFieldDvrPass->setLineData(lineData, isNewData);
}

void LineDensityMapRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageSettings.exportMemory = true;
    sgl::vk::ImageSamplerSettings samplerSettings;
    renderTextureVk = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    renderTextureGl = sgl::TexturePtr(new sgl::TextureGLExternalMemoryVk(renderTextureVk));

    lineDensityFieldDvrPass->setOutputImage(renderTextureVk->getImageView());
    lineDensityFieldDvrPass->recreateSwapchain(width, height);
}

void LineDensityMapRenderer::render() {
    LineRenderer::render();

    if (lineData && lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        return;
    }

    GLenum dstLayout = GL_NONE;
	sgl::vk::Device* device = rendererVk->getDevice();
	if (device->getDeviceDriverId() == VK_DRIVER_ID_INTEL_PROPRIETARY_WINDOWS) {
		dstLayout = GL_LAYOUT_GENERAL_EXT;
	}
    renderReadySemaphore->signalSemaphoreGl(renderTextureGl, dstLayout);

    rendererVk->beginCommandBuffer();
    rendererVk->setViewMatrix((*sceneData->camera)->getViewMatrix());
    rendererVk->setProjectionMatrix((*sceneData->camera)->getProjectionMatrixVulkan());
    lineDensityFieldDvrPass->setBackgroundColor(sceneData->clearColor->getFloatColorRGBA());
    lineDensityFieldDvrPass->render();
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

    sgl::Renderer->bindFBO(*sceneData->framebuffer);
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->blitTexture(
            renderTextureGl, sgl::AABB2(glm::vec2(-1, -1), glm::vec2(1, 1)),
            true);
}

void LineDensityMapRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool somethingChanged = lineDensityFieldDvrPass->renderGuiPropertyEditorNodes(propertyEditor);

    if (somethingChanged) {
        reRender = true;
    }
}



LineDensityFieldDvrPass::LineDensityFieldDvrPass(sgl::vk::Renderer* renderer, sgl::CameraPtr* camera)
        : ComputePass(renderer), camera(camera) {
    cameraSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(CameraSettings),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    renderSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(RenderSettingsData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void LineDensityFieldDvrPass::setOutputImage(sgl::vk::ImageViewPtr& colorImage) {
    sceneImageView = colorImage;

    if (computeData) {
        computeData->setStaticImageView(sceneImageView, "outputImage");
    }
}

void LineDensityFieldDvrPass::setBackgroundColor(const glm::vec4& color) {
    renderSettingsData.backgroundColor = color;
}

bool LineDensityFieldDvrPass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool changed = false;
    changed |= propertyEditor.addSliderFloat(
            "Attenuation Coefficient", &renderSettingsData.attenuationCoefficient, 0, 500);

    return changed;
}

void LineDensityFieldDvrPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in AabbRenderPassCompute::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");
        return;
    }

    lineDataScatteringRenderData = {};
    std::shared_ptr<LineDataScattering> lineDataScattering = std::static_pointer_cast<LineDataScattering>(lineData);
    lineDataScatteringRenderData = lineDataScattering->getVulkanLineDataScatteringRenderData();
    AABB3 aabb = lineDataScattering->getGridBoundingBox();

    renderSettingsData.minBoundingBox = aabb.min;
    renderSettingsData.maxBoundingBox = aabb.max;
    renderSettingsData.voxelSize = aabb.getDimensions().x / lineDataScattering->getGridSizeX();

    dataDirty = true;
}

void LineDensityFieldDvrPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"LineDensityFieldDvrShader.Compute"});
}

void LineDensityFieldDvrPass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(cameraSettingsBuffer, "CameraSettingsBuffer");
    computeData->setStaticBuffer(renderSettingsBuffer, "RenderSettingsBuffer");
    computeData->setStaticImageView(sceneImageView, "outputImage");
    computeData->setStaticTexture(lineDataScatteringRenderData.lineDensityFieldTexture, "lineDensityField");

    lineData->setVulkanRenderDataDescriptors(std::static_pointer_cast<vk::RenderData>(computeData));
}

void LineDensityFieldDvrPass::_render() {
    cameraSettings.viewMatrix = (*camera)->getViewMatrix();
    cameraSettings.projectionMatrix = (*camera)->getProjectionMatrixVulkan();
    cameraSettings.inverseViewMatrix = glm::inverse((*camera)->getViewMatrix());
    cameraSettings.inverseProjectionMatrix = glm::inverse((*camera)->getProjectionMatrixVulkan());
    cameraSettingsBuffer->updateData(
            sizeof(CameraSettings), &cameraSettings, renderer->getVkCommandBuffer());

    renderSettingsBuffer->updateData(
            sizeof(RenderSettingsData), &renderSettingsData, renderer->getVkCommandBuffer());

    renderer->insertImageMemoryBarrier(
            sceneImageView->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT);

    int width = int(sceneImageView->getImage()->getImageSettings().width);
    int height = int(sceneImageView->getImage()->getImageSettings().height);
    int groupCountX = sgl::iceil(width, 16);
    int groupCountY = sgl::iceil(height, 16);
    renderer->dispatch(computeData, groupCountX, groupCountY, 1);

    renderer->insertImageMemoryBarrier(
            sceneImageView->getImage(),
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);
}
