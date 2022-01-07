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

#include <Utils/StringUtils.hpp>
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
#include <Graphics/Vulkan/Render/CommandBuffer.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "VolumetricPathTracingPass.hpp"
#include "VolumetricPathTracingRenderer.hpp"

using namespace sgl;

VolumetricPathTracingRenderer::VolumetricPathTracingRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk)
        : LineRenderer("Volumetric Path Tracer", sceneData, transferFunctionWindow),
          rendererVk(rendererVk) {
    isVulkanRenderer = true;
    isRasterizer = true;

    vptPass = std::make_shared<VolumetricPathTracingPass>(rendererVk, sceneData->camera);

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    renderReadySemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);
    renderFinishedSemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);

#ifndef _WIN32
    if (device->getDeviceDriverId() == VK_DRIVER_ID_NVIDIA_PROPRIETARY) {
        std::string driverVersionString = device->getDeviceDriverInfo();
        std::vector<std::string> splitVersionString;
        sgl::splitString(driverVersionString, '.', splitVersionString);
        if (splitVersionString.size() >= 2 && sgl::fromString<int>(splitVersionString.at(0)) >= 495) {
            isLinuxAndNvidia495OrNewer = true;
        }
    }
#endif

    onResolutionChanged();
}

VolumetricPathTracingRenderer::~VolumetricPathTracingRenderer() {
    sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();

    vptPass = {};

    if (rendererVk) {
        delete rendererVk;
        rendererVk = nullptr;
    }
}

void VolumetricPathTracingRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    dirty = false;
    reRender = true;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in ScatteredLinesRenderer::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");
        return;
    }

    LineDataScatteringPtr lineDataScattering = std::static_pointer_cast<LineDataScattering>(lineData);
    vptPass->setLineData(lineDataScattering, isNewData);
}

void VolumetricPathTracingRenderer::onResolutionChanged() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.usage =
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
            | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    imageSettings.exportMemory = true;
    sgl::vk::ImageSamplerSettings samplerSettings;
    renderTextureVk = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    renderTextureGl = sgl::TexturePtr(new sgl::TextureGLExternalMemoryVk(renderTextureVk));

    vptPass->setOutputImage(renderTextureVk->getImageView());
    vptPass->recreateSwapchain(width, height);
}

bool VolumetricPathTracingRenderer::needsReRender() {
    bool reRenderParent = LineRenderer::needsReRender();
    return vptPass->needsReRender() || reRenderParent;
}

void VolumetricPathTracingRenderer::onHasMoved() {
    vptPass->onHasMoved();
}

void VolumetricPathTracingRenderer::notifyReRenderTriggeredExternally() {
    LineRenderer::notifyReRenderTriggeredExternally();
    vptPass->onHasMoved();
}

void VolumetricPathTracingRenderer::render() {
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

    sgl::vk::SemaphorePtr renderReadySemaphoreVk =
            std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderReadySemaphore);
    sgl::vk::SemaphorePtr renderFinishedSemaphoreVk =
            std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderFinishedSemaphore);

    rendererVk->beginCommandBuffer();
    rendererVk->getCommandBuffer()->pushWaitSemaphore(
            renderReadySemaphoreVk, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);
    rendererVk->setViewMatrix((*sceneData->camera)->getViewMatrix());
    rendererVk->setProjectionMatrix((*sceneData->camera)->getProjectionMatrixVulkan());
    vptPass->render();
    //renderTextureVk->getImage()->transitionImageLayout(VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    //renderTextureVk->getImageView()->clearColor(
    //        glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), rendererVk->getVkCommandBuffer());
    rendererVk->getCommandBuffer()->pushSignalSemaphore(renderFinishedSemaphoreVk);
    rendererVk->endCommandBuffer();

    // Submit the rendering operation in Vulkan.
    rendererVk->submitToQueue();

    /*
     * For some reason, on NVIDIA driver versions > 470.xx, a second frame may not be rendered immediately after the first frame.
     * Otherwise, the driver will hang.
     */
    if (isFirstFrame && isLinuxAndNvidia495OrNewer) {
        rendererVk->getDevice()->waitGraphicsQueueIdle();
    }
    isFirstFrame = false;

    // Wait for the rendering to finish on the Vulkan side.
    //renderFinishedSemaphore->waitSemaphoreGl(renderTextureGl, GL_LAYOUT_SHADER_READ_ONLY_EXT);
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
            false);
}

void VolumetricPathTracingRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool somethingChanged = vptPass->renderGuiPropertyEditorNodes(propertyEditor);

    if (somethingChanged) {
        reRender = true;
    }
}

void VolumetricPathTracingRenderer::renderGuiOverlay() {
}
