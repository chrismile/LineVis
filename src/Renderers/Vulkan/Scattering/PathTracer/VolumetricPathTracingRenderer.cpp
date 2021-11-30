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

    renderReadySemaphore->signalSemaphoreGl(renderTextureGl, GL_NONE);

    rendererVk->beginCommandBuffer();
    rendererVk->setViewMatrix((*sceneData->camera)->getViewMatrix());
    rendererVk->setProjectionMatrix((*sceneData->camera)->getProjectionMatrixVulkan());
    vptPass->render();
    rendererVk->endCommandBuffer();

    // Submit the rendering operation in Vulkan.
    sgl::vk::FencePtr fence;
    sgl::vk::SemaphorePtr renderReadySemaphoreVk =
            std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderReadySemaphore);
    sgl::vk::SemaphorePtr renderFinishedSemaphoreVk =
            std::static_pointer_cast<sgl::vk::Semaphore, sgl::SemaphoreVkGlInterop>(renderFinishedSemaphore);
    //rendererVk->submitToQueue(
    //        renderReadySemaphoreVk, renderFinishedSemaphoreVk, fence, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
    rendererVk->submitToQueue(
            renderReadySemaphoreVk, renderFinishedSemaphoreVk, fence, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
    //rendererVk->getDevice()->waitIdle();

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
