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

#include "VulkanTestRenderer.hpp"

using namespace sgl;

VulkanTestRenderer::VulkanTestRenderer(
        SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk)
        : LineRenderer("Vulkan Test Renderer", sceneData, transferFunctionWindow),
          rendererVk(rendererVk) {
    isVulkanRenderer = true;
    isRasterizer = true;

    testRenderPass = std::make_shared<TestRenderPass>(rendererVk, sceneData.camera);
    testRenderPass->setBackgroundColor(sceneData.clearColor.getFloatColorRGBA());

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    renderReadySemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);
    renderFinishedSemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);

    onResolutionChanged();
}

VulkanTestRenderer::~VulkanTestRenderer() {
    sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();
}

void VulkanTestRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    testRenderPass->setLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void VulkanTestRenderer::onResolutionChanged() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    auto width = uint32_t(window->getWidth());
    auto height = uint32_t(window->getHeight());

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = window->getWidth();
    imageSettings.height = window->getHeight();
    imageSettings.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageSettings.exportMemory = true;
    sgl::vk::ImageSamplerSettings samplerSettings;
    renderTextureVk = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    renderTextureGl = sgl::TexturePtr(new sgl::TextureGLExternalMemoryVk(renderTextureVk));

    testRenderPass->setOutputImage(renderTextureVk->getImageView());
    testRenderPass->recreateSwapchain(width, height);
}

void VulkanTestRenderer::render() {
    LineRenderer::render();

    renderReadySemaphore->signalSemaphoreGl(renderTextureGl, GL_NONE);

    rendererVk->beginCommandBuffer();
    rendererVk->setViewMatrix(sceneData.camera->getViewMatrix());
    rendererVk->setProjectionMatrix(sceneData.camera->getProjectionMatrixVulkan());
    testRenderPass->setBackgroundColor(sceneData.clearColor.getFloatColorRGBA());
    testRenderPass->render();
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
            renderTextureGl, sgl::AABB2(glm::vec2(-1, -1), glm::vec2(1, 1)),
            true);
}

void VulkanTestRenderer::renderGui() {
    LineRenderer::renderGui();

    // Add GUI code here.
}

void VulkanTestRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    // Add GUI code here.
}



TestRenderPass::TestRenderPass(sgl::vk::Renderer* renderer, sgl::CameraPtr camera)
        : RasterPass(renderer), camera(std::move(camera)) {
    renderSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(RenderSettingsData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void TestRenderPass::setOutputImage(sgl::vk::ImageViewPtr& colorImage) {
    sceneImageView = colorImage;
}

void TestRenderPass::setBackgroundColor(const glm::vec4& color) {
    if (framebuffer && backgroundColor != color) {
        framebuffer->setClearColor(0, color);
    }
    backgroundColor = color;
}

void TestRenderPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;

    std::vector<std::vector<glm::vec3>> filteredLines = lineData->getFilteredLines();

    sgl::AABB3 aabb;
    for (const std::vector<glm::vec3>& line : filteredLines) {
        for (const glm::vec3& point : line) {
            aabb.combine(point);
        }
    }

    std::vector<uint32_t> triangleIndices = {
            0, 4, 6, 0, 6, 2, // left
            1, 3, 7, 1, 7, 5, // right
            0, 1, 5, 0, 5, 4, // bottom
            2, 6, 7, 2, 7, 3, // top
            0, 2, 3, 0, 3, 1, // back
            4, 5, 7, 4, 7, 6, // front
    };
    indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, triangleIndices.size() * sizeof(uint32_t), triangleIndices.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    std::vector<glm::vec3> vertexPositions = {
            { aabb.getMinimum().x, aabb.getMinimum().y, aabb.getMinimum().z }, // 0
            { aabb.getMaximum().x, aabb.getMinimum().y, aabb.getMinimum().z }, // 1
            { aabb.getMinimum().x, aabb.getMaximum().y, aabb.getMinimum().z }, // 2
            { aabb.getMaximum().x, aabb.getMaximum().y, aabb.getMinimum().z }, // 3
            { aabb.getMinimum().x, aabb.getMinimum().y, aabb.getMaximum().z }, // 4
            { aabb.getMaximum().x, aabb.getMinimum().y, aabb.getMaximum().z }, // 5
            { aabb.getMinimum().x, aabb.getMaximum().y, aabb.getMaximum().z }, // 6
            { aabb.getMaximum().x, aabb.getMaximum().y, aabb.getMaximum().z }, // 7
    };
    vertexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPositions.size() * sizeof(glm::vec3), vertexPositions.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    dataDirty = true;
}

void TestRenderPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);

    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    framebuffer->setColorAttachment(sceneImageView, 0, attachmentState, backgroundColor);

    sgl::vk::ImageSettings depthImageSettings;
    depthImageSettings.width = width;
    depthImageSettings.height = height;
    depthImageSettings.format = VK_FORMAT_D32_SFLOAT;
    depthImageSettings.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    sgl::vk::ImagePtr depthImage(new sgl::vk::Image(device, depthImageSettings));
    depthImageView = std::make_shared<sgl::vk::ImageView>(
            depthImage, VK_IMAGE_VIEW_TYPE_2D, VK_IMAGE_ASPECT_DEPTH_BIT);

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(depthImageView, depthAttachmentState, 1.0f);

    framebufferDirty = true;
}

void TestRenderPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"TestShader.Vertex", "TestShader.Fragment"});
}

void TestRenderPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setVertexBufferBinding(0, sizeof(glm::vec3));
    pipelineInfo.setInputAttributeDescription(0, 0, "vertexPosition");
    //pipelineInfo.setCullMode(vk::CullMode::CULL_NONE);
}

void TestRenderPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(indexBuffer);
    rasterData->setVertexBuffer(vertexBuffer, "vertexPosition");
    rasterData->setStaticBuffer(renderSettingsBuffer, "RenderSettingsBuffer");
}

void TestRenderPass::_render() {
    glm::vec3 cameraPosition = camera->getPosition();
    renderSettingsData.cameraPosition = cameraPosition;
    renderSettingsBuffer->updateData(
            sizeof(RenderSettingsData), &renderSettingsData, renderer->getVkCommandBuffer());

    RasterPass::_render();
}
