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
#include <Graphics/Renderer.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Scene/RenderTarget.hpp>
#include <Graphics/OpenGL/Texture.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>

#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Image/Image.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Utils/Interop.hpp>
#include <utility>
#endif

#include "LineData/LineData.hpp"
#include "Renderers/LineRenderer.hpp"
#include "DataView.hpp"

DataView::DataView(SceneData* parentSceneData, sgl::ShaderProgramPtr gammaCorrectionShader)
        : parentSceneData(parentSceneData), gammaCorrectionShader(std::move(gammaCorrectionShader)),
          sceneData(*parentSceneData) {
    sceneData.framebuffer = &sceneFramebuffer;
    sceneData.sceneTexture = &sceneTexture;
    sceneData.sceneDepthRBO = &sceneDepthRBO;
    sceneData.viewportWidth = &viewportWidth;
    sceneData.viewportHeight = &viewportHeight;

    sgl::CameraPtr parentCamera = *parentSceneData->camera;

    camera = std::make_shared<sgl::Camera>();
    camera->setNearClipDistance(parentCamera->getNearClipDistance());
    camera->setFarClipDistance(parentCamera->getFarClipDistance());
    camera->setYaw(parentCamera->getYaw());
    camera->setPitch(parentCamera->getPitch());
    camera->setFOVy(parentCamera->getFOVy());
    camera->setPosition(parentCamera->getPosition());
}

DataView::~DataView() {
    if (lineRenderer) {
        delete lineRenderer;
        lineRenderer = nullptr;
    }
}

void DataView::resize(int newWidth, int newHeight) {
    viewportWidth = uint32_t(std::max(newWidth, 0));
    viewportHeight = uint32_t(std::max(newHeight, 0));

    if (viewportWidth == 0 || viewportHeight == 0) {
        sceneFramebuffer = {};
        sceneTexture = {};
        resolvedSceneFramebuffer = {};
        resolvedSceneTexture = {};
        sceneDepthRBO = {};
        return;
    }

    // Buffers for off-screen rendering.
    sceneFramebuffer = sgl::Renderer->createFBO();
    resolvedSceneFramebuffer = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA16;
    } else {
        textureSettings.internalFormat = GL_RGBA8;
    }
    sceneTexture = sgl::TextureManager->createEmptyTexture(int(viewportWidth), int(viewportHeight), textureSettings);
    sceneFramebuffer->bindTexture(sceneTexture);
    sceneDepthRBO = sgl::Renderer->createRBO(int(viewportWidth), int(viewportHeight), sceneDepthRBOType);
    sceneFramebuffer->bindRenderbuffer(sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);
    camera->setRenderTarget(std::make_shared<sgl::RenderTarget>(sceneFramebuffer), false);
    camera->onResolutionChanged({});

    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA8;
        resolvedSceneTexture = sgl::TextureManager->createEmptyTexture(
                int(viewportWidth), int(viewportHeight), textureSettings);
        resolvedSceneFramebuffer->bindTexture(resolvedSceneTexture);
    }
}

void DataView::beginRender() {
    if (syncWithParentCamera) {
        sgl::CameraPtr parentCamera = *parentSceneData->camera;
        camera->setNearClipDistance(parentCamera->getNearClipDistance());
        camera->setFarClipDistance(parentCamera->getFarClipDistance());
        camera->setYaw(parentCamera->getYaw());
        camera->setPitch(parentCamera->getPitch());
        camera->setFOVy(parentCamera->getFOVy());
        camera->setPosition(parentCamera->getPosition());
    }

    //sgl::Renderer->bindFBO(sceneFramebuffer);
    sgl::Renderer->setCamera(camera, true);
    sgl::Renderer->clearFramebuffer(
            GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, clearColor);

    sgl::Renderer->setProjectionMatrix(camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glEnable(GL_DEPTH_TEST);
}

void DataView::endRender() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    sgl::Renderer->bindFBO(resolvedSceneFramebuffer);
    if (useLinearRGB) {
        sgl::Renderer->blitTexture(
                sceneTexture, sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)),
                gammaCorrectionShader);
    }
    sgl::Renderer->unbindFBO();
}


#ifdef USE_VULKAN_INTEROP
DataViewVulkan::DataViewVulkan(
        SceneData* parentSceneData, sgl::ShaderProgramPtr gammaCorrectionShader, sgl::vk::Renderer* renderer)
        : DataView(parentSceneData, std::move(gammaCorrectionShader)), renderer(renderer) {
    sgl::vk::Device* device = renderer->getDevice();
    renderReadySemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);
    renderFinishedSemaphore = std::make_shared<sgl::SemaphoreVkGlInterop>(device);
}

void DataViewVulkan::resize(int newWidth, int newHeight) {
    sgl::vk::Device* device = renderer->getDevice();
    viewportWidth = newWidth;
    viewportHeight = newHeight;

    if (viewportWidth == 0 || viewportHeight == 0) {
        sceneFramebuffer = {};
        sceneTexture = {};
        resolvedSceneFramebuffer = {};
        resolvedSceneTexture = {};
        sceneTextureVk = {};
        return;
    }

    // Buffers for off-screen rendering.
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = viewportWidth;
    imageSettings.height = viewportHeight;
    imageSettings.usage =
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT
            | VK_IMAGE_USAGE_STORAGE_BIT;
    if (useLinearRGB) {
        imageSettings.format = VK_FORMAT_R16G16B16A16_UNORM;
    } else {
        imageSettings.format = VK_FORMAT_R8G8B8A8_UNORM;
    }
    imageSettings.exportMemory = true;
    sceneTextureVk = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, sgl::vk::ImageSamplerSettings());
    sceneTexture = sgl::TexturePtr(new sgl::TextureGLExternalMemoryVk(sceneTextureVk));

    sceneTextureVk = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, sgl::vk::ImageSamplerSettings(),
            VK_IMAGE_ASPECT_COLOR_BIT);

    // Create scene depth texture.
    imageSettings.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    imageSettings.format = VK_FORMAT_D32_SFLOAT;
    sceneDepthTextureVk = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, sgl::vk::ImageSamplerSettings(),
            VK_IMAGE_ASPECT_DEPTH_BIT);
}

void DataViewVulkan::beginRender() {
    DataView::beginRender();
    renderReadySemaphore->signalSemaphoreGl(sceneTexture, GL_NONE);
}

void DataViewVulkan::endRender() {
    renderFinishedSemaphore->waitSemaphoreGl(sceneTexture, GL_LAYOUT_SHADER_READ_ONLY_EXT);
    DataView::endRender();
}
#endif
