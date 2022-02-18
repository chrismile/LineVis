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
#include <Utils/SciVis/Navigation/CameraNavigator2D.hpp>
#include <Input/Mouse.hpp>
#include <Graphics/Scene/RenderTarget.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_impl_vulkan.h>

#include "LineData/LineData.hpp"
#include "Renderers/LineRenderer.hpp"
#include "DataView.hpp"

DataView::DataView(SceneData* parentSceneData)
        : parentSceneData(parentSceneData), renderer(*parentSceneData->renderer), sceneData(*parentSceneData) {
    sceneData.sceneTexture = &sceneTextureVk;
    sceneData.sceneDepthTexture = &sceneDepthTextureVk;
    sceneData.viewportWidth = &viewportWidth;
    sceneData.viewportHeight = &viewportHeight;

    sgl::CameraPtr parentCamera = *parentSceneData->camera;

    camera = std::make_shared<sgl::Camera>();
    camera->copyState(parentCamera);
    sceneData.camera = &camera;

    camera2d = std::make_shared<sgl::Camera>();
    camera2d->setPosition(glm::vec3(0.0f, 0.0f, 1.0f));
    camera2d->setFOVy(sgl::PI / 2.0f);
    camera2d->setNearClipDistance(0.001f);
    camera2d->setFarClipDistance(100.0f);

    camera2dNavigator = std::make_shared<sgl::CameraNavigator2D>(
            *sceneData.MOVE_SPEED, *sceneData.MOUSE_ROT_SPEED);

    sceneTextureBlitPass = std::make_shared<sgl::vk::BlitRenderPass>(renderer);
    sceneTextureGammaCorrectionPass = sgl::vk::BlitRenderPassPtr(new sgl::vk::BlitRenderPass(
            renderer, {"GammaCorrection.Vertex", "GammaCorrection.Fragment"}));

    screenshotReadbackHelper = std::make_shared<sgl::vk::ScreenshotReadbackHelper>(renderer);
}

DataView::~DataView() {
    if (lineRenderer) {
        delete lineRenderer;
        lineRenderer = nullptr;
    }
    if (descriptorSetImGui) {
        sgl::ImGuiWrapper::get()->freeDescriptorSet(descriptorSetImGui);
        descriptorSetImGui = nullptr;
    }
}

void DataView::resize(int newWidth, int newHeight) {
    viewportWidth = uint32_t(std::max(newWidth, 0));
    viewportHeight = uint32_t(std::max(newHeight, 0));

    if (viewportWidth == 0 || viewportHeight == 0) {
        sceneTextureVk = {};
        sceneDepthTextureVk = {};
        compositedTextureVk = {};
        return;
    }

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = viewportWidth;
    imageSettings.height = viewportHeight;

    // Create scene texture.
    imageSettings.usage =
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
            | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
    if (useLinearRGB) {
        imageSettings.format = VK_FORMAT_R16G16B16A16_UNORM;
    } else {
        imageSettings.format = VK_FORMAT_R8G8B8A8_UNORM;
    }
    dataViewTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, sgl::vk::ImageSamplerSettings(),
            VK_IMAGE_ASPECT_COLOR_BIT);
    dataViewTexture->getImage()->transitionImageLayout(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    dataViewTexture->getImageView()->clearColor(
            clearColor.getFloatColorRGBA(), rendererVk->getVkCommandBuffer());

    // Create composited (gamma-resolved, if VK_FORMAT_R16G16B16A16_UNORM for scene texture) scene texture.
    imageSettings.usage =
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imageSettings.format = VK_FORMAT_R8G8B8A8_UNORM;
    compositedDataViewTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, sgl::vk::ImageSamplerSettings(),
            VK_IMAGE_ASPECT_COLOR_BIT);

    // Pass the textures to the render passes.
    sceneTextureBlitPass->setInputTexture(dataViewTexture);
    sceneTextureBlitPass->setOutputImage(compositedDataViewTexture->getImageView());
    sceneTextureBlitPass->recreateSwapchain(viewportWidth, viewportHeight);

    sceneTextureGammaCorrectionPass->setInputTexture(dataViewTexture);
    sceneTextureGammaCorrectionPass->setOutputImage(compositedDataViewTexture->getImageView());
    sceneTextureGammaCorrectionPass->recreateSwapchain(viewportWidth, viewportHeight);

    screenshotReadbackHelper->onSwapchainRecreated(viewportWidth, viewportHeight);

    if (descriptorSetImGui) {
        sgl::ImGuiWrapper::get()->freeDescriptorSet(descriptorSetImGui);
        descriptorSetImGui = nullptr;
    }
    descriptorSetImGui = ImGui_ImplVulkan_AddTexture(
            compositedDataViewTexture->getImageSampler()->getVkSampler(),
            compositedDataViewTexture->getImageView()->getVkImageView(),
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    volumetricPathTracingPass->setOutputImage(dataViewTexture->getImageView());
    volumetricPathTracingPass->recreateSwapchain(viewportWidth, viewportHeight);

    


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
    auto renderTarget = std::make_shared<sgl::RenderTarget>(
            int(viewportWidth), int(viewportHeight));
    camera->setRenderTarget(renderTarget, false);
    camera->onResolutionChanged({});
    camera2d->setRenderTarget(renderTarget, false);
    camera2d->onResolutionChanged({});

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
        camera->copyState(parentCamera);
    }

    renderer->setProjectionMatrix(camera->getProjectionMatrix());
    renderer->setViewMatrix(camera->getViewMatrix());
    renderer->setModelMatrix(sgl::matrixIdentity());
}

void DataView::endRender() {
    renderer->transitionImageLayout(
            sceneTextureVk->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    if (useLinearRGB) {
        sceneTextureGammaCorrectionPass->render();
    } else {
        sceneTextureBlitPass->render();
    }
}

void DataView::saveScreenshot(const std::string& filename) {
    renderer->transitionImageLayout(
            compositedTextureVk->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    screenshotReadbackHelper->requestScreenshotReadback(compositedTextureVk->getImage(), filename);
}

void DataView::saveScreenshotDataIfAvailable() {
    if (viewportWidth == 0 || viewportHeight == 0) {
        return;
    }
    sgl::vk::Swapchain* swapchain = sgl::AppSettings::get()->getSwapchain();
    uint32_t imageIndex = swapchain ? swapchain->getImageIndex() : 0;
    screenshotReadbackHelper->saveDataIfAvailable(imageIndex);
}

ImTextureID DataView::getImGuiTextureId() const {
    compositedTextureVk->getImage()->transitionImageLayout(
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, renderer->getVkCommandBuffer());
    return reinterpret_cast<ImTextureID>(descriptorSetImGui);
}

void DataView::updateCameraMode() {
    bool useCamera2dNew = lineRenderer->getRenderingMode() == RENDERING_MODE_SPHERICAL_HEAT_MAP_RENDERER;
    if (useCamera2dNew) {
        sceneData.camera = &camera2d;
    } else {
        sceneData.camera = &camera;
    }
}

void DataView::moveCamera2dKeyboard(float dt) {
    bool cameraMoved = camera2dNavigator->moveCameraKeyboard(camera2d, dt);
    if (cameraMoved) {
        reRender = true;
        if (lineRenderer != nullptr) {
            lineRenderer->onHasMoved();
        }
    }
}

void DataView::moveCamera2dMouse(float dt) {
    bool cameraMoved = camera2dNavigator->moveCameraMouse(camera2d, dt);
    if (cameraMoved) {
        reRender = true;
        if (lineRenderer != nullptr) {
            lineRenderer->onHasMoved();
        }
    }
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
#endif
