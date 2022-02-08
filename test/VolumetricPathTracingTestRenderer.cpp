/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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
#include <ImGui/imgui.h>

#include "Renderers/Vulkan/Scattering/PathTracer/VolumetricPathTracingPass.hpp"
#include "VolumetricPathTracingTestRenderer.hpp"

VolumetricPathTracingTestRenderer::VolumetricPathTracingTestRenderer(sgl::vk::Renderer* renderer) : renderer(renderer) {
    camera = std::make_shared<sgl::Camera>();
    camera->setNearClipDistance(0.01f);
    camera->setFarClipDistance(100.0f);
    camera->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    camera->setYaw(-sgl::PI / 2.0f); //< around y axis
    camera->setPitch(0.0f); //< around x axis
    camera->setPosition(glm::vec3(0.0f, 0.0f, 0.8f));
    camera->setFOVy(std::atan(1.0f / 2.0f) * 2.0f);
    camera->resetLookAtLocation();

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    vptPass = std::make_shared<VolumetricPathTracingPass>(renderer, &camera);
    renderFinishedFence = std::make_shared<sgl::vk::Fence>(device);
}

VolumetricPathTracingTestRenderer::~VolumetricPathTracingTestRenderer() {
    if (imageData) {
        delete[] imageData;
        imageData = nullptr;
    }
}

void VolumetricPathTracingTestRenderer::setCloudData(const CloudDataPtr& cloudData) {
    vptPass->setCloudData(cloudData, true);
}

void VolumetricPathTracingTestRenderer::setRenderingResolution(uint32_t width, uint32_t height) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    if (imageData) {
        delete[] imageData;
        imageData = nullptr;
    }
    imageData = new float[width * height * 3];

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettings.usage =
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT
            | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    renderImageView = std::make_shared<sgl::vk::ImageView>(std::make_shared<sgl::vk::Image>(
            device, imageSettings));

    imageSettings.tiling = VK_IMAGE_TILING_LINEAR;
    imageSettings.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    imageSettings.memoryUsage = VMA_MEMORY_USAGE_GPU_TO_CPU;
    renderImageStaging = std::make_shared<sgl::vk::Image>(device, imageSettings);

    vptPass->setOutputImage(renderImageView);
    vptPass->recreateSwapchain(width, height);
    camera->onResolutionChanged(width, height);
}

void VolumetricPathTracingTestRenderer::setUseSparseGrid(bool useSparseGrid) {
    vptPass->setUseSparseGrid(useSparseGrid);
}

void VolumetricPathTracingTestRenderer::setGridInterpolationType(GridInterpolationType type) {
    vptPass->setSparseGridInterpolationType(type);
}

void VolumetricPathTracingTestRenderer::setCustomSeedOffset(uint32_t offset) {
    vptPass->setCustomSeedOffset(offset);
}

void VolumetricPathTracingTestRenderer::setUseLinearRGB(bool useLinearRGB) {
    vptPass->setUseLinearRGB(useLinearRGB);
}

void VolumetricPathTracingTestRenderer::setVptMode(VptMode vptMode) {
    vptPass->setVptMode(vptMode);
}

void VolumetricPathTracingTestRenderer::setVptModeFromString(const std::string& vptModeName) {
    for (int i = 0; i < IM_ARRAYSIZE(VPT_MODE_NAMES); i++) {
        if (vptModeName == VPT_MODE_NAMES[i]) {
            vptPass->setVptMode(VptMode(i));
            return;
        }
    }
    sgl::Logfile::get()->throwError(
            "Error in VolumetricPathTracingTestRenderer::setVptModeFromString: Unknown VPT mode \""
            + vptModeName + "\".");
}

float* VolumetricPathTracingTestRenderer::renderFrame(int numFrames) {
    // TODO: Allow multiple frames in flight.
    for (int i = 0; i < numFrames; i++) {
        renderer->beginCommandBuffer();
        vptPass->render();
        if (i == numFrames - 1) {
            renderImageView->getImage()->transitionImageLayout(
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, renderer->getVkCommandBuffer());
            renderImageStaging->transitionImageLayout(
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, renderer->getVkCommandBuffer());
            renderImageView->getImage()->copyToImage(
                    renderImageStaging, VK_IMAGE_ASPECT_COLOR_BIT,
                    renderer->getVkCommandBuffer());
        }
        renderer->endCommandBuffer();

        // Submit the rendering operations in Vulkan.
        renderer->submitToQueue(
                {}, {}, renderFinishedFence,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
        renderFinishedFence->wait();
        renderFinishedFence->reset();
    }

    uint32_t width = renderImageStaging->getImageSettings().width;
    uint32_t height = renderImageStaging->getImageSettings().height;
    VkSubresourceLayout subresourceLayout =
            renderImageStaging->getSubresourceLayout(VK_IMAGE_ASPECT_COLOR_BIT);
    auto* mappedData = (float*)renderImageStaging->mapMemory();
    const auto rowPitch = uint32_t(subresourceLayout.rowPitch / sizeof(float));

    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            uint32_t writeLocation = (x + y * width) * 3;
            // We don't need to add "uint32_t(subresourceLayout.offset)" here, as this is automatically done by VMA.
            uint32_t readLocation = x * 4 + rowPitch * y;
            for (uint32_t c = 0; c < 3; c++) {
                imageData[writeLocation + c] = mappedData[readLocation + c];
            }
        }
    }

    renderImageStaging->unmapMemory();
    return imageData;
}
