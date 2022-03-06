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
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "SphericalHeatMapRenderer.hpp"

SphericalHeatMapRenderer::SphericalHeatMapRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Spherical Heat Map Renderer", sceneData, transferFunctionWindow) {
    isRasterizer = true;
    blitRenderPass = std::make_shared<sgl::vk::BlitRenderPass>(
            renderer, std::vector<std::string>{ "Blit.Vertex.Transformed", "Blit.Fragment" });
    blitRenderPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
    blitRenderPass->setAttachmentClearColor({ 0.0f, 0.0f, 0.0f, 0.0f });
    // Set cull mode to front, as we are using OpenGL-style projection matrix.
    blitRenderPass->setCullMode(sgl::vk::CullMode::CULL_FRONT);
}

SphericalHeatMapRenderer::~SphericalHeatMapRenderer() {
    if (heat_map.pixels) {
        heat_map.free();
        heat_map = {};
        heatMapTexture = {};
    }
}

void SphericalHeatMapRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    reRender = true;
    dirty = false;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in SphericalHeatMapRenderer::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");

        if (heat_map.pixels) {
            heat_map.free();
            heat_map = {};
        }

        return;
    }

    std::shared_ptr<LineDataScattering> lineDataScattering = std::static_pointer_cast<LineDataScattering>(lineData);

    KdTree<Empty>* exit_dirs = lineDataScattering->getExitDirections();
    Image heat_map = create_spherical_heatmap_image(exit_dirs, 300);

    setHeatMapData(heat_map);
}

void SphericalHeatMapRenderer::setHeatMapData(Image data) {
    if (heat_map.pixels) {
        heat_map.free();
        heat_map = {};
        heatMapTexture = {};
    }

    heat_map = data;

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = heat_map.width;
    imageSettings.height = heat_map.height;
    imageSettings.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    heatMapTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    heatMapTexture->getImage()->uploadData(heat_map.width * heat_map.height * 4, heat_map.pixels);

    blitRenderPass->setInputTexture(heatMapTexture);
    blitRenderPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    blitRenderPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void SphericalHeatMapRenderer::render() {
    if (!heatMapTexture) {
        return;
    }

    const uint32_t px_width = *sceneData->viewportWidth;
    const uint32_t px_height = *sceneData->viewportHeight;

    float heat_map_aspect_ratio = 2; // dim_x / dim_y
    float fb_aspect_ratio = 1.0f * float(px_width) / float(px_height);

    glm::vec2 texture_lower_left = { -heat_map_aspect_ratio, -1};

    if (heat_map_aspect_ratio > 1.0f * fb_aspect_ratio) {
        // bars top and bottom
        texture_lower_left *=  fb_aspect_ratio / heat_map_aspect_ratio;
    }

    sgl::AABB2 aabb { texture_lower_left, -texture_lower_left };

    // Don't use a 3D camera, but normalized device coordinate space ([-1, 1]^3).
    renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    renderer->setModelMatrix(sgl::matrixIdentity());

    blitRenderPass->setNormalizedCoordinatesAabb(aabb);
    blitRenderPass->render();
}

void SphericalHeatMapRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
}
