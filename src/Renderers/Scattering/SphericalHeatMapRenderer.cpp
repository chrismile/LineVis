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
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "Renderers/Helpers/Sphere.hpp"
#define FTB_MOLLWEIDE_IMPL
#include "mollweide.hpp"
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

    sphereRasterPass = std::make_shared<TexturedSphereRasterPass>(this);
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

    lineData->rebuildInternalRepresentationIfNecessary();

    this->lineData = lineData;
    recreateMapImage();
}

void SphericalHeatMapRenderer::recreateMapImage() {
    std::shared_ptr<LineDataScattering> lineDataScattering = std::static_pointer_cast<LineDataScattering>(lineData);

    Image heat_map;
    if (sphericalMapType == SphericalMapType::MOLLWEIDE_KD_TREE) {
        sgl::KdTree<sgl::Empty>* kd_tree_exit_dirs = new sgl::KdTree<sgl::Empty>;
        kd_tree_exit_dirs->build(lineDataScattering->getExitDirections());
        heat_map = create_spherical_heatmap_image(kd_tree_exit_dirs, 300);
    } else if (sphericalMapType == SphericalMapType::MOLLWEIDE
            || sphericalMapType == SphericalMapType::MOLLWEIDE_SPHERE) {
        Mollweide_Grid<sgl::Empty>* grid = new Mollweide_Grid<sgl::Empty>;
        grid->init(6);
        const auto& exitDirections = lineDataScattering->getExitDirections();
        for (const glm::vec3& direction : exitDirections) {
            grid->insert(direction, {});
        }
        heat_map.pixels = reinterpret_cast<Pixel*>(grid->render_heatmap(256));
        heat_map.width = 512;
        heat_map.height = 256;
    }

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

    sphereRasterPass->setMollweideMapImage(heatMapTexture);

    sphereRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void SphericalHeatMapRenderer::render() {
    if (sphericalMapType == SphericalMapType::MOLLWEIDE_SPHERE) {
        renderSphere();
    } else {
        renderImage();
    }
}

void SphericalHeatMapRenderer::renderImage() {
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

void SphericalHeatMapRenderer::renderSphere() {
    if (!heatMapTexture) {
        return;
    }

    renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    renderer->setModelMatrix(sgl::matrixIdentity());

    sgl::vk::ImageViewPtr colorRenderTargetImage = (*sceneData->sceneTexture)->getImageView();
    sgl::vk::ImageViewPtr depthRenderTargetImage = (*sceneData->sceneDepthTexture)->getImageView();

    renderer->transitionImageLayout(
            colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    colorRenderTargetImage->clearColor(
            sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
    renderer->transitionImageLayout(
            colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    renderer->transitionImageLayout(
            depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    depthRenderTargetImage->clearDepthStencil(1.0f, 0, renderer->getVkCommandBuffer());
    renderer->transitionImageLayout(
            depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

    sphereRasterPass->render();
}

void SphericalHeatMapRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    if (propertyEditor.addCombo(
            "Map Type", (int*)&sphericalMapType,
            SPHERICAL_MAP_TYPE_NAMES, IM_ARRAYSIZE(SPHERICAL_MAP_TYPE_NAMES))) {
        recreateMapImage();
        reRender = true;
    }
}



void getSphereSurfaceRenderDataMollweide(
        const glm::vec3& center, float radius, int sectorCount, int stackCount,
        std::vector<glm::vec3>& vertexPositions, std::vector<glm::vec2>& vertexTexCoords,
        std::vector<uint32_t>& triangleIndices) {
    float phi, theta, sinPhi, cosPhi;
    float sectorStep = sgl::TWO_PI / sectorCount;
    float stackStep = sgl::PI / stackCount;

    // 1. Build the vertex buffers.
    for (int stackIdx = 0; stackIdx <= stackCount; ++stackIdx) {
        phi = sgl::HALF_PI - stackIdx * stackStep;
        cosPhi = std::cos(phi);
        sinPhi = std::sin(phi);
        for (int sectorIdx = 0; sectorIdx <= sectorCount; ++sectorIdx) {
            theta = sectorIdx * sectorStep;
            glm::vec3 normal(cosPhi * std::cos(theta), cosPhi * std::sin(theta), sinPhi);
            glm::vec3 position = center + radius * normal;
            vertexPositions.push_back(position);

            float texU, texV;
            lat_lng_to_mollweide(phi, theta, &texU, &texV);
            texU = (texU + 1.0f) * 0.5f;
            texV = texV + 0.5f;
            vertexTexCoords.emplace_back(texU, texV);
        }
    }

    // 2. Build the index buffer.
    uint32_t k1, k2;
    for (int stackIdx = 0; stackIdx <= stackCount; ++stackIdx) {
        k1 = stackIdx * (sectorCount + 1);
        k2 = k1 + sectorCount + 1;
        for (int sectorIdx = 0; sectorIdx <= sectorCount; ++sectorIdx) {
            if(stackIdx != 0) {
                triangleIndices.push_back(k1);
                triangleIndices.push_back(k2);
                triangleIndices.push_back(k1 + 1);
            }
            if(stackIdx != (stackCount-1)) {
                triangleIndices.push_back(k1 + 1);
                triangleIndices.push_back(k2);
                triangleIndices.push_back(k2 + 1);
            }
            k1++;
            k2++;
        }
    }
}

TexturedSphereRasterPass::TexturedSphereRasterPass(LineRenderer* lineRenderer)
        : RasterPass(*lineRenderer->getSceneData()->renderer),
          lineRenderer(lineRenderer), sceneData(lineRenderer->getSceneData()), camera(&sceneData->camera) {
    std::vector<glm::vec3> sphereVertexPositions;
    std::vector<glm::vec2> sphereVertexTexCoords;
    std::vector<uint32_t> sphereIndices;
    getSphereSurfaceRenderDataMollweide(
            glm::vec3(0, 0, 0), 0.25f, 32, 32,
            sphereVertexPositions, sphereVertexTexCoords, sphereIndices);

    vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(glm::vec3) * sphereVertexPositions.size(), sphereVertexPositions.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    vertexTexCoordBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(glm::vec2) * sphereVertexTexCoords.size(), sphereVertexTexCoords.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(uint32_t) * sphereIndices.size(), sphereIndices.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void TexturedSphereRasterPass::setMollweideMapImage(const sgl::vk::TexturePtr& _texture) {
    mollweideMapImage = _texture;
    if (rasterData) {
        rasterData->setStaticTexture(mollweideMapImage, "mollweideMapImage");
    }
}

void TexturedSphereRasterPass::setAttachmentLoadOp(VkAttachmentLoadOp loadOp) {
    this->attachmentLoadOp = loadOp;
    recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void TexturedSphereRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);
    lineRenderer->setFramebufferAttachments(framebuffer, attachmentLoadOp);

    framebufferDirty = true;
    dataDirty = true;
}

void TexturedSphereRasterPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"Sphere.Vertex.Textured", "Sphere.Fragment.Textured"});
}

void TexturedSphereRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexPosition", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexTexCoords", sizeof(glm::vec2));
}

void TexturedSphereRasterPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(indexBuffer);
    rasterData->setVertexBuffer(vertexPositionBuffer, "vertexPosition");
    rasterData->setVertexBuffer(vertexTexCoordBuffer, "vertexTexCoords");
    rasterData->setStaticTexture(mollweideMapImage, "mollweideMapImage");
}

void TexturedSphereRasterPass::_render() {
    RasterPass::_render();
}
