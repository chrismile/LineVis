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
#include <Graphics/Renderer.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Scene/RenderTarget.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "SphericalHeatMapRenderer.hpp"

SphericalHeatMapRenderer::SphericalHeatMapRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Spherical Heat Map Renderer", sceneData, transferFunctionWindow) {
    ;
}

SphericalHeatMapRenderer::~SphericalHeatMapRenderer() {
    if (heatMap) {
        delete[] heatMap;
        heatMap = nullptr;
        heatMapTexture = {};
    }
}

void SphericalHeatMapRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    reRender = true;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in SphericalHeatMapRenderer::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");

        if (heatMap) {
            delete[] heatMap;
            heatMap = nullptr;
            heatMapTexture = {};
        }

        return;
    }

    std::shared_ptr<LineDataScattering> lineDataScattering = std::static_pointer_cast<LineDataScattering>(lineData);

    // TODO: Test data just uses horizontal gradient.
    uint32_t w = 200, h = 200;
    auto* testData = new float[w * h];
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            testData[x + y * w] = float(x) / float(w - 1);
        }
    }

    setHeatMapData(testData, w, h);

    delete[] testData;
}

void SphericalHeatMapRenderer::setHeatMapData(float* data, uint32_t width, uint32_t height) {
    if (heatMap) {
        delete[] heatMap;
        heatMap = nullptr;
        heatMapTexture = {};
    }

    heatMapWidth = width;
    heatMapHeight = height;

    heatMap = new float[heatMapWidth * heatMapHeight];
    memcpy(heatMap, data, sizeof(float) * heatMapWidth * heatMapHeight);

    sgl::PixelFormat pixelFormat;
    pixelFormat.pixelFormat = GL_RED;
    pixelFormat.pixelType = GL_FLOAT;
    sgl::TextureSettings textureSettings;
    textureSettings.internalFormat = GL_R32F;
    heatMapTexture = sgl::TextureManager->createTexture(
            heatMap, int(width), int(height), pixelFormat, textureSettings);
}

void SphericalHeatMapRenderer::render() {
    if (!heatMapTexture) {
        return;
    }

    sgl::FramebufferObjectPtr fbo = (*sceneData->camera)->getRenderTarget()->getFramebufferObject();

    // Don't use a 3D camera, but normalized device coordinate space ([-1, 1]^3).
    sgl::Renderer->setProjectionMatrix((*sceneData->camera)->getProjectionMatrix());
    sgl::Renderer->setViewMatrix((*sceneData->camera)->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    /*
     * TODO: Use framebuffer width and height in combination with heatMapWidth and heatMapHeight
     * to determine which part of the [-1, 1]^2 normalized viewport area should be covered.
     */

    sgl::Renderer->clearFramebuffer(GL_COLOR_BUFFER_BIT, sgl::Color(0, 0, 0, 0));
    sgl::Renderer->blitTexture(
            heatMapTexture, sgl::AABB2(glm::vec2(-1), glm::vec2(1)));
}

void SphericalHeatMapRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    ;
}

