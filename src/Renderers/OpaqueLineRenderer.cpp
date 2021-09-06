/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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

#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>
#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>

#include "LineData/LineDataStress.hpp"
#include "Helpers/Sphere.hpp"
#include "OpaqueLineRenderer.hpp"

OpaqueLineRenderer::OpaqueLineRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opaque Line Renderer", sceneData, transferFunctionWindow) {
    // Get all available multisampling modes.
    glGetIntegerv(GL_MAX_SAMPLES, &maximumNumberOfSamples);
    if (maximumNumberOfSamples <= 1) {
        useMultisampling = false;
    }
    numSampleModes = sgl::intlog2(maximumNumberOfSamples) + 1;
    sampleModeSelection = std::min(sgl::intlog2(numSamples), numSampleModes - 1);
    for (int i = 1; i <= maximumNumberOfSamples; i *= 2) {
        sampleModeNames.push_back(std::to_string(i));
    }

    reloadSphereRenderData();
    onResolutionChanged();
}

void OpaqueLineRenderer::setVisualizeSeedingProcess(bool visualizeSeedingProcess) {
    if (this->visualizeSeedingProcess != visualizeSeedingProcess) {
        this->visualizeSeedingProcess = visualizeSeedingProcess;
        if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
            reloadGatherShader();
        }
    }
}

void OpaqueLineRenderer::reloadSphereRenderData() {
    std::vector<glm::vec3> sphereVertexPositions;
    std::vector<glm::vec3> sphereVertexNormals;
    std::vector<uint32_t> sphereIndices;
    getSphereSurfaceRenderData(
            glm::vec3(0,0,0), 1.0f, 32, 32,
            sphereVertexPositions, sphereVertexNormals, sphereIndices);

    gatherShaderSphere = sgl::ShaderManager->getShaderProgram({
            "Sphere.Vertex", "Sphere.Fragment"
    });

    shaderAttributesSphere = sgl::ShaderManager->createShaderAttributes(gatherShaderSphere);
    shaderAttributesSphere->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    sgl::GeometryBufferPtr focusPointVertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            sphereVertexPositions.size() * sizeof(glm::vec3), sphereVertexPositions.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSphere->addGeometryBuffer(
            focusPointVertexPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    sgl::GeometryBufferPtr focusPointVertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
            sphereVertexNormals.size() * sizeof(glm::vec3), sphereVertexNormals.data(), sgl::VERTEX_BUFFER);
    shaderAttributesSphere->addGeometryBuffer(
            focusPointVertexNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);
    sgl::GeometryBufferPtr focusPointIndexBuffer = sgl::Renderer->createGeometryBuffer(
            sphereIndices.size() * sizeof(uint32_t), sphereIndices.data(), sgl::INDEX_BUFFER);
    shaderAttributesSphere->setIndexGeometryBuffer(focusPointIndexBuffer, sgl::ATTRIB_UNSIGNED_INT);
}

void OpaqueLineRenderer::renderSphere(const glm::vec3& position, float radius, const sgl::Color& color) {
    gatherShaderSphere->setUniform("spherePosition", position);
    gatherShaderSphere->setUniform("sphereRadius", radius);
    gatherShaderSphere->setUniform("sphereColor", color);
    gatherShaderSphere->setUniform("cameraPosition", sceneData.camera->getPosition());
    if (gatherShaderSphere->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        gatherShaderSphere->setUniform("backgroundColor", backgroundColor);
    }
    glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
    glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
    if (gatherShaderSphere->hasUniform("foregroundColor")) {
        gatherShaderSphere->setUniform("foregroundColor", foregroundColor);
    }
    sgl::Renderer->render(shaderAttributesSphere);
}

void OpaqueLineRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    if (visualizeSeedingProcess) {
        sgl::ShaderManager->addPreprocessorDefine("VISUALIZE_SEEDING_PROCESS", "");
    }
    LineRenderer::reloadGatherShader();
    gatherShader = lineData->reloadGatherShader();
    gatherShaderPoints = sgl::ShaderManager->getShaderProgram({
            "Point.Vertex", "Point.Geometry", "Point.Fragment"
    });
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");

    if (visualizeSeedingProcess) {
        sgl::ShaderManager->removePreprocessorDefine("VISUALIZE_SEEDING_PROCESS");
    }
    if (canCopyShaderAttributes && shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }
    if (canCopyShaderAttributes && shaderAttributesDegeneratePoints) {
        shaderAttributesDegeneratePoints = shaderAttributesDegeneratePoints->copy(gatherShaderPoints);
    }
    if (canCopyShaderAttributes && shaderAttributesSphere) {
        shaderAttributesSphere = shaderAttributesSphere->copy(gatherShaderSphere);
    }
}

void OpaqueLineRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();
    shaderAttributesDegeneratePoints = sgl::ShaderAttributesPtr();

    shaderAttributes = lineData->getGatherShaderAttributes(gatherShader);
    hasDegeneratePoints =
            lineData->getType() == DATA_SET_TYPE_STRESS_LINES &&
            static_cast<LineDataStress*>(lineData.get())->getHasDegeneratePoints();
    if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES && hasDegeneratePoints) {
        PointRenderData pointRenderData = static_cast<LineDataStress*>(lineData.get())->getDegeneratePointsRenderData();
        shaderAttributesDegeneratePoints = sgl::ShaderManager->createShaderAttributes(gatherShaderPoints);
        shaderAttributesDegeneratePoints->setVertexMode(sgl::VERTEX_MODE_POINTS);
        shaderAttributesDegeneratePoints->addGeometryBuffer(
                pointRenderData.vertexPositionBuffer, "vertexPosition",
                sgl::ATTRIB_FLOAT, 3);
    }

    dirty = false;
    reRender = true;
}

void OpaqueLineRenderer::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    if (useMultisampling) {
        msaaSceneFBO = sgl::Renderer->createFBO();
        msaaRenderTexture = sgl::TextureManager->createMultisampledTexture(
                width, height, numSamples, sceneData.sceneTexture->getSettings().internalFormat, true);
        msaaDepthRBO = sgl::Renderer->createRBO(width, height, sgl::RBO_DEPTH24_STENCIL8, numSamples);
        msaaSceneFBO->bindTexture(msaaRenderTexture);
        msaaSceneFBO->bindRenderbuffer(msaaDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);
    } else {
        msaaSceneFBO = sgl::FramebufferObjectPtr();
        msaaRenderTexture = sgl::TexturePtr();
        msaaDepthRBO = sgl::RenderbufferObjectPtr();
    }
}

void OpaqueLineRenderer::render() {
    LineRenderer::render();

    gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    gatherShader->setUniform("lineWidth", lineWidth);
    if (gatherShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        gatherShader->setUniform("backgroundColor", backgroundColor);
    }
    glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
    glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
    if (gatherShader->hasUniform("foregroundColor")) {
        gatherShader->setUniform("foregroundColor", foregroundColor);
    }
    lineData->setUniformGatherShaderData(gatherShader);
    setUniformData_Pass(gatherShader);

    if (useMultisampling) {
        sgl::Renderer->bindFBO(msaaSceneFBO);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, sceneData.clearColor);
    }

    //if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    //}
    sgl::Renderer->render(shaderAttributes);
    //if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    //}
    if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
        LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
        if (lineDataStress->getShallRenderSeedingProcess()) {
            int currentSeedIdx = lineDataStress->getCurrentSeedIdx();
            if (currentSeedIdx >= 0) {
                renderSphere(
                        lineDataStress->getCurrentSeedPosition(), 0.006f, sgl::Color(255, 40, 0));
            }
        }
    }

    if (shaderAttributesDegeneratePoints && showDegeneratePoints && hasDegeneratePoints) {
        gatherShaderPoints->setUniform("cameraPosition", sceneData.camera->getPosition());
        gatherShaderPoints->setUniform("pointWidth", pointWidth);
        gatherShaderPoints->setUniform("foregroundColor", foregroundColor);
        gatherShaderPoints->setUniform("pointColor", glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        sgl::Renderer->render(shaderAttributesDegeneratePoints);
    }

    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);
    renderHull();
    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);

    if (useMultisampling) {
        sgl::Renderer->bindFBO(sceneData.framebuffer);
        sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
        sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        sgl::Renderer->blitTexture(
                msaaRenderTexture, sgl::AABB2(glm::vec2(-1, -1), glm::vec2(1, 1)));
    }
}

void OpaqueLineRenderer::renderGui() {
    LineRenderer::renderGui();

    if (shaderAttributesDegeneratePoints && hasDegeneratePoints
            && ImGui::Checkbox("Show Degenerate Points", &showDegeneratePoints)) {
        reRender = true;
    }
    if (shaderAttributesDegeneratePoints && showDegeneratePoints && hasDegeneratePoints) {
        if (shaderAttributesDegeneratePoints && ImGui::SliderFloat(
                "Point Width", &pointWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH)) {
            reRender = true;
        }
    }
    if (maximumNumberOfSamples > 1) {
        if (ImGui::Checkbox("Multisampling", &useMultisampling)) {
            onResolutionChanged();
            reRender = true;
        }
        if (useMultisampling) {
            if (ImGui::Combo("Samples", &sampleModeSelection, sampleModeNames.data(), numSampleModes)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                onResolutionChanged();
                reRender = true;
            }
        }
    }
}
