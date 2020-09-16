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

#include "Widgets/TransferFunctionWindow.hpp"
#include "OpaqueLineRenderer.hpp"

OpaqueLineRenderer::OpaqueLineRenderer(SceneData& sceneData, TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer(sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("DIRECT_BLIT_GATHER", "");
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "GatherDummy.glsl");
    shaderProgramGeometryShader = sgl::ShaderManager->getShaderProgram({
        "GeometryPassNormal.VBO.Vertex",
        "GeometryPassNormal.VBO.Geometry",
        "GeometryPassNormal.Fragment"
    });
    shaderProgramProgrammableFetch = sgl::ShaderManager->getShaderProgram({
        "GeometryPassNormal.Programmable.Vertex",
        "GeometryPassNormal.Fragment"
    });
    sgl::ShaderManager->removePreprocessorDefine("DIRECT_BLIT_GATHER");

    // Get all available multisampling modes.
    glGetIntegerv(GL_MAX_SAMPLES, &maximumNumberOfSamples);
    if (maximumNumberOfSamples <= 1) {
        useMultisampling = false;
    }
    numSampleModes = sgl::intlog2(maximumNumberOfSamples) + 1;
    sampleModeSelection = std::min(sgl::intlog2(4), numSampleModes - 1);
    for (int i = 1; i <= maximumNumberOfSamples; i *= 2) {
        sampleModeNames.push_back(std::to_string(i));
    }

    onResolutionChanged();
}

void OpaqueLineRenderer::setLineData(LineDataPtr& lineData, bool isNewMesh) {
    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();

    if (useProgrammableFetch) {
        TubeRenderDataProgrammableFetch tubeRenderData = lineData->getTubeRenderDataProgrammableFetch();
        linePointDataSSBO = tubeRenderData.linePointsBuffer;

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgramProgrammableFetch);
        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    } else {
        TubeRenderData tubeRenderData = lineData->getTubeRenderData();
        linePointDataSSBO = sgl::GeometryBufferPtr();

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(shaderProgramGeometryShader);

        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        shaderAttributes->addGeometryBuffer(
                tubeRenderData.vertexPositionBuffer, "vertexPosition",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBuffer(
                tubeRenderData.vertexAttributeBuffer, "vertexAttribute",
                sgl::ATTRIB_FLOAT, 1);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexNormalBuffer, "vertexNormal",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexTangentBuffer, "vertexTangent",
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
    sgl::ShaderProgram* shaderProgram = shaderAttributes->getShaderProgram();
    shaderProgram->setUniform("cameraPosition", sceneData.camera->getPosition());
    shaderProgram->setUniform("lineWidth", lineWidth);
    shaderProgram->setUniform(
            "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    if (shaderProgram->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        shaderProgram->setUniform("backgroundColor", backgroundColor);
    }
    if (shaderProgram->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        shaderProgram->setUniform("foregroundColor", foregroundColor);
    }
    if (useProgrammableFetch) {
        sgl::ShaderManager->bindShaderStorageBuffer(2, linePointDataSSBO);
    }

    if (useMultisampling) {
        sgl::Renderer->bindFBO(msaaSceneFBO);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, sceneData.clearColor);
    }

    sgl::Renderer->render(shaderAttributes);

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
    if (ImGui::Begin("Opaque Line Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
        if (ImGui::Checkbox("Programmable Fetch", &useProgrammableFetch)) {
            dirty = true;
            reRender = true;
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
    ImGui::End();
}
