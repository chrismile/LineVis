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
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/SystemGL.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "WBOITRenderer.hpp"
#include "WBOITRenderer.hpp"

using namespace sgl;

// Use stencil buffer to mask unused pixels
const bool useStencilBuffer = true;

WBOITRenderer::WBOITRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Weighted Blended Order Independent Transparency",
                       sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"WBOITGather.glsl\"");
    onResolutionChanged();
    reloadResolveShader();

    // Create blitting data (fullscreen rectangle in normalized device coordinates)
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuadPos{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    GeometryBufferPtr geomBufferPos = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuadPos.size(), (void*)&fullscreenQuadPos.front());
    std::vector<glm::vec2> fullscreenQuadTex{
            glm::vec2(1,1), glm::vec2(0,0), glm::vec2(1,0),
            glm::vec2(0,0), glm::vec2(1,1), glm::vec2(0,1)};
    GeometryBufferPtr geomBufferTex = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec2)*fullscreenQuadTex.size(), (void*)&fullscreenQuadTex.front());
    blitRenderData->addGeometryBuffer(geomBufferPos, "vertexPosition", ATTRIB_FLOAT, 3);
    blitRenderData->addGeometryBuffer(geomBufferTex, "vertexTexCoord", ATTRIB_FLOAT, 2);
}

void WBOITRenderer::reloadShaders() {
    reloadGatherShader();
    reloadResolveShader();

    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
}

void WBOITRenderer::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    resolveShader = sgl::ShaderManager->getShaderProgram({"WBOITResolve.Vertex", "WBOITResolve.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
}

void WBOITRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    LineRenderer::reloadGatherShader();
    gatherShader = lineData->reloadGatherShader();
    if (canCopyShaderAttributes && shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }
}

void WBOITRenderer::setLineData(LineDataPtr& lineData, bool isNewMesh) {
    updateNewLineData(lineData, isNewMesh);

    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();
    shaderAttributes = lineData->getGatherShaderAttributes(gatherShader);

    dirty = false;
    reRender = true;
}

void WBOITRenderer::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    TextureSettings textureSettingsColor;
    textureSettingsColor.internalFormat = GL_RGBA32F; // GL_RGBA16F?
    TextureSettings textureSettingsDepth;
    textureSettingsDepth.internalFormat = GL_DEPTH_COMPONENT;
    textureSettingsDepth.pixelFormat = GL_DEPTH_COMPONENT;
    textureSettingsDepth.pixelType = GL_FLOAT;

    gatherPassFBO = sgl::Renderer->createFBO();
    accumulationRenderTexture = TextureManager->createEmptyTexture(width, height, textureSettingsColor);
    textureSettingsColor.internalFormat = GL_R32F; // GL_R16F?
    revealageRenderTexture = TextureManager->createEmptyTexture(width, height, textureSettingsColor);
    gatherPassFBO->bindTexture(accumulationRenderTexture, COLOR_ATTACHMENT0);
    gatherPassFBO->bindTexture(revealageRenderTexture, COLOR_ATTACHMENT1);
    gatherPassFBO->bindRenderbuffer(sceneData.sceneDepthRBO, DEPTH_ATTACHMENT);
}

void WBOITRenderer::setUniformData() {
    gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    gatherShader->setUniform("lineWidth", lineWidth);
    if (gatherShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        gatherShader->setUniform("backgroundColor", backgroundColor);
    }
    if (gatherShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        gatherShader->setUniform("foregroundColor", foregroundColor);
    }
    lineData->setUniformGatherShaderData(gatherShader);
    setUniformData_Pass(gatherShader);

    resolveShader->setUniform("accumulationTexture", accumulationRenderTexture, 0);
    resolveShader->setUniform("revealageTexture", revealageRenderTexture, 1);
}

void WBOITRenderer::render() {
    setUniformData();

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    sgl::Renderer->bindFBO(gatherPassFBO);
    const float rgba32Zero[] = { 0, 0, 0, 0 };
    glClearBufferfv(GL_COLOR, 0, rgba32Zero);
    const float r32One[] = { 1, 0, 0, 0 };
    glClearBufferfv(GL_COLOR, 1, r32One);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_BLEND);
    glBlendFunci(0, GL_ONE, GL_ONE);
    glBlendFunci(1, GL_ZERO, GL_ONE_MINUS_SRC_COLOR);

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(shaderAttributes);
    renderHull();
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    sgl::Renderer->setProjectionMatrix(matrixIdentity());
    sgl::Renderer->setViewMatrix(matrixIdentity());
    sgl::Renderer->setModelMatrix(matrixIdentity());

    sgl::Renderer->bindFBO(sceneData.framebuffer);
    // Normal alpha blending
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
    sgl::Renderer->render(blitRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

void WBOITRenderer::renderGui() {
    if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
        reRender = true;
    }
    if (lineData) {
        lineData->renderGuiRenderingSettings();
    }
}
