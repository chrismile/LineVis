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

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <Utils/AppSettings.hpp>
#include <Graphics/Window.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/SystemGL.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "DepthPeelingRenderer.hpp"

DepthPeelingRenderer::DepthPeelingRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Depth Peeling", sceneData, transferFunctionWindow) {
    onResolutionChanged();
}

void DepthPeelingRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"DepthPeelingGather.glsl\"");
    gatherShader = lineData->reloadGatherShader();
    LineRenderer::reloadGatherShader();
    if (canCopyShaderAttributes && shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }

    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"DepthComplexityGatherInc.glsl\"");
    depthComplexityGatherShader = lineData->reloadGatherShader();
    if (canCopyShaderAttributes && depthComplexityShaderAttributes) {
        depthComplexityShaderAttributes = depthComplexityShaderAttributes->copy(depthComplexityGatherShader);
    }
}

void DepthPeelingRenderer::setNewState(const InternalState& newState) {
}

void DepthPeelingRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();
    depthComplexityShaderAttributes = sgl::ShaderAttributesPtr();
    shaderAttributes = lineData->getGatherShaderAttributes(gatherShader);
    depthComplexityShaderAttributes = lineData->getGatherShaderAttributes(depthComplexityGatherShader);

    dirty = false;
    reRender = true;
}

void DepthPeelingRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);

    sgl::TextureSettings textureSettingsColor;
    textureSettingsColor.internalFormat = GL_RGBA32F;
    sgl::TextureSettings textureSettingsDepth;
    textureSettingsDepth.internalFormat = GL_DEPTH_COMPONENT;

    accumulatorFBO = sgl::Renderer->createFBO();
    colorAccumulatorTexture = sgl::TextureManager->createEmptyTexture(width, height, textureSettingsColor);
    accumulatorFBO->bindTexture(colorAccumulatorTexture, sgl::COLOR_ATTACHMENT);

    for (int i = 0; i < 2; i++) {
        depthPeelingFBOs[i] = sgl::Renderer->createFBO();

        colorRenderTextures[i] = sgl::TextureManager->createEmptyTexture(width, height, textureSettingsColor);
        depthPeelingFBOs[i]->bindTexture(colorRenderTextures[i], sgl::COLOR_ATTACHMENT);

        depthRenderTextures[i] = sgl::TextureManager->createEmptyTexture(width, height, textureSettingsDepth);
        depthPeelingFBOs[i]->bindTexture(depthRenderTextures[i], sgl::DEPTH_ATTACHMENT);
    }

    // Buffer for determining the (maximum) depth complexity of the scene
    size_t numFragmentsBufferSizeBytes = sizeof(uint32_t) * width * height;
    fragmentCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentCounterBuffer = sgl::Renderer->createGeometryBuffer(
            numFragmentsBufferSizeBytes, nullptr, sgl::SHADER_STORAGE_BUFFER);

    size_t bufferSizeColorAccumulators = 2 * sizeof(float) * 4 * width * height;
    size_t bufferSizeDepthTextures = 2 * sizeof(float) * width * height;
    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(
                numFragmentsBufferSizeBytes + bufferSizeColorAccumulators + bufferSizeDepthTextures);
    }
}

void DepthPeelingRenderer::setUniformData() {
    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentCounterBuffer);
    lineData->setUniformGatherShaderData_AllPasses();

    int width = int(*sceneData->viewportWidth);

    depthComplexityGatherShader->setUniformOptional("cameraPosition", (*sceneData->camera)->getPosition());
    depthComplexityGatherShader->setUniform("lineWidth", lineWidth);
    depthComplexityGatherShader->setUniform("viewportW", width);
    lineData->setUniformGatherShaderData_Pass(depthComplexityGatherShader);

    gatherShader->setUniform("cameraPosition", (*sceneData->camera)->getPosition());
    gatherShader->setUniform("lineWidth", lineWidth);
    if (gatherShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        gatherShader->setUniform("backgroundColor", backgroundColor);
    }
    if (gatherShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        gatherShader->setUniform("foregroundColor", foregroundColor);
    }
    lineData->setUniformGatherShaderData_Pass(gatherShader);
    setUniformData_Pass(gatherShader);
}

void DepthPeelingRenderer::render() {
    LineRenderer::render();

    setUniformData();
    gather();
    resolve();
}

void DepthPeelingRenderer::gather() {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    computeDepthComplexity();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);

    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_ONE_MINUS_DST_ALPHA, GL_ONE, GL_ONE_MINUS_DST_ALPHA, GL_ONE); // Front-to-back blending

    for (int i = 0; i < 2; i++) {
        sgl::Renderer->bindFBO(depthPeelingFBOs[i]);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, sgl::Color(0, 0, 0, 0), 1.0f);
    }

    sgl::Renderer->bindFBO(accumulatorFBO);
    sgl::Renderer->clearFramebuffer(GL_COLOR_BUFFER_BIT, sgl::Color(0, 0, 0, 0));

    int SWAP_INTERVAL = 400;
    // Sorry Intel, your drivers are currently too slow :)
    if (boost::contains(boost::to_lower_copy(sgl::SystemGL::get()->getVendorString()), "intel")) {
        SWAP_INTERVAL = 200;
    }

    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }

    // TODO: Set layers to higher numbers for better quality.
    for (uint64_t i = 0; i < std::min(maxDepthComplexity, uint64_t(2000ul)); i++) {
        // 1. Peel one layer of the scene
        glDisable(GL_BLEND); // Replace with current surface
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        sgl::Renderer->bindFBO(depthPeelingFBOs[i%2]);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, sgl::Color(0, 0, 0, 0), 1.0f);
        gatherShader->setUniform("depthReadTexture", depthRenderTextures[(i+1)%2], 7);
        gatherShader->setUniform("iteration", int(i));

        sgl::Renderer->setProjectionMatrix((*sceneData->camera)->getProjectionMatrix());
        sgl::Renderer->setViewMatrix((*sceneData->camera)->getViewMatrix());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

        sgl::Renderer->render(shaderAttributes);
        //renderHull();

        // 2. Store it in the accumulator
        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        sgl::Renderer->bindFBO(accumulatorFBO);
        sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
        sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
        sgl::Renderer->blitTexture(
                colorRenderTextures[i%2],
                sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)));

        // NVIDIA OpenGL Linux driver assumes the application has hung if we don't swap buffers at regular intervals.
        if (i % SWAP_INTERVAL == 0 && i > 0) {
            sgl::AppSettings::get()->getMainWindow()->flip();
        }
    }

    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }

    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void DepthPeelingRenderer::resolve() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    sgl::Renderer->bindFBO(*sceneData->framebuffer);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); // Pre-multiplied alpha
    sgl::Renderer->blitTexture(
            colorAccumulatorTexture,
            sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)));

    // Revert to normal alpha blending
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

void DepthPeelingRenderer::computeDepthComplexity() {
    // Clear numFragmentsBuffer.
    GLuint bufferID = static_cast<sgl::GeometryBufferGL*>(fragmentCounterBuffer.get())->getBuffer();
    GLubyte val = 0;
    glClearNamedBufferData(bufferID, GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);

    // Render to numFragmentsBuffer to determine the depth complexity of the scene.
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    sgl::Renderer->setProjectionMatrix((*sceneData->camera)->getProjectionMatrix());
    sgl::Renderer->setViewMatrix((*sceneData->camera)->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(depthComplexityShaderAttributes);
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Compute the maximum depth complexity of the scene.
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int bufferSize = width * height;
    uint32_t* data = (uint32_t*)fragmentCounterBuffer->mapBuffer(sgl::BUFFER_MAP_READ_ONLY);

    uint64_t maxDepthComplexity = 0;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(max:maxDepthComplexity) default(none) shared(data, bufferSize) schedule(static)
#endif
    for (int i = 0; i < bufferSize; i++) {
        maxDepthComplexity = std::max(maxDepthComplexity, (uint64_t)data[i]);
    }
    this->maxDepthComplexity = maxDepthComplexity;

    fragmentCounterBuffer->unmapBuffer();
}

void DepthPeelingRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);
    propertyEditor.addText("Max. Depth Complexity", std::to_string(maxDepthComplexity));
}
