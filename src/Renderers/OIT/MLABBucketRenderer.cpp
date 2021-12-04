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
#include <Utils/AppSettings.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "TilingMode.hpp"
#include "MLABBucketRenderer.hpp"

// Opacity threshold for lower back buffer boundary.
static float lowerBackBufferOpacity = 0.2f;

// Opacity threshold for upper back buffer boundary.
static float upperBackBufferOpacity = 0.98f;

MLABBucketRenderer::MLABBucketRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : MLABRenderer("Multi-Layer Alpha Blending Renderer with Depth Buckets",
                       sceneData, transferFunctionWindow) {
}

void MLABBucketRenderer::initialize() {
    sgl::ShaderManager->addPreprocessorDefine("MLAB_MIN_DEPTH_BUCKETS", "");
    sgl::ShaderManager->addPreprocessorDefine("USE_SCREEN_SPACE_POSITION", "");
    MLABRenderer::initialize();
}

MLABBucketRenderer::~MLABBucketRenderer() {
    sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
    sgl::ShaderManager->removePreprocessorDefine("MLAB_MIN_DEPTH_BUCKETS");
}

void MLABBucketRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    MLABRenderer::reloadGatherShader();

    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SYNC_FRAGMENT_SHADER_INTERLOCK", "");
        if (!useOrderedFragmentShaderInterlock) {
            sgl::ShaderManager->addPreprocessorDefine("INTERLOCK_UNORDERED", "");
        }
    } else if (syncMode == SYNC_SPINLOCK) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SYNC_SPINLOCK", "");
        // Do not discard while keeping the spinlock locked.
        sgl::ShaderManager->addPreprocessorDefine("GATHER_NO_DISCARD", "");
    }

    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"MinDepthPass.glsl\"");

    minDepthPassShader = lineData->reloadGatherShader();
    if (canCopyShaderAttributes && minDepthPassShaderAttributes) {
        minDepthPassShaderAttributes = minDepthPassShaderAttributes->copy(minDepthPassShader);
    }

    sgl::ShaderManager->removePreprocessorDefine("OIT_GATHER_HEADER");

    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SYNC_FRAGMENT_SHADER_INTERLOCK");
        if (!useOrderedFragmentShaderInterlock) {
            sgl::ShaderManager->removePreprocessorDefine("INTERLOCK_UNORDERED");
        }
    } else if (syncMode == SYNC_SPINLOCK) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SYNC_SPINLOCK");
        sgl::ShaderManager->removePreprocessorDefine("GATHER_NO_DISCARD");
    }
}

void MLABBucketRenderer::setNewState(const InternalState& newState) {
    MLABRenderer::setNewState(newState);
    lowerBackBufferOpacity = newState.rendererSettings.getFloatValue("lowerOpacity");
    upperBackBufferOpacity = newState.rendererSettings.getFloatValue("upperOpacity");
}

void MLABBucketRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    MLABRenderer::setLineData(lineData, isNewData);

    // Unload old data.
    minDepthPassShaderAttributes = sgl::ShaderAttributesPtr();
    minDepthPassShaderAttributes = lineData->getGatherShaderAttributes(minDepthPassShader);
}

void MLABBucketRenderer::reallocateFragmentBuffer() {
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    size_t fragmentBufferSizeBytes =
            (sizeof(uint32_t) + sizeof(float)) * size_t(numLayers) * size_t(paddedWidth) * size_t(paddedHeight);
    if (fragmentBufferSizeBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
        fragmentBufferSizeBytes = (1ull << 32ull) - 12ull;
    } else {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Fragment buffer size GiB: "
                + std::to_string(fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));
    }

    size_t minDepthBufferSizeBytes = sizeof(float) * 2 * size_t(paddedWidth) * size_t(paddedHeight);

    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(
                fragmentBufferSizeBytes + minDepthBufferSizeBytes);
    }

    fragmentBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    minDepthBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    minDepthBuffer = sgl::Renderer->createGeometryBuffer(
            minDepthBufferSizeBytes, nullptr, sgl::SHADER_STORAGE_BUFFER);

    updateSyncMode();

    // Buffer has to be cleared again.
    clearBitSet = true;
}

void MLABBucketRenderer::render() {
    LineRenderer::render();

    setUniformData();
    computeDepthRange();
    clear();
    gather();
    resolve();
}

void MLABBucketRenderer::setUniformData() {
    MLABRenderer::setUniformData();
    sgl::ShaderManager->bindShaderStorageBuffer(1, minDepthBuffer);

    minDepthPassShader->setUniform("viewportW", paddedWindowWidth);
    minDepthPassShader->setUniform("cameraPosition", (*sceneData->camera)->getPosition());
    minDepthPassShader->setUniform("lineWidth", lineWidth);
    if (minDepthPassShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        minDepthPassShader->setUniform("backgroundColor", backgroundColor);
    }
    if (minDepthPassShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        minDepthPassShader->setUniform("foregroundColor", foregroundColor);
    }
    minDepthPassShader->setUniform("lowerBackBufferOpacity", lowerBackBufferOpacity);
    minDepthPassShader->setUniform("upperBackBufferOpacity", upperBackBufferOpacity);
    lineData->setUniformGatherShaderData(minDepthPassShader);
}

void MLABBucketRenderer::gather() {
    // Enable the depth test, but disable depth write for gathering.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // We can use the stencil buffer to mask used pixels for the resolve pass.
    if (useStencilBuffer) {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);
        glClear(GL_STENCIL_BUFFER_BIT);
    }

    sgl::Renderer->setProjectionMatrix((*sceneData->camera)->getProjectionMatrix());
    sgl::Renderer->setViewMatrix((*sceneData->camera)->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }

    sgl::Renderer->render(minDepthPassShaderAttributes);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    sgl::Renderer->render(shaderAttributes);
    renderHull();
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }
}

void MLABBucketRenderer::computeDepthRange() {
    const sgl::AABB3& boundingBox = lineData->getModelBoundingBox();
    sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed((*sceneData->camera)->getViewMatrix());

    // Add offset of 0.1 for e.g. point data sets where additonal vertices may be added in the shader for quads.
    float minViewZ = screenSpaceBoundingBox.getMaximum().z + 0.1f;
    float maxViewZ = screenSpaceBoundingBox.getMinimum().z - 0.1f;
    minViewZ = std::max(-minViewZ, (*sceneData->camera)->getNearClipDistance());
    maxViewZ = std::min(-maxViewZ, (*sceneData->camera)->getFarClipDistance());
    minViewZ = std::min(minViewZ, (*sceneData->camera)->getFarClipDistance());
    maxViewZ = std::max(maxViewZ, (*sceneData->camera)->getNearClipDistance());
    float logmin = log(minViewZ);
    float logmax = log(maxViewZ);
    minDepthPassShader->setUniform("logDepthMin", logmin);
    minDepthPassShader->setUniform("logDepthMax", logmax);
}

void MLABBucketRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    MLABRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addSliderFloat("Back Bucket Lower Opacity", &lowerBackBufferOpacity, 0.0f, 1.0f)) {
        reRender = true;
    }

    if (propertyEditor.addSliderFloat("Back Bucket Upper Opacity", &upperBackBufferOpacity, 0.0f, 1.0f)) {
        reRender = true;
    }
}
