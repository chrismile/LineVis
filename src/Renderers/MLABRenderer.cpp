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

#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Graphics/OpenGL/SystemGL.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "TilingMode.hpp"
#include "MLABRenderer.hpp"

// Whether to use stencil buffer to mask unused pixels.
static bool useStencilBuffer = false;

MLABRenderer::MLABRenderer(SceneData& sceneData, TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer(sceneData, transferFunctionWindow) {
    clearBitSet = true;
    syncMode = getSupportedSyncMode();

    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"MLABGather.glsl\"");
    updateLayerMode();
    reloadShaders();

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    blitRenderData = sgl::ShaderManager->createShaderAttributes(resolveShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    blitRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    clearRenderData = sgl::ShaderManager->createShaderAttributes(clearShader);
    clearRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
}

void MLABRenderer::updateSyncMode() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    spinlockViewportBuffer = sgl::GeometryBufferPtr();
    if (syncMode == SYNC_SPINLOCK) {
        spinlockViewportBuffer = sgl::Renderer->createGeometryBuffer(
                sizeof(uint32_t) * size_t(paddedWidth) * size_t(paddedHeight),
                NULL, sgl::SHADER_STORAGE_BUFFER);

        // Set all values in the buffer to zero.
        GLuint bufferId = static_cast<sgl::GeometryBufferGL*>(spinlockViewportBuffer.get())->getBuffer();
        uint32_t val = 0;
        glClearNamedBufferData(bufferId, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&val);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}

void MLABRenderer::updateLayerMode() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_LAYERS", sgl::toString(numLayers));
    onResolutionChanged();
}

void MLABRenderer::reloadShaders() {
    reloadGatherShader();
    reloadResolveShader();

    clearShader = sgl::ShaderManager->getShaderProgram({"MLABClear.Vertex", "MLABClear.Fragment"});
    if (clearRenderData) {
        clearRenderData = clearRenderData->copy(clearShader);
    }
}

void MLABRenderer::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    resolveShader = sgl::ShaderManager->getShaderProgram({"MLABResolve.Vertex", "MLABResolve.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
}

void MLABRenderer::reloadGatherShader() {
    sgl::ShaderManager->invalidateShaderCache();
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->addPreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX", "");
    }
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
    if (useProgrammableFetch) {
        gatherShader = sgl::ShaderManager->getShaderProgram({
            "GeometryPassNormal.Programmable.Vertex",
            "GeometryPassNormal.Fragment"
        });
    } else {
        gatherShader = sgl::ShaderManager->getShaderProgram({
            "GeometryPassNormal.VBO.Vertex",
            "GeometryPassNormal.VBO.Geometry",
            "GeometryPassNormal.Fragment"
        });
    }
    if (shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SYNC_FRAGMENT_SHADER_INTERLOCK");
        if (!useOrderedFragmentShaderInterlock) {
            sgl::ShaderManager->removePreprocessorDefine("INTERLOCK_UNORDERED");
        }
    } else if (syncMode == SYNC_SPINLOCK) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SYNC_SPINLOCK");
        sgl::ShaderManager->removePreprocessorDefine("GATHER_NO_DISCARD");
    }
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->removePreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX");
    }
}

void MLABRenderer::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    newState.rendererSettings.getValueOpt("numLayers", numLayers);
    newState.rendererSettings.getValueOpt("useStencilBuffer", useStencilBuffer);

    timerDataIsWritten = false;
    if (sceneData.performanceMeasurer && !timerDataIsWritten) {
        if (timer) {
            delete timer;
        }
        timer = new sgl::TimerGL;
        // TODO
        //sceneData.performanceMeasurer->setMlabTimer(timer);
    }
}

void MLABRenderer::setLineData(LineDataPtr& lineData, bool isNewMesh) {
    // Unload old data.
    this->lineData = lineData;
    shaderAttributes = sgl::ShaderAttributesPtr();

    if (useProgrammableFetch) {
        TubeRenderDataProgrammableFetch tubeRenderData = lineData->getTubeRenderDataProgrammableFetch();
        linePointDataSSBO = tubeRenderData.linePointsBuffer;

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    } else {
        TubeRenderData tubeRenderData = lineData->getTubeRenderData();
        linePointDataSSBO = sgl::GeometryBufferPtr();

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);

        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        shaderAttributes->addGeometryBuffer(
                tubeRenderData.vertexPositionBuffer, "vertexPosition",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexAttributeBuffer, "vertexAttribute",
                sgl::ATTRIB_FLOAT, 1);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexNormalBuffer, "vertexNormal",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexTangentBuffer, "vertexTangent",
                sgl::ATTRIB_FLOAT, 3);
        if (tubeRenderData.vertexPrincipalStressIndexBuffer) {
            shaderAttributes->addGeometryBufferOptional(
                    tubeRenderData.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex",
                    sgl::ATTRIB_UNSIGNED_INT,
                    1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);
        }
    }

    dirty = false;
    reRender = true;
}

void MLABRenderer::reallocateFragmentBuffer() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
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

    if (sceneData.performanceMeasurer) {
        sceneData.performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }

    fragmentBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentBuffer = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    updateSyncMode();


    // Buffer has to be cleared again.
    clearBitSet = true;
}

void MLABRenderer::setUsePrincipalStressDirectionIndex(bool usePrincipalStressDirectionIndex) {
    this->usePrincipalStressDirectionIndex = usePrincipalStressDirectionIndex;
    reloadGatherShader();
    if (shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }
    reRender = true;
}

void MLABRenderer::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    windowWidth = window->getWidth();
    windowHeight = window->getHeight();
    paddedWindowWidth = windowWidth, paddedWindowHeight = windowHeight;
    getScreenSizeWithTiling(paddedWindowWidth, paddedWindowHeight);

    reallocateFragmentBuffer();
}

void MLABRenderer::render() {
    setUniformData();
    clear();
    gather();
    resolve();
}

void MLABRenderer::setUniformData() {
    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBuffer);
    if (syncMode == SYNC_SPINLOCK) {
        sgl::ShaderManager->bindShaderStorageBuffer(1, spinlockViewportBuffer);
    }
    if (useProgrammableFetch) {
        sgl::ShaderManager->bindShaderStorageBuffer(2, linePointDataSSBO);
    }

    gatherShader->setUniform("viewportW", paddedWindowWidth);
    gatherShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    gatherShader->setUniform("lineWidth", lineWidth);
    if (gatherShader->hasUniform("transferFunctionTexture")) {
        gatherShader->setUniform(
                "transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    }
    if (gatherShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        gatherShader->setUniform("backgroundColor", backgroundColor);
    }
    if (gatherShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        gatherShader->setUniform("foregroundColor", foregroundColor);
    }

    resolveShader->setUniform("viewportW", paddedWindowWidth);
    clearShader->setUniform("viewportW", paddedWindowWidth);
}

void MLABRenderer::clear() {
    glDepthMask(GL_FALSE);

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    if (clearBitSet) {
        sgl::Renderer->render(clearRenderData);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        clearBitSet = false;
    }
}

void MLABRenderer::gather() {
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

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    sgl::Renderer->render(shaderAttributes);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void MLABRenderer::resolve() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);

    if (useStencilBuffer) {
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilMask(0x00);
    }

    sgl::Renderer->render(blitRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void MLABRenderer::renderGui() {
    if (ImGui::Begin("Opaque Line Renderer", &showRendererWindow)) {
        if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
            reRender = true;
        }
        if (ImGui::Checkbox("Programmable Fetch", &useProgrammableFetch)) {
            reloadGatherShader();
            dirty = true;
            reRender = true;
        }
        if (ImGui::SliderInt("Num Layers", &numLayers, 1, 64)) {
            updateLayerMode();
            reloadShaders();
            reRender = true;
        }
        const char *syncModeNames[] = { "No Sync (Unsafe)", "Fragment Shader Interlock", "Spinlock" };
        if (ImGui::Combo(
                "Sync Mode", (int*)&syncMode, syncModeNames, IM_ARRAYSIZE(syncModeNames))) {
            updateSyncMode();
            reloadGatherShader();
            reRender = true;
        }
        if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK && ImGui::Checkbox(
                "Ordered Sync", &useOrderedFragmentShaderInterlock)) {
            reloadGatherShader();
            reRender = true;
        }
        if (selectTilingModeUI()) {
            reloadShaders();
            clearBitSet = true;
            reRender = true;
        }
        if (ImGui::Button("Reload Shader")) {
            reloadGatherShader();
            if (shaderAttributes) {
                shaderAttributes = shaderAttributes->copy(gatherShader);
            }
            reRender = true;
        }
    }
    ImGui::End();
}
