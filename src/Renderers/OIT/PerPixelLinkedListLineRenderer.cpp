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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Utils/AppSettings.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "TilingMode.hpp"
#include "PerPixelLinkedListLineRenderer.hpp"

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = true;

PerPixelLinkedListLineRenderer::PerPixelLinkedListLineRenderer(
        SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Per-Pixel Linked List Renderer", sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");

    reloadResolveShader();
    clearShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClear.Vertex", "LinkedListClear.Fragment"});

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

    onResolutionChanged();
}

PerPixelLinkedListLineRenderer::~PerPixelLinkedListLineRenderer() {
    if (sceneData.performanceMeasurer && !timerDataIsWritten && timer) {
        delete timer;
        sceneData.performanceMeasurer->setPpllTimer(nullptr);
    }
}

void PerPixelLinkedListLineRenderer::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity));

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
        || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = int(std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4);
        sgl::ShaderManager->addPreprocessorDefine("STACK_SIZE", sgl::toString(stackSize));
    }

    resolveShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolve.Vertex", "LinkedListResolve.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(resolveShader);
    }
}

void PerPixelLinkedListLineRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    LineRenderer::reloadGatherShader();
    gatherShader = lineData->reloadGatherShader();
    if (canCopyShaderAttributes && shaderAttributes) {
        shaderAttributes = shaderAttributes->copy(gatherShader);
    }
}

void PerPixelLinkedListLineRenderer::setSortingAlgorithmDefine() {
    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_PRIORITY_QUEUE) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "frontToBackPQ");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BUBBLE_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "bubbleSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_INSERTION_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "insertionSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_SHELL_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "shellSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_MAX_HEAP) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "heapSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "quicksort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "quicksortHybrid");
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
        || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        sgl::ShaderManager->addPreprocessorDefine("USE_QUICKSORT", "");
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_QUICKSORT");
    }
}

void PerPixelLinkedListLineRenderer::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    timerDataIsWritten = false;
    if (sceneData.performanceMeasurer && !timerDataIsWritten) {
        if (timer) {
            delete timer;
        }
        timer = new sgl::TimerGL;
        sceneData.performanceMeasurer->setPpllTimer(timer);
    }
}

void PerPixelLinkedListLineRenderer::updateLargeMeshMode() {
    // More than one million cells?
    LargeMeshMode newMeshLargeMeshMode = MESH_SIZE_MEDIUM;
    if (lineData->getNumLineSegments() > 1e6) { // > 1m line segments
        newMeshLargeMeshMode = MESH_SIZE_LARGE;
    }
    if (newMeshLargeMeshMode != largeMeshMode) {
        largeMeshMode = newMeshLargeMeshMode;
        expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][0];
        expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][1];
        reallocateFragmentBuffer();
        reloadResolveShader();
    }
}

void PerPixelLinkedListLineRenderer::setLineData(LineDataPtr& lineData, bool isNewMesh) {
    updateNewLineData(lineData, isNewMesh);

    // Unload old data.
    shaderAttributes = sgl::ShaderAttributesPtr();
    updateLargeMeshMode();

    shaderAttributes = lineData->getGatherShaderAttributes(gatherShader);

    dirty = false;
    reRender = true;
}

void PerPixelLinkedListLineRenderer::reallocateFragmentBuffer() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    fragmentBufferSize = size_t(expectedAvgDepthComplexity) * size_t(paddedWidth) * size_t(paddedHeight);
    size_t fragmentBufferSizeBytes = 12ull * fragmentBufferSize;
    if (fragmentBufferSizeBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
        fragmentBufferSizeBytes = (1ull << 32ull) - 12ull;
        fragmentBufferSize = fragmentBufferSizeBytes / 12ull;
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
}

void PerPixelLinkedListLineRenderer::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    windowWidth = window->getWidth();
    windowHeight = window->getHeight();
    paddedWindowWidth = windowWidth, paddedWindowHeight = windowHeight;
    getScreenSizeWithTiling(paddedWindowWidth, paddedWindowHeight);

    reallocateFragmentBuffer();

    size_t startOffsetBufferSizeBytes = sizeof(uint32_t) * paddedWindowWidth * paddedWindowHeight;
    startOffsetBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    startOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            startOffsetBufferSizeBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    atomicCounterBuffer = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    atomicCounterBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);
}

void PerPixelLinkedListLineRenderer::render() {
    setUniformData();
    if (sceneData.performanceMeasurer) {
        timer->startGPU("PPLLClear", frameCounter);
        clear();
        timer->end();
        timer->startGPU("FCGather", frameCounter);
        gather();
        timer->end();
        timer->startGPU("PPLLResolve", frameCounter);
        resolve();
        timer->end();
    } else {
        clear();
        gather();
        resolve();
    }
    frameCounter++;
}

void PerPixelLinkedListLineRenderer::setUniformData() {
    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, startOffsetBuffer);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBuffer);

    gatherShader->setUniform("viewportW", paddedWindowWidth);
    gatherShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
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

    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        gatherShaderHull->setUniform("viewportW", paddedWindowWidth);
        gatherShaderHull->setUniform("linkedListSize", (unsigned int)fragmentBufferSize);
    }

    resolveShader->setUniform("viewportW", paddedWindowWidth);
    clearShader->setUniform("viewportW", paddedWindowWidth);
}

void PerPixelLinkedListLineRenderer::clear() {
    glDepthMask(GL_FALSE);

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(clearRenderData);

    // Set atomic counter to zero.
    GLuint bufferID = static_cast<sgl::GeometryBufferGL*>(atomicCounterBuffer.get())->getBuffer();
    GLubyte val = 0;
    glClearNamedBufferData(bufferID, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void PerPixelLinkedListLineRenderer::gather() {
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
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(shaderAttributes);
    renderHull();
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void PerPixelLinkedListLineRenderer::resolve() {
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

void PerPixelLinkedListLineRenderer::renderGui() {
    if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
        reRender = true;
    }
    if (lineData) {
        lineData->renderGuiRenderingSettings();
    }
    if (ImGui::Combo(
            "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
        setSortingAlgorithmDefine();
        reloadResolveShader();
        reRender = true;
    }
    if (ImGui::Button("Reload Shader")) {
        reloadGatherShader();
        reRender = true;
    }
}
