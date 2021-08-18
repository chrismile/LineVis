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

#include <iostream>

#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>

#include "LineRenderer.hpp"

float LineRenderer::lineWidth = STANDARD_LINE_WIDTH;
float LineRenderer::bandWidth = STANDARD_BAND_WIDTH;

// For compute shaders.
constexpr size_t BLOCK_SIZE = 256;

void LineRenderer::initialize() {
    updateDepthCueMode();
    sgl::ShaderManager->addPreprocessorDefine("COMPUTE_DEPTH_CUES_GPU", "");
    computeDepthValuesShaderProgram = sgl::ShaderManager->getShaderProgram({"ComputeDepthValues.Compute"});
    minMaxReduceDepthShaderProgram = sgl::ShaderManager->getShaderProgram({"MinMaxReduceDepth.Compute"});
}

LineRenderer::~LineRenderer() {
    if (useDepthCues) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
    sgl::ShaderManager->removePreprocessorDefine("COMPUTE_DEPTH_CUES_GPU");
}

void LineRenderer::update(float dt) {
}

void LineRenderer::updateDepthCueMode() {
    if (useDepthCues) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SCREEN_SPACE_POSITION", "");
        sgl::ShaderManager->addPreprocessorDefine("USE_DEPTH_CUES", "");

        if (lineData && filteredLines.empty()) {
            updateDepthCueGeometryData();
        }
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
}

void LineRenderer::updateDepthCueGeometryData() {
    filteredLines = lineData->getFilteredLines();
    std::vector<glm::vec4> filteredLinesVertices;
    for (std::vector<glm::vec3>& line : filteredLines) {
        for (const glm::vec3& point : line) {
            filteredLinesVertices.push_back(glm::vec4(point.x, point.y, point.z, 1.0f));
        }
    }
    filteredLinesVerticesBuffer = sgl::Renderer->createGeometryBuffer(
            filteredLinesVertices.size() * sizeof(glm::vec4), filteredLinesVertices.data(),
            sgl::SHADER_STORAGE_BUFFER);

    depthMinMaxBuffers[0] = sgl::Renderer->createGeometryBuffer(
            sgl::iceil(filteredLinesVertices.size(), BLOCK_SIZE) * sizeof(glm::vec2),
            sgl::SHADER_STORAGE_BUFFER);
    depthMinMaxBuffers[1] = sgl::Renderer->createGeometryBuffer(
            sgl::iceil(filteredLinesVertices.size(), BLOCK_SIZE * BLOCK_SIZE * 2) * sizeof(glm::vec2),
            sgl::SHADER_STORAGE_BUFFER);
}

void LineRenderer::setUniformData_Pass(sgl::ShaderProgramPtr shaderProgram) {
    if (useDepthCues && lineData && computeDepthCuesOnGpu) {
        sgl::ShaderManager->bindShaderStorageBuffer(12, filteredLinesVerticesBuffer);
        sgl::ShaderManager->bindShaderStorageBuffer(11, depthMinMaxBuffers[0]);
        uint32_t numVertices = filteredLinesVerticesBuffer->getSize() / sizeof(glm::vec4);
        uint32_t numBlocks = sgl::iceil(numVertices, BLOCK_SIZE);
        computeDepthValuesShaderProgram->setUniform("numVertices", numVertices);
        computeDepthValuesShaderProgram->setUniform("nearDist", sceneData.camera->getNearClipDistance());
        computeDepthValuesShaderProgram->setUniform("farDist", sceneData.camera->getFarClipDistance());
        computeDepthValuesShaderProgram->setUniform("cameraViewMatrix", sceneData.camera->getViewMatrix());
        computeDepthValuesShaderProgram->setUniform(
                "cameraProjectionMatrix", sceneData.camera->getProjectionMatrix());
        computeDepthValuesShaderProgram->dispatchCompute(numBlocks);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        minMaxReduceDepthShaderProgram->setUniform("nearDist", sceneData.camera->getNearClipDistance());
        minMaxReduceDepthShaderProgram->setUniform("farDist", sceneData.camera->getFarClipDistance());
        int iteration = 0;
        uint32_t inputSize;
        while (numBlocks > 1) {
            if (iteration != 0) {
                // Already bound for computeDepthValuesShaderProgram if i == 0.
                sgl::ShaderManager->bindShaderStorageBuffer(11, depthMinMaxBuffers[iteration % 2]);
            }
            sgl::ShaderManager->bindShaderStorageBuffer(12, depthMinMaxBuffers[(iteration + 1) % 2]);

            inputSize = numBlocks;
            numBlocks = sgl::iceil(numBlocks, BLOCK_SIZE*2);
            minMaxReduceDepthShaderProgram->setUniform("sizeOfInput", inputSize);
            minMaxReduceDepthShaderProgram->dispatchCompute(numBlocks);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

            iteration++;
        }

        // Bind the output of ComputeDepthValues.glsl to position 12 for Lighting.glsl if no reduction was necessary.
        if (iteration == 0) {
            sgl::ShaderManager->bindShaderStorageBuffer(12, depthMinMaxBuffers[0]);
        }
    }

    if (useDepthCues && lineData && !computeDepthCuesOnGpu) {
        bool useBoundingBox = lineData->getNumLines() > 1000;
        if (useBoundingBox) {
            const sgl::AABB3& boundingBox = lineData->getModelBoundingBox();
            sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed(sceneData.camera->getViewMatrix());
            minDepth = -screenSpaceBoundingBox.getMaximum().z;
            maxDepth = -screenSpaceBoundingBox.getMinimum().z;
        } else {
            glm::mat4 viewMatrix = sceneData.camera->getViewMatrix();
            minDepth = std::numeric_limits<float>::max();
            maxDepth = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
            #pragma omp parallel for default(none) shared(viewMatrix, filteredLines) \
            reduction(min: minDepth) reduction(max: maxDepth)
#endif
            for (size_t lineIdx = 0; lineIdx < filteredLines.size(); lineIdx++) {
                const std::vector<glm::vec3>& line = filteredLines.at(lineIdx);
                for (const glm::vec3& point : line) {
                    float depth = -sgl::transformPoint(viewMatrix, point).z;
                    minDepth = std::min(minDepth, depth);
                    maxDepth = std::max(maxDepth, depth);
                }
            }
        }

        minDepth = glm::clamp(
                minDepth, sceneData.camera->getFarClipDistance(), sceneData.camera->getNearClipDistance());
        maxDepth = glm::clamp(
                maxDepth, sceneData.camera->getFarClipDistance(), sceneData.camera->getNearClipDistance());

        shaderProgram->setUniformOptional("minDepth", minDepth);
        shaderProgram->setUniformOptional("maxDepth", maxDepth);
    }

    shaderProgram->setUniformOptional("depthCueStrength", depthCueStrength);

    if (ambientOcclusionBaker && ambientOcclusionBaker->getHasComputationFinished()) {
        //sgl::GeometryBufferPtr aoBuffer = ambientOcclusionBaker->getAmbientOcclusionBuffer();
        //sgl::ShaderManager->bindShaderStorageBuffer(13, aoBuffer);
    }
}

void LineRenderer::setAmbientOcclusionBaker(AmbientOcclusionBakerPtr& aoBaker) {
    ambientOcclusionBaker = aoBaker;
}

bool LineRenderer::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = false;

    settings.getValueOpt("line_width", lineWidth);
    settings.getValueOpt("band_width", bandWidth);

    if (settings.getValueOpt("depth_cue_strength", depthCueStrength)) {
        if (depthCueStrength <= 0.0f && useDepthCues) {
            useDepthCues = false;
            updateDepthCueMode();
            shallReloadGatherShader = true;
        }
        if (depthCueStrength > 0.0f && !useDepthCues) {
            useDepthCues = true;
            updateDepthCueMode();
            shallReloadGatherShader = true;
        }
    }

    return shallReloadGatherShader;
}

void LineRenderer::reloadGatherShaderExternal() {
    if (lineData) {
        reloadGatherShader(false);
        setLineData(lineData, false);
    }
}

void LineRenderer::renderGuiWindow() {
    bool shallReloadGatherShader = false;

    if (windowName == "Opaque Line Renderer") {
        sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(2, 14, 735, 564);
    }
    if (ImGui::Begin(windowName.c_str(), &showRendererWindow)) {
        this->renderGui();
        if (lineData) {
            ImGui::Separator();
            if (lineData->renderGuiRenderer(isRasterizer)) {
                shallReloadGatherShader = true;
            }
            if (ImGui::SliderFloat(
                    "Depth Cue Strength", &depthCueStrength, 0.0f, 1.0f)) {
                if (depthCueStrength <= 0.0f && useDepthCues) {
                    useDepthCues = false;
                    updateDepthCueMode();
                    shallReloadGatherShader = true;
                }
                if (depthCueStrength > 0.0f && !useDepthCues) {
                    useDepthCues = true;
                    updateDepthCueMode();
                    shallReloadGatherShader = true;
                }
                reRender = true;
            }
        }
    }
    ImGui::End();

    if (lineData && lineData->renderGuiWindow(isRasterizer)) {
        shallReloadGatherShader = true;
    }
    if (lineData && lineData->renderGuiWindowSecondary(isRasterizer)) {
        shallReloadGatherShader = true;
    }

    if (shallReloadGatherShader) {
        reloadGatherShaderExternal();
    }
}

void LineRenderer::updateNewLineData(LineDataPtr& lineData, bool isNewData) {
    if (!this->lineData || lineData->getType() != this->lineData->getType()
            || lineData->settingsDiffer(this->lineData.get())) {
        this->lineData = lineData;
        //ambientOcclusionBaker->startAmbientOcclusionBaking(lineData);
        reloadGatherShader(false);
    }
    this->lineData = lineData;

    filteredLines.clear();
    filteredLinesVerticesBuffer = sgl::GeometryBufferPtr();
    depthMinMaxBuffers[0] = sgl::GeometryBufferPtr();
    depthMinMaxBuffers[1] = sgl::GeometryBufferPtr();
    if (useDepthCues) {
        updateDepthCueGeometryData();
    }

    if (lineData && lineData->hasSimulationMeshOutline()) {
        shaderAttributesHull = sgl::ShaderAttributesPtr();
        shaderAttributesHull = lineData->getGatherShaderAttributesHull(gatherShaderHull);
    }
}

void LineRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    if (lineData && lineData->hasSimulationMeshOutline()) {
        gatherShaderHull = lineData->reloadGatherShaderHull();
        if (canCopyShaderAttributes && shaderAttributesHull) {
            shaderAttributesHull = shaderAttributesHull->copy(gatherShaderHull);
        }
    }
}

void LineRenderer::renderHull() {
    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        lineData->setUniformGatherShaderDataHull_Pass(gatherShaderHull);
        gatherShaderHull->setUniformOptional("cameraPosition", sceneData.camera->getPosition());
        glDisable(GL_CULL_FACE);
        sgl::Renderer->render(shaderAttributesHull);
        glEnable(GL_CULL_FACE);
    }
}
