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

void LineRenderer::initialize() {
    updateDepthCueMode();
}

LineRenderer::~LineRenderer() {
    if (useDepthCues && lineData) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
}

void LineRenderer::update(float dt) {
}

void LineRenderer::updateDepthCueMode() {
    if (useDepthCues) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SCREEN_SPACE_POSITION", "");
        sgl::ShaderManager->addPreprocessorDefine("USE_DEPTH_CUES", "");
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
}

void LineRenderer::setUniformData_Pass(sgl::ShaderProgramPtr shaderProgram) {
    if (useDepthCues && lineData) {
        bool useBoundingBox = false; // lineData->getNumLines() > 1000
        if (useBoundingBox) {
            const sgl::AABB3& boundingBox = lineData->getModelBoundingBox();
            sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed(sceneData.camera->getViewMatrix());
            minDepth = -screenSpaceBoundingBox.getMaximum().z;
            maxDepth = -screenSpaceBoundingBox.getMinimum().z;
        } else {
            std::cout << lineData->getNumLines() << std::endl;
            glm::mat4 viewMatrix = sceneData.camera->getViewMatrix();
            minDepth = std::numeric_limits<float>::max();
            maxDepth = std::numeric_limits<float>::lowest();
            #pragma omp parallel for default(none) shared(viewMatrix, filteredLines) \
            reduction(min: minDepth) reduction(max: maxDepth)
            for (size_t lineIdx = 0; lineIdx < filteredLines.size(); lineIdx++) {
                const std::vector<glm::vec3>& line = filteredLines.at(lineIdx);
                for (const glm::vec3& point : line) {
                    float depth = -sgl::transformPoint(viewMatrix, point).z;
                    minDepth = std::min(minDepth, depth);
                    maxDepth = std::max(maxDepth, depth);
                }
            }
        }

        minDepth = std::max(minDepth, sceneData.camera->getNearClipDistance());
        maxDepth = std::min(maxDepth, sceneData.camera->getFarClipDistance());
        minDepth = std::min(minDepth, sceneData.camera->getFarClipDistance());
        maxDepth = std::max(maxDepth, sceneData.camera->getNearClipDistance());
    }

    if (useDepthCues) {
        shaderProgram->setUniformOptional("minDepth", minDepth);
        shaderProgram->setUniformOptional("maxDepth", maxDepth);
    }
}

void LineRenderer::renderGuiWindow() {
    bool shallReloadGatherShader = false;

    if (ImGui::Begin(windowName.c_str(), &showRendererWindow)) {
        this->renderGui();
        if (lineData) {
            ImGui::Separator();
            if (lineData->renderGui(isRasterizer)) {
                shallReloadGatherShader = true;
            }
            if (ImGui::Checkbox("Depth Cues", &useDepthCues)) {
                updateDepthCueMode();
                shallReloadGatherShader = true;
                reRender = true;
            }
        }
    }
    ImGui::End();

    if (lineData && lineData->renderGuiWindow(isRasterizer)) {
        shallReloadGatherShader = true;
    }

    if (shallReloadGatherShader) {
        reloadGatherShader(false);
        if (lineData) {
            setLineData(lineData, false);
        }
    }
}

void LineRenderer::updateNewLineData(LineDataPtr& lineData, bool isNewMesh) {
    if (!this->lineData || lineData->getType() != this->lineData->getType()
            || lineData->settingsDiffer(this->lineData.get())) {
        this->lineData = lineData;
        reloadGatherShader(false);
    }
    this->lineData = lineData;
    filteredLines = lineData->getFilteredLines();

    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        shaderAttributesHull = sgl::ShaderAttributesPtr();
        shaderAttributesHull = lineData->getGatherShaderAttributesHull(gatherShaderHull);
    }
}

void LineRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
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
