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

#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include "VoxelRayCastingRenderer.hpp"

VoxelRayCastingRenderer::VoxelRayCastingRenderer(
        SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Voxel Ray Casting Renderer", sceneData, transferFunctionWindow) {
    isVulkanRenderer = false;
    isRasterizer = false;

    onResolutionChanged();
}

void VoxelRayCastingRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine(
            "gridResolution", uvec3ToString(gridResolution));
    sgl::ShaderManager->addPreprocessorDefine(
            "GRID_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(gridResolution.x))));
    sgl::ShaderManager->addPreprocessorDefine(
            "GRID_RESOLUTION", gridResolution.x);
    sgl::ShaderManager->addPreprocessorDefine(
            "quantizationResolution", uvec3ToString(quantizationResolution));
    sgl::ShaderManager->addPreprocessorDefine(
            "QUANTIZATION_RESOLUTION", sgl::toString(quantizationResolution.x));
    sgl::ShaderManager->addPreprocessorDefine(
            "QUANTIZATION_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(quantizationResolution.x))));
    sgl::ShaderManager->addPreprocessorDefine(
            "MAX_NUM_HITS", maxNumHits);
    sgl::ShaderManager->addPreprocessorDefine(
            "MAX_NUM_LINES_PER_VOXEL", maxNumLinesPerVoxel);

    renderShader = sgl::ShaderManager->getShaderProgram(
            { "VoxelRayCasting.Vertex", "VoxelRayCasting.Fragment" });

    sgl::ShaderManager->removePreprocessorDefine("gridResolution");
    sgl::ShaderManager->removePreprocessorDefine("GRID_RESOLUTION_LOG2");
    sgl::ShaderManager->removePreprocessorDefine("GRID_RESOLUTION");
    sgl::ShaderManager->removePreprocessorDefine("quantizationResolution");
    sgl::ShaderManager->removePreprocessorDefine("QUANTIZATION_RESOLUTION");
    sgl::ShaderManager->removePreprocessorDefine("QUANTIZATION_RESOLUTION_LOG2");
    sgl::ShaderManager->removePreprocessorDefine("MAX_NUM_HITS");
    sgl::ShaderManager->removePreprocessorDefine("MAX_NUM_LINES_PER_VOXEL");

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    blitRenderData = sgl::ShaderManager->createShaderAttributes(renderShader);
    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());
    blitRenderData->addGeometryBuffer(geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
}

void VoxelRayCastingRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    voxelCurveDiscretizer.loadLineData(lineData, gridResolution1D, quantizationResolution1D);
    if (useGpuForVoxelization) {
        voxelCurveDiscretizer.createVoxelGridGpu();
    } else {
        voxelCurveDiscretizer.createVoxelGridCpu();
    }
    voxelGridLineSegmentOffsetsBuffer = voxelCurveDiscretizer.getVoxelGridLineSegmentOffsetsBuffer();
    voxelGridNumLineSegmentsBuffer = voxelCurveDiscretizer.getVoxelGridNumLineSegmentsBuffer();
    voxelGridLineSegmentsBuffer = voxelCurveDiscretizer.getVoxelGridLineSegmentsBuffer();

    worldToVoxelGridMatrix = voxelCurveDiscretizer.getWorldToVoxelGridMatrix();
    gridResolution = voxelCurveDiscretizer.getGridResolution();
    quantizationResolution = voxelCurveDiscretizer.getQuantizationResolution();

    reloadGatherShader();

    dirty = false;
    reRender = true;
}

void VoxelRayCastingRenderer::onResolutionChanged() {
}

void VoxelRayCastingRenderer::setUniformData() {
    // Set camera data
    renderShader->setUniform("fov", sceneData.camera->getFOVy());
    renderShader->setUniform("aspectRatio", sceneData.camera->getAspectRatio());
    glm::mat4 inverseViewMatrix = glm::inverse(sceneData.camera->getViewMatrix());
    renderShader->setUniform("inverseViewMatrix", inverseViewMatrix);

    float voxelSpaceLineRadius = lineWidth * 0.5f * glm::length(worldToVoxelGridMatrix[0]);
    renderShader->setUniform("lineRadius", voxelSpaceLineRadius);
    renderShader->setUniform("clearColor", sceneData.clearColor);

    if (renderShader->hasUniform("cameraPosition")) {
        renderShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    }

    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    renderShader->setUniform("viewportSize", glm::ivec2(width, height));

    sgl::ShaderManager->bindShaderStorageBuffer(0, voxelGridLineSegmentOffsetsBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(1, voxelGridNumLineSegmentsBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(2, voxelGridLineSegmentsBuffer);
    //if (renderShader->hasUniform("densityTexture")) {
    //    renderShader->setUniform("densityTexture", densityTexture, 0);
    //}
    //if (renderShader->hasUniform("aoTexture")) {
    //    renderShader->setUniform("aoTexture", aoTexture, 1);
    //}
    if (renderShader->hasUniform("transferFunctionTexture")) {
        renderShader->setUniform("transferFunctionTexture", transferFunctionWindow.getTransferFunctionMapTexture(), 2);
    }

    renderShader->setUniform("worldSpaceToVoxelSpace", worldToVoxelGridMatrix);

    lineData->setUniformGatherShaderData(renderShader);
    setUniformData_Pass(renderShader);
}

void VoxelRayCastingRenderer::render() {
    setUniformData();

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    sgl::Renderer->render(blitRenderData);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

void VoxelRayCastingRenderer::renderGui() {
    LineRenderer::renderGui();

    bool voxelGridDirty = false;

    if (ImGui::Checkbox("Use GPU for Voxelization", &useGpuForVoxelization)) {
        voxelGridDirty = true;
    }

    uint32_t minVal = 4;
    uint32_t maxVal = 256;
    if (ImGui::SliderInt("Grid Resolution", &gridResolution1D, 4, 256)) {
        voxelGridDirty = true;
    }
    int quantizationResolutionLog = sgl::intlog2(quantizationResolution1D);
    if (ImGui::SliderIntPowerOfTwo("Quantization Resolution", &quantizationResolution1D, 1, 64)) {
        voxelGridDirty = true;
    }

    if (voxelGridDirty) {
        dirty = true;
        internalReRender = true;
        reRender = true;
    }
}
