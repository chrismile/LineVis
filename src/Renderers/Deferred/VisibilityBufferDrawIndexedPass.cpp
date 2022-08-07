/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#include "Renderers/LineRenderer.hpp"
#include "VisibilityBufferDrawIndexedPass.hpp"

VisibilityBufferDrawIndexedPass::VisibilityBufferDrawIndexedPass(LineRenderer* lineRenderer)
        : LineRasterPass(lineRenderer) {
}

void VisibilityBufferDrawIndexedPass::setDrawIndexedGeometryMode(DrawIndexedGeometryMode geometryModeNew) {
    if (geometryMode != geometryModeNew) {
        geometryMode = geometryModeNew;
        setShaderDirty();
        setDataDirty();
    }
}

void VisibilityBufferDrawIndexedPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    std::vector<std::string> shaderModuleNames;
    if (geometryMode == DrawIndexedGeometryMode::TRIANGLES) {
        shaderModuleNames = {
                "VisibilityBufferDrawIndexed.Vertex",
                "VisibilityBufferDrawIndexed.Fragment"
        };
    } else if (geometryMode == DrawIndexedGeometryMode::PROGRAMMABLE_PULLING) {
        shaderModuleNames = {
                "VisibilityBufferDrawIndexed.Vertex.ProgrammablePull",
                "VisibilityBufferDrawIndexed.Fragment"
        };
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            shaderModuleNames, preprocessorDefines);
}

void VisibilityBufferDrawIndexedPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);
    if (geometryMode == DrawIndexedGeometryMode::TRIANGLES) {
        pipelineInfo.setVertexBufferBindingByLocationIndex(
                "vertexPosition", sizeof(TubeTriangleVertexData));
    }

    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    if (lineData->getUseCappedTubes() && geometryMode == DrawIndexedGeometryMode::TRIANGLES) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }

    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setDepthTestEnabled(true);
    pipelineInfo.setDepthWriteEnabled(true);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::OVERWRITE);
}

void VisibilityBufferDrawIndexedPass::createRasterData(
        sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    //lineRenderer->setRenderDataBindings(rasterData);

    if (geometryMode == DrawIndexedGeometryMode::TRIANGLES) {
        TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderData(
                true, false);
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setVertexBuffer(tubeRenderData.vertexBuffer, "vertexPosition");
    } else if (geometryMode == DrawIndexedGeometryMode::PROGRAMMABLE_PULLING) {
        LinePassTubeRenderDataProgrammablePull tubeRenderData = lineData->getLinePassTubeRenderDataProgrammablePull();
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setStaticBufferOptional(
                tubeRenderData.linePointDataBuffer, "LinePointDataBuffer");
        if (tubeRenderData.stressLinePointDataBuffer) {
            rasterData->setStaticBufferOptional(
                    tubeRenderData.stressLinePointDataBuffer,
                    "StressLinePointDataBuffer");
        }
        if (tubeRenderData.stressLinePointPrincipalStressDataBuffer) {
            rasterData->setStaticBufferOptional(
                    tubeRenderData.stressLinePointPrincipalStressDataBuffer,
                    "StressLinePointPrincipalStressDataBuffer");
        }
    }
}
