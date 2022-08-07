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

#include <Math/Math.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include "Renderers/LineRenderer.hpp"
#include "LineData/LineData.hpp"
#include "LineData/TrianglePayload/MeshletsTaskMeshShaderPayload.hpp"
#include "MeshletTaskMeskPass.hpp"

MeshletTaskMeshPass::MeshletTaskMeshPass(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
    const VkPhysicalDeviceMeshShaderPropertiesNV& meshShaderProperties =
            device->getPhysicalDeviceMeshShaderPropertiesNV();
    WORKGROUP_SIZE = std::min(
            meshShaderProperties.maxTaskWorkGroupSize[0], meshShaderProperties.maxMeshWorkGroupSize[0]);
}

void MeshletTaskMeshPass::setRecheckOccludedOnly(bool recheck) {
    if (recheckOccludedOnly != recheck) {
        recheckOccludedOnly = recheck;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setMaxNumPrimitivesPerMeshlet(uint32_t num) {
    if (maxNumPrimitivesPerMeshlet != num) {
        maxNumPrimitivesPerMeshlet = num;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setMaxNumVerticesPerMeshlet(uint32_t num) {
    if (maxNumVerticesPerMeshlet != num) {
        maxNumVerticesPerMeshlet = num;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer) {
    visibilityCullingUniformBuffer = uniformBuffer;
}

void MeshletTaskMeshPass::setDepthBufferTexture(const sgl::vk::TexturePtr& texture) {
    depthBufferTexture = texture;
}

void MeshletTaskMeshPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    preprocessorDefines.insert(std::make_pair(
            "WORKGROUP_SIZE", std::to_string(WORKGROUP_SIZE)));
    preprocessorDefines.insert(std::make_pair(
            "MESHLET_MAX_VERTICES", std::to_string(maxNumVerticesPerMeshlet)));
    preprocessorDefines.insert(std::make_pair(
            "MESHLET_MAX_PRIMITIVES", std::to_string(maxNumPrimitivesPerMeshlet)));
    if (recheckOccludedOnly) {
        preprocessorDefines.insert(std::make_pair("RECHECK_OCCLUDED_ONLY", ""));
    }

    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "MeshletTaskMeshPass.Task", "MeshletTaskMeshPass.Mesh", "MeshletTaskMeshPass.Fragment" },
            preprocessorDefines);
}

void MeshletTaskMeshPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);

    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    if (lineData->getUseCappedTubes()) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }

    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setDepthTestEnabled(true);
    pipelineInfo.setDepthWriteEnabled(true);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::OVERWRITE);
}

void MeshletTaskMeshPass::createRasterData(
        sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    //lineRenderer->setRenderDataBindings(rasterData);

    TubeTriangleRenderDataPayloadPtr payloadSuperClass(new MeshletsTaskMeshShaderPayload(
            maxNumPrimitivesPerMeshlet, maxNumVerticesPerMeshlet));
    TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
            true, false, payloadSuperClass);

    if (!tubeRenderData.indexBuffer) {
        numMeshlets = 0;
        return;
    }
    auto* payload = static_cast<MeshletsTaskMeshShaderPayload*>(payloadSuperClass.get());

    numMeshlets = payload->getNumMeshlets();
    rasterData->setStaticBuffer(payload->getMeshletDataBuffer(), "MeshletDataBuffer");
    rasterData->setStaticBuffer(payload->getMeshletVisibilityArrayBuffer(), "MeshletVisibilityArrayBuffer");
    rasterData->setStaticBuffer(payload->getDedupVerticesBuffer(), "DedupVerticesBuffer");
    rasterData->setStaticBuffer(payload->getDedupTriangleIndicesBuffer(), "DedupTriangleIndicesBuffer");
    rasterData->setStaticBuffer(visibilityCullingUniformBuffer, "VisibilityCullingUniformBuffer");
    rasterData->setStaticTexture(depthBufferTexture, "depthBuffer");
    rasterData->setMeshTasks(sgl::uiceil(numMeshlets, WORKGROUP_SIZE), 0);
}

void MeshletTaskMeshPass::_render() {
    renderer->render(rasterData);
}
