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
#include "LineData/LineData.hpp"
#include "LineData/TrianglePayload/MeshletsDrawIndirectPayload.hpp"
#include "MeshletVisibilityPass.hpp"

MeshletVisibilityPass::MeshletVisibilityPass(sgl::vk::Renderer* renderer) : ComputePass(renderer) {}

void MeshletVisibilityPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    dataDirty = true;
}

void MeshletVisibilityPass::setRecheckOccludedOnly(bool _recheckOccludedOnly) {
    if (recheckOccludedOnly != _recheckOccludedOnly) {
        recheckOccludedOnly = _recheckOccludedOnly;
        setShaderDirty();
    }
}

void MeshletVisibilityPass::setMaxNumPrimitivesPerMeshlet(uint32_t _maxNumPrimitivesPerMeshlet) {
    if (maxNumPrimitivesPerMeshlet != _maxNumPrimitivesPerMeshlet) {
        maxNumPrimitivesPerMeshlet = _maxNumPrimitivesPerMeshlet;
        setShaderDirty();
    }
}

void MeshletVisibilityPass::setShallVisualizeNodes(uint32_t _shallVisualizeNodes) {
    if (shallVisualizeNodes != _shallVisualizeNodes) {
        shallVisualizeNodes = _shallVisualizeNodes;
        setDataDirty();
        setShaderDirty();
    }
}

void MeshletVisibilityPass::setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer) {
    visibilityCullingUniformBuffer = uniformBuffer;
}

void MeshletVisibilityPass::setDepthBufferTexture(const sgl::vk::TexturePtr& texture) {
    depthBufferTexture = texture;
    setDataDirty();
}

void MeshletVisibilityPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    preprocessorDefines.insert(std::make_pair("WORKGROUP_SIZE", std::to_string(WORKGROUP_SIZE)));
    if (recheckOccludedOnly) {
        preprocessorDefines.insert(std::make_pair("RECHECK_OCCLUDED_ONLY", ""));
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "MeshletVisibilityPass.Compute" }, preprocessorDefines);
}

void MeshletVisibilityPass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);

    TubeTriangleRenderDataPayloadPtr payloadSuperClass(new MeshletsDrawIndirectPayload(
            maxNumPrimitivesPerMeshlet, shallVisualizeNodes));
    TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
            true, false, payloadSuperClass);

    if (!tubeRenderData.indexBuffer) {
        return;
    }
    auto* payload = static_cast<MeshletsDrawIndirectPayload*>(payloadSuperClass.get());

    numMeshlets = payload->getNumMeshlets();
    computeData->setStaticBuffer(payload->getMeshletDataBuffer(), "MeshletDataBuffer");
    computeData->setStaticBuffer(payload->getMeshletVisibilityArrayBuffer(), "MeshletVisibilityArrayBuffer");
    computeData->setStaticBuffer(visibilityCullingUniformBuffer, "VisibilityCullingUniformBuffer");
    computeData->setStaticTexture(depthBufferTexture, "depthBuffer");
}

void MeshletVisibilityPass::_render() {
    renderer->dispatch(computeData, sgl::uiceil(numMeshlets, WORKGROUP_SIZE), 1, 1);
}
