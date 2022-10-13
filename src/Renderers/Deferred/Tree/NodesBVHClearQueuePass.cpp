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
#include "LineData/TrianglePayload/NodesBVHTreePayload.hpp"
#include "NodesBVHClearQueuePass.hpp"

NodesBVHClearQueuePass::NodesBVHClearQueuePass(sgl::vk::Renderer* renderer) : ComputePass(renderer) {}

void NodesBVHClearQueuePass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    dataDirty = true;
}

void NodesBVHClearQueuePass::setDrawIndexedIndirectMode(bool _drawIndexedIndirectMode) {
    if (drawIndexedIndirectMode != _drawIndexedIndirectMode) {
        drawIndexedIndirectMode = _drawIndexedIndirectMode;
        setShaderDirty();
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setUseMeshShaderNV(bool _useMeshShaderNV) {
    if (useMeshShaderNV != _useMeshShaderNV) {
        useMeshShaderNV = _useMeshShaderNV;
        setShaderDirty();
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setMaxNumPrimitivesPerMeshlet(uint32_t _maxNumPrimitivesPerMeshlet) {
    if (maxNumPrimitivesPerMeshlet != _maxNumPrimitivesPerMeshlet) {
        maxNumPrimitivesPerMeshlet = _maxNumPrimitivesPerMeshlet;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setMaxNumVerticesPerMeshlet(uint32_t _maxNumVerticesPerMeshlet) {
    if (maxNumVerticesPerMeshlet != _maxNumVerticesPerMeshlet) {
        maxNumVerticesPerMeshlet = _maxNumVerticesPerMeshlet;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
        bool _useMeshShaderWritePackedPrimitiveIndices) {
    if (useMeshShaderWritePackedPrimitiveIndicesIfAvailable != _useMeshShaderWritePackedPrimitiveIndices) {
        useMeshShaderWritePackedPrimitiveIndicesIfAvailable = _useMeshShaderWritePackedPrimitiveIndices;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setBvhBuildAlgorithm(BvhBuildAlgorithm _bvhBuildAlgorithm) {
    if (bvhBuildAlgorithm != _bvhBuildAlgorithm) {
        bvhBuildAlgorithm = _bvhBuildAlgorithm;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setBvhBuildGeometryMode(BvhBuildGeometryMode _bvhBuildGeometryMode) {
    if (bvhBuildGeometryMode != _bvhBuildGeometryMode) {
        bvhBuildGeometryMode = _bvhBuildGeometryMode;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setBvhBuildPrimitiveCenterMode(BvhBuildPrimitiveCenterMode _bvhBuildPrimitiveCenterMode) {
    if (bvhBuildPrimitiveCenterMode != _bvhBuildPrimitiveCenterMode) {
        bvhBuildPrimitiveCenterMode = _bvhBuildPrimitiveCenterMode;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setUseStdBvhParameters(bool _useStdBvhParameters) {
    if (useStdBvhParameters != _useStdBvhParameters) {
        useStdBvhParameters = _useStdBvhParameters;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setMaxLeafSizeBvh(uint32_t _maxLeafSizeBvh) {
    if (maxLeafSizeBvh != _maxLeafSizeBvh) {
        maxLeafSizeBvh = _maxLeafSizeBvh;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setMaxTreeDepthBvh(uint32_t _maxTreeDepthBvh) {
    if (maxTreeDepthBvh != _maxTreeDepthBvh) {
        maxTreeDepthBvh = _maxTreeDepthBvh;
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setShallVisualizeNodes(uint32_t _shallVisualizeNodes) {
    if (shallVisualizeNodes != _shallVisualizeNodes) {
        shallVisualizeNodes = _shallVisualizeNodes;
        setShaderDirty();
        setDataDirty();
    }
}

void NodesBVHClearQueuePass::setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer) {
    visibilityCullingUniformBuffer = uniformBuffer;
}

void NodesBVHClearQueuePass::setDepthBufferTexture(const sgl::vk::TexturePtr& texture) {
    depthBufferTexture = texture;
    setDataDirty();
}

void NodesBVHClearQueuePass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    if (shallVisualizeNodes) {
        preprocessorDefines.insert(std::make_pair("VISUALIZE_BVH_HIERARCHY", ""));
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "NodesBVHDrawCountPass.Initialize.Compute" }, preprocessorDefines);
}

void NodesBVHClearQueuePass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);

    useMeshShaderWritePackedPrimitiveIndices = useMeshShaderWritePackedPrimitiveIndicesIfAvailable && useMeshShaderNV;
    if (useMeshShaderWritePackedPrimitiveIndicesIfAvailable) {
        /*
         * The number of indices needs to be divisible by four to be able to use writePackedPrimitiveIndices4x8NV.
         * The number of vertices and primitives is (for N = subdivisions, x = segments):
         * (N + Nx) vertices and (2Nx) primitives.
         * Thus: The number of primitives is divisible by four if N is divisible by two.
         */
        if (lineData->getUseCappedTubes() || lineData->getTubeNumSubdivisions() % 2 != 0) {
            useMeshShaderWritePackedPrimitiveIndices = false;
        }
    }

    TubeTriangleRenderDataPayloadPtr payloadSuperClass(new NodesBVHTreePayload(
            drawIndexedIndirectMode,
            maxNumPrimitivesPerMeshlet, maxNumVerticesPerMeshlet, useMeshShaderWritePackedPrimitiveIndices,
            bvhBuildAlgorithm, bvhBuildGeometryMode, bvhBuildPrimitiveCenterMode,
            useStdBvhParameters, maxLeafSizeBvh, maxTreeDepthBvh, shallVisualizeNodes));
    TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
            true, false, payloadSuperClass);

    if (!tubeRenderData.indexBuffer) {
        return;
    }
    auto* payload = static_cast<NodesBVHTreePayload*>(payloadSuperClass.get());

    queueBuffer = payload->getQueueBuffer();
    queueBufferRecheck = payload->getQueueBufferRecheck();

    if (shallVisualizeNodes) {
        computeData->setStaticBuffer(payload->getNodeAabbBuffer(), "NodeAabbBuffer");
        computeData->setStaticBuffer(payload->getNodeAabbCountBuffer(), "NodeAabbCountBuffer");
    }
    computeData->setStaticBuffer(payload->getNodeDataBuffer(), "NodeBuffer");
    computeData->setStaticBuffer(payload->getQueueBuffer(), "QueueBuffer");
    computeData->setStaticBuffer(payload->getQueueStateBuffer(), "QueueStateBuffer");
    computeData->setStaticBuffer(payload->getQueueBufferRecheck(), "QueueBufferRecheck");
    computeData->setStaticBuffer(payload->getQueueStateBufferRecheck(), "QueueStateBufferRecheck");
    computeData->setStaticBuffer(visibilityCullingUniformBuffer, "VisibilityCullingUniformBuffer");
    computeData->setStaticTexture(depthBufferTexture, "depthBuffer");
}

void NodesBVHClearQueuePass::_render() {
    // Remove / check performance compared to write in shader.
    /*uint32_t emptyData = 0xFFFFFFFFu;
    queueBuffer->fill(
            0, queueBuffer->getSizeInBytes(), emptyData,
            renderer->getVkCommandBuffer());
    queueBufferRecheck->fill(
            0, queueBufferRecheck->getSizeInBytes(), emptyData,
            renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);*/

    renderer->dispatch(computeData, 1, 1, 1);
}
