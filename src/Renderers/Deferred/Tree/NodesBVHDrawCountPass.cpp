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
#include "PersistentThreadHelper.hpp"
#include "NodesBVHDrawCountPass.hpp"

NodesBVHDrawCountPass::NodesBVHDrawCountPass(sgl::vk::Renderer* renderer) : ComputePass(renderer) {}

void NodesBVHDrawCountPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    dataDirty = true;
}

void NodesBVHDrawCountPass::setRecheckOccludedOnly(bool recheck) {
    if (recheckOccludedOnly != recheck) {
        recheckOccludedOnly = recheck;
        setShaderDirty();
    }
}

void NodesBVHDrawCountPass::setUseSubgroupOps(bool _useSubgroupOps) {
    if (useSubgroupOps != _useSubgroupOps) {
        useSubgroupOps = _useSubgroupOps;
        setShaderDirty();
    }
}

void NodesBVHDrawCountPass::setDrawIndexedIndirectMode(bool _drawIndexedIndirectMode) {
    if (drawIndexedIndirectMode != _drawIndexedIndirectMode) {
        drawIndexedIndirectMode = _drawIndexedIndirectMode;
        setShaderDirty();
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setUseMeshShaderNV(bool _useMeshShaderNV) {
    if (useMeshShaderNV != _useMeshShaderNV) {
        useMeshShaderNV = _useMeshShaderNV;
        setShaderDirty();
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setMaxNumPrimitivesPerMeshlet(uint32_t _maxNumPrimitivesPerMeshlet) {
    if (maxNumPrimitivesPerMeshlet != _maxNumPrimitivesPerMeshlet) {
        maxNumPrimitivesPerMeshlet = _maxNumPrimitivesPerMeshlet;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setMaxNumVerticesPerMeshlet(uint32_t _maxNumVerticesPerMeshlet) {
    if (maxNumVerticesPerMeshlet != _maxNumVerticesPerMeshlet) {
        maxNumVerticesPerMeshlet = _maxNumVerticesPerMeshlet;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
        bool _useMeshShaderWritePackedPrimitiveIndices) {
    if (useMeshShaderWritePackedPrimitiveIndicesIfAvailable != _useMeshShaderWritePackedPrimitiveIndices) {
        useMeshShaderWritePackedPrimitiveIndicesIfAvailable = _useMeshShaderWritePackedPrimitiveIndices;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setBvhBuildAlgorithm(BvhBuildAlgorithm _bvhBuildAlgorithm) {
    if (bvhBuildAlgorithm != _bvhBuildAlgorithm) {
        bvhBuildAlgorithm = _bvhBuildAlgorithm;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setBvhBuildGeometryMode(BvhBuildGeometryMode _bvhBuildGeometryMode) {
    if (bvhBuildGeometryMode != _bvhBuildGeometryMode) {
        bvhBuildGeometryMode = _bvhBuildGeometryMode;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setBvhBuildPrimitiveCenterMode(BvhBuildPrimitiveCenterMode _bvhBuildPrimitiveCenterMode) {
    if (bvhBuildPrimitiveCenterMode != _bvhBuildPrimitiveCenterMode) {
        bvhBuildPrimitiveCenterMode = _bvhBuildPrimitiveCenterMode;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setUseStdBvhParameters(bool _useStdBvhParameters) {
    if (useStdBvhParameters != _useStdBvhParameters) {
        useStdBvhParameters = _useStdBvhParameters;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setMaxLeafSizeBvh(uint32_t _maxLeafSizeBvh) {
    if (maxLeafSizeBvh != _maxLeafSizeBvh) {
        maxLeafSizeBvh = _maxLeafSizeBvh;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setMaxTreeDepthBvh(uint32_t _maxTreeDepthBvh) {
    if (maxTreeDepthBvh != _maxTreeDepthBvh) {
        maxTreeDepthBvh = _maxTreeDepthBvh;
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setShallVisualizeNodes(uint32_t _shallVisualizeNodes) {
    if (shallVisualizeNodes != _shallVisualizeNodes) {
        shallVisualizeNodes = _shallVisualizeNodes;
        setShaderDirty();
        setDataDirty();
    }
}

void NodesBVHDrawCountPass::setNumWorkgroups(uint32_t numWorkgroupsBvh) {
    numWorkgroups = numWorkgroupsBvh;
}

void NodesBVHDrawCountPass::setWorkgroupSize(uint32_t workgroupSizeBvh) {
    workgroupSize = workgroupSizeBvh;
    setShaderDirty();
}

void NodesBVHDrawCountPass::setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer) {
    visibilityCullingUniformBuffer = uniformBuffer;
}

void NodesBVHDrawCountPass::setDepthBufferTexture(const sgl::vk::TexturePtr& texture) {
    depthBufferTexture = texture;
    setDataDirty();
}

void NodesBVHDrawCountPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    preprocessorDefines.insert(std::make_pair("WORKGROUP_SIZE", std::to_string(workgroupSize)));
    preprocessorDefines.insert(std::make_pair("SUBGROUP_SIZE", std::to_string(
            device->getPhysicalDeviceSubgroupProperties().subgroupSize)));
    if (recheckOccludedOnly) {
        preprocessorDefines.insert(std::make_pair("RECHECK_OCCLUDED_ONLY", ""));
    }
    if (drawIndexedIndirectMode) {
        preprocessorDefines.insert(std::make_pair("OUTPUT_DRAW_INDEXED_INDIRECT", ""));
    } else {
        preprocessorDefines.insert(std::make_pair("OUTPUT_MESH_SHADER", ""));
    }
    if (useSubgroupOps) {
        preprocessorDefines.insert(std::make_pair("USE_SUBGROUP_OPS", ""));
    }
    if (shallVisualizeNodes) {
        preprocessorDefines.insert(std::make_pair("VISUALIZE_BVH_HIERARCHY", ""));
    }
    if (useSpinlock) {
        shaderStages = sgl::vk::ShaderManager->getShaderStages(
                { "NodesBVHDrawCountPass.TraverseSpinlock.Compute" }, preprocessorDefines);
    } else {
        shaderStages = sgl::vk::ShaderManager->getShaderStages(
                { "NodesBVHDrawCountPass.Traverse.Compute" }, preprocessorDefines);
    }
}

void NodesBVHDrawCountPass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
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

    numNodes = payload->getNumNodes();
    indirectDrawCountBuffer = payload->getIndirectDrawCountBuffer();
    computeData->setStaticBuffer(payload->getNodeDataBuffer(), "NodeBuffer");
    if (shallVisualizeNodes) {
        nodeAabbBuffer = payload->getNodeAabbBuffer();
        nodeAabbCountBuffer = payload->getNodeAabbCountBuffer();
        nodeIdxToTreeHeightBuffer = payload->getNodeIdxToTreeHeightBuffer();
        computeData->setStaticBuffer(nodeAabbBuffer, "NodeAabbBuffer");
        computeData->setStaticBuffer(nodeAabbCountBuffer, "NodeAabbCountBuffer");
        computeData->setStaticBuffer(nodeIdxToTreeHeightBuffer, "NodeIdxToTreeHeightBuffer");
    }
    if (recheckOccludedOnly) {
        computeData->setStaticBuffer(payload->getQueueBufferRecheck(), "QueueBuffer");
        computeData->setStaticBuffer(payload->getQueueStateBufferRecheck(), "QueueStateBuffer");
    } else {
        computeData->setStaticBuffer(payload->getQueueBuffer(), "QueueBuffer");
        computeData->setStaticBuffer(payload->getQueueStateBuffer(), "QueueStateBuffer");
        computeData->setStaticBuffer(payload->getQueueBufferRecheck(), "QueueBufferRecheck");
        computeData->setStaticBuffer(payload->getQueueStateBufferRecheck(), "QueueStateBufferRecheck");
    }
    computeData->setStaticBuffer(payload->getQueueInfoBuffer(), "QueueInfoBuffer");
    if (drawIndexedIndirectMode) {
        computeData->setStaticBuffer(
                payload->getIndirectDrawBuffer(), "DrawIndexedIndirectCommandBuffer");
    } else {
        computeData->setStaticBuffer(
                payload->getVisibleMeshletIndexArrayBuffer(), "VisibleMeshletIndexArrayBuffer");
    }
    computeData->setStaticBuffer(indirectDrawCountBuffer, "IndirectDrawCountBuffer");
    computeData->setStaticBuffer(visibilityCullingUniformBuffer, "VisibilityCullingUniformBuffer");
    computeData->setStaticTexture(depthBufferTexture, "depthBuffer");
    computeData->setStaticBuffer(payload->getMaxWorkLeftTestBuffer(), "TestBuffer"); // For debugging.
    maxWorkLeftTestBuffer = payload->getMaxWorkLeftTestBuffer();
}

void NodesBVHDrawCountPass::_render() {
    indirectDrawCountBuffer->fill(0, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            indirectDrawCountBuffer);
    renderer->dispatch(
            computeData, std::min(sgl::uiceil(numNodes, workgroupSize), numWorkgroups),
            1, 1);
}
