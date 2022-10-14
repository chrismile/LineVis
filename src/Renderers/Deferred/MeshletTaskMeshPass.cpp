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
#include "LineData/TrianglePayload/NodesBVHTreePayload.hpp"
#include "MeshletTaskMeskPass.hpp"

MeshletTaskMeshPass::MeshletTaskMeshPass(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
    const VkPhysicalDeviceMeshShaderPropertiesNV& meshShaderPropertiesNV =
            device->getPhysicalDeviceMeshShaderPropertiesNV();
    WORKGROUP_SIZE_NV = std::min(
            meshShaderPropertiesNV.maxTaskWorkGroupSize[0], meshShaderPropertiesNV.maxMeshWorkGroupSize[0]);
#ifdef VK_EXT_mesh_shader
    const VkPhysicalDeviceMeshShaderPropertiesEXT& meshShaderPropertiesEXT =
            device->getPhysicalDeviceMeshShaderPropertiesEXT();
    WORKGROUP_SIZE_EXT = meshShaderPropertiesEXT.maxPreferredTaskWorkGroupInvocations;
#endif
}

void MeshletTaskMeshPass::setRecheckOccludedOnly(bool recheck) {
    if (recheckOccludedOnly != recheck) {
        recheckOccludedOnly = recheck;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setUseMeshShaderNV(bool _useMeshShaderNV) {
    if (useMeshShaderNV != _useMeshShaderNV) {
        useMeshShaderNV = _useMeshShaderNV;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setMaxNumPrimitivesPerMeshlet(uint32_t numPrimitives) {
    if (maxNumPrimitivesPerMeshlet != numPrimitives) {
        maxNumPrimitivesPerMeshlet = numPrimitives;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setMaxNumVerticesPerMeshlet(uint32_t numVertices) {
    if (maxNumVerticesPerMeshlet != numVertices) {
        maxNumVerticesPerMeshlet = numVertices;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(bool useWritePacked) {
    if (useMeshShaderWritePackedPrimitiveIndicesIfAvailable != useWritePacked) {
        useMeshShaderWritePackedPrimitiveIndicesIfAvailable = useWritePacked;
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setShallVisualizeNodes(bool _shallVisualizeNodes) {
    if (shallVisualizeNodes != _shallVisualizeNodes) {
        shallVisualizeNodes = _shallVisualizeNodes;
        setDataDirty();
        setShaderDirty();
    }
}

void MeshletTaskMeshPass::setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer) {
    visibilityCullingUniformBuffer = uniformBuffer;
}

void MeshletTaskMeshPass::setDepthBufferTexture(const sgl::vk::TexturePtr& texture) {
    depthBufferTexture = texture;
    setDataDirty();
}

void MeshletTaskMeshPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    uint32_t WORKGROUP_SIZE = useMeshShaderNV ? WORKGROUP_SIZE_NV : WORKGROUP_SIZE_EXT;
    preprocessorDefines.insert(std::make_pair(
            "WORKGROUP_SIZE", std::to_string(WORKGROUP_SIZE)));
    preprocessorDefines.insert(std::make_pair(
            "MESHLET_MAX_VERTICES", std::to_string(maxNumVerticesPerMeshlet)));
    preprocessorDefines.insert(std::make_pair(
            "MESHLET_MAX_PRIMITIVES", std::to_string(maxNumPrimitivesPerMeshlet)));
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
    if (useMeshShaderWritePackedPrimitiveIndices) {
        preprocessorDefines.insert(std::make_pair("WRITE_PACKED_PRIMITIVE_INDICES", ""));
    }
    if (recheckOccludedOnly) {
        preprocessorDefines.insert(std::make_pair("RECHECK_OCCLUDED_ONLY", ""));
    }
    if (bvhMeshlets) {
        preprocessorDefines.insert(std::make_pair("BVH_MESHLETS", ""));
    }
    if (!bvhMeshlets && shallVisualizeNodes) {
        preprocessorDefines.insert(std::make_pair("VISUALIZE_BVH_HIERARCHY", ""));
    }

    std::vector<std::string> shaderIds;
    shaderIds.reserve(bvhMeshlets ? 2 : 3);
    if (!bvhMeshlets) {
        shaderIds.emplace_back(useMeshShaderNV ? "MeshletTaskMeshPass.TaskNV" : "MeshletTaskMeshPass.TaskEXT");
    }
    shaderIds.emplace_back(useMeshShaderNV ? "MeshletTaskMeshPass.MeshNV" : "MeshletTaskMeshPass.MeshEXT");
    shaderIds.emplace_back("MeshletTaskMeshPass.Fragment");
    shaderStages = sgl::vk::ShaderManager->getShaderStages(shaderIds, preprocessorDefines);
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
            maxNumPrimitivesPerMeshlet, maxNumVerticesPerMeshlet, useMeshShaderWritePackedPrimitiveIndices,
            shallVisualizeNodes));
    TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
            true, false, payloadSuperClass);

    if (!tubeRenderData.indexBuffer) {
        numMeshlets = 0;
        return;
    }
    auto* payload = static_cast<MeshletsTaskMeshShaderPayload*>(payloadSuperClass.get());

    numMeshlets = payload->getNumMeshlets();
    if (shallVisualizeNodes) {
        nodeAabbBuffer = payload->getNodeAabbBuffer();
        nodeAabbCountBuffer = payload->getNodeAabbCountBuffer();
        rasterData->setStaticBuffer(nodeAabbBuffer, "NodeAabbBuffer");
        rasterData->setStaticBuffer(nodeAabbCountBuffer, "NodeAabbCountBuffer");
    }
    rasterData->setStaticBuffer(payload->getMeshletDataBuffer(), "MeshletDataBuffer");
    rasterData->setStaticBuffer(payload->getMeshletVisibilityArrayBuffer(), "MeshletVisibilityArrayBuffer");
    rasterData->setStaticBuffer(payload->getDedupVerticesBuffer(), "DedupVerticesBuffer");
    rasterData->setStaticBuffer(payload->getDedupTriangleIndicesBuffer(), "DedupTriangleIndicesBuffer");
    rasterData->setStaticBuffer(visibilityCullingUniformBuffer, "VisibilityCullingUniformBuffer");
    rasterData->setStaticTexture(depthBufferTexture, "depthBuffer");
    if (useMeshShaderNV) {
        rasterData->setMeshTasksNV(sgl::uiceil(numMeshlets, WORKGROUP_SIZE_NV), 0);
    } else {
        rasterData->setMeshTasksGroupCountEXT(
                sgl::uiceil(numMeshlets, WORKGROUP_SIZE_EXT), 1, 1);
    }
}

void MeshletTaskMeshPass::_render() {
    renderer->render(rasterData);
}



MeshletMeshBVHPass::MeshletMeshBVHPass(LineRenderer* lineRenderer) : MeshletTaskMeshPass(lineRenderer) {
    bvhMeshlets = true;
}

void MeshletMeshBVHPass::setBvhBuildAlgorithm(BvhBuildAlgorithm _bvhBuildAlgorithm) {
    if (bvhBuildAlgorithm != _bvhBuildAlgorithm) {
        bvhBuildAlgorithm = _bvhBuildAlgorithm;
        setDataDirty();
    }
}

void MeshletMeshBVHPass::setBvhBuildGeometryMode(BvhBuildGeometryMode _bvhBuildGeometryMode) {
    if (bvhBuildGeometryMode != _bvhBuildGeometryMode) {
        bvhBuildGeometryMode = _bvhBuildGeometryMode;
        setDataDirty();
    }
}

void MeshletMeshBVHPass::setBvhBuildPrimitiveCenterMode(BvhBuildPrimitiveCenterMode _bvhBuildPrimitiveCenterMode) {
    if (bvhBuildPrimitiveCenterMode != _bvhBuildPrimitiveCenterMode) {
        bvhBuildPrimitiveCenterMode = _bvhBuildPrimitiveCenterMode;
        setDataDirty();
    }
}

void MeshletMeshBVHPass::setUseStdBvhParameters(bool _useStdBvhParameters) {
    if (useStdBvhParameters != _useStdBvhParameters) {
        useStdBvhParameters = _useStdBvhParameters;
        setDataDirty();
    }
}

void MeshletMeshBVHPass::setMaxLeafSizeBvh(uint32_t _maxLeafSizeBvh) {
    if (maxLeafSizeBvh != _maxLeafSizeBvh) {
        maxLeafSizeBvh = _maxLeafSizeBvh;
        setDataDirty();
    }
}

void MeshletMeshBVHPass::setMaxTreeDepthBvh(uint32_t _maxTreeDepthBvh) {
    if (maxTreeDepthBvh != _maxTreeDepthBvh) {
        maxTreeDepthBvh = _maxTreeDepthBvh;
        setDataDirty();
    }
}

void MeshletMeshBVHPass::createRasterData(
        sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    //lineRenderer->setRenderDataBindings(rasterData);

    TubeTriangleRenderDataPayloadPtr payloadSuperClass(new NodesBVHTreePayload(
            false, maxNumPrimitivesPerMeshlet, maxNumVerticesPerMeshlet, useMeshShaderWritePackedPrimitiveIndices,
            bvhBuildAlgorithm, bvhBuildGeometryMode, bvhBuildPrimitiveCenterMode,
            useStdBvhParameters, maxLeafSizeBvh, maxTreeDepthBvh, shallVisualizeNodes));
    TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
            true, false, payloadSuperClass);

    if (!tubeRenderData.indexBuffer) {
        numMeshlets = 0;
        return;
    }
    auto* payload = static_cast<NodesBVHTreePayload*>(payloadSuperClass.get());

    numMeshlets = payload->getNumTreeLeafMeshlets();
    treeHeight = payload->getTreeHeight();
    if (shallVisualizeNodes) {
        nodeAabbBuffer = payload->getNodeAabbBuffer();
        nodeAabbCountBuffer = payload->getNodeAabbBuffer();
    }
    rasterData->setStaticBuffer(payload->getTreeLeafMeshletsBuffer(), "MeshletDataBuffer");
    rasterData->setStaticBuffer(payload->getDedupVerticesBuffer(), "DedupVerticesBuffer");
    rasterData->setStaticBuffer(payload->getDedupTriangleIndicesBuffer(), "DedupTriangleIndicesBuffer");
    rasterData->setStaticBuffer(
            payload->getVisibleMeshletIndexArrayBuffer(), "VisibleMeshletIndexArrayBuffer");
    uint32_t stride;
    if (useMeshShaderNV) {
        stride = 8; //< sizeof(VkDrawMeshTasksIndirectCommandNV)
    } else {
        stride = 12; //< sizeof(VkDrawMeshTasksIndirectCommandEXT)
    }
    rasterData->setIndirectDrawBuffer(payload->getTasksIndirectCommandBuffer(), stride);
    if (useMeshShaderNV) {
        const auto& meshShaderPropertiesNV =
                device->getPhysicalDeviceMeshShaderPropertiesNV();
        uint32_t numTreeLeafMeshlets = payload->getNumTreeLeafMeshlets();
        uint32_t numDrawCallsMax = sgl::uiceil(numTreeLeafMeshlets, meshShaderPropertiesNV.maxDrawMeshTasksCount);
        rasterData->setIndirectDrawCountBuffer(
                payload->getTasksIndirectCommandsCountBuffer(), numDrawCallsMax);
        rasterData->setMeshTasksNV(sgl::uiceil(numMeshlets, WORKGROUP_SIZE_NV), 0);
    } else {
        rasterData->setIndirectDrawCount(1);
        rasterData->setMeshTasksGroupCountEXT(
                sgl::uiceil(numMeshlets, WORKGROUP_SIZE_EXT), 1, 1);
    }
}
