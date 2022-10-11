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
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include "LineData/LineData.hpp"
#include "LineData/TrianglePayload/NodesBVHTreePayload.hpp"
#include "ConvertMeshletCommandsBVHPass.hpp"

ConvertMeshletCommandsBVHPass::ConvertMeshletCommandsBVHPass(sgl::vk::Renderer* renderer) : ComputePass(renderer) {}

void ConvertMeshletCommandsBVHPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    dataDirty = true;
}

void ConvertMeshletCommandsBVHPass::setUseMeshShaderNV(bool _useMeshShaderNV) {
    if (useMeshShaderNV != _useMeshShaderNV) {
        useMeshShaderNV = _useMeshShaderNV;
        setShaderDirty();
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setMaxNumPrimitivesPerMeshlet(uint32_t _maxNumPrimitivesPerMeshlet) {
    if (maxNumPrimitivesPerMeshlet != _maxNumPrimitivesPerMeshlet) {
        maxNumPrimitivesPerMeshlet = _maxNumPrimitivesPerMeshlet;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setMaxNumVerticesPerMeshlet(uint32_t _maxNumVerticesPerMeshlet) {
    if (maxNumVerticesPerMeshlet != _maxNumVerticesPerMeshlet) {
        maxNumVerticesPerMeshlet = _maxNumVerticesPerMeshlet;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
        bool _useMeshShaderWritePackedPrimitiveIndices) {
    if (useMeshShaderWritePackedPrimitiveIndicesIfAvailable != _useMeshShaderWritePackedPrimitiveIndices) {
        useMeshShaderWritePackedPrimitiveIndicesIfAvailable = _useMeshShaderWritePackedPrimitiveIndices;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setBvhBuildAlgorithm(BvhBuildAlgorithm _bvhBuildAlgorithm) {
    if (bvhBuildAlgorithm != _bvhBuildAlgorithm) {
        bvhBuildAlgorithm = _bvhBuildAlgorithm;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setBvhBuildGeometryMode(BvhBuildGeometryMode _bvhBuildGeometryMode) {
    if (bvhBuildGeometryMode != _bvhBuildGeometryMode) {
        bvhBuildGeometryMode = _bvhBuildGeometryMode;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setBvhBuildPrimitiveCenterMode(BvhBuildPrimitiveCenterMode _bvhBuildPrimitiveCenterMode) {
    if (bvhBuildPrimitiveCenterMode != _bvhBuildPrimitiveCenterMode) {
        bvhBuildPrimitiveCenterMode = _bvhBuildPrimitiveCenterMode;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setUseStdBvhParameters(bool _useStdBvhParameters) {
    if (useStdBvhParameters != _useStdBvhParameters) {
        useStdBvhParameters = _useStdBvhParameters;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setMaxLeafSizeBvh(uint32_t _maxLeafSizeBvh) {
    if (maxLeafSizeBvh != _maxLeafSizeBvh) {
        maxLeafSizeBvh = _maxLeafSizeBvh;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::setMaxTreeDepthBvh(uint32_t _maxTreeDepthBvh) {
    if (maxTreeDepthBvh != _maxTreeDepthBvh) {
        maxTreeDepthBvh = _maxTreeDepthBvh;
        setDataDirty();
    }
}

void ConvertMeshletCommandsBVHPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    if (useMeshShaderNV) {
        preprocessorDefines.insert(std::make_pair("VK_NV_mesh_shader", ""));
    } else {
        preprocessorDefines.insert(std::make_pair("VK_EXT_mesh_shader", ""));
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "ConvertMeshletCommands.Compute" }, preprocessorDefines);
}

void ConvertMeshletCommandsBVHPass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
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
            false, maxNumPrimitivesPerMeshlet, maxNumVerticesPerMeshlet,
            useMeshShaderWritePackedPrimitiveIndices,
            bvhBuildAlgorithm, bvhBuildGeometryMode, bvhBuildPrimitiveCenterMode,
            useStdBvhParameters, maxLeafSizeBvh, maxTreeDepthBvh));
    TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
            true, false, payloadSuperClass);

    if (!tubeRenderData.indexBuffer) {
        return;
    }
    auto* payload = static_cast<NodesBVHTreePayload*>(payloadSuperClass.get());

    computeData->setStaticBuffer(payload->getIndirectDrawCountBuffer(), "IndirectDrawCountBuffer");
    computeData->setStaticBuffer(payload->getTasksIndirectCommandBuffer(), "TasksIndirectCommandBuffer");
    if (useMeshShaderNV) {
        computeData->setStaticBuffer(
                payload->getTasksIndirectCommandsCountBuffer(), "TasksIndirectCommandsCountBuffer");
    }
}

void ConvertMeshletCommandsBVHPass::_render() {
    if (useMeshShaderNV) {
        uint32_t maxDrawMeshTasksCount = device->getPhysicalDeviceMeshShaderPropertiesNV().maxDrawMeshTasksCount;
        renderer->pushConstants(
                computeData->getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT,
                0, maxDrawMeshTasksCount);
    }

    renderer->dispatch(computeData, 1, 1, 1);
}
