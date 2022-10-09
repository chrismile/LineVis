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

#include "DeferredRenderer.hpp"

#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include <memory>
#include <utility>

#include "LineData/LineDataStress.hpp"
#include "LineData/TrianglePayload/MeshletsDrawIndirectPayload.hpp"
#include "../HullRasterPass.hpp"

#include "VisibilityBufferDrawIndexedPass.hpp"
#include "VisibilityBufferDrawIndexedIndirectPass.hpp"
#include "MeshletDrawCountNoReductionPass.hpp"
#include "MeshletDrawCountAtomicPass.hpp"
#include "MeshletVisibilityPass.hpp"
#include "VisibilityBufferPrefixSumScanPass.hpp"
#include "MeshletDrawCountPass.hpp"
#include "MeshletTaskMeskPass.hpp"
#include "Tree/PersistentThreadHelper.hpp"
#include "Tree/NodesBVHClearQueuePass.hpp"
#include "Tree/NodesBVHDrawCountPass.hpp"
#include "Tree/ConvertMeshletCommandsBVHPass.hpp"
#include "DeferredRenderer.hpp"

DeferredRenderer::DeferredRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opaque Line Renderer", sceneData, transferFunctionWindow) {
    sgl::vk::Device* device = (*sceneData->renderer)->getDevice();

    // Needs gl_DrawIDARB via shaderDrawParameters in the shader to get the indirect draw call-corrected gl_PrimitiveID.
    supportsDrawIndirect =
            device->getPhysicalDeviceShaderDrawParametersFeatures().shaderDrawParameters
            && device->getPhysicalDeviceFeatures().multiDrawIndirect;

    supportsDrawIndirectCount =
            supportsDrawIndirect && device->isDeviceExtensionSupported(VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME);
    if (!supportsDrawIndirectCount) {
        drawIndirectReductionMode = DrawIndirectReductionMode::NO_REDUCTION;
    }

    const auto& meshShaderFeaturesNV =
            device->getPhysicalDeviceMeshShaderFeaturesNV();
    const auto& meshShaderPropertiesNV =
            device->getPhysicalDeviceMeshShaderPropertiesNV();
    supportsTaskMeshShadersNV = meshShaderFeaturesNV.taskShader && meshShaderFeaturesNV.meshShader;
    taskMeshShaderMaxNumPrimitivesSupportedNV = meshShaderPropertiesNV.maxMeshOutputPrimitives;
    // Support a maximum of 256 vertices, as the mesh shader uses 8-bit unsigned indices for meshlets.
    taskMeshShaderMaxNumVerticesSupportedNV = std::min(meshShaderPropertiesNV.maxMeshOutputVertices, uint32_t(256));

#ifdef VK_EXT_mesh_shader
    const auto& meshShaderFeaturesEXT =
            device->getPhysicalDeviceMeshShaderFeaturesEXT();
    const auto& meshShaderPropertiesEXT =
            device->getPhysicalDeviceMeshShaderPropertiesEXT();
    supportsTaskMeshShadersEXT = meshShaderFeaturesEXT.taskShader && meshShaderFeaturesEXT.meshShader;
    taskMeshShaderMaxNumPrimitivesSupportedEXT = meshShaderPropertiesEXT.maxMeshOutputPrimitives;
    // Support a maximum of 256 vertices, as the mesh shader uses 8-bit unsigned indices for meshlets.
    taskMeshShaderMaxNumVerticesSupportedEXT = std::min(meshShaderPropertiesEXT.maxMeshOutputVertices, uint32_t(256));
#endif

    if (!device->isDeviceExtensionSupported(VK_KHR_SHADER_FLOAT16_INT8_EXTENSION_NAME)
            || !device->isDeviceExtensionSupported(VK_KHR_8BIT_STORAGE_EXTENSION_NAME)) {
        supportsTaskMeshShadersNV = false;
#ifdef VK_EXT_mesh_shader
        supportsTaskMeshShadersEXT = false;
#endif
    } else {
        if (!device->getPhysicalDeviceShaderFloat16Int8Features().shaderInt8
                || !device->getPhysicalDevice8BitStorageFeatures().storageBuffer8BitAccess) {
            supportsTaskMeshShadersNV = false;
#ifdef VK_EXT_mesh_shader
            supportsTaskMeshShadersEXT = false;
#endif
        }
    }
    supportsTaskMeshShaders = supportsTaskMeshShadersNV || supportsTaskMeshShadersEXT;
    useMeshShaderNV = !supportsTaskMeshShadersEXT;

    if (supportsTaskMeshShaders) {
        updateTaskMeshShaderMode();
    }
}

void DeferredRenderer::initialize() {
    LineRenderer::initialize();

    visibilityCullingUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(VisibilityCullingUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    deferredResolvePass = std::make_shared<DeferredResolvePass>(this);
    deferredResolvePass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    downsampleBlitPass = std::make_shared<DownsampleBlitPass>(renderer);

    updateRenderingMode();
    onClearColorChanged();
}

void DeferredRenderer::reloadShaders() {
    reloadGatherShader();
    reloadResolveShader();
}

void DeferredRenderer::reloadResolveShader() {
    deferredResolvePass->setShaderDirty();
}

void DeferredRenderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();
    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        visibilityBufferDrawIndexedPass->setShaderDirty();
    } else if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        for (int i = 0; i < 2; i++) {
            visibilityBufferDrawIndexedIndirectPasses[i]->setShaderDirty();
        }
        if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
            for (int i = 0; i < 2; i++) {
                meshletDrawCountNoReductionPasses[i]->setShaderDirty();
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
            for (int i = 0; i < 2; i++) {
                meshletDrawCountAtomicPasses[i]->setShaderDirty();
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
            for (int i = 0; i < 2; i++) {
                meshletVisibilityPasses[i]->setShaderDirty();
            }
            visibilityBufferPrefixSumScanPass->setShaderDirty();
            meshletDrawCountPass->setShaderDirty();
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            meshletTaskMeshPasses[i]->setShaderDirty();
        }
    } else if (getIsBvhRenderingMode()) {
        nodesBVHClearQueuePass->setShaderDirty();
        if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
            convertMeshletCommandsBVHPass->setShaderDirty();
        }
        for (int i = 0; i < 2; i++) {
            nodesBVHDrawCountPasses[i]->setShaderDirty();
            if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setShaderDirty();
            } else {
                meshletMeshBVHPasses[i]->setShaderDirty();
            }
        }
    }
}

void DeferredRenderer::updateRenderingMode() {
    frameNumber = 0;
    visibilityBufferDrawIndexedPass = {};
    visibilityBufferPrefixSumScanPass = {};
    meshletDrawCountPass = {};
    nodesBVHClearQueuePass = {};
    for (int i = 0; i < 2; i++) {
        visibilityBufferDrawIndexedIndirectPasses[i] = {};
        meshletDrawCountAtomicPasses[i] = {};
        meshletVisibilityPasses[i] = {};
        meshletTaskMeshPasses[i] = {};
        nodesBVHDrawCountPasses[i] = {};
        visibilityBufferBVHDrawIndexedIndirectPasses[i] = {};
    }
    deferredResolvePass->setDataDirty();

    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        visibilityBufferDrawIndexedPass = std::make_shared<VisibilityBufferDrawIndexedPass>(this);
        updateGeometryMode();
        onResolutionChangedDeferredRenderingMode();
        if (lineData) {
            setLineData(lineData, false);
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        for (int i = 0; i < 2; i++) {
            visibilityBufferDrawIndexedIndirectPasses[i] =
                    std::make_shared<VisibilityBufferDrawIndexedIndirectPass>(this);
            visibilityBufferDrawIndexedIndirectPasses[i]->setMaxNumPrimitivesPerMeshlet(
                    drawIndirectMaxNumPrimitivesPerMeshlet);
        }
        visibilityBufferDrawIndexedIndirectPasses[0]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
        visibilityBufferDrawIndexedIndirectPasses[1]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_LOAD);
        updateGeometryMode();
        updateDrawIndirectReductionMode(); // Already contains call to @see onResolutionChangedDeferredRenderingMode.
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            meshletTaskMeshPasses[i] = std::make_shared<MeshletTaskMeshPass>(this);
            meshletTaskMeshPasses[i]->setUseMeshShaderNV(useMeshShaderNV);
            meshletTaskMeshPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
            meshletTaskMeshPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
            meshletTaskMeshPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                    useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            meshletTaskMeshPasses[i]->setVisibilityCullingUniformBuffer(visibilityCullingUniformDataBuffer);
        }
        meshletTaskMeshPasses[0]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
        meshletTaskMeshPasses[1]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_LOAD);
        meshletTaskMeshPasses[1]->setRecheckOccludedOnly(true);
        updateGeometryMode();
        onResolutionChangedDeferredRenderingMode();
        if (lineData) {
            setLineData(lineData, false);
        }
    } else if (getIsBvhRenderingMode()) {
        initializeOptimalNumWorkgroups();
        nodesBVHClearQueuePass = std::make_shared<NodesBVHClearQueuePass>(renderer);
        nodesBVHClearQueuePass->setDrawIndexedIndirectMode(
                deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
        } else {
            nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
            nodesBVHClearQueuePass->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
            nodesBVHClearQueuePass->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                    useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
        }
        nodesBVHClearQueuePass->setBvhBuildAlgorithm(bvhBuildAlgorithm);
        nodesBVHClearQueuePass->setBvhBuildGeometryMode(bvhBuildGeometryMode);
        nodesBVHClearQueuePass->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
        nodesBVHClearQueuePass->setVisibilityCullingUniformBuffer(visibilityCullingUniformDataBuffer);
        nodesBVHClearQueuePass->setUseStdBvhParameters(useStdBvhParameters);
        nodesBVHClearQueuePass->setMaxLeafSizeBvh(maxLeafSizeBvh);
        nodesBVHClearQueuePass->setMaxTreeDepthBvh(maxTreeDepthBvh);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
            convertMeshletCommandsBVHPass = std::make_shared<ConvertMeshletCommandsBVHPass>(renderer);
            convertMeshletCommandsBVHPass->setUseMeshShaderNV(useMeshShaderNV);
            convertMeshletCommandsBVHPass->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
            convertMeshletCommandsBVHPass->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
            convertMeshletCommandsBVHPass->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                    useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            convertMeshletCommandsBVHPass->setBvhBuildAlgorithm(bvhBuildAlgorithm);
            convertMeshletCommandsBVHPass->setBvhBuildGeometryMode(bvhBuildGeometryMode);
            convertMeshletCommandsBVHPass->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
            convertMeshletCommandsBVHPass->setUseStdBvhParameters(useStdBvhParameters);
            convertMeshletCommandsBVHPass->setMaxLeafSizeBvh(maxLeafSizeBvh);
            convertMeshletCommandsBVHPass->setMaxTreeDepthBvh(maxTreeDepthBvh);
        }
        for (int i = 0; i < 2; i++) {
            nodesBVHDrawCountPasses[i] = std::make_shared<NodesBVHDrawCountPass>(renderer);
            nodesBVHDrawCountPasses[i]->setDrawIndexedIndirectMode(
                    deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT);
            if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
                nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
            } else {
                nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                nodesBVHDrawCountPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                nodesBVHDrawCountPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
            nodesBVHDrawCountPasses[i]->setBvhBuildAlgorithm(bvhBuildAlgorithm);
            nodesBVHDrawCountPasses[i]->setBvhBuildGeometryMode(bvhBuildGeometryMode);
            nodesBVHDrawCountPasses[i]->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
            nodesBVHDrawCountPasses[i]->setUseStdBvhParameters(useStdBvhParameters);
            nodesBVHDrawCountPasses[i]->setMaxLeafSizeBvh(maxLeafSizeBvh);
            nodesBVHDrawCountPasses[i]->setMaxTreeDepthBvh(maxTreeDepthBvh);
            nodesBVHDrawCountPasses[i]->setNumWorkgroups(numWorkgroupsBvh);
            nodesBVHDrawCountPasses[i]->setWorkgroupSize(workgroupSizeBvh);
            nodesBVHDrawCountPasses[i]->setUseSubgroupOps(useSubgroupOps);
            nodesBVHDrawCountPasses[i]->setVisibilityCullingUniformBuffer(visibilityCullingUniformDataBuffer);
            if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
                visibilityBufferBVHDrawIndexedIndirectPasses[i] =
                        std::make_shared<VisibilityBufferBVHDrawIndexedIndirectPass>(this);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        drawIndirectMaxNumPrimitivesPerMeshlet);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setBvhBuildAlgorithm(bvhBuildAlgorithm);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setBvhBuildGeometryMode(bvhBuildGeometryMode);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setUseStdBvhParameters(useStdBvhParameters);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxLeafSizeBvh(maxLeafSizeBvh);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxTreeDepthBvh(maxTreeDepthBvh);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setUseDrawIndexedIndirectCount(true);
            } else {
                meshletMeshBVHPasses[i] =
                        std::make_shared<MeshletMeshBVHPass>(this);
                meshletMeshBVHPasses[i]->setUseMeshShaderNV(useMeshShaderNV);
                meshletMeshBVHPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                meshletMeshBVHPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                meshletMeshBVHPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
                meshletMeshBVHPasses[i]->setBvhBuildAlgorithm(bvhBuildAlgorithm);
                meshletMeshBVHPasses[i]->setBvhBuildGeometryMode(bvhBuildGeometryMode);
                meshletMeshBVHPasses[i]->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
                meshletMeshBVHPasses[i]->setUseStdBvhParameters(useStdBvhParameters);
                meshletMeshBVHPasses[i]->setMaxLeafSizeBvh(maxLeafSizeBvh);
                meshletMeshBVHPasses[i]->setMaxTreeDepthBvh(maxTreeDepthBvh);
            }
        }
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[0]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
            visibilityBufferBVHDrawIndexedIndirectPasses[1]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_LOAD);
        } else {
            meshletMeshBVHPasses[0]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
            meshletMeshBVHPasses[1]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_LOAD);
        }
        nodesBVHDrawCountPasses[1]->setRecheckOccludedOnly(true);
        updateGeometryMode();
        onResolutionChangedDeferredRenderingMode();
        if (lineData) {
            setLineData(lineData, false);
        }
    } else {
        sgl::Logfile::get()->throwError("Error in DeferredRenderer::updateRenderingMode: Invalid rendering mode.");
    }
}

void DeferredRenderer::updateGeometryMode() {
    deferredResolvePass->setDrawIndexedGeometryMode(
            deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED
            ? drawIndexedGeometryMode : DrawIndexedGeometryMode::TRIANGLES);
    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        visibilityBufferDrawIndexedPass->setDrawIndexedGeometryMode(drawIndexedGeometryMode);
    }
}

void DeferredRenderer::updateDrawIndirectReductionMode() {
    for (int i = 0; i < 2; i++) {
        meshletDrawCountNoReductionPasses[i] = {};
        meshletDrawCountAtomicPasses[i] = {};
        meshletVisibilityPasses[i] = {};
    }
    visibilityBufferPrefixSumScanPass = {};
    meshletDrawCountPass = {};

    if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
        for (int i = 0; i < 2; i++) {
            meshletDrawCountNoReductionPasses[i] = std::make_shared<MeshletDrawCountNoReductionPass>(renderer);
            meshletDrawCountNoReductionPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
            meshletDrawCountNoReductionPasses[i]->setVisibilityCullingUniformBuffer(visibilityCullingUniformDataBuffer);
            visibilityBufferDrawIndexedIndirectPasses[i]->setUseDrawIndexedIndirectCount(false);
        }
        meshletDrawCountNoReductionPasses[1]->setRecheckOccludedOnly(true);
    } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
        for (int i = 0; i < 2; i++) {
            meshletDrawCountAtomicPasses[i] = std::make_shared<MeshletDrawCountAtomicPass>(renderer);
            meshletDrawCountAtomicPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
            meshletDrawCountAtomicPasses[i]->setVisibilityCullingUniformBuffer(visibilityCullingUniformDataBuffer);
            visibilityBufferDrawIndexedIndirectPasses[i]->setUseDrawIndexedIndirectCount(true);
        }
        meshletDrawCountAtomicPasses[1]->setRecheckOccludedOnly(true);
    } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
        for (int i = 0; i < 2; i++) {
            meshletVisibilityPasses[i] = std::make_shared<MeshletVisibilityPass>(renderer);
            meshletVisibilityPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
            meshletVisibilityPasses[i]->setVisibilityCullingUniformBuffer(visibilityCullingUniformDataBuffer);
            visibilityBufferDrawIndexedIndirectPasses[i]->setUseDrawIndexedIndirectCount(true);
        }
        meshletVisibilityPasses[1]->setRecheckOccludedOnly(true);
        visibilityBufferPrefixSumScanPass = std::make_shared<VisibilityBufferPrefixSumScanPass>(renderer);
        meshletDrawCountPass = std::make_shared<MeshletDrawCountPass>(renderer);
        meshletDrawCountPass->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
    }

    onResolutionChangedDeferredRenderingMode();
    if (lineData) {
        setLineData(lineData, false);
    }

    if (showVisibleMeshletStatistics) {
        for (size_t i = 0; i < frameHasNewStagingDataList.size(); i++) {
            frameHasNewStagingDataList.at(i) = false;
        }
        for (int passIdx = 0; passIdx < 2; passIdx++) {
            visibleMeshletCounters[passIdx] = 0;
        }
    }
}

void DeferredRenderer::updateTaskMeshShaderMode() {
    if (useMeshShaderNV) {
        taskMeshShaderMaxNumPrimitivesSupported = taskMeshShaderMaxNumPrimitivesSupportedNV;
        taskMeshShaderMaxNumVerticesSupported = taskMeshShaderMaxNumVerticesSupportedNV;
    } else {
        taskMeshShaderMaxNumPrimitivesSupported = taskMeshShaderMaxNumPrimitivesSupportedEXT;
        taskMeshShaderMaxNumVerticesSupported = taskMeshShaderMaxNumVerticesSupportedEXT;
    }
    taskMeshShaderMaxNumPrimitivesPerMeshlet = std::min(
            taskMeshShaderMaxNumPrimitivesPerMeshlet, taskMeshShaderMaxNumPrimitivesSupported);
    taskMeshShaderMaxNumVerticesPerMeshlet = std::min(
            taskMeshShaderMaxNumVerticesPerMeshlet, taskMeshShaderMaxNumVerticesSupported);

    if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            if (meshletTaskMeshPasses[i]) {
                meshletTaskMeshPasses[i]->setUseMeshShaderNV(useMeshShaderNV);
                meshletTaskMeshPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        taskMeshShaderMaxNumPrimitivesPerMeshlet);
                meshletTaskMeshPasses[i]->setMaxNumVerticesPerMeshlet(
                        taskMeshShaderMaxNumVerticesPerMeshlet);
                meshletTaskMeshPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            if (nodesBVHClearQueuePass) {
                nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                nodesBVHClearQueuePass->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                nodesBVHClearQueuePass->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
            if (convertMeshletCommandsBVHPass) {
                convertMeshletCommandsBVHPass->setUseMeshShaderNV(useMeshShaderNV);
                convertMeshletCommandsBVHPass->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                convertMeshletCommandsBVHPass->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                convertMeshletCommandsBVHPass->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
            if (meshletMeshBVHPasses[i]) {
                meshletMeshBVHPasses[i]->setUseMeshShaderNV(useMeshShaderNV);
                meshletMeshBVHPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        taskMeshShaderMaxNumPrimitivesPerMeshlet);
                meshletMeshBVHPasses[i]->setMaxNumVerticesPerMeshlet(
                        taskMeshShaderMaxNumVerticesPerMeshlet);
                meshletMeshBVHPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
            if (nodesBVHDrawCountPasses[i]) {
                nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                nodesBVHDrawCountPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                nodesBVHDrawCountPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
        }
    }
}

void DeferredRenderer::initializeOptimalNumWorkgroups() {
    if (optimalNumWorkgroups == 0 || optimalWorkgroupSize == 0) {
        DevicePersistentThreadInfo threadInfo = getDevicePersistentThreadInfo(renderer->getDevice());
        optimalNumWorkgroups = threadInfo.optimalNumWorkgroups;
        optimalWorkgroupSize = threadInfo.optimalWorkgroupSize;
        numWorkgroupsBvh = optimalNumWorkgroups;
        workgroupSizeBvh = optimalWorkgroupSize;
        maxNumWorkgroups = sgl::nextPowerOfTwo(2 * int(optimalNumWorkgroups));
        maxWorkgroupSize = renderer->getDevice()->getLimits().maxComputeWorkGroupSize[0];
    }
}

bool DeferredRenderer::getIsTriangleRepresentationUsed() const {
    return deferredRenderingMode != DeferredRenderingMode::DRAW_INDEXED
           || drawIndexedGeometryMode != DrawIndexedGeometryMode::PROGRAMMABLE_PULLING
           || (useAmbientOcclusion && ambientOcclusionBaker);
}

bool DeferredRenderer::getUsesTriangleMeshInternally() const {
    return deferredRenderingMode != DeferredRenderingMode::DRAW_INDEXED
           || drawIndexedGeometryMode != DrawIndexedGeometryMode::PROGRAMMABLE_PULLING;
}

void DeferredRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    bool isDataEmpty = true;
    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        visibilityBufferDrawIndexedPass->setLineData(lineData, isNewData);
        visibilityBufferDrawIndexedPass->buildIfNecessary();
        isDataEmpty = visibilityBufferDrawIndexedPass->getIsDataEmpty();
    } else if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        for (int i = 0; i < 2; i++) {
            visibilityBufferDrawIndexedIndirectPasses[i]->setLineData(lineData, isNewData);
        }
        if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
            for (int i = 0; i < 2; i++) {
                meshletDrawCountNoReductionPasses[i]->setLineData(lineData, isNewData);
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
            for (int i = 0; i < 2; i++) {
                meshletDrawCountAtomicPasses[i]->setLineData(lineData, isNewData);
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
            for (int i = 0; i < 2; i++) {
                meshletVisibilityPasses[i]->setLineData(lineData, isNewData);
            }
            meshletDrawCountPass->setLineData(lineData, isNewData);

            TubeTriangleRenderDataPayloadPtr payloadSuperClass(new MeshletsDrawIndirectPayload(
                    drawIndirectMaxNumPrimitivesPerMeshlet));
            TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
                    true, false, payloadSuperClass);

            if (tubeRenderData.indexBuffer) {
                auto* payload = static_cast<MeshletsDrawIndirectPayload*>(payloadSuperClass.get());
                visibilityBufferPrefixSumScanPass->setInputBuffer(payload->getMeshletVisibilityArrayBuffer());
                meshletDrawCountPass->setPrefixSumScanBuffer(visibilityBufferPrefixSumScanPass->getOutputBuffer());
            }
        }
        visibilityBufferDrawIndexedIndirectPasses[0]->buildIfNecessary();
        isDataEmpty = visibilityBufferDrawIndexedIndirectPasses[0]->getIsDataEmpty();
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            meshletTaskMeshPasses[i]->setLineData(lineData, isNewData);
        }
        meshletTaskMeshPasses[0]->buildIfNecessary();
        isDataEmpty = meshletTaskMeshPasses[0]->getNumMeshlets() == 0;
    } else if (getIsBvhRenderingMode()) {
        nodesBVHClearQueuePass->setLineData(lineData, isNewData);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
            convertMeshletCommandsBVHPass->setLineData(lineData, isNewData);
        }
        for (int i = 0; i < 2; i++) {
            nodesBVHDrawCountPasses[i]->setLineData(lineData, isNewData);
            if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setLineData(lineData, isNewData);
            } else {
                meshletMeshBVHPasses[i]->setLineData(lineData, isNewData);
            }
        }
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[0]->buildIfNecessary();
            isDataEmpty = visibilityBufferBVHDrawIndexedIndirectPasses[0]->getIsDataEmpty();
        } else {
            meshletMeshBVHPasses[0]->buildIfNecessary();
            isDataEmpty = meshletMeshBVHPasses[0]->getIsDataEmpty();
        }
    }

    reloadResolveShader();
    deferredResolvePass->setDataDirty();

    if (hullRasterPass) {
        framebufferMode = FramebufferMode::HULL_RASTER_PASS;
        hullRasterPass->setAttachmentLoadOp(
                isDataEmpty ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD);
        hullRasterPass->updateFramebuffer();
    }

    dirty = false;
    reRender = true;
    frameNumber = 0;
}

void DeferredRenderer::getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("DIRECT_BLIT_GATHER", ""));
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "GatherDummy.glsl"));
    //if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
    //    preprocessorDefines.insert(std::make_pair("WORKGROUP_SIZE", std::to_string(meshWorkgroupSize)));
    //}
}

void DeferredRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
}

void DeferredRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);

    bool isDeferredShadingTriangle = renderData->getShaderStages()->getHasModuleId(
            "DeferredShading.Fragment");
    bool isDeferredShadingProgrammablePull = renderData->getShaderStages()->getHasModuleId(
            "DeferredShading.Fragment.ProgrammablePull");
    bool isDeferredShading = isDeferredShadingTriangle || isDeferredShadingProgrammablePull;
    if (isDeferredShading) {
        lineData->setVulkanRenderDataDescriptors(renderData);
    }
    if (isDeferredShadingTriangle) {
        TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderData(
                true, false);
        if (tubeRenderData.indexBuffer) {
            renderData->setStaticBuffer(tubeRenderData.indexBuffer, "TriangleIndexBuffer");
            renderData->setStaticBuffer(tubeRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
            renderData->setStaticBuffer(tubeRenderData.linePointDataBuffer, "LinePointDataBuffer");
            if (tubeRenderData.multiVarAttributeDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.multiVarAttributeDataBuffer, "AttributeDataArrayBuffer");
            }
            if (tubeRenderData.stressLinePointDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.stressLinePointDataBuffer,
                        "StressLinePointDataBuffer");
            }
            if (tubeRenderData.stressLinePointPrincipalStressDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.stressLinePointPrincipalStressDataBuffer,
                        "StressLinePointPrincipalStressDataBuffer");
            }
        }
    }
    if (isDeferredShadingProgrammablePull) {
        LinePassTubeRenderDataProgrammablePull tubeRenderData = lineData->getLinePassTubeRenderDataProgrammablePull();
        if (tubeRenderData.indexBuffer) {
            renderData->setStaticBuffer(tubeRenderData.indexBuffer, "TriangleIndexBuffer");
            renderData->setStaticBuffer(tubeRenderData.linePointDataBuffer, "LinePointDataBuffer");
            if (tubeRenderData.multiVarAttributeDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.multiVarAttributeDataBuffer, "AttributeDataArrayBuffer");
            }
            if (tubeRenderData.stressLinePointDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.stressLinePointDataBuffer,
                        "StressLinePointDataBuffer");
            }
            if (tubeRenderData.stressLinePointPrincipalStressDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.stressLinePointPrincipalStressDataBuffer,
                        "StressLinePointPrincipalStressDataBuffer");
            }
        }
    }
    if (isDeferredShading) {
        renderData->setStaticTexture(primitiveIndexTexture, "primitiveIndexBuffer");
        renderData->setStaticTexture(depthMipLevelTextures.at(0), "depthBuffer");
    }
}

void DeferredRenderer::setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    if (framebufferMode == FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_PASS) {
        sgl::vk::AttachmentState primitiveIndexAttachmentState;
        primitiveIndexAttachmentState.loadOp = loadOp;
        primitiveIndexAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        framebuffer->setColorAttachmentUint(
                primitiveIndexImage, 0, primitiveIndexAttachmentState,
                glm::uvec4(std::numeric_limits<uint32_t>::max()));

        sgl::vk::AttachmentState depthAttachmentState;
        depthAttachmentState.loadOp = loadOp;
        depthAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        framebuffer->setDepthStencilAttachment(
                depthRenderTargetImage, depthAttachmentState, 1.0f);
    } else if (framebufferMode == FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_INDIRECT_PASS
            || framebufferMode == FramebufferMode::VISIBILITY_BUFFER_TASK_MESH_SHADER_PASS) {
        sgl::vk::AttachmentState primitiveIndexAttachmentState;
        primitiveIndexAttachmentState.loadOp = loadOp;
        primitiveIndexAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        framebuffer->setColorAttachmentUint(
                primitiveIndexImage, 0, primitiveIndexAttachmentState,
                glm::uvec4(std::numeric_limits<uint32_t>::max()));

        sgl::vk::AttachmentState depthAttachmentState;
        depthAttachmentState.loadOp = loadOp;
        depthAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        framebuffer->setDepthStencilAttachment(
                depthRenderTargetImagePingPong[(framebufferModeIndex + 1) % 2],
                depthAttachmentState, 1.0f);
    } else if (framebufferMode == FramebufferMode::DEFERRED_RESOLVE_PASS) {
        // Deferred pass.
        sgl::vk::AttachmentState colorAttachmentState;
        colorAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE; // TODO: VK_ATTACHMENT_LOAD_OP_CLEAR?
        colorAttachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        framebuffer->setColorAttachment(
                colorRenderTargetImage, 0, colorAttachmentState,
                sceneData->clearColor->getFloatColorRGBA());
    } else if (framebufferMode == FramebufferMode::HULL_RASTER_PASS) {
        // Hull pass.
        sgl::vk::AttachmentState colorAttachmentState;
        colorAttachmentState.loadOp = loadOp;
        colorAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        framebuffer->setColorAttachment(
                colorRenderTargetImage, 0, colorAttachmentState,
                sceneData->clearColor->getFloatColorRGBA());

        sgl::vk::AttachmentState depthAttachmentState;
        depthAttachmentState.loadOp = loadOp;
        depthAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        framebuffer->setDepthStencilAttachment(
                depthRenderTargetImage, depthAttachmentState, 1.0f);
    } else {
        sgl::Logfile::get()->throwError(
                "Error in DeferredRenderer::setFramebufferAttachments: Invalid framebuffer mode.");
    }
}

void DeferredRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    sgl::vk::Device* device = renderer->getDevice();
    auto scalingFactor = uint32_t(getResolutionIntegerScalingFactor());
    renderWidth = *sceneData->viewportWidth * scalingFactor;
    renderHeight = *sceneData->viewportHeight * scalingFactor;
    finalWidth = *sceneData->viewportWidth;
    finalHeight = *sceneData->viewportHeight;

    sgl::vk::ImageSamplerSettings samplerSettings;
    samplerSettings.minFilter = VK_FILTER_NEAREST;
    samplerSettings.magFilter = VK_FILTER_NEAREST;
    samplerSettings.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;

    sgl::vk::ImageSettings imageSettings = (*sceneData->sceneTexture)->getImage()->getImageSettings();
    imageSettings.width = renderWidth;
    imageSettings.height = renderHeight;
    imageSettings.format = VK_FORMAT_R32_UINT;
    imageSettings.usage =
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    primitiveIndexImage = std::make_shared<sgl::vk::ImageView>(
            std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);
    primitiveIndexTexture = std::make_shared<sgl::vk::Texture>(primitiveIndexImage, samplerSettings);

    // Create scene depth texture.
    depthMipLevelImageViews = {};
    depthRenderTargetImage = {};
    depthBufferTexture = {};
    depthMipLevelTextures = {};
    depthMipBlitRenderPasses = {};
    for (int i = 0; i < 2; i++) {
        depthMipLevelImageViewsPingPong[i] = {};
        depthRenderTargetImagePingPong[i] = {};
        depthBufferTexturePingPong[i] = {};
        depthMipLevelTexturesPingPong[i] = {};
        depthMipBlitRenderPassesPingPong[i] = {};
    }
    for (int i = 0; i < 2; i++) {
        if (i == 0) {
            imageSettings.usage =
                    VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                    | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        } else {
            imageSettings.usage =
                    VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                    | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        }
        imageSettings.format = device->getSupportedDepthFormat();
        // https://registry.khronos.org/OpenGL/extensions/ARB/ARB_texture_non_power_of_two.txt
        imageSettings.mipLevels = 1 + uint32_t(std::floor(std::log2(std::max(int(renderWidth), int(renderHeight)))));
        sgl::vk::ImageSamplerSettings depthSamplerSettings = samplerSettings;
        depthSamplerSettings.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        depthBufferTexturePingPong[i] = std::make_shared<sgl::vk::Texture>(
                device, imageSettings, depthSamplerSettings, VK_IMAGE_ASPECT_DEPTH_BIT);

        depthMipLevelImageViewsPingPong[i].clear();
        depthMipLevelImageViewsPingPong[i].resize(imageSettings.mipLevels);
        depthMipLevelTexturesPingPong[i].clear();
        depthMipLevelTexturesPingPong[i].resize(imageSettings.mipLevels);
        for (uint32_t level = 0; level < imageSettings.mipLevels; level++) {
            depthMipLevelImageViewsPingPong[i].at(level) = std::make_shared<sgl::vk::ImageView>(
                    depthBufferTexturePingPong[i]->getImage(), VK_IMAGE_VIEW_TYPE_2D,
                    level, 1, 0, 1,
                    VK_IMAGE_ASPECT_DEPTH_BIT);
            sgl::vk::ImageSamplerSettings samplerSettingsMipLevel = depthSamplerSettings;
            samplerSettingsMipLevel.minLod = float(level);
            samplerSettingsMipLevel.maxLod = float(level);
            depthMipLevelTexturesPingPong[i].at(level) = std::make_shared<sgl::vk::Texture>(
                    depthMipLevelImageViewsPingPong[i].at(level), samplerSettingsMipLevel);
        }
        depthRenderTargetImagePingPong[i] = depthMipLevelTexturesPingPong[i].at(0)->getImageView();
        depthMipBlitRenderPassesPingPong[i].clear();
        depthMipBlitRenderPassesPingPong[i].resize(imageSettings.mipLevels - 1);
        auto mipWidth = renderWidth;
        auto mipHeight = renderHeight;
        for (uint32_t level = 0; level < imageSettings.mipLevels - 1; level++) {
            if (mipWidth > 1) mipWidth /= 2;
            if (mipHeight > 1) mipHeight /= 2;
            sgl::vk::BlitRenderPassPtr depthMipBlitRenderPass = std::make_shared<sgl::vk::BlitRenderPass>(
                    renderer, std::vector<std::string>{"GenerateHZB.Vertex", "GenerateHZB.Fragment"});
            depthMipBlitRenderPass->setColorWriteEnabled(false);
            depthMipBlitRenderPass->setDepthWriteEnabled(true);
            depthMipBlitRenderPass->setDepthTestEnabled(true);
            depthMipBlitRenderPass->setDepthCompareOp(VK_COMPARE_OP_ALWAYS);
            depthMipBlitRenderPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
            depthMipBlitRenderPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_STORE);
            depthMipBlitRenderPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
            depthMipBlitRenderPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            depthMipBlitRenderPass->setInputTexture(depthMipLevelTexturesPingPong[i].at(level));
            depthMipBlitRenderPass->setOutputImage(depthMipLevelImageViewsPingPong[i].at(level + 1));
            depthMipBlitRenderPass->recreateSwapchain(mipWidth, mipHeight);
            depthMipBlitRenderPassesPingPong[i].at(level) = depthMipBlitRenderPass;
        }
    }
    depthMipLevelImageViews = depthMipLevelImageViewsPingPong[0];
    depthRenderTargetImage = depthRenderTargetImagePingPong[0];
    depthBufferTexture = depthBufferTexturePingPong[0];
    depthMipLevelTextures = depthMipLevelTexturesPingPong[0];
    depthMipBlitRenderPasses = depthMipBlitRenderPassesPingPong[0];

    onResolutionChangedDeferredRenderingMode();

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    frameHasNewStagingDataList.clear();
    frameHasNewStagingDataList.resize(swapchain->getNumImages(), false);
    for (int passIdx = 0; passIdx < 2; passIdx++) {
        visibleMeshletCounters[passIdx] = 0;
    }
    visibleMeshletsStagingBuffers.clear();
    visibleMeshletsStagingBuffers.reserve(swapchain->getNumImages());
    for (size_t i = 0; i < swapchain->getNumImages(); i++) {
        visibleMeshletsStagingBuffers.push_back(std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), 2 * sizeof(uint32_t),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU));
    }

    if (showMaxWorkLeftDebugInfo) {
        maxWorkLeftStagingBuffers.clear();
        maxWorkLeftStagingBuffers.reserve(swapchain->getNumImages());
        for (size_t i = 0; i < swapchain->getNumImages(); i++) {
            int32_t initData[2] = { 0, 0 };
            maxWorkLeftStagingBuffers.push_back(std::make_shared<sgl::vk::Buffer>(
                    renderer->getDevice(), 2 * sizeof(int32_t), initData,
                    VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU));
        }
    }

    framebufferMode = FramebufferMode::DEFERRED_RESOLVE_PASS;
    if (supersamplingMode == 0) {
        // No supersampling, thus we can directly draw the result to the scene data color image.
        colorRenderTargetImage = (*sceneData->sceneTexture)->getImageView();
        colorRenderTargetTexture = {};
    } else {
        // Use intermediate high-resolution texture, which is then downsampled in the next step.
        imageSettings.format = (*sceneData->sceneTexture)->getImage()->getImageSettings().format;
        imageSettings.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageSettings.mipLevels = 1;
        samplerSettings.minFilter = VK_FILTER_LINEAR;
        samplerSettings.magFilter = VK_FILTER_LINEAR;
        samplerSettings.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        colorRenderTargetImage = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);
        colorRenderTargetTexture = std::make_shared<sgl::vk::Texture>(
                colorRenderTargetImage, samplerSettings);

        downsampleBlitPass->setScalingFactor(int(scalingFactor));
        downsampleBlitPass->setInputTexture(colorRenderTargetTexture);
        downsampleBlitPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
        downsampleBlitPass->recreateSwapchain(finalWidth, finalHeight);
    }

    if (hullRasterPass) {
        framebufferMode = FramebufferMode::HULL_RASTER_PASS;
        hullRasterPass->recreateSwapchain(renderWidth, renderHeight);
    }

    deferredResolvePass->setOutputImage(colorRenderTargetImage);
    deferredResolvePass->recreateSwapchain(renderWidth, renderHeight);
    frameNumber = 0;
}

void DeferredRenderer::onResolutionChangedDeferredRenderingMode() {
    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        framebufferMode = FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_PASS;
        visibilityBufferDrawIndexedPass->recreateSwapchain(renderWidth, renderHeight);
    } else if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        for (int i = 0; i < 2; i++) {
            framebufferMode = FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_INDIRECT_PASS;
            framebufferModeIndex = i;
            visibilityBufferDrawIndexedIndirectPasses[i]->recreateSwapchain(renderWidth, renderHeight);
        }
        framebufferModeIndex = 0;
        if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
            for (int i = 0; i < 2; i++) {
                meshletDrawCountNoReductionPasses[i]->setDepthBufferTexture(depthBufferTexturePingPong[i]);
                meshletDrawCountNoReductionPasses[i]->recreateSwapchain(renderWidth, renderHeight);
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
            for (int i = 0; i < 2; i++) {
                meshletDrawCountAtomicPasses[i]->setDepthBufferTexture(depthBufferTexturePingPong[i]);
                meshletDrawCountAtomicPasses[i]->recreateSwapchain(renderWidth, renderHeight);
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
            for (int i = 0; i < 2; i++) {
                meshletVisibilityPasses[i]->setDepthBufferTexture(depthBufferTexturePingPong[i]);
                meshletVisibilityPasses[i]->recreateSwapchain(renderWidth, renderHeight);
            }
            visibilityBufferPrefixSumScanPass->recreateSwapchain(renderWidth, renderHeight);
            meshletDrawCountPass->recreateSwapchain(renderWidth, renderHeight);
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            framebufferMode = FramebufferMode::VISIBILITY_BUFFER_TASK_MESH_SHADER_PASS;
            framebufferModeIndex = i;
            meshletTaskMeshPasses[i]->setDepthBufferTexture(depthBufferTexturePingPong[i]);
            meshletTaskMeshPasses[i]->recreateSwapchain(renderWidth, renderHeight);
        }
        framebufferModeIndex = 0;
    } else if (getIsBvhRenderingMode()) {
        for (int i = 0; i < 2; i++) {
            framebufferMode = FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_INDIRECT_PASS;
            framebufferModeIndex = i;
            if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->recreateSwapchain(renderWidth, renderHeight);
            } else {
                meshletMeshBVHPasses[i]->recreateSwapchain(renderWidth, renderHeight);
            }
        }
        framebufferModeIndex = 0;
        nodesBVHClearQueuePass->setDepthBufferTexture(depthBufferTexturePingPong[0]);
        nodesBVHClearQueuePass->recreateSwapchain(renderWidth, renderHeight);
        for (int i = 0; i < 2; i++) {
            nodesBVHDrawCountPasses[i]->setDepthBufferTexture(depthBufferTexturePingPong[i]);
            nodesBVHDrawCountPasses[i]->recreateSwapchain(renderWidth, renderHeight);
        }
    }
}

void DeferredRenderer::onClearColorChanged() {
    deferredResolvePass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
    if (hullRasterPass && !hullRasterPass->getIsDataEmpty()) {
        framebufferMode = FramebufferMode::HULL_RASTER_PASS;
        hullRasterPass->updateFramebuffer();
    }
}

void DeferredRenderer::setUniformData() {
    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        return;
    }
    visibilityCullingUniformData.viewportSize = glm::ivec2(renderWidth, renderHeight);
    // Upload and updating is done before rendering.
    //visibilityCullingUniformData.modelViewProjectionMatrix =
    //        sceneData->camera->getProjectionMatrix() * sceneData->camera->getViewMatrix();
    //visibilityCullingUniformData.numMeshlets = 0;
    //visibilityCullingUniformData.rootNodeIdx = 0;
    //visibilityCullingUniformDataBuffer->updateData(
    //        sizeof(VisibilityCullingUniformData), &visibilityCullingUniformData,
    //        renderer->getVkCommandBuffer());
    //renderer->insertMemoryBarrier(
    //        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
    //        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void DeferredRenderer::render() {
    LineRenderer::renderBase();
    setUniformData();

    bool isDataEmpty = true;
    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        visibilityBufferDrawIndexedPass->buildIfNecessary();
        isDataEmpty = visibilityBufferDrawIndexedPass->getIsDataEmpty();
    } else if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        visibilityBufferDrawIndexedIndirectPasses[0]->buildIfNecessary();
        isDataEmpty = visibilityBufferDrawIndexedIndirectPasses[0]->getIsDataEmpty();
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        meshletTaskMeshPasses[0]->buildIfNecessary();
        isDataEmpty = meshletTaskMeshPasses[0]->getNumMeshlets() == 0;
    } else if (getIsBvhRenderingMode()) {
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[0]->buildIfNecessary();
            isDataEmpty = visibilityBufferBVHDrawIndexedIndirectPasses[0]->getIsDataEmpty();
        } else {
            meshletMeshBVHPasses[0]->buildIfNecessary();
            isDataEmpty = meshletMeshBVHPasses[0]->getIsDataEmpty();
        }
    }

    if (isDataEmpty) {
        renderDataEmpty();
    } else {
        if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
            renderDrawIndexed();
        } else {
            // If this is the first frame: Just clear the depth image and use the matrices of this frame.
            if (frameNumber == 0) {
                renderer->insertImageMemoryBarrier(
                        depthBufferTexturePingPong[0]->getImageView(),
                        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                        VK_ACCESS_NONE_KHR, VK_ACCESS_TRANSFER_WRITE_BIT);
                depthBufferTexturePingPong[0]->getImageView()->clearDepthStencil(
                        1.0f, 0, renderer->getVkCommandBuffer());
                lastFrameViewMatrix = sceneData->camera->getViewMatrix();
                lastFrameProjectionMatrix = sceneData->camera->getProjectionMatrix();
            }

            if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
                visibilityBufferDrawIndexedIndirectPasses[0]->buildIfNecessary();
                visibilityCullingUniformData.numMeshlets =
                        visibilityBufferDrawIndexedIndirectPasses[0]->getNumMeshlets();
            } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
                meshletTaskMeshPasses[0]->buildIfNecessary();
                visibilityCullingUniformData.numMeshlets =
                        meshletTaskMeshPasses[0]->getNumMeshlets();
            } else if (getIsBvhRenderingMode()) {
                if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
                    visibilityBufferBVHDrawIndexedIndirectPasses[0]->buildIfNecessary();
                    visibilityCullingUniformData.numMeshlets =
                            visibilityBufferBVHDrawIndexedIndirectPasses[0]->getNumMeshlets();
                } else {
                    meshletMeshBVHPasses[0]->buildIfNecessary();
                    visibilityCullingUniformData.numMeshlets = meshletMeshBVHPasses[0]->getNumMeshlets();
                }
            }

                VkPipelineStageFlags pipelineStageFlags = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
            if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
                pipelineStageFlags = VK_PIPELINE_STAGE_TASK_SHADER_BIT_NV;
#ifdef VK_EXT_mesh_shader
                if (!useMeshShaderNV) {
                    pipelineStageFlags = VK_PIPELINE_STAGE_TASK_SHADER_BIT_EXT;
                }
#endif
            }

            // Set old MVP matrix as uniform.
            visibilityCullingUniformData.modelViewProjectionMatrix = lastFrameProjectionMatrix * lastFrameViewMatrix;
            visibilityCullingUniformDataBuffer->updateData(
                    sizeof(VisibilityCullingUniformData), &visibilityCullingUniformData,
                    renderer->getVkCommandBuffer());
            renderer->insertMemoryBarrier(
                    VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
                    VK_PIPELINE_STAGE_TRANSFER_BIT, pipelineStageFlags);

            // Prepare depth buffer 0 for reading.
            renderer->transitionImageLayout(
                    depthBufferTexturePingPong[0]->getImageView(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

            // Draw the meshlets that would have been visible in the last frame using the HZB of last frame.
            renderDrawIndexedIndirectOrTaskMesh(0);
            renderer->transitionImageLayout(
                    depthRenderTargetImagePingPong[1], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

            // Rebuild the HZB.
            renderComputeHZB(1);

            // Copy MIP level 0 of D1 to D0.
            renderer->transitionImageLayout(
                    depthRenderTargetImagePingPong[1], VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            renderer->insertImageMemoryBarrier(
                    depthRenderTargetImagePingPong[0],
                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                    VK_ACCESS_NONE_KHR, VK_ACCESS_TRANSFER_WRITE_BIT);
            depthRenderTargetImagePingPong[1]->getImage()->copyToImage(
                    depthRenderTargetImagePingPong[0]->getImage(), VK_IMAGE_ASPECT_DEPTH_BIT,
                    renderer->getVkCommandBuffer());

            // Bring depth render targets into the correct format.
            renderer->insertImageMemoryBarrier(
                    depthRenderTargetImagePingPong[0],
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
                    VK_ACCESS_TRANSFER_WRITE_BIT,
                    VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
            renderer->insertImageMemoryBarrier(
                    depthRenderTargetImagePingPong[1],
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                    VK_PIPELINE_STAGE_TRANSFER_BIT, pipelineStageFlags,
                    VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_SHADER_READ_BIT);

            // Set new MVP matrix as uniform.
            visibilityCullingUniformData.modelViewProjectionMatrix =
                    sceneData->camera->getProjectionMatrix() * sceneData->camera->getViewMatrix();
            visibilityCullingUniformDataBuffer->updateData(
                    sizeof(VisibilityCullingUniformData), &visibilityCullingUniformData,
                    renderer->getVkCommandBuffer());
            renderer->insertMemoryBarrier(
                    VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
                    VK_PIPELINE_STAGE_TRANSFER_BIT, pipelineStageFlags);

            // Draw the meshlets that were marked as occluded in the last pass, but are visible using the new HZB.
            renderDrawIndexedIndirectOrTaskMesh(1);
            renderer->transitionImageLayout(
                    depthRenderTargetImagePingPong[0], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

            // Rebuild the HZB for next frame.
            renderComputeHZB(0);

            renderer->transitionImageLayout(
                    primitiveIndexImage->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            renderer->transitionImageLayout(
                    depthRenderTargetImage, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

            lastFrameViewMatrix = sceneData->camera->getViewMatrix();
            lastFrameProjectionMatrix = sceneData->camera->getProjectionMatrix();
        }
        deferredResolvePass->render();
    }

    // Render the hull mesh after the resolve pass has finished and use the resulting depth buffer for depth testing.
    //renderHull();
    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        // Can't use transitionImageLayout, as subresource transition by HZB build leaves wrong internal layout set.
        //renderer->transitionImageLayout(
        //        depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
        if (!isDataEmpty) {
            renderer->insertImageMemoryBarrier(
                    depthRenderTargetImage,//->getImage(),
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
                    VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT);
        }
        hullRasterPass->render();
    }

    if (supersamplingMode != 0) {
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        downsampleBlitPass->render();
    }

    frameNumber++;
}

void DeferredRenderer::renderDataEmpty() {
    // In case the data is empty, we can simply clear the color and depth render target.
    renderer->transitionImageLayout(
            colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    colorRenderTargetImage->clearColor(
            sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
    renderer->transitionImageLayout(
            colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    renderer->transitionImageLayout(
            depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    depthRenderTargetImage->clearDepthStencil(
            1.0f, 0, renderer->getVkCommandBuffer());
}

void DeferredRenderer::renderDrawIndexed() {
    visibilityBufferDrawIndexedPass->render();
    renderer->transitionImageLayout(
            primitiveIndexImage->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(
            depthRenderTargetImage, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    //for (uint32_t level = 0; level < uint32_t(depthMipBlitRenderPasses.size()); level++) {
    //    depthMipBlitRenderPasses.at(level)->render();
    //}
}

void DeferredRenderer::renderDrawIndexedIndirectOrTaskMesh(int passIndex) {
    sgl::vk::BufferPtr drawCountBuffer;
    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
            meshletDrawCountNoReductionPasses[passIndex]->render();
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
            meshletDrawCountAtomicPasses[passIndex]->render();
            if (showVisibleMeshletStatistics) {
                drawCountBuffer = meshletDrawCountAtomicPasses[passIndex]->getDrawCountBuffer();
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
            meshletVisibilityPasses[passIndex]->render();
            renderer->insertMemoryBarrier(
                    VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            visibilityBufferPrefixSumScanPass->render();
            renderer->insertMemoryBarrier(
                    VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            meshletDrawCountPass->render();
            if (showVisibleMeshletStatistics) {
                drawCountBuffer = meshletDrawCountPass->getDrawCountBuffer();
            }
        }
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_INDIRECT_COMMAND_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT);
        visibilityBufferDrawIndexedIndirectPasses[passIndex]->render();
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        meshletTaskMeshPasses[passIndex]->render();
    } else if (getIsBvhRenderingMode()) {
        if (passIndex == 0) {
            nodesBVHClearQueuePass->render();
            renderer->insertMemoryBarrier(
                    VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
        }
        nodesBVHDrawCountPasses[passIndex]->render();
        if (showVisibleMeshletStatistics) {
            drawCountBuffer = nodesBVHDrawCountPasses[passIndex]->getDrawCountBuffer();
        }
        if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
            renderer->insertMemoryBarrier(
                    VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            convertMeshletCommandsBVHPass->render();
        }
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_INDIRECT_COMMAND_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[passIndex]->render();
        } else {
            meshletMeshBVHPasses[passIndex]->render();
        }
    }

    if (showVisibleMeshletStatistics && drawCountBuffer) {
        auto* swapchain = sgl::AppSettings::get()->getSwapchain();
        drawCountBuffer->copyDataTo(
                visibleMeshletsStagingBuffers.at(swapchain->getImageIndex()),
                0, passIndex * sizeof(uint32_t), sizeof(uint32_t),
                renderer->getVkCommandBuffer());
        frameHasNewStagingDataList.at(swapchain->getImageIndex()) = true;
    }
    if (showMaxWorkLeftDebugInfo && getIsBvhRenderingMode()) {
        auto* swapchain = sgl::AppSettings::get()->getSwapchain();
        nodesBVHDrawCountPasses[passIndex]->getMaxWorkLeftTestBuffer()->copyDataTo(
                maxWorkLeftStagingBuffers.at(swapchain->getImageIndex()),
                0, passIndex * sizeof(int32_t), sizeof(int32_t),
                renderer->getVkCommandBuffer());
        frameHasNewStagingDataList.at(swapchain->getImageIndex()) = true;
    }
}

void DeferredRenderer::renderComputeHZB(int passIndex) {
    auto mipWidth = renderWidth;
    auto mipHeight = renderHeight;
    for (uint32_t level = 0; level < uint32_t(depthMipBlitRenderPasses.size()); level++) {
        depthMipBlitRenderPassesPingPong[passIndex].at(level)->buildIfNecessary();
        renderer->pushConstants(
                depthMipBlitRenderPassesPingPong[passIndex].at(level)->getGraphicsPipeline(),
                VK_SHADER_STAGE_FRAGMENT_BIT, 0, glm::ivec2(mipWidth, mipHeight));
        renderer->pushConstants(
                depthMipBlitRenderPassesPingPong[passIndex].at(level)->getGraphicsPipeline(),
                VK_SHADER_STAGE_FRAGMENT_BIT, sizeof(glm::ivec2), level);
        depthMipBlitRenderPassesPingPong[passIndex].at(level)->render();
        if (mipWidth > 1) mipWidth /= 2;
        if (mipHeight > 1) mipHeight /= 2;
    }
}

void DeferredRenderer::updateWritePackedPrimitives() {
    if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            if (meshletTaskMeshPasses[i]) {
                meshletTaskMeshPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        for (int i = 0; i < 2; i++) {
            if (nodesBVHClearQueuePass) {
                nodesBVHClearQueuePass->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
            if (convertMeshletCommandsBVHPass) {
                convertMeshletCommandsBVHPass->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
            if (meshletMeshBVHPasses[i]) {
                meshletMeshBVHPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
            if (nodesBVHDrawCountPasses[i]) {
                nodesBVHDrawCountPasses[i]->setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(
                        useMeshShaderWritePackedPrimitiveIndicesIfAvailable);
            }
        }
    }
    reRender = true;
}

void DeferredRenderer::updateBvhBuildAlgorithm() {
    if (!getIsBvhRenderingMode()) {
        return;
    }
    nodesBVHClearQueuePass->setBvhBuildAlgorithm(bvhBuildAlgorithm);
    for (int i = 0; i < 2; i++) {
        nodesBVHDrawCountPasses[i]->setBvhBuildAlgorithm(bvhBuildAlgorithm);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[i]->setBvhBuildAlgorithm(bvhBuildAlgorithm);
        } else {
            meshletMeshBVHPasses[i]->setBvhBuildAlgorithm(bvhBuildAlgorithm);
        }
    }
    if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        convertMeshletCommandsBVHPass->setBvhBuildAlgorithm(bvhBuildAlgorithm);
    }
    deferredResolvePass->setDataDirty();
    reRender = true;
}

void DeferredRenderer::updateBvhBuildGeometryMode() {
    if (!getIsBvhRenderingMode()) {
        return;
    }
    nodesBVHClearQueuePass->setBvhBuildGeometryMode(bvhBuildGeometryMode);
    for (int i = 0; i < 2; i++) {
        nodesBVHDrawCountPasses[i]->setBvhBuildGeometryMode(bvhBuildGeometryMode);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[i]->setBvhBuildGeometryMode(bvhBuildGeometryMode);
        } else {
            meshletMeshBVHPasses[i]->setBvhBuildGeometryMode(bvhBuildGeometryMode);
        }
    }
    if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        convertMeshletCommandsBVHPass->setBvhBuildGeometryMode(bvhBuildGeometryMode);
    }
    deferredResolvePass->setDataDirty();
    reRender = true;
}

void DeferredRenderer::updateBvhBuildPrimitiveCenterMode() {
    if (!getIsBvhRenderingMode()) {
        return;
    }
    nodesBVHClearQueuePass->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
    for (int i = 0; i < 2; i++) {
        nodesBVHDrawCountPasses[i]->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[i]->setBvhBuildPrimitiveCenterMode(
                    bvhBuildPrimitiveCenterMode);
        } else {
            meshletMeshBVHPasses[i]->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
        }
    }
    if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        convertMeshletCommandsBVHPass->setBvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode);
    }
    deferredResolvePass->setDataDirty();
    reRender = true;
}

void DeferredRenderer::updateUseStdBvhParameters() {
    if (!getIsBvhRenderingMode()) {
        return;
    }
    nodesBVHClearQueuePass->setUseStdBvhParameters(useStdBvhParameters);
    for (int i = 0; i < 2; i++) {
        nodesBVHDrawCountPasses[i]->setUseStdBvhParameters(useStdBvhParameters);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[i]->setUseStdBvhParameters(useStdBvhParameters);
        } else {
            meshletMeshBVHPasses[i]->setUseStdBvhParameters(useStdBvhParameters);
        }
    }
    if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        convertMeshletCommandsBVHPass->setUseStdBvhParameters(useStdBvhParameters);
    }
    deferredResolvePass->setDataDirty();
    reRender = true;
}

void DeferredRenderer::updateMaxLeafSizeBvh() {
    if (!getIsBvhRenderingMode()) {
        return;
    }
    nodesBVHClearQueuePass->setMaxLeafSizeBvh(maxLeafSizeBvh);
    for (int i = 0; i < 2; i++) {
        nodesBVHDrawCountPasses[i]->setMaxLeafSizeBvh(maxLeafSizeBvh);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxLeafSizeBvh(maxLeafSizeBvh);
        } else {
            meshletMeshBVHPasses[i]->setMaxLeafSizeBvh(maxLeafSizeBvh);
        }
    }
    if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        convertMeshletCommandsBVHPass->setMaxLeafSizeBvh(maxLeafSizeBvh);
    }
    deferredResolvePass->setDataDirty();
    reRender = true;
}

void DeferredRenderer::updateMaxTreeDepthBvh() {
    if (!getIsBvhRenderingMode()) {
        return;
    }
    nodesBVHClearQueuePass->setMaxTreeDepthBvh(maxTreeDepthBvh);
    for (int i = 0; i < 2; i++) {
        nodesBVHDrawCountPasses[i]->setMaxTreeDepthBvh(maxTreeDepthBvh);
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxTreeDepthBvh(maxTreeDepthBvh);
        } else {
            meshletMeshBVHPasses[i]->setMaxTreeDepthBvh(maxTreeDepthBvh);
        }
    }
    if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
        convertMeshletCommandsBVHPass->setMaxTreeDepthBvh(maxTreeDepthBvh);
    }
    deferredResolvePass->setDataDirty();
    reRender = true;
}

void DeferredRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    int numRenderingModes = IM_ARRAYSIZE(deferredRenderingModeNames);
    if (!supportsDrawIndirect) {
        numRenderingModes -= 3;
    } if (!supportsDrawIndirectCount) {
        numRenderingModes -= 2;
    } else if (!supportsTaskMeshShaders) {
        numRenderingModes -= 1;
    }
    if (propertyEditor.addCombo(
            "Deferred Mode", (int*)&deferredRenderingMode,
            deferredRenderingModeNames, numRenderingModes)) {
        updateRenderingMode();
        reloadGatherShader();
        onResolutionChanged();
        reRender = true;
    }

    if (propertyEditor.addCombo(
            "Supersampling", &supersamplingMode,
            supersamplingModeNames, IM_ARRAYSIZE(supersamplingModeNames))) {
        onResolutionChanged();
        reRender = true;
    }

    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED) {
        if (propertyEditor.addCombo(
                "Geometry Mode", (int*)&drawIndexedGeometryMode,
                drawIndexedGeometryModeNames, IM_ARRAYSIZE(drawIndexedGeometryModeNames))) {
            renderer->getDevice()->waitIdle();
            updateGeometryMode();
            reloadGatherShader();
            reRender = true;
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        int numReductionModes = IM_ARRAYSIZE(drawIndirectReductionModeNames);
        if (!supportsDrawIndirectCount) {
            numReductionModes -= 2;
        }
        if (propertyEditor.addCombo(
                "Reduction Mode", (int*)&drawIndirectReductionMode,
                drawIndirectReductionModeNames, numReductionModes)) {
            renderer->getDevice()->waitIdle();
            updateDrawIndirectReductionMode();
            reloadGatherShader();
            reRender = true;
        }
        if (propertyEditor.addSliderIntEdit(
                "#Tri/Meshlet", (int*)&drawIndirectMaxNumPrimitivesPerMeshlet,
                32, 1024) == ImGui::EditMode::INPUT_FINISHED) {
            if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
                for (int i = 0; i < 2; i++) {
                    meshletDrawCountNoReductionPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
                for (int i = 0; i < 2; i++) {
                    meshletDrawCountAtomicPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
                for (int i = 0; i < 2; i++) {
                    meshletVisibilityPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
                }
                meshletDrawCountPass->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
            }
            reloadGatherShader();
            reRender = true;
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        if (propertyEditor.addSliderIntEdit(
                "#Tri/Meshlet", (int*)&taskMeshShaderMaxNumPrimitivesPerMeshlet,
                32, int(taskMeshShaderMaxNumPrimitivesSupportedNV)) == ImGui::EditMode::INPUT_FINISHED) {
            for (int i = 0; i < 2; i++) {
                meshletTaskMeshPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
            }
            reloadGatherShader();
            reRender = true;
        }
        if (propertyEditor.addSliderIntEdit(
                "#Verts/Meshlet", (int*)&taskMeshShaderMaxNumVerticesPerMeshlet,
                16, int(taskMeshShaderMaxNumVerticesSupported)) == ImGui::EditMode::INPUT_FINISHED) {
            for (int i = 0; i < 2; i++) {
                meshletTaskMeshPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
            }
            reloadGatherShader();
            reRender = true;
        }
        if (supportsTaskMeshShadersNV && supportsTaskMeshShadersEXT) {
            if (propertyEditor.addCheckbox("Use NVIDIA Mesh Shaders", &useMeshShaderNV)) {
                updateTaskMeshShaderMode();
                reRender = true;
            }
        }
        if (useMeshShaderNV) {
            if (propertyEditor.addCheckbox(
                    "Write Packed Primitives", &useMeshShaderWritePackedPrimitiveIndicesIfAvailable)) {
                updateWritePackedPrimitives();
            }
        }
    } else if (getIsBvhRenderingMode()) {
        if (propertyEditor.addCombo(
                "BVH Build Algorithm", (int*)&bvhBuildAlgorithm,
                bvhBuildAlgorithmNames, IM_ARRAYSIZE(bvhBuildAlgorithmNames))) {
            updateBvhBuildAlgorithm();
        }
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT && propertyEditor.addCombo(
                "BVH Geometry Mode", (int*)&bvhBuildGeometryMode,
                bvhBuildGeometryModeNames, IM_ARRAYSIZE(bvhBuildGeometryModeNames))) {
            updateBvhBuildGeometryMode();
        }
        if (propertyEditor.addCombo(
                "BVH Primitive Centers", (int*)&bvhBuildPrimitiveCenterMode,
                bvhBuildPrimitiveCenterModeNames, IM_ARRAYSIZE(bvhBuildPrimitiveCenterModeNames))) {
            updateBvhBuildPrimitiveCenterMode();
        }
        bool meshletSizeConfigurable = deferredRenderingMode ==
                DeferredRenderingMode::BVH_MESH_SHADER
                || bvhBuildGeometryMode == BvhBuildGeometryMode::MESHLETS
                || (bvhBuildGeometryMode == BvhBuildGeometryMode::TRIANGLES && (
                        bvhBuildAlgorithm == BvhBuildAlgorithm::BINNED_SAH_CPU
                        || bvhBuildAlgorithm == BvhBuildAlgorithm::SWEEP_SAH_CPU));
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT && meshletSizeConfigurable) {
            if (propertyEditor.addSliderIntEdit(
                    "#Tri/Meshlet", (int*)&drawIndirectMaxNumPrimitivesPerMeshlet,
                    32, 1024) == ImGui::EditMode::INPUT_FINISHED) {
                nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
                for (int i = 0; i < 2; i++) {
                    nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
                    if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
                        visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxNumPrimitivesPerMeshlet(
                                drawIndirectMaxNumPrimitivesPerMeshlet);
                    } else {
                        meshletMeshBVHPasses[i]->setMaxNumPrimitivesPerMeshlet(
                                drawIndirectMaxNumPrimitivesPerMeshlet);
                    }
                }
                reloadGatherShader();
                reRender = true;
            }
        } else if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER && meshletSizeConfigurable) {
            if (propertyEditor.addSliderIntEdit(
                    "#Tri/Meshlet", (int*)&taskMeshShaderMaxNumPrimitivesPerMeshlet,
                    32, int(taskMeshShaderMaxNumPrimitivesSupportedNV)) == ImGui::EditMode::INPUT_FINISHED) {
                nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                for (int i = 0; i < 2; i++) {
                    nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                    meshletMeshBVHPasses[i]->setMaxNumPrimitivesPerMeshlet(
                            taskMeshShaderMaxNumPrimitivesPerMeshlet);
                }
                reloadGatherShader();
                reRender = true;
            }
            if (propertyEditor.addSliderIntEdit(
                    "#Verts/Meshlet", (int*)&taskMeshShaderMaxNumVerticesPerMeshlet,
                    16, int(taskMeshShaderMaxNumVerticesSupported)) == ImGui::EditMode::INPUT_FINISHED) {
                nodesBVHClearQueuePass->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                for (int i = 0; i < 2; i++) {
                    nodesBVHDrawCountPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                    meshletMeshBVHPasses[i]->setMaxNumVerticesPerMeshlet(
                            taskMeshShaderMaxNumVerticesPerMeshlet);
                }
                reloadGatherShader();
                reRender = true;
            }
            if (supportsTaskMeshShadersNV && supportsTaskMeshShadersEXT) {
                if (propertyEditor.addCheckbox("Use NVIDIA Mesh Shaders", &useMeshShaderNV)) {
                    updateTaskMeshShaderMode();
                    reRender = true;
                }
            }
            if (useMeshShaderNV) {
                if (propertyEditor.addCheckbox(
                        "Write Packed Primitives", &useMeshShaderWritePackedPrimitiveIndicesIfAvailable)) {
                    updateWritePackedPrimitives();
                }
            }
        }

        if (propertyEditor.beginNode("Advanced Settings##bvh")) {
            bool bvhOptionsAvailable =
                    bvhBuildAlgorithm == BvhBuildAlgorithm::SWEEP_SAH_CPU
                    || bvhBuildAlgorithm == BvhBuildAlgorithm::BINNED_SAH_CPU;
            if (bvhOptionsAvailable) {
                if (propertyEditor.addCheckbox("Standard BVH Parameters", &useStdBvhParameters)) {
                    updateUseStdBvhParameters();
                }
                if (!useStdBvhParameters) {
                    if (propertyEditor.addSliderIntEdit(
                            "Max. Leaf Size", (int*)&maxLeafSizeBvh,
                            1, int(maxNumWorkgroups)) == ImGui::EditMode::INPUT_FINISHED) {
                        updateMaxLeafSizeBvh();
                    }
                    if (propertyEditor.addSliderIntEdit(
                            "Max. Tree Depth", (int*)&maxTreeDepthBvh,
                            1, int(maxNumWorkgroups)) == ImGui::EditMode::INPUT_FINISHED) {
                        updateMaxTreeDepthBvh();
                    }
                }
            }

            if (propertyEditor.addSliderIntEdit(
                    "#Workgroups", (int*)&numWorkgroupsBvh,
                    1, int(maxNumWorkgroups)) == ImGui::EditMode::INPUT_FINISHED) {
                numWorkgroupsBvh = std::clamp(numWorkgroupsBvh, 1u, maxNumWorkgroups);
                for (int i = 0; i < 2; i++) {
                    nodesBVHDrawCountPasses[i]->setNumWorkgroups(numWorkgroupsBvh);
                }
                reRender = true;
            }
            if (propertyEditor.addSliderIntEdit(
                    "Workgroup Size", (int*)&workgroupSizeBvh,
                    1, int(maxWorkgroupSize)) == ImGui::EditMode::INPUT_FINISHED) {
                workgroupSizeBvh = std::clamp(workgroupSizeBvh, 1u, maxWorkgroupSize);
                for (int i = 0; i < 2; i++) {
                    nodesBVHDrawCountPasses[i]->setWorkgroupSize(workgroupSizeBvh);
                }
                reRender = true;
            }
            if (propertyEditor.addCheckbox("Use Subgroup Ops", &useSubgroupOps)) {
                for (int i = 0; i < 2; i++) {
                    nodesBVHDrawCountPasses[i]->setUseSubgroupOps(useSubgroupOps);
                }
                reRender = true;
            }
            propertyEditor.endNode();
        }
    }

    if ((deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT
                && drawIndirectReductionMode != DrawIndirectReductionMode::NO_REDUCTION)
            || getIsBvhRenderingMode()) {
        bool showStatisticsChanged = false;
        if (propertyEditor.addCheckbox("Show Statistics", &showVisibleMeshletStatistics)) {
            reRender = true;
            showStatisticsChanged = true;
            auto* swapchain = sgl::AppSettings::get()->getSwapchain();
            frameHasNewStagingDataList.clear();
            frameHasNewStagingDataList.resize(swapchain->getNumImages(), false);
            for (int passIdx = 0; passIdx < 2; passIdx++) {
                visibleMeshletCounters[passIdx] = 0;
            }
        }
        if (showVisibleMeshletStatistics && !showStatisticsChanged) {
            auto* swapchain = sgl::AppSettings::get()->getSwapchain();
            if (frameHasNewStagingDataList.at(swapchain->getImageIndex())) {
                auto stagingBuffer = visibleMeshletsStagingBuffers.at(swapchain->getImageIndex());
                auto* visibleMeshletsCountBuffer = reinterpret_cast<uint32_t*>(stagingBuffer->mapMemory());
                visibleMeshletCounters[0] = visibleMeshletsCountBuffer[0];
                visibleMeshletCounters[1] = visibleMeshletsCountBuffer[1];
                stagingBuffer->unmapMemory();
                frameHasNewStagingDataList.at(swapchain->getImageIndex()) = false;
            }

            float numVisiblePass1Pct =
                    100.0f * float(visibleMeshletCounters[0]) / float(visibilityCullingUniformData.numMeshlets);
            float numVisiblePass2Pct =
                    100.0f * float(visibleMeshletCounters[1]) / float(visibilityCullingUniformData.numMeshlets);
            float numVisiblePct =
                    100.0f * float(visibleMeshletCounters[0] + visibleMeshletCounters[1])
                    / float(visibilityCullingUniformData.numMeshlets);
            if (visibilityCullingUniformData.numMeshlets == 0) {
                numVisiblePass1Pct = 0.0f;
                numVisiblePass2Pct = 0.0f;
                numVisiblePct = 0.0f;
            }
            std::string str1 =
                    std::to_string(visibleMeshletCounters[0]) + " of "
                    + std::to_string(visibilityCullingUniformData.numMeshlets)
                    + " (" + sgl::toString(numVisiblePass1Pct, 1) + "%)";
            std::string str2 =
                    std::to_string(visibleMeshletCounters[1]) + " of "
                    + std::to_string(visibilityCullingUniformData.numMeshlets)
                    + " (" + sgl::toString(numVisiblePass2Pct, 1) + "%)";
            std::string str =
                    std::to_string(visibleMeshletCounters[0] + visibleMeshletCounters[1]) + " of "
                    + std::to_string(visibilityCullingUniformData.numMeshlets)
                    + " (" + sgl::toString(numVisiblePct, 1) + "%)";
            propertyEditor.addText("#Meshlets Visible (1): ", str1);
            propertyEditor.addText("#Meshlets Visible (2): ", str2);
            propertyEditor.addText("#Meshlets Visible Total: ", str);
        }
        if (getIsBvhRenderingMode() && showMaxWorkLeftDebugInfo) {
            auto* swapchain = sgl::AppSettings::get()->getSwapchain();
            auto stagingBuffer = maxWorkLeftStagingBuffers.at(swapchain->getImageIndex());
            auto* maxWorkLeftBuffer = reinterpret_cast<int32_t*>(stagingBuffer->mapMemory());
            maxWorkLeft0 = std::max(maxWorkLeft0, maxWorkLeftBuffer[0]);
            maxWorkLeft1 = std::max(maxWorkLeft1, maxWorkLeftBuffer[1]);
            stagingBuffer->unmapMemory();
            propertyEditor.addText("Max work left (1): ", std::to_string(maxWorkLeft0));
            propertyEditor.addText("Max work left (2): ", std::to_string(maxWorkLeft1));
        }
    }
}

void DeferredRenderer::setNewState(const InternalState& newState) {
    std::string deferredRenderingModeString;
    if (newState.rendererSettings.getValueOpt("deferredRenderingMode", deferredRenderingModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(deferredRenderingModeNames); i++) {
            if (deferredRenderingModeString == deferredRenderingModeNames[i]) {
                deferredRenderingMode = DeferredRenderingMode(i);
                updateGeometryMode();
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    int supersamplingFactor = 1;
    if (newState.rendererSettings.getValueOpt("supersampling", supersamplingFactor)) {
        for (int i = 0; i < IM_ARRAYSIZE(supersamplingModeNames); i++) {
            if (std::to_string(supersamplingFactor) + "x" == supersamplingModeNames[i]) {
                supersamplingMode = i;
                onResolutionChanged();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndexedGeometryModeString;
    if (newState.rendererSettings.getValueOpt("drawIndexedGeometryMode", drawIndexedGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndexedGeometryModeNames); i++) {
            if (drawIndexedGeometryModeString == drawIndexedGeometryModeNames[i]) {
                drawIndexedGeometryMode = DrawIndexedGeometryMode(i);
                updateGeometryMode();
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndirectReductionModeString;
    if (newState.rendererSettings.getValueOpt("drawIndirectReductionMode", drawIndirectReductionModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndirectReductionModeNames); i++) {
            if (drawIndirectReductionModeString == drawIndirectReductionModeNames[i]) {
                drawIndirectReductionMode = DrawIndirectReductionMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    if (newState.rendererSettings.getValueOpt(
            "drawIndirectMaxNumPrimitivesPerMeshlet", drawIndirectMaxNumPrimitivesPerMeshlet)) {
        if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
            for (int i = 0; i < 2; i++) {
                if (meshletDrawCountNoReductionPasses[i]) {
                    meshletDrawCountNoReductionPasses[i]->setMaxNumPrimitivesPerMeshlet(
                            drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
            for (int i = 0; i < 2; i++) {
                if (meshletDrawCountAtomicPasses[i]) {
                    meshletDrawCountAtomicPasses[i]->setMaxNumPrimitivesPerMeshlet(
                            drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
            for (int i = 0; i < 2; i++) {
                if (meshletVisibilityPasses[i]) {
                    meshletVisibilityPasses[i]->setMaxNumPrimitivesPerMeshlet(
                            drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            }
            if (meshletDrawCountPass) {
                meshletDrawCountPass->setMaxNumPrimitivesPerMeshlet(
                        drawIndirectMaxNumPrimitivesPerMeshlet);
            }
        }
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        drawIndirectMaxNumPrimitivesPerMeshlet);
            }
        }
        reloadGatherShader();
        reRender = true;
    }

    if (newState.rendererSettings.getValueOpt(
            "taskMeshShaderMaxNumPrimitivesPerMeshlet", taskMeshShaderMaxNumPrimitivesPerMeshlet)) {
        if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
            for (int i = 0; i < 2; i++) {
                meshletTaskMeshPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        taskMeshShaderMaxNumPrimitivesPerMeshlet);
            }
        } else if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
            nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                meshletMeshBVHPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        taskMeshShaderMaxNumPrimitivesPerMeshlet);
            }
        }
        reloadGatherShader();
        reRender = true;
    }
    if (newState.rendererSettings.getValueOpt(
            "taskMeshShaderMaxNumVerticesPerMeshlet", taskMeshShaderMaxNumVerticesPerMeshlet)) {
        for (int i = 0; i < 2; i++) {
            if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
                for (int i = 0; i < 2; i++) {
                    meshletTaskMeshPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                }
            } else if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
                nodesBVHClearQueuePass->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                for (int i = 0; i < 2; i++) {
                    nodesBVHDrawCountPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                    meshletMeshBVHPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                }
            }
        }
        reloadGatherShader();
        reRender = true;
    }

    if (newState.rendererSettings.getValueOpt("useMeshShaderNV", useMeshShaderNV)) {
        updateTaskMeshShaderMode();
        reRender = true;
    }

    if (newState.rendererSettings.getValueOpt(
            "useMeshShaderWritePackedPrimitiveIndicesIfAvailable",
            useMeshShaderWritePackedPrimitiveIndicesIfAvailable)) {
        updateWritePackedPrimitives();
    }

    std::string bvhBuildAlgorithmString;
    if (newState.rendererSettings.getValueOpt("bvhBuildAlgorithm", bvhBuildAlgorithmString)) {
        for (int i = 0; i < IM_ARRAYSIZE(bvhBuildAlgorithmNames); i++) {
            if (bvhBuildAlgorithmString == bvhBuildAlgorithmNames[i]) {
                bvhBuildAlgorithm = BvhBuildAlgorithm(i);
                updateBvhBuildAlgorithm();
                break;
            }
        }
    }

    std::string bvhBuildGeometryModeString;
    if (newState.rendererSettings.getValueOpt("bvhBuildGeometryMode", bvhBuildGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(bvhBuildGeometryModeNames); i++) {
            if (bvhBuildGeometryModeString == bvhBuildGeometryModeNames[i]) {
                bvhBuildGeometryMode = BvhBuildGeometryMode(i);
                updateBvhBuildGeometryMode();
                break;
            }
        }
    }

    std::string bvhBuildPrimitiveCenterModeString;
    if (newState.rendererSettings.getValueOpt("bvhBuildPrimitiveCenterMode", bvhBuildPrimitiveCenterModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(bvhBuildPrimitiveCenterModeNames); i++) {
            if (bvhBuildPrimitiveCenterModeString == bvhBuildPrimitiveCenterModeNames[i]) {
                bvhBuildPrimitiveCenterMode = BvhBuildPrimitiveCenterMode(i);
                updateBvhBuildPrimitiveCenterMode();
                break;
            }
        }
    }

    if (newState.rendererSettings.getValueOpt("useStdBvhParameters", useStdBvhParameters)) {
        updateUseStdBvhParameters();
    }
    if (newState.rendererSettings.getValueOpt("maxTreeDepthBvh", maxLeafSizeBvh)) {
        updateMaxLeafSizeBvh();
    }
    if (newState.rendererSettings.getValueOpt("maxTreeDepthBvh", maxTreeDepthBvh)) {
        updateMaxTreeDepthBvh();
    }

    if (newState.rendererSettings.getValueOpt("numWorkgroupsBvh", numWorkgroupsBvh)) {
        numWorkgroupsBvh = std::clamp(numWorkgroupsBvh, 1u, maxNumWorkgroups);
        if (getIsBvhRenderingMode()) {
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setNumWorkgroups(numWorkgroupsBvh);
            }
        }
        reRender = true;
    }
    if (newState.rendererSettings.getValueOpt("workgroupSizeBvh", workgroupSizeBvh)) {
        workgroupSizeBvh = std::clamp(workgroupSizeBvh, 1u, maxWorkgroupSize);
        if (getIsBvhRenderingMode()) {
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setWorkgroupSize(workgroupSizeBvh);
            }
        }
        reRender = true;
    }
    if (newState.rendererSettings.getValueOpt("useSubgroupOps", useSubgroupOps)) {
        if (getIsBvhRenderingMode()) {
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setUseSubgroupOps(useSubgroupOps);
            }
        }
        reRender = true;
    }
}

bool DeferredRenderer::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = LineRenderer::setNewSettings(settings);

    std::string deferredRenderingModeString;
    if (settings.getValueOpt("deferred_rendering_mode", deferredRenderingModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(deferredRenderingModeNames); i++) {
            if (deferredRenderingModeString == deferredRenderingModeNames[i]) {
                deferredRenderingMode = DeferredRenderingMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    int supersamplingFactor = 1;
    if (settings.getValueOpt("supersampling", supersamplingFactor)) {
        for (int i = 0; i < IM_ARRAYSIZE(supersamplingModeNames); i++) {
            if (std::to_string(supersamplingFactor) + "x" == supersamplingModeNames[i]) {
                supersamplingMode = i;
                onResolutionChanged();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndexedGeometryModeString;
    if (settings.getValueOpt("draw_indexed_geometry_mode", drawIndexedGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndexedGeometryModeNames); i++) {
            if (drawIndexedGeometryModeString == drawIndexedGeometryModeNames[i]) {
                drawIndexedGeometryMode = DrawIndexedGeometryMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndirectReductionModeString;
    if (settings.getValueOpt("draw_indirect_reduction_mode", drawIndirectReductionModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndirectReductionModeNames); i++) {
            if (drawIndirectReductionModeString == drawIndirectReductionModeNames[i]) {
                drawIndirectReductionMode = DrawIndirectReductionMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    if (settings.getValueOpt(
            "draw_indirect_max_num_primitives_per_meshlet", drawIndirectMaxNumPrimitivesPerMeshlet)) {
        if (drawIndirectReductionMode == DrawIndirectReductionMode::NO_REDUCTION) {
            for (int i = 0; i < 2; i++) {
                if (meshletDrawCountNoReductionPasses[i]) {
                    meshletDrawCountNoReductionPasses[i]->setMaxNumPrimitivesPerMeshlet(
                            drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::ATOMIC_COUNTER) {
            for (int i = 0; i < 2; i++) {
                if (meshletDrawCountAtomicPasses[i]) {
                    meshletDrawCountAtomicPasses[i]->setMaxNumPrimitivesPerMeshlet(
                            drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            }
        } else if (drawIndirectReductionMode == DrawIndirectReductionMode::PREFIX_SUM_SCAN) {
            for (int i = 0; i < 2; i++) {
                if (meshletVisibilityPasses[i]) {
                    meshletVisibilityPasses[i]->setMaxNumPrimitivesPerMeshlet(
                            drawIndirectMaxNumPrimitivesPerMeshlet);
                }
            }
            if (meshletDrawCountPass) {
                meshletDrawCountPass->setMaxNumPrimitivesPerMeshlet(
                        drawIndirectMaxNumPrimitivesPerMeshlet);
            }
        }
        if (deferredRenderingMode == DeferredRenderingMode::BVH_DRAW_INDIRECT) {
            nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(drawIndirectMaxNumPrimitivesPerMeshlet);
                visibilityBufferBVHDrawIndexedIndirectPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        drawIndirectMaxNumPrimitivesPerMeshlet);
            }
        }
        reloadGatherShader();
        reRender = true;
    }

    if (settings.getValueOpt(
            "task_mesh_shader_max_num_primitives_per_meshlet", taskMeshShaderMaxNumPrimitivesPerMeshlet)) {
        if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
            for (int i = 0; i < 2; i++) {
                meshletTaskMeshPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        taskMeshShaderMaxNumPrimitivesPerMeshlet);
            }
        } else if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
            nodesBVHClearQueuePass->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setMaxNumPrimitivesPerMeshlet(taskMeshShaderMaxNumPrimitivesPerMeshlet);
                meshletMeshBVHPasses[i]->setMaxNumPrimitivesPerMeshlet(
                        taskMeshShaderMaxNumPrimitivesPerMeshlet);
            }
        }
        reloadGatherShader();
        reRender = true;
    }
    if (settings.getValueOpt(
            "task_mesh_shader_max_num_vertices_per_meshlet", taskMeshShaderMaxNumVerticesPerMeshlet)) {
        if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
            for (int i = 0; i < 2; i++) {
                meshletTaskMeshPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
            }
        } else if (deferredRenderingMode == DeferredRenderingMode::BVH_MESH_SHADER) {
            nodesBVHClearQueuePass->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
                meshletMeshBVHPasses[i]->setMaxNumVerticesPerMeshlet(taskMeshShaderMaxNumVerticesPerMeshlet);
            }
        }
        reloadGatherShader();
        reRender = true;
    }

    if (settings.getValueOpt("use_mesh_shader_nv", useMeshShaderNV)) {
        updateTaskMeshShaderMode();
        reRender = true;
    }

    if (settings.getValueOpt(
            "use_mesh_shader_write_packed_primitive_indices_if_available",
            useMeshShaderWritePackedPrimitiveIndicesIfAvailable)) {
        updateWritePackedPrimitives();
    }

    std::string bvhBuildAlgorithmString;
    if (settings.getValueOpt("bvh_build_algorithm", bvhBuildAlgorithmString)) {
        for (int i = 0; i < IM_ARRAYSIZE(bvhBuildAlgorithmNames); i++) {
            if (bvhBuildAlgorithmString == bvhBuildAlgorithmNames[i]) {
                bvhBuildAlgorithm = BvhBuildAlgorithm(i);
                updateBvhBuildAlgorithm();
                break;
            }
        }
    }

    std::string bvhBuildGeometryModeString;
    if (settings.getValueOpt("bvh_build_geometry_mode", bvhBuildGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(bvhBuildGeometryModeNames); i++) {
            if (bvhBuildGeometryModeString == bvhBuildGeometryModeNames[i]) {
                bvhBuildGeometryMode = BvhBuildGeometryMode(i);
                updateBvhBuildGeometryMode();
                break;
            }
        }
    }

    std::string bvhBuildPrimitiveCenterModeString;
    if (settings.getValueOpt("bvh_build_primitive_center_mode", bvhBuildPrimitiveCenterModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(bvhBuildPrimitiveCenterModeNames); i++) {
            if (bvhBuildPrimitiveCenterModeString == bvhBuildPrimitiveCenterModeNames[i]) {
                bvhBuildPrimitiveCenterMode = BvhBuildPrimitiveCenterMode(i);
                updateBvhBuildPrimitiveCenterMode();
                break;
            }
        }
    }

    if (settings.getValueOpt("use_std_bvh_parameters", useStdBvhParameters)) {
        updateUseStdBvhParameters();
    }
    if (settings.getValueOpt("max_tree_depth_bvh", maxLeafSizeBvh)) {
        updateMaxLeafSizeBvh();
    }
    if (settings.getValueOpt("max_tree_depth_bvh", maxTreeDepthBvh)) {
        updateMaxTreeDepthBvh();
    }

    if (settings.getValueOpt("num_workgroups_bvh", numWorkgroupsBvh)) {
        numWorkgroupsBvh = std::clamp(numWorkgroupsBvh, 1u, maxNumWorkgroups);
        if (getIsBvhRenderingMode()) {
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setNumWorkgroups(numWorkgroupsBvh);
            }
        }
        reRender = true;
    }
    if (settings.getValueOpt("workgroup_size_bvh", workgroupSizeBvh)) {
        workgroupSizeBvh = std::clamp(workgroupSizeBvh, 1u, maxWorkgroupSize);
        if (getIsBvhRenderingMode()) {
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setWorkgroupSize(workgroupSizeBvh);
            }
        }
        reRender = true;
    }
    if (settings.getValueOpt("use_subgroup_ops", useSubgroupOps)) {
        if (getIsBvhRenderingMode()) {
            for (int i = 0; i < 2; i++) {
                nodesBVHDrawCountPasses[i]->setUseSubgroupOps(useSubgroupOps);
            }
        }
        reRender = true;
    }

    return shallReloadGatherShader;
}



DeferredResolvePass::DeferredResolvePass(LineRenderer* lineRenderer) : ResolvePass(lineRenderer) {}

void DeferredResolvePass::setDrawIndexedGeometryMode(DrawIndexedGeometryMode geometryModeNew) {
    if (geometryMode != geometryModeNew) {
        geometryMode = geometryModeNew;
        setShaderDirty();
        setDataDirty();
    }
}

void DeferredResolvePass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineRenderer->getLineData()->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("RESOLVE_PASS", ""));

    std::vector<std::string> shaderModuleNames;
    if (geometryMode == DrawIndexedGeometryMode::TRIANGLES) {
        shaderModuleNames = { "DeferredShading.Vertex", "DeferredShading.Fragment" };
    } else if (geometryMode == DrawIndexedGeometryMode::PROGRAMMABLE_PULLING) {
        shaderModuleNames = { "DeferredShading.Vertex", "DeferredShading.Fragment.ProgrammablePull" };
    }

    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            shaderModuleNames, preprocessorDefines);
}



DownsampleBlitPass::DownsampleBlitPass(sgl::vk::Renderer* renderer) : sgl::vk::BlitRenderPass(
        renderer, { "DownsampleBlit.Vertex", "DownsampleBlit.Fragment" }) {
}

void DownsampleBlitPass::_render() {
    renderer->pushConstants(
            rasterData->getGraphicsPipeline(), VK_SHADER_STAGE_FRAGMENT_BIT,
            0, scalingFactor);
    BlitRenderPass::_render();
}
