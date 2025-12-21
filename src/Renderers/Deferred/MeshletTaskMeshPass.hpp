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

#ifndef LINEVIS_MESHLETTASKMESHPASS_HPP
#define LINEVIS_MESHLETTASKMESHPASS_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include "Renderers/LineRasterPass.hpp"

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class MeshletTaskMeshPass : public LineRasterPass {
public:
    explicit MeshletTaskMeshPass(LineRenderer* lineRenderer);

    // Public interface.
    void setRecheckOccludedOnly(bool recheck);
    void setUseMeshShaderNV(bool _useMeshShaderNV);
    void setMaxNumPrimitivesPerMeshlet(uint32_t numPrimitives);
    void setMaxNumVerticesPerMeshlet(uint32_t numVertices);
    void setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(bool useWritePacked);
    void setShallVisualizeNodes(bool _shallVisualizeNodes);
    void setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer);
    void setDepthBufferTexture(const sgl::vk::TexturePtr& texture);
    [[nodiscard]] inline uint32_t getNumMeshlets() const { return numMeshlets; }

    // Visualization of meshlet bounds.
    [[nodiscard]] inline const sgl::vk::BufferPtr& getNodeAabbBuffer() const { return nodeAabbBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getNodeAabbCountBuffer() const { return nodeAabbCountBuffer; }

protected:
    uint32_t WORKGROUP_SIZE_NV = 32;
    uint32_t WORKGROUP_SIZE_EXT = 32;
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

    bool recheckOccludedOnly = false;
    bool useMeshShaderNV = false; ///< Whether to use VK_EXT_mesh_shader oder VK_NV_mesh_shader.
    uint32_t maxNumPrimitivesPerMeshlet = 126;
    uint32_t maxNumVerticesPerMeshlet = 64;
    bool useMeshShaderWritePackedPrimitiveIndicesIfAvailable = true; ///< Sub-mode for VK_NV_mesh_shader.
    bool useMeshShaderWritePackedPrimitiveIndices = false;
    bool shallVisualizeNodes = false; ///< Whether to visualize the BVH hierarchy and meshlet bounds.
    uint32_t numMeshlets = 0;
    bool bvhMeshlets = false;
    sgl::vk::BufferPtr visibilityCullingUniformBuffer;
    sgl::vk::TexturePtr depthBufferTexture;

    // Visualization of meshlet bounds.
    sgl::vk::BufferPtr nodeAabbBuffer;
    sgl::vk::BufferPtr nodeAabbCountBuffer;
};

class MeshletMeshBVHPass : public MeshletTaskMeshPass {
public:
    explicit MeshletMeshBVHPass(LineRenderer* lineRenderer);
    void setBvhBuildAlgorithm(BvhBuildAlgorithm _bvhBuildAlgorithm);
    void setBvhBuildGeometryMode(BvhBuildGeometryMode _bvhBuildGeometryMode);
    void setBvhBuildPrimitiveCenterMode(BvhBuildPrimitiveCenterMode _bvhBuildPrimitiveCenterMode);
    void setUseStdBvhParameters(bool _useStdBvhParameters);
    void setMaxLeafSizeBvh(uint32_t _maxLeafSizeBvh);
    void setMaxTreeDepthBvh(uint32_t _maxTreeDepthBvh);
    [[nodiscard]] inline uint32_t getTreeHeight() const { return treeHeight; }

protected:
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;

private:
    BvhBuildAlgorithm bvhBuildAlgorithm = BvhBuildAlgorithm::SWEEP_SAH_CPU;
    BvhBuildGeometryMode bvhBuildGeometryMode = BvhBuildGeometryMode::TRIANGLES;
    BvhBuildPrimitiveCenterMode bvhBuildPrimitiveCenterMode = BvhBuildPrimitiveCenterMode::PRIMITIVE_CENTROID;
    // For bvhBuildAlgorithm == BvhBuildAlgorithm::BINNED_SAH_CPU and SWEEP_SAH_CPU.
    bool useStdBvhParameters = true; ///< Whether to use the settings below.
    uint32_t maxLeafSizeBvh = 16;
    uint32_t maxTreeDepthBvh = 64;
    uint32_t treeHeight = 0;
};

#endif //LINEVIS_MESHLETTASKMESHPASS_HPP
