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

#ifndef LINEVIS_NODESBVHDRAWCOUNTPASS_HPP
#define LINEVIS_NODESBVHDRAWCOUNTPASS_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class NodesBVHDrawCountPass : public sgl::vk::ComputePass {
public:
    explicit NodesBVHDrawCountPass(sgl::vk::Renderer* renderer);

    // Public interface.
    void setLineData(LineDataPtr& lineData, bool isNewData);
    void setRecheckOccludedOnly(bool recheck);
    void setUseSubgroupOps(bool _useSubgroupOps);
    void setDrawIndexedIndirectMode(bool _drawIndexedIndirectMode);
    void setMaxNumPrimitivesPerMeshlet(uint32_t _maxNumPrimitivesPerMeshlet);
    void setMaxNumVerticesPerMeshlet(uint32_t _maxNumVerticesPerMeshlet);
    void setUseMeshShaderWritePackedPrimitiveIndicesIfAvailable(bool _useMeshShaderWritePackedPrimitiveIndices);
    void setBvhBuildAlgorithm(BvhBuildAlgorithm _bvhBuildAlgorithm);
    void setBvhBuildGeometryMode(BvhBuildGeometryMode _bvhBuildGeometryMode);
    void setBvhBuildPrimitiveCenterMode(BvhBuildPrimitiveCenterMode _bvhBuildPrimitiveCenterMode);
    void setUseStdBvhParameters(bool _useStdBvhParameters);
    void setMaxLeafSizeBvh(uint32_t _maxLeafSizeBvh);
    void setMaxTreeDepthBvh(uint32_t _maxTreeDepthBvh);
    void setNumWorkgroups(uint32_t numWorkgroupsBvh);
    void setWorkgroupSize(uint32_t workgroupSizeBvh);
    void setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer);
    void setDepthBufferTexture(const sgl::vk::TexturePtr& texture);
    [[nodiscard]] inline const sgl::vk::BufferPtr& getDrawCountBuffer() const { return indirectDrawCountBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getMaxWorkLeftTestBuffer() const { return maxWorkLeftTestBuffer; }

protected:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

private:
    LineDataPtr lineData;
    bool recheckOccludedOnly = false;
    bool useSubgroupOps = false; ///< Use subgroup operations in NodesBVHDrawCountPass.glsl?
    bool useSpinlock = false;

    bool drawIndexedIndirectMode = true; ///< Draw indexed indirect or mesh shader mode?
    uint32_t maxNumPrimitivesPerMeshlet = 126;
    uint32_t maxNumVerticesPerMeshlet = 64;
    bool useMeshShaderWritePackedPrimitiveIndices = false;
    BvhBuildAlgorithm bvhBuildAlgorithm = BvhBuildAlgorithm::SWEEP_SAH_CPU;
    BvhBuildGeometryMode bvhBuildGeometryMode = BvhBuildGeometryMode::TRIANGLES;
    BvhBuildPrimitiveCenterMode bvhBuildPrimitiveCenterMode = BvhBuildPrimitiveCenterMode::PRIMITIVE_CENTROID;
    // For bvhBuildAlgorithm == BvhBuildAlgorithm::BINNED_SAH_CPU and SWEEP_SAH_CPU.
    bool useStdBvhParameters = true; ///< Whether to use the settings below.
    uint32_t maxLeafSizeBvh = 16;
    uint32_t maxTreeDepthBvh = 64;

    uint32_t numNodes = 0;
    uint32_t numWorkgroups = 0;
    uint32_t workgroupSize = 0;
    sgl::vk::BufferPtr indirectDrawCountBuffer;
    sgl::vk::BufferPtr visibilityCullingUniformBuffer;
    sgl::vk::TexturePtr depthBufferTexture;
    sgl::vk::BufferPtr maxWorkLeftTestBuffer;
};

#endif //LINEVIS_NODESBVHDRAWCOUNTPASS_HPP
