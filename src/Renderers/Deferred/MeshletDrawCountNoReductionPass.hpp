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

#ifndef LINEVIS_MESHLETDRAWCOUNTNOREDUCTIONPASS_HPP
#define LINEVIS_MESHLETDRAWCOUNTNOREDUCTIONPASS_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class MeshletDrawCountNoReductionPass : public sgl::vk::ComputePass {
public:
    explicit MeshletDrawCountNoReductionPass(sgl::vk::Renderer* renderer);

    // Public interface.
    void setLineData(LineDataPtr& lineData, bool isNewData);
    void setRecheckOccludedOnly(bool _recheckOccludedOnly);
    void setMaxNumPrimitivesPerMeshlet(uint32_t _maxNumPrimitivesPerMeshlet);
    void setShallVisualizeNodes(bool _shallVisualizeNodes);
    void setVisibilityCullingUniformBuffer(const sgl::vk::BufferPtr& uniformBuffer);
    void setDepthBufferTexture(const sgl::vk::TexturePtr& texture);

protected:
    const uint32_t WORKGROUP_SIZE = 256;
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

private:
    LineDataPtr lineData;
    bool recheckOccludedOnly = false;
    uint32_t maxNumPrimitivesPerMeshlet = 128;
    bool shallVisualizeNodes = false; ///< Whether to visualize the BVH hierarchy and meshlet bounds.
    uint32_t numMeshlets = 0;
    sgl::vk::BufferPtr visibilityCullingUniformBuffer;
    sgl::vk::TexturePtr depthBufferTexture;
};

#endif //LINEVIS_MESHLETDRAWCOUNTNOREDUCTIONPASS_HPP
