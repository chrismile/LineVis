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

#ifndef LINEVIS_VISIBILITYBUFFERDRAWINDEXEDINDIRECTPASS_HPP
#define LINEVIS_VISIBILITYBUFFERDRAWINDEXEDINDIRECTPASS_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include "Renderers/LineRasterPass.hpp"
#include "DeferredModes.hpp"

/**
 * Rasterizes the geometry to the visibility and depth buffer using indirect rendering.
 * Currently, only TaskMeshShaderGeometryMode::TRIANGLES is supported, i.e., no programmable pulling.
 *
 * For reference regarding rendering with hierarchical z-buffers, please refer to:
 * - https://www.rastergrid.com/blog/2010/10/hierarchical-z-map-based-occlusion-culling/
 * - http://advances.realtimerendering.com/s2021/Karis_Nanite_SIGGRAPH_Advances_2021_final.pdf
 * - https://blog.selfshadow.com/publications/practical-visibility/
 */
class VisibilityBufferDrawIndexedIndirectPass : public LineRasterPass {
public:
    explicit VisibilityBufferDrawIndexedIndirectPass(LineRenderer* lineRenderer);
    void setMaxNumPrimitivesPerMeshlet(uint32_t numPrimitives);
    void setUseDrawIndexedIndirectCount(bool useIndirectCount);
    [[nodiscard]] inline uint32_t getNumMeshlets() const { return numMeshlets; }

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;

private:
    uint32_t maxNumPrimitivesPerMeshlet = 128;
    bool useDrawIndexedIndirectCount = true;
    uint32_t numMeshlets = 0;
};

#endif //LINEVIS_VISIBILITYBUFFERDRAWINDEXEDINDIRECTPASS_HPP
