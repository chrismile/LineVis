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

#ifndef LINEVIS_VISIBILITYBUFFERPREFIXSUMSCANPASS_HPP
#define LINEVIS_VISIBILITYBUFFERPREFIXSUMSCANPASS_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>

class VisibilityBufferPrefixSumScanPass : public sgl::vk::ComputePass {
public:
    explicit VisibilityBufferPrefixSumScanPass(sgl::vk::Renderer* renderer);
    void setInputBuffer(const sgl::vk::BufferPtr& _inputBuffer);
    const sgl::vk::BufferPtr& getOutputBuffer();

protected:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

private:
    void createPrefixSumDataRecursive(
            size_t level, uint32_t N, sgl::vk::BufferPtr& bufferIn, sgl::vk::BufferPtr& bufferOut);
    void parallelPrefixSumRecursive(size_t level, uint32_t N);

    const int BLOCK_SIZE = 256;
    sgl::vk::BufferPtr inputBuffer;
    sgl::vk::BufferPtr outputBuffer;

    sgl::vk::ComputePipelinePtr prefixSumScanPipeline;
    sgl::vk::ComputePipelinePtr prefixSumBlockIncrementPipeline;
    std::vector<sgl::vk::ComputeDataPtr> prefixSumScanDataList;
    std::vector<sgl::vk::ComputeDataPtr> prefixSumBlockIncrementDataList;
};


#endif //LINEVIS_VISIBILITYBUFFERPREFIXSUMSCANPASS_HPP
