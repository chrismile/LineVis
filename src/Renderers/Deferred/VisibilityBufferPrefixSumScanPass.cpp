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
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include "VisibilityBufferPrefixSumScanPass.hpp"

VisibilityBufferPrefixSumScanPass::VisibilityBufferPrefixSumScanPass(
        sgl::vk::Renderer* renderer) : ComputePass(renderer) {
}

void VisibilityBufferPrefixSumScanPass::setInputBuffer(const sgl::vk::BufferPtr& _inputBuffer) {
    inputBuffer = _inputBuffer;
    uint32_t N = inputBuffer->getSizeInBytes() / sizeof(uint32_t);
    outputBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), N * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    setDataDirty();
}

const sgl::vk::BufferPtr& VisibilityBufferPrefixSumScanPass::getOutputBuffer() {
    return outputBuffer;
}

void VisibilityBufferPrefixSumScanPass::loadShader() {
    sgl::vk::ComputePipelineInfo computePipelineInfo(sgl::vk::ShaderManager->getShaderStages(
            { "PrefixSumBlockIncrement.Compute" }));
    prefixSumBlockIncrementPipeline = std::make_shared<sgl::vk::ComputePipeline>(device, computePipelineInfo);

    computePipelineInfo = sgl::vk::ComputePipelineInfo(sgl::vk::ShaderManager->getShaderStages(
            { "PrefixSumScan.Compute" }));
    prefixSumScanPipeline = std::make_shared<sgl::vk::ComputePipeline>(device, computePipelineInfo);

    // Just use something, as CompuePass::_build expects it to not be a nullptr.
    shaderStages = sgl::vk::ShaderManager->getShaderStages({ "PrefixSumScan.Compute" });
}

void VisibilityBufferPrefixSumScanPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    prefixSumScanDataList.clear();
    prefixSumBlockIncrementDataList.clear();
    uint32_t N = inputBuffer->getSizeInBytes() / sizeof(uint32_t);
    createPrefixSumDataRecursive(0, N, inputBuffer, outputBuffer);
}

void VisibilityBufferPrefixSumScanPass::_render() {
    uint32_t N = inputBuffer->getSizeInBytes() / sizeof(uint32_t);
    parallelPrefixSumRecursive(0, N);
}

void VisibilityBufferPrefixSumScanPass::createPrefixSumDataRecursive(
        size_t level, uint32_t N, sgl::vk::BufferPtr& bufferIn, sgl::vk::BufferPtr& bufferOut) {
    if (N <= 0) {
        return;
    }

    const int numBlocks = sgl::iceil(int(N), 2 * BLOCK_SIZE);

    sgl::vk::BufferPtr sumIn = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numBlocks * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    sgl::vk::BufferPtr sumOut = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numBlocks * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    sgl::vk::ComputeDataPtr prefixSumScanData = std::make_shared<sgl::vk::ComputeData>(
            renderer, prefixSumScanPipeline);
    prefixSumScanData->setStaticBuffer(bufferIn, 0);
    prefixSumScanData->setStaticBuffer(bufferOut, 1);
    prefixSumScanData->setStaticBuffer(sumIn, 2);
    if (level >= prefixSumScanDataList.size()) {
        prefixSumScanDataList.resize(level + 1);
    }
    prefixSumScanDataList.at(level) = prefixSumScanData;

    // Array of block increments that is added to the elements in the blocks.
    if (numBlocks > 1) {
        createPrefixSumDataRecursive(level + 1, numBlocks, sumIn, sumOut);

        sgl::vk::ComputeDataPtr prefixSumBlockIncrementData = std::make_shared<sgl::vk::ComputeData>(
                renderer, prefixSumBlockIncrementPipeline);
        prefixSumBlockIncrementData->setStaticBuffer(bufferOut, 1);
        prefixSumBlockIncrementData->setStaticBuffer(sumOut, 2);
        if (level >= prefixSumBlockIncrementDataList.size()) {
            prefixSumBlockIncrementDataList.resize(level + 1);
        }
        prefixSumBlockIncrementDataList.at(level) = prefixSumBlockIncrementData;
    }
}

void VisibilityBufferPrefixSumScanPass::parallelPrefixSumRecursive(size_t level, uint32_t N) {
    if (N <= 0) {
        return;
    }

    const int numBlocks = sgl::iceil(int(N), 2 * BLOCK_SIZE);

    sgl::vk::ComputeDataPtr prefixSumScanData = prefixSumScanDataList.at(level);
    renderer->pushConstants(
            prefixSumScanData->getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT,
            0, N);
    renderer->dispatch(prefixSumScanData, numBlocks, 1, 1);
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    // Array of block increments that is added to the elements in the blocks.
    if (numBlocks > 1) {
        parallelPrefixSumRecursive(level + 1, numBlocks);

        sgl::vk::ComputeDataPtr prefixSumBlockIncrementData = prefixSumBlockIncrementDataList.at(level);
        renderer->pushConstants(
                prefixSumBlockIncrementData->getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT,
                0, N);
        renderer->dispatch(prefixSumBlockIncrementData, numBlocks, 1, 1);
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    }
}
