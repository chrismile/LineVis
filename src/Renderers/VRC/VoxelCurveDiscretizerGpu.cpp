/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#include <chrono>

#include <Math/Math.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <memory>

#include "VoxelCurveDiscretizer.hpp"

struct LinePoint {
    LinePoint(glm::vec3 linePoint, float lineAttribute) : linePoint(linePoint), lineAttribute(lineAttribute) {}
    glm::vec3 linePoint;
    float lineAttribute;
};

std::string ivec3ToString(const glm::ivec3& v) {
    return std::string() + "ivec3(" + sgl::toString(v.x) + ", " + sgl::toString(v.y) + ", " + sgl::toString(v.z) + ")";
}

std::string uvec3ToString(const glm::uvec3& v) {
    return std::string() + "uvec3(" + sgl::toString(v.x) + ", " + sgl::toString(v.y) + ", " + sgl::toString(v.z) + ")";
}

void VoxelCurveDiscretizer::createVoxelGridGpu() {
    sgl::vk::Device* device = renderer->getDevice();
    uint32_t gridSize1D = gridResolution.x * gridResolution.y * gridResolution.z;

    std::map<std::string, std::string> preprocessorDefines;

    // Set the preprocessor defines for the shaders.
    preprocessorDefines.insert(std::make_pair(
            "gridResolution", ivec3ToString(gridResolution)));
    preprocessorDefines.insert(std::make_pair(
            "GRID_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(gridResolution.x)))));
    preprocessorDefines.insert(std::make_pair(
            "GRID_RESOLUTION", sgl::toString(gridResolution.x)));
    preprocessorDefines.insert(std::make_pair(
            "quantizationResolution", uvec3ToString(quantizationResolution)));
    preprocessorDefines.insert(std::make_pair(
            "QUANTIZATION_RESOLUTION", sgl::toString(quantizationResolution.x)));
    preprocessorDefines.insert(std::make_pair(
            "QUANTIZATION_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(quantizationResolution.x)))));

    // PART 1: Create the LinePointBuffer, LineOffsetBuffer, NumSegmentsBuffer (empty) and LineSegmentsBuffer.
    auto startBuffers = std::chrono::system_clock::now();
    VkCommandBuffer commandBuffer = device->beginSingleTimeCommands(
            device->getComputeQueueIndex(), false);
    renderer->setCustomCommandBuffer(commandBuffer, false);
    renderer->beginCommandBuffer();
    std::vector<LinePoint> linePoints;
    std::vector<uint32_t> lineOffsets;
    lineOffsets.push_back(0);
    uint32_t offsetCounter = 0;
    for (Curve& curve : curves) {
        size_t curveNumPoints = curve.points.size();
        for (size_t i = 0; i < curveNumPoints; i++) {
            linePoints.emplace_back(curve.points.at(i), curve.attributes.at(i));
        }
        offsetCounter += uint32_t(curveNumPoints);
        lineOffsets.push_back(offsetCounter);
    }

    voxelGridLineSegmentOffsetsBuffer = {};
    voxelGridNumLineSegmentsBuffer = {};
    voxelGridLineSegmentsBuffer = {};

    if (linePoints.empty()) {
        return;
    }

    sgl::vk::BufferPtr linePointBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), (linePoints.size() + 1) * sizeof(LinePoint), &linePoints.front(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    sgl::vk::BufferPtr lineOffsetBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), (curves.size() + 1) * sizeof(uint32_t), &lineOffsets.front(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    voxelGridNumLineSegmentsBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), gridSize1D * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    sgl::vk::BufferPtr voxelGridNumLineSegmentsBufferTmp = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), gridSize1D * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    voxelGridNumLineSegmentsBuffer->fill(0, renderer->getVkCommandBuffer());
    voxelGridNumLineSegmentsBufferTmp->fill(0, renderer->getVkCommandBuffer());
    auto endBuffers = std::chrono::system_clock::now();
    auto elapsedBuffers = std::chrono::duration_cast<std::chrono::milliseconds>(endBuffers - startBuffers);
    sgl::Logfile::get()->writeInfo(
            std::string() + "Computational time to create the buffers: "
            + std::to_string(elapsedBuffers.count()) + "ms");


    // PART 2: Count the number of voxelized line segments passing through each voxel.
    auto startCountLineSegments = std::chrono::system_clock::now();
    uint32_t numWorkGroupsLines = sgl::iceil(int(curves.size()), 256);
    sgl::vk::ShaderManager->invalidateShaderCache();
    sgl::vk::ComputePipelineInfo computePipelineInfo(sgl::vk::ShaderManager->getShaderStages(
            { "DiscretizeLines.Compute" }, preprocessorDefines));
    sgl::vk::ComputePipelinePtr discretizeLinesComputePipeline(
            new sgl::vk::ComputePipeline(device, computePipelineInfo));
    sgl::vk::ComputeDataPtr discretizeLinesData = std::make_shared<sgl::vk::ComputeData>(
            renderer, discretizeLinesComputePipeline);
    discretizeLinesData->setStaticBuffer(linePointBuffer, "LinePointBuffer");
    discretizeLinesData->setStaticBuffer(lineOffsetBuffer, "LineOffsetBuffer");
    discretizeLinesData->setStaticBuffer(voxelGridNumLineSegmentsBuffer, "NumLinesBuffer");
    renderer->pushConstants(
            discretizeLinesComputePipeline, VK_SHADER_STAGE_COMPUTE_BIT,
            0, uint32_t(curves.size()));
    renderer->dispatch(discretizeLinesData, numWorkGroupsLines, 1, 1);
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            voxelGridNumLineSegmentsBuffer);
    auto endCountLineSegments = std::chrono::system_clock::now();
    auto elapsedCountLineSegments = std::chrono::duration_cast<std::chrono::milliseconds>(
            endCountLineSegments - startCountLineSegments);
    sgl::Logfile::get()->writeInfo(
            "Computational time to count the voxelized line segments: "
            + std::to_string(elapsedCountLineSegments.count()) + "ms");


    // PART 3: Run a parallel prefix sum scan on the number of lines.
    auto startPrefixSum = std::chrono::system_clock::now();
    voxelGridLineSegmentOffsetsBuffer = parallelPrefixSumReduce(gridSize1D, voxelGridNumLineSegmentsBuffer);
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            voxelGridLineSegmentOffsetsBuffer);
    sgl::vk::BufferPtr stagingBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), voxelGridLineSegmentOffsetsBuffer->getSizeInBytes(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU);
    voxelGridLineSegmentOffsetsBuffer->copyDataTo(stagingBuffer, renderer->getVkCommandBuffer());
    renderer->endCommandBuffer();
    renderer->resetCustomCommandBuffer();
    device->endSingleTimeCommands(commandBuffer, device->getComputeQueueIndex(), false);
    uint32_t numVoxelizedLineSegments = 0;
    if (voxelGridLineSegmentOffsetsBuffer) {
        auto voxelGridPrefixSumArray = static_cast<uint32_t*>(stagingBuffer->mapMemory());
        numVoxelizedLineSegments = voxelGridPrefixSumArray[gridSize1D];
        stagingBuffer->unmapMemory();
    }
    parallelPrefixSumReduceCache.clear();
    auto endPrefixSum = std::chrono::system_clock::now();
    auto elapsedPrefixSum = std::chrono::duration_cast<std::chrono::milliseconds>(endPrefixSum - startPrefixSum);
    sgl::Logfile::get()->writeInfo(
            "Computational time to run a parallel prefix sum scan on the number of voxelized line segments: "
            + std::to_string(elapsedPrefixSum.count()) + "ms");


    // PART 4: Discretize, quantize and voxelize the lines.
    auto startVoxelizeLines = std::chrono::system_clock::now();
    voxelGridLineSegmentsBuffer = {};
    if (numVoxelizedLineSegments > 0) {
        VkCommandBuffer commandBuffer = device->beginSingleTimeCommands(
                device->getComputeQueueIndex(), false);
        renderer->setCustomCommandBuffer(commandBuffer, false);
        renderer->beginCommandBuffer();

        voxelGridLineSegmentsBuffer = std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), numVoxelizedLineSegments * sizeof(LineSegmentCompressed),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
        sgl::vk::ShaderManager->invalidateShaderCache();
        preprocessorDefines.insert(std::make_pair("WRITE_LINE_SEGMENTS_PASS", ""));
        computePipelineInfo = sgl::vk::ComputePipelineInfo(sgl::vk::ShaderManager->getShaderStages(
                { "DiscretizeLines.Compute" }, preprocessorDefines));
        discretizeLinesComputePipeline = std::make_shared<sgl::vk::ComputePipeline>(
                device, computePipelineInfo);
        discretizeLinesData = std::make_shared<sgl::vk::ComputeData>(
                renderer, discretizeLinesComputePipeline);
        preprocessorDefines.erase("WRITE_LINE_SEGMENTS_PASS");
        discretizeLinesData->setStaticBuffer(linePointBuffer, "LinePointBuffer");
        discretizeLinesData->setStaticBuffer(lineOffsetBuffer, "LineOffsetBuffer");
        discretizeLinesData->setStaticBuffer(voxelGridNumLineSegmentsBufferTmp, "NumLinesBuffer");
        discretizeLinesData->setStaticBuffer(voxelGridLineSegmentOffsetsBuffer, "VoxelLineListOffsetBuffer");
        discretizeLinesData->setStaticBuffer(voxelGridLineSegmentsBuffer, "LineSegmentsBuffer");
        renderer->pushConstants(
                discretizeLinesComputePipeline, VK_SHADER_STAGE_COMPUTE_BIT,
                0, uint32_t(curves.size()));
        renderer->dispatch(discretizeLinesData, numWorkGroupsLines, 1, 1);
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                voxelGridNumLineSegmentsBuffer);

        renderer->endCommandBuffer();
        renderer->resetCustomCommandBuffer();
        device->endSingleTimeCommands(commandBuffer, device->getComputeQueueIndex(), false);
    }
    auto endVoxelizeLines = std::chrono::system_clock::now();
    auto elapsedVoxelizeLines = std::chrono::duration_cast<std::chrono::milliseconds>(
            endVoxelizeLines - startVoxelizeLines);
    sgl::Logfile::get()->writeInfo(
            "Computational time to voxelize all lines: "
            + std::to_string(elapsedVoxelizeLines.count()) + "ms");
    sgl::Logfile::get()->writeInfo(
            "Total number of voxelized line segments: " + std::to_string(numVoxelizedLineSegments));
}

sgl::vk::BufferPtr VoxelCurveDiscretizer::parallelPrefixSumReduce(uint32_t N, sgl::vk::BufferPtr& bufferIn) {
    if (N <= 0) {
        return {};
    }

    sgl::vk::BufferPtr bufferOut = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), (N + 1) * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    parallelPrefixSumRecursive(N, bufferIn, bufferOut);

    prefixSumWriteFinalElementData->setStaticBuffer(bufferIn, 0);
    prefixSumWriteFinalElementData->setStaticBuffer(bufferOut, 1);
    renderer->pushConstants(
            prefixSumWriteFinalElementData->getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT,
            0, N);
    renderer->dispatch(prefixSumWriteFinalElementData, 1, 1, 1);

    return bufferOut;
}

void VoxelCurveDiscretizer::parallelPrefixSumRecursive(
        uint32_t N, sgl::vk::BufferPtr& bufferIn, sgl::vk::BufferPtr& bufferOut) {
    if (N <= 0) {
        return;
    }

    const int BLOCK_SIZE = 256;
    const int numBlocks = sgl::iceil(int(N), 2 * BLOCK_SIZE);

    sgl::vk::BufferPtr sumIn = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numBlocks * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    sgl::vk::BufferPtr sumOut = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numBlocks * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    prefixSumScanData = std::make_shared<sgl::vk::ComputeData>(renderer, prefixSumScanPipeline);
    prefixSumScanData->setStaticBuffer(bufferIn, 0);
    prefixSumScanData->setStaticBuffer(bufferOut, 1);
    prefixSumScanData->setStaticBuffer(sumIn, 2);
    renderer->pushConstants(
            prefixSumScanData->getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT,
            0, N);
    renderer->dispatch(prefixSumScanData, numBlocks, 1, 1);
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    parallelPrefixSumReduceCache.push_back(prefixSumScanData);

    // Array of block increments that is added to the elements in the blocks.
    if (numBlocks > 1) {
        parallelPrefixSumRecursive(numBlocks, sumIn, sumOut);

        prefixSumBlockIncrementData = std::make_shared<sgl::vk::ComputeData>(
                renderer, prefixSumBlockIncrementPipeline);
        prefixSumBlockIncrementData->setStaticBuffer(bufferOut, 1);
        prefixSumBlockIncrementData->setStaticBuffer(sumOut, 2);
        renderer->pushConstants(
                prefixSumBlockIncrementData->getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT,
                0, N);
        renderer->dispatch(prefixSumBlockIncrementData, numBlocks, 1, 1);
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
        parallelPrefixSumReduceCache.push_back(prefixSumBlockIncrementData);
    }
}
