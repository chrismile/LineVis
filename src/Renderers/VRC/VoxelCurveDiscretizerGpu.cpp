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
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>

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
    uint32_t gridSize1D = gridResolution.x * gridResolution.y * gridResolution.z;

    // Set the preprocessor defines for the shaders.
    sgl::ShaderManager->addPreprocessorDefine(
            "gridResolution", ivec3ToString(gridResolution));
    sgl::ShaderManager->addPreprocessorDefine(
            "GRID_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(gridResolution.x))));
    sgl::ShaderManager->addPreprocessorDefine(
            "GRID_RESOLUTION", gridResolution.x);
    sgl::ShaderManager->addPreprocessorDefine(
            "quantizationResolution", uvec3ToString(quantizationResolution));
    sgl::ShaderManager->addPreprocessorDefine(
            "QUANTIZATION_RESOLUTION", sgl::toString(quantizationResolution.x));
    sgl::ShaderManager->addPreprocessorDefine(
            "QUANTIZATION_RESOLUTION_LOG2", sgl::toString(sgl::intlog2(int(quantizationResolution.x))));


    // PART 1: Create the LinePointBuffer, LineOffsetBuffer, NumSegmentsBuffer (empty) and LineSegmentsBuffer.
    auto startBuffers = std::chrono::system_clock::now();
    std::vector<LinePoint> linePoints;
    std::vector<uint32_t> lineOffsets;
    lineOffsets.push_back(0);
    uint32_t offsetCounter = 0;
    for (Curve& curve : curves) {
        size_t curveNumPoints = curve.points.size();
        for (size_t i = 0; i < curveNumPoints; i++) {
            linePoints.emplace_back(curve.points.at(i), curve.attributes.at(i));
        }
        offsetCounter += curveNumPoints;
        lineOffsets.push_back(offsetCounter);
    }
    sgl::GeometryBufferPtr linePointBuffer = sgl::Renderer->createGeometryBuffer(
            (linePoints.size() + 1) * sizeof(LinePoint), &linePoints.front(),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    sgl::GeometryBufferPtr lineOffsetBuffer = sgl::Renderer->createGeometryBuffer(
            (curves.size() + 1) * sizeof(uint32_t), &lineOffsets.front(),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    voxelGridNumLineSegmentsBuffer = sgl::Renderer->createGeometryBuffer(
            gridSize1D * sizeof(uint32_t),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    sgl::GeometryBufferPtr voxelGridNumLineSegmentsBufferTmp = sgl::Renderer->createGeometryBuffer(
            gridSize1D * sizeof(uint32_t),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    uint32_t zeroData = 0u;
    glClearNamedBufferData(
            ((sgl::GeometryBufferGL*)voxelGridNumLineSegmentsBuffer.get())->getBuffer(),
            GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&zeroData);
    glClearNamedBufferData(
            ((sgl::GeometryBufferGL*)voxelGridNumLineSegmentsBufferTmp.get())->getBuffer(),
            GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&zeroData);
    auto endBuffers = std::chrono::system_clock::now();
    auto elapsedBuffers = std::chrono::duration_cast<std::chrono::milliseconds>(endBuffers - startBuffers);
    sgl::Logfile::get()->writeInfo(
            std::string() + "Computational time to create the buffers: "
            + std::to_string(elapsedBuffers.count()) + "ms");


    // PART 2: Count the number of voxelized line segments passing through each voxel.
    auto startCountLineSegments = std::chrono::system_clock::now();
    uint32_t numWorkGroupsLines = sgl::iceil(int(curves.size()), 256);
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderProgramPtr discretizeLinesShader = sgl::ShaderManager->getShaderProgram(
            {"DiscretizeLines.Compute"});
    sgl::ShaderManager->bindShaderStorageBuffer(2, linePointBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(3, lineOffsetBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(4, voxelGridNumLineSegmentsBuffer);
    discretizeLinesShader->setUniform("numLines", uint32_t(curves.size()));
    discretizeLinesShader->dispatchCompute(int(numWorkGroupsLines));
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    auto endCountLineSegments = std::chrono::system_clock::now();
    auto elapsedCountLineSegments = std::chrono::duration_cast<std::chrono::milliseconds>(
            endCountLineSegments - startCountLineSegments);
    sgl::Logfile::get()->writeInfo(
            "Computational time to count the voxelized line segments: "
            + std::to_string(elapsedCountLineSegments.count()) + "ms");


    // PART 3: Run a parallel prefix sum scan on the number of lines.
    auto startPrefixSum = std::chrono::system_clock::now();
    voxelGridLineSegmentOffsetsBuffer = parallelPrefixSumReduce(gridSize1D, voxelGridNumLineSegmentsBuffer);
    uint32_t numVoxelizedLineSegments = 0;
    if (voxelGridLineSegmentOffsetsBuffer) {
        auto voxelGridPrefixSumArray = static_cast<uint32_t*>(
                voxelGridLineSegmentOffsetsBuffer->mapBuffer(sgl::BUFFER_MAP_READ_ONLY));
        numVoxelizedLineSegments = voxelGridPrefixSumArray[gridSize1D];
        voxelGridLineSegmentOffsetsBuffer->unmapBuffer();
    }
    auto endPrefixSum = std::chrono::system_clock::now();
    auto elapsedPrefixSum = std::chrono::duration_cast<std::chrono::milliseconds>(endPrefixSum - startPrefixSum);
    sgl::Logfile::get()->writeInfo(
            "Computational time to run a parallel prefix sum scan on the number of voxelized line segments: "
            + std::to_string(elapsedPrefixSum.count()) + "ms");


    // PART 4: Discretize, quantize and voxelize the lines.
    auto startVoxelizeLines = std::chrono::system_clock::now();
    voxelGridLineSegmentsBuffer = {};
    if (numVoxelizedLineSegments > 0) {
        voxelGridLineSegmentsBuffer = sgl::Renderer->createGeometryBuffer(
                numVoxelizedLineSegments * sizeof(LineSegmentCompressed),
                sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
        sgl::ShaderManager->invalidateShaderCache();
        sgl::ShaderManager->addPreprocessorDefine("WRITE_LINE_SEGMENTS_PASS", "");
        discretizeLinesShader = sgl::ShaderManager->getShaderProgram({"DiscretizeLines.Compute"});
        sgl::ShaderManager->removePreprocessorDefine("WRITE_LINE_SEGMENTS_PASS");
        sgl::ShaderManager->bindShaderStorageBuffer(2, linePointBuffer);
        sgl::ShaderManager->bindShaderStorageBuffer(3, lineOffsetBuffer);
        sgl::ShaderManager->bindShaderStorageBuffer(4, voxelGridNumLineSegmentsBufferTmp);
        sgl::ShaderManager->bindShaderStorageBuffer(5, voxelGridLineSegmentOffsetsBuffer);
        sgl::ShaderManager->bindShaderStorageBuffer(6, voxelGridLineSegmentsBuffer);
        discretizeLinesShader->setUniform("numLines", uint32_t(curves.size()));
        discretizeLinesShader->dispatchCompute(int(numWorkGroupsLines));
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
    auto endVoxelizeLines = std::chrono::system_clock::now();
    auto elapsedVoxelizeLines = std::chrono::duration_cast<std::chrono::milliseconds>(
            endVoxelizeLines - startVoxelizeLines);
    sgl::Logfile::get()->writeInfo(
            "Computational time to voxelize all lines: "
            + std::to_string(elapsedVoxelizeLines.count()) + "ms");
    sgl::Logfile::get()->writeInfo(
            "Total number of voxelized line segments: " + std::to_string(numVoxelizedLineSegments));


    // Remove the preprocessor defines for the shaders.
    sgl::ShaderManager->removePreprocessorDefine("gridResolution");
    sgl::ShaderManager->removePreprocessorDefine("GRID_RESOLUTION_LOG2");
    sgl::ShaderManager->removePreprocessorDefine("GRID_RESOLUTION");
    sgl::ShaderManager->removePreprocessorDefine("quantizationResolution");
    sgl::ShaderManager->removePreprocessorDefine("QUANTIZATION_RESOLUTION");
    sgl::ShaderManager->removePreprocessorDefine("QUANTIZATION_RESOLUTION_LOG2");
}

sgl::GeometryBufferPtr VoxelCurveDiscretizer::parallelPrefixSumReduce(uint32_t N, sgl::GeometryBufferPtr& bufferIn) {
    if (N <= 0) {
        return {};
    }

    sgl::GeometryBufferPtr bufferOut = sgl::Renderer->createGeometryBuffer(
            (N + 1) * sizeof(uint32_t), sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);

    parallelPrefixSumRecursive(N, bufferIn, bufferOut);

    sgl::ShaderManager->bindShaderStorageBuffer(0, bufferIn);
    sgl::ShaderManager->bindShaderStorageBuffer(1, bufferOut);
    prefixSumWriteFinalElementShader->setUniform("N", N);
    prefixSumWriteFinalElementShader->dispatchCompute(1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    return bufferOut;
}

void VoxelCurveDiscretizer::parallelPrefixSumRecursive(
        uint32_t N, sgl::GeometryBufferPtr& bufferIn, sgl::GeometryBufferPtr& bufferOut) {
    if (N <= 0) {
        return;
    }

    const int BLOCK_SIZE = 256;
    const int numBlocks = sgl::iceil(int(N), 2 * BLOCK_SIZE);

    sgl::GeometryBufferPtr sumIn = sgl::Renderer->createGeometryBuffer(
            numBlocks * sizeof(uint32_t), sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    sgl::GeometryBufferPtr sumOut = sgl::Renderer->createGeometryBuffer(
            numBlocks * sizeof(uint32_t), sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);

    sgl::ShaderManager->bindShaderStorageBuffer(0, bufferIn);
    sgl::ShaderManager->bindShaderStorageBuffer(1, bufferOut);
    sgl::ShaderManager->bindShaderStorageBuffer(2, sumIn);
    prefixSumScanShader->setUniform("N", N);
    prefixSumScanShader->dispatchCompute(numBlocks);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Array of block increments that is added to the elements in the blocks.
    if (numBlocks > 1) {
        parallelPrefixSumRecursive(numBlocks, sumIn, sumOut);
        sgl::ShaderManager->bindShaderStorageBuffer(1, bufferOut);
        sgl::ShaderManager->bindShaderStorageBuffer(2, sumOut);
        prefixSumBlockIncrementShader->setUniform("N", N);
        prefixSumBlockIncrementShader->dispatchCompute(numBlocks);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}
