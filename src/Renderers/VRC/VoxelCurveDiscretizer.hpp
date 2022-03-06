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

#ifndef LINEVIS_VOXELCURVEDISCRETIZER_HPP
#define LINEVIS_VOXELCURVEDISCRETIZER_HPP

#include <Graphics/Buffers/GeometryBuffer.hpp>
#include <Graphics/Shader/Shader.hpp>

#include "LineData/LineData.hpp"
#include "VoxelData.hpp"

struct AttributePoint
{
    AttributePoint(const glm::vec3 &v, float a) : v(v), a(a) {}
    glm::vec3 v;
    float a;
};

class VoxelDiscretizer {
public:
    // Returns true if the passed line intersects the voxel boundaries
    bool addPossibleIntersections(const glm::vec3& v1, const glm::vec3& v2, float a1, float a2);
    void setIndex(glm::uvec3 index);
    [[nodiscard]] const glm::uvec3& getIndex() const { return index; }
    float computeDensity();

    glm::uvec3 index;
    std::vector<LineSegment> lines;

    // For clipping lines to voxel
    std::vector<AttributePoint> currentCurveIntersections;
};

class VoxelCurveDiscretizer {
public:
    VoxelCurveDiscretizer(sgl::vk::Device* device);
    ~VoxelCurveDiscretizer();
    void loadLineData(LineDataPtr& lineData, uint32_t gridResolution1D = 256, uint32_t quantizationResolution1D = 8);
    void createVoxelGridGpu();
    void createVoxelGridCpu();
    void createLineHullMesh();

    // Get finished data.
    [[nodiscard]] inline const glm::mat4& getWorldToVoxelGridMatrix() const { return linesToVoxel; }
    [[nodiscard]] inline const glm::mat4& getVoxelGridToWorldMatrix() const { return voxelToLines; }
    [[nodiscard]] inline const glm::ivec3& getGridResolution() const { return gridResolution; }
    [[nodiscard]] inline const glm::uvec3& getQuantizationResolution() const { return quantizationResolution; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getVoxelGridLineSegmentOffsetsBuffer() const { return voxelGridLineSegmentOffsetsBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getVoxelGridNumLineSegmentsBuffer() const { return voxelGridNumLineSegmentsBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getVoxelGridLineSegmentsBuffer() const { return voxelGridLineSegmentsBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getLineHullIndexBuffer() const { return lineHullIndexBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getLineHullVertexBuffer() const { return lineHullVertexBuffer; }

private:
    // CPU grid generation utility functions.
    void nextStreamline(const Curve& line);
    void compressData();
    std::vector<VoxelDiscretizer*> getVoxelsInAABB(const sgl::AABB3& aabb);
    VoxelDiscretizer* voxels = nullptr;

    // Compression
    void quantizeLine(
            const glm::vec3& voxelPos, const LineSegment& line, LineSegmentQuantized& lineQuantized,
            int faceIndex1, int faceIndex2);
    void compressLine(const glm::ivec3& voxelIndex, const LineSegment& line, LineSegmentCompressed& lineCompressed);
    void quantizePoint(const glm::vec3& v, glm::ivec2& qv, int faceIndex);
    static int computeFaceIndex(const glm::vec3& v, const glm::ivec3& voxelIndex);

    // Test decompression
    [[nodiscard]] glm::vec3 getQuantizedPositionOffset(uint32_t faceIndex, uint32_t quantizedPos1D) const;
    void decompressLine(const glm::vec3& voxelPosition, const LineSegmentCompressed& compressedLine,
                        LineSegment& decompressedLine);
    bool checkLinesEqual(const LineSegment& originalLine, const LineSegment& decompressedLine);

    // Prefix sum data.
    sgl::vk::BufferPtr parallelPrefixSumReduce(uint32_t N, sgl::vk::BufferPtr& bufferIn);
    void parallelPrefixSumRecursive(uint32_t N, sgl::vk::BufferPtr& bufferIn, sgl::vk::BufferPtr& bufferOut);
    sgl::vk::ComputePipelinePtr prefixSumBlockIncrementPipeline;
    sgl::vk::ComputePipelinePtr prefixSumScanPipeline;
    sgl::vk::ComputeDataPtr prefixSumBlockIncrementData;
    sgl::vk::ComputeDataPtr prefixSumScanData;
    sgl::vk::ComputeDataPtr prefixSumWriteFinalElementData;
    std::vector<sgl::vk::ComputeDataPtr> parallelPrefixSumReduceCache;

    // Line hull mesh helpers.
    bool isVoxelFilled(const uint32_t* voxelGridNumLineSegmentsArray, int x, int y, int z) const;
    bool isVoxelFilledDilated(const uint32_t* voxelGridNumLineSegmentsArray, int x, int y, int z) const;

    // Line data.
    std::vector<Curve> curves;
    sgl::AABB3 linesBoundingBox{};
    glm::mat4 linesToVoxel{}, voxelToLines{};
    glm::ivec3 gridResolution{};
    glm::uvec3 quantizationResolution{};

    sgl::vk::Renderer* renderer = nullptr;
    sgl::vk::BufferPtr voxelGridLineSegmentOffsetsBuffer;
    sgl::vk::BufferPtr voxelGridNumLineSegmentsBuffer;
    sgl::vk::BufferPtr voxelGridLineSegmentsBuffer;
    sgl::vk::BufferPtr lineHullIndexBuffer;
    sgl::vk::BufferPtr lineHullVertexBuffer;
};

std::string ivec3ToString(const glm::ivec3& v);
std::string uvec3ToString(const glm::uvec3& v);

#endif //LINEVIS_VOXELCURVEDISCRETIZER_HPP
