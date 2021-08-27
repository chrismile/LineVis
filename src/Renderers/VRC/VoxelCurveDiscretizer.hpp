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
    const glm::uvec3& getIndex() const { return index; }
    float computeDensity();

    glm::uvec3 index;
    std::vector<LineSegment> lines;

    // For clipping lines to voxel
    std::vector<AttributePoint> currentCurveIntersections;
};

class VoxelCurveDiscretizer {
public:
    VoxelCurveDiscretizer();
    void loadLineData(LineDataPtr& lineData, uint32_t gridResolution1D = 256, uint32_t quantizationResolution1D = 8);
    void createVoxelGridGpu();
    void createVoxelGridCpu();

    // Get finished data.
    inline const glm::mat4& getWorldToVoxelGridMatrix() const { return linesToVoxel; }
    inline const glm::uvec3& getGridResolution() const { return gridResolution; }
    inline const glm::uvec3& getQuantizationResolution() const { return quantizationResolution; }
    inline const sgl::GeometryBufferPtr& getVoxelGridLineSegmentOffsetsBuffer() const { return voxelGridLineSegmentOffsetsBuffer; }
    inline const sgl::GeometryBufferPtr& getVoxelGridNumLineSegmentsBuffer() const { return voxelGridNumLineSegmentsBuffer; }
    inline const sgl::GeometryBufferPtr& getVoxelGridLineSegmentsBuffer() const { return voxelGridLineSegmentsBuffer; }

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
    glm::vec3 getQuantizedPositionOffset(uint32_t faceIndex, uint32_t quantizedPos1D) const;
    void decompressLine(const glm::vec3& voxelPosition, const LineSegmentCompressed& compressedLine,
                        LineSegment& decompressedLine);
    bool checkLinesEqual(const LineSegment& originalLine, const LineSegment& decompressedLine);

    // Prefix sum data.
    sgl::GeometryBufferPtr parallelPrefixSumReduce(uint32_t N, sgl::GeometryBufferPtr& bufferIn);
    void parallelPrefixSumRecursive(uint32_t N, sgl::GeometryBufferPtr& bufferIn, sgl::GeometryBufferPtr& bufferOut);
    sgl::ShaderProgramPtr prefixSumBlockIncrementShader;
    sgl::ShaderProgramPtr prefixSumScanShader;
    sgl::ShaderProgramPtr prefixSumWriteFinalElementShader;

    // Line data.
    std::vector<Curve> curves;
    sgl::AABB3 linesBoundingBox{};
    glm::mat4 linesToVoxel{}, voxelToLines{};
    glm::uvec3 gridResolution{};
    glm::uvec3 quantizationResolution{};

    sgl::GeometryBufferPtr voxelGridLineSegmentOffsetsBuffer;
    sgl::GeometryBufferPtr voxelGridNumLineSegmentsBuffer;
    sgl::GeometryBufferPtr voxelGridLineSegmentsBuffer;
};

std::string uvec3ToString(const glm::uvec3& v);

#endif //LINEVIS_VOXELCURVEDISCRETIZER_HPP
