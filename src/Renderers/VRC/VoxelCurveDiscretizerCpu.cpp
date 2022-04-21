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

#include <unordered_set>
#include <chrono>

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>

#include "VoxelCurveDiscretizer.hpp"

#define BIAS 0.001

/**
 * Helper function for rayBoxIntersection (see below).
 */
bool rayBoxPlaneIntersection(
        float rayOriginX, float rayDirectionX, float lowerX, float upperX, float& tNear, float& tFar) {
    if (std::abs(rayDirectionX) < BIAS) {
        // Ray is parallel to the x planes
        if (rayOriginX < lowerX || rayOriginX > upperX) {
            return false;
        }
    } else {
        // Not parallel to the x planes. Compute the intersection distance to the planes.
        float t0 = (lowerX - rayOriginX) / rayDirectionX;
        float t1 = (upperX - rayOriginX) / rayDirectionX;
        if (t0 > t1) {
            // Since t0 intersection with near plane
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }

        if (t0 > tNear) {
            // We want the largest tNear
            tNear = t0;
        }
        if (t1 < tFar) {
            // We want the smallest tFar
            tFar = t1;
        }
        if (tNear > tFar) {
            // Box is missed
            return false;
        }
        if (tFar < 0) {
            // Box is behind ray
            return false;
        }
    }
    return true;
}

/**
 * Implementation of ray-box intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter3.htm
 */
bool rayBoxIntersection(
        const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& lower, const glm::vec3& upper,
        float& tNear, float& tFar) {
    tNear = -1e7;
    tFar = 1e7;
    for (int i = 0; i < 3; i++) {
        if (!rayBoxPlaneIntersection(rayOrigin[i], rayDirection[i], lower[i], upper[i], tNear, tFar)) {
            return false;
        }
    }

    //entrancePoint = rayOrigin + tNear * rayDirection;
    //exitPoint = rayOrigin + tFar * rayDirection;
    return true;
}

bool VoxelDiscretizer::addPossibleIntersections(const glm::vec3& v1, const glm::vec3& v2, float a1, float a2) {
    float tNear, tFar;
    glm::vec3 voxelLower = glm::vec3(index);
    glm::vec3 voxelUpper = glm::vec3(index + glm::uvec3(1, 1, 1));
    if (rayBoxIntersection(v1, (v2 - v1), voxelLower, voxelUpper, tNear, tFar)) {
        bool intersectionNear = 0.0f <= tNear && tNear <= 1.0f;
        bool intersectionFar = 0.0f <= tFar && tFar <= 1.0f;
        if (intersectionNear) {
            glm::vec3 entrancePoint = v1 + tNear * (v2 - v1);
            float interpolatedAttribute = a1 + tNear * (a2 - a1);
            currentCurveIntersections.emplace_back(entrancePoint, interpolatedAttribute);
        }
        if (intersectionFar) {
            glm::vec3 exitPoint = v1 + tFar * (v2 - v1);
            float interpolatedAttribute = a1 + tFar * (a2 - a1);
            currentCurveIntersections.emplace_back(exitPoint, interpolatedAttribute);
        }
        if (intersectionNear || intersectionFar) {
            return true; // Intersection found
        }
    }

    return false;
}

void VoxelDiscretizer::setIndex(glm::uvec3 index) {
    this->index = index;
}

float VoxelDiscretizer::computeDensity() {
    float density = 0.0f;
    for (LineSegment& line : lines) {
        density += line.length(); // * line.avgOpacity();
    }
    return density;
}


void VoxelCurveDiscretizer::createVoxelGridCpu() {
    auto startVoxelizeLines = std::chrono::system_clock::now();

    voxels = new VoxelDiscretizer[gridResolution.x * gridResolution.y * gridResolution.z];
    for (uint32_t z = 0; z < uint32_t(gridResolution.z); z++) {
        for (uint32_t y = 0; y < uint32_t(gridResolution.y); y++) {
            for (uint32_t x = 0; x < uint32_t(gridResolution.x); x++) {
                uint32_t index = x + y * gridResolution.x + z * gridResolution.x * gridResolution.y;
                voxels[index].setIndex(glm::uvec3(x, y, z));
            }
        }
    }

    // Insert lines into voxel representation
    //int lineNum = 0;
    for (const Curve& curve : curves) {
        nextStreamline(curve);
    }
    compressData();

    delete[] voxels;
    voxels = nullptr;

    auto endVoxelizeLines = std::chrono::system_clock::now();
    auto elapsedVoxelizeLines = std::chrono::duration_cast<std::chrono::milliseconds>(
            endVoxelizeLines - startVoxelizeLines);
    sgl::Logfile::get()->writeInfo(
            "Computational time to voxelize all lines: "
            + std::to_string(elapsedVoxelizeLines.count()) + "ms");
}

void VoxelCurveDiscretizer::nextStreamline(const Curve& line) {
    int N = int(line.points.size());

    // Add intersections to voxels.
    std::unordered_set<VoxelDiscretizer*> usedVoxels;
    for (int i = 0; i < N - 1; i++) {
        // Get line segment
        glm::vec3 v1 = line.points.at(i);
        glm::vec3 v2 = line.points.at(i+1);
        float a1 = line.attributes.at(i);
        float a2 = line.attributes.at(i+1);

        // Remove invalid line points (large values are used in some scientific datasets to indicate invalid lines).
        const float MAX_VAL = 1e10;
        if (std::fabs(v1.x) > MAX_VAL || std::fabs(v1.y) > MAX_VAL || std::fabs(v1.z) > MAX_VAL
            || std::fabs(v2.x) > MAX_VAL || std::fabs(v2.y) > MAX_VAL || std::fabs(v2.z) > MAX_VAL) {
            continue;
        }

        // Compute AABB of current segment.
        sgl::AABB3 segmentAABB = sgl::AABB3();
        segmentAABB.combine(v1);
        segmentAABB.combine(v2);

        // Iterate over all voxels with possible intersections
        std::vector<VoxelDiscretizer*> voxelsInAABB = getVoxelsInAABB(segmentAABB);

        for (VoxelDiscretizer *voxel : voxelsInAABB) {
            // Line-voxel intersection
            if (voxel->addPossibleIntersections(v1, v2, a1, a2)) {
                // Intersection(s) added to "currentLineIntersections", voxel used
                usedVoxels.insert(voxel);
            }
        }
    }

    // Convert intersections to clipped line segments
    for (VoxelDiscretizer *voxel : usedVoxels) {
        if (voxel->currentCurveIntersections.size() < 2) {
            voxel->currentCurveIntersections.clear();
            continue;
        }
        auto it1 = voxel->currentCurveIntersections.begin();
        auto it2 = voxel->currentCurveIntersections.begin();
        it2++;
        while (it2 != voxel->currentCurveIntersections.end()) {
            voxel->lines.emplace_back(it1->v, it1->a, it2->v, it2->a, line.lineID);
            it1++; it1++;
            if (it1 == voxel->currentCurveIntersections.end()) break;
            it2++; it2++;
        }
        voxel->currentCurveIntersections.clear();
    }
}

void VoxelCurveDiscretizer::compressData() {
    uint32_t n = gridResolution.x * gridResolution.y * gridResolution.z;
    std::vector<float> voxelDensities;
    std::vector<uint32_t> usedVoxels;
    voxelDensities.reserve(n);
    usedVoxels.reserve(n);

    uint32_t lineOffset = 0;
#ifdef PACK_LINES
    std::vector<LineSegmentCompressed> lineSegments;
#else
    std::vector<LineSegment> lineSegments;
#endif
    std::vector<uint32_t> voxelLineListOffsets;
    std::vector<uint32_t> numLinesInVoxel;
    voxelLineListOffsets.reserve(n);
    numLinesInVoxel.reserve(n);

    for (uint32_t i = 0; i < n; i++) {
        voxelLineListOffsets.push_back(lineOffset);
        uint32_t numLines = uint32_t(voxels[i].lines.size());
        numLinesInVoxel.push_back(numLines);
        voxelDensities.push_back(voxels[i].computeDensity());
        usedVoxels.push_back(numLines > 0 ? 1 : 0);

#ifdef PACK_LINES
        std::vector<LineSegmentCompressed> lineSegmentsLocal;
        lineSegmentsLocal.resize(voxels[i].lines.size());
        for (size_t j = 0; j < voxels[i].lines.size(); j++) {
            compressLine(voxels[i].getIndex(), voxels[i].lines[j], lineSegmentsLocal[j]);

            // Test
            /*LineSegment originalLine = voxels[i].lines[j];
            LineSegment decompressedLine;
            decompressLine(glm::vec3(voxels[i].getIndex()), lineSegments[j], decompressedLine);
            if (!checkLinesEqual(originalLine, decompressedLine)) {
                compressLine(voxels[i].getIndex(), voxels[i].lines[j], lineSegments[j]);
                decompressLine(glm::vec3(voxels[i].getIndex()), lineSegments[j], decompressedLine);
            }*/
        }
        lineSegments.insert(lineSegments.end(), lineSegmentsLocal.begin(), lineSegmentsLocal.end());
#else
        lineSegments.insert(lineSegments.end(), voxels[i].lines.begin(), voxels[i].lines.end());
#endif

        lineOffset += numLines;
    }

    voxelGridLineSegmentOffsetsBuffer = {};
    voxelGridNumLineSegmentsBuffer = {};
    voxelGridLineSegmentsBuffer = {};

    if (lineSegments.empty()) {
        return;
    }

    //std::vector<float> voxelAOFactors;
    //voxelAOFactors.resize(n);
    //generateVoxelAOFactorsFromDensity(voxelDensities, voxelAOFactors, gridResolution, isHairDataset);

    // Upload to GPU.
    voxelGridLineSegmentOffsetsBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t) * voxelLineListOffsets.size(), voxelLineListOffsets.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    voxelGridNumLineSegmentsBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t) * numLinesInVoxel.size(), numLinesInVoxel.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    //densityTexture = generateDensityTexture(compressedData.voxelDensities, gpuData.gridResolution);
    //aoTexture = generateDensityTexture(compressedData.voxelAOFactors, gpuData.gridResolution);

#ifdef PACK_LINES
    size_t baseSize = sizeof(LineSegmentCompressed);
#else
    size_t baseSize = sizeof(LineSegment);
#endif

    sgl::Logfile::get()->writeInfo(
            "Total number of voxelized line segments: " + std::to_string(lineSegments.size()));

    voxelGridLineSegmentsBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), baseSize * lineSegments.size(), lineSegments.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

std::vector<VoxelDiscretizer*> VoxelCurveDiscretizer::getVoxelsInAABB(const sgl::AABB3& aabb) {
    std::vector<VoxelDiscretizer*> voxelsInAABB;
    glm::vec3 minimum = aabb.getMinimum();
    glm::vec3 maximum = aabb.getMaximum();

    glm::ivec3 lower = glm::ivec3(minimum); // Round down
    glm::ivec3 upper = glm::ivec3(ceil(maximum.x), ceil(maximum.y), ceil(maximum.z)); // Round up
    lower = glm::clamp(lower, glm::ivec3(0), gridResolution - glm::ivec3(1));
    upper = glm::clamp(upper, glm::ivec3(0), gridResolution - glm::ivec3(1));

    for (uint32_t z = lower.z; z <= uint32_t(upper.z); z++) {
        for (uint32_t y = lower.y; y <= uint32_t(upper.y); y++) {
            for (uint32_t x = lower.x; x <= uint32_t(upper.x); x++) {
                uint32_t index = x + y*gridResolution.x + z*gridResolution.x*gridResolution.y;
                voxelsInAABB.push_back(voxels + index);
            }
        }
    }
    return voxelsInAABB;
}

void VoxelCurveDiscretizer::quantizeLine(
        const glm::vec3& voxelPos, const LineSegment& line, LineSegmentQuantized& lineQuantized,
        int faceIndex1, int faceIndex2) {
    lineQuantized.a1 = line.a1;
    lineQuantized.a2 = line.a2;

    glm::ivec2 facePosition3D1, facePosition3D2;
    quantizePoint(line.v1 - voxelPos, facePosition3D1, faceIndex1);
    quantizePoint(line.v2 - voxelPos, facePosition3D2, faceIndex2);
    lineQuantized.lineID = line.lineID;
    lineQuantized.faceIndex1 = faceIndex1;
    lineQuantized.faceIndex2 = faceIndex2;
    lineQuantized.facePositionQuantized1 = facePosition3D1.x + facePosition3D1.y*quantizationResolution.x;
    lineQuantized.facePositionQuantized2 = facePosition3D2.x + facePosition3D2.y*quantizationResolution.x;
}

int intlog2(int x) {
    int exponent = 0;
    while (x > 1) {
        x /= 2;
        exponent++;
    }
    return exponent;
}

void VoxelCurveDiscretizer::compressLine(
        const glm::ivec3& voxelIndex, const LineSegment& line, LineSegmentCompressed& lineCompressed) {
    LineSegmentQuantized lineQuantized;
    int faceIndex1 = computeFaceIndex(line.v1, voxelIndex);
    int faceIndex2 = computeFaceIndex(line.v2, voxelIndex);
    quantizeLine(glm::vec3(voxelIndex), line, lineQuantized, faceIndex1, faceIndex2);

    uint8_t attr1Unorm = uint8_t(glm::clamp(std::round(lineQuantized.a1*255.0f), 0.0f, 255.0f));
    uint8_t attr2Unorm = uint8_t(glm::clamp(std::round(lineQuantized.a2*255.0f), 0.0f, 255.0f));

    int c = int(std::round(2 * intlog2(quantizationResolution.x)));
    lineCompressed.linePosition = lineQuantized.faceIndex1;
    lineCompressed.linePosition |= lineQuantized.faceIndex2 << 3;
    lineCompressed.linePosition |= lineQuantized.facePositionQuantized1 << 6;
    lineCompressed.linePosition |= lineQuantized.facePositionQuantized2 << (6 + c);
    lineCompressed.attributes = 0;
    if (c > 12) {
        // Quantization resolution of 128 or 256
        lineCompressed.attributes |= lineQuantized.facePositionQuantized2 >> (c - (6 + 2*c - 32));
    }
    lineCompressed.attributes |= (lineQuantized.lineID & 31u) << 11;
    lineCompressed.attributes |= attr1Unorm << 16;
    lineCompressed.attributes |= attr2Unorm << 24;
}

void VoxelCurveDiscretizer::quantizePoint(const glm::vec3& v, glm::ivec2& qv, int faceIndex) {
    int dimensions[2];
    if (faceIndex == 0 || faceIndex == 1) {
        // x face
        dimensions[0] = 1;
        dimensions[1] = 2;
    } else if (faceIndex == 2 || faceIndex == 3) {
        // y face
        dimensions[0] = 0;
        dimensions[1] = 2;
    } else {
        // z face
        dimensions[0] = 0;
        dimensions[1] = 1;
    }

    // Iterate over all dimensions
    for (int i = 0; i < 2; i++) {
        uint32_t quantizationPos = uint32_t(std::floor(v[dimensions[i]] * quantizationResolution[dimensions[i]]));
        qv[i] = glm::clamp(quantizationPos, 0u, quantizationResolution[dimensions[i]]-1);
    }
}

int VoxelCurveDiscretizer::computeFaceIndex(const glm::vec3& v, const glm::ivec3& voxelIndex) {
    glm::ivec3 lower = voxelIndex, upper = voxelIndex + glm::ivec3(1);
    for (int i = 0; i < 3; i++) {
        if (std::abs(v[i] - float(lower[i])) < 1e-5f) {
            return 2*i;
        }
        if (std::abs(v[i] - float(upper[i])) < 1e-5f) {
            return 2*i+1;
        }
    }
    sgl::Logfile::get()->writeError("Error in VoxelCurveDiscretizer::computeFaceIndex: Invalid position.");
    return 0;
}

glm::vec3 VoxelCurveDiscretizer::getQuantizedPositionOffset(uint32_t faceIndex, uint32_t quantizedPos1D) const {
    glm::vec2 quantizedFacePosition = glm::vec2(
            float(quantizedPos1D % quantizationResolution.x),
            float(quantizedPos1D / quantizationResolution.x)) / float(quantizationResolution.x);

    // Whether the face is the face in x/y/z direction with greater dimensions (offset factor)
    float face0or1 = float(faceIndex % 2);

    glm::vec3 offset;
    if (faceIndex <= 1) {
        offset = glm::vec3(face0or1, quantizedFacePosition.x, quantizedFacePosition.y);
    } else if (faceIndex <= 3) {
        offset = glm::vec3(quantizedFacePosition.x, face0or1, quantizedFacePosition.y);
    } else if (faceIndex <= 5) {
        offset = glm::vec3(quantizedFacePosition.x, quantizedFacePosition.y, face0or1);
    }
    return offset;
}


void VoxelCurveDiscretizer::decompressLine(
        const glm::vec3& voxelPosition, const LineSegmentCompressed& compressedLine, LineSegment& decompressedLine) {
    const uint32_t c = 2 * intlog2(int(quantizationResolution.x));
    const uint32_t bitmaskQuantizedPos = quantizationResolution.x*quantizationResolution.x-1;
    uint32_t faceStartIndex = compressedLine.linePosition & 0x7u;
    uint32_t faceEndIndex = (compressedLine.linePosition >> 3) & 0x7u;
    uint32_t quantizedStartPos1D = (compressedLine.linePosition >> 6) & bitmaskQuantizedPos;
    uint32_t quantizedEndPos1D = (compressedLine.linePosition >> (6+c)) & bitmaskQuantizedPos;
    if (c > 12) {
        quantizedEndPos1D |= (compressedLine.attributes << (c - (6 + 2*c - 32))) & bitmaskQuantizedPos;
    }
    uint32_t lineID = (compressedLine.attributes >> 11) & 31u;
    uint32_t attr1 = (compressedLine.attributes >> 16) & 0xFFu;
    uint32_t attr2 = (compressedLine.attributes >> 24) & 0xFFu;

    decompressedLine.v1 = voxelPosition + getQuantizedPositionOffset(faceStartIndex, quantizedStartPos1D);
    decompressedLine.v2 = voxelPosition + getQuantizedPositionOffset(faceEndIndex, quantizedEndPos1D);
    decompressedLine.a1 = float(attr1) / 255.0f;
    decompressedLine.a2 = float(attr2) / 255.0f;
    decompressedLine.lineID = lineID;
}

bool VoxelCurveDiscretizer::checkLinesEqual(const LineSegment& originalLine, const LineSegment& decompressedLine) {
    bool linesEqual = true;
    if (originalLine.lineID % 256 != decompressedLine.lineID) {
        linesEqual = false;
        sgl::Logfile::get()->writeError("Mismatch in VoxelCurveDiscretizer::checkLinesEqual: lineID");
    }

    if (std::abs(originalLine.a1 - decompressedLine.a1) > 0.01f
        || std::abs(originalLine.a2 - decompressedLine.a2) > 0.01f) {
        linesEqual = false;
        sgl::Logfile::get()->writeError("Mismatch in VoxelCurveDiscretizer::checkLinesEqual: attribute");
    }

    if (glm::length(originalLine.v1 - decompressedLine.v1) > 0.5f) {
        linesEqual = false;
        sgl::Logfile::get()->writeError(
                "Mismatch in VoxelCurveDiscretizer::checkLinesEqual: position2, error: "
                + sgl::toString(glm::length(originalLine.v2 - decompressedLine.v2)));
    }

    if (glm::length(originalLine.v2 - decompressedLine.v2) > 0.5f) {
        linesEqual = false;
        sgl::Logfile::get()->writeError(
                "Mismatch in VoxelCurveDiscretizer::checkLinesEqual: position1, error: "
                + sgl::toString(glm::length(originalLine.v2 - decompressedLine.v2)));
    }

    return linesEqual;
}



bool VoxelCurveDiscretizer::isVoxelFilled(
        const uint32_t* voxelGridNumLineSegmentsArray, int x, int y, int z) const {
    if (x < 0 || y < 0 || z < 0 || x >= gridResolution.x || y >= gridResolution.y || z >= gridResolution.z) {
        return false;
    }
    return voxelGridNumLineSegmentsArray[x + y * gridResolution.x + z * gridResolution.x * gridResolution.y] > 0;
}

bool VoxelCurveDiscretizer::isVoxelFilledDilated(
        const uint32_t* voxelGridNumLineSegmentsArray, int x, int y, int z) const {
    if (x < -1 || y < -1 || z < -1 || x > gridResolution.x || y > gridResolution.y || z > gridResolution.z) {
        return false;
    }
    return voxelGridNumLineSegmentsArray[
            (x + 1)
            + (y + 1) * (gridResolution.x + 2)
            + (z + 1) * (gridResolution.x + 2) * (gridResolution.y + 2)] > 0;
}

void VoxelCurveDiscretizer::createLineHullMesh() {
    lineHullIndexBuffer = {};
    lineHullVertexBuffer = {};

    if (!voxelGridNumLineSegmentsBuffer) {
        return;
    }

    std::vector<uint32_t> lineHullIndices;
    std::vector<glm::vec3> lineHullVertices;

    auto* voxelGridNumLineSegmentsArray = new uint32_t[gridResolution.x * gridResolution.y * gridResolution.z];
    auto* voxelGridDilatedNumSegmentsArray = new uint32_t[
            (gridResolution.x + 2) * (gridResolution.y + 2) * (gridResolution.z + 2)];

    sgl::vk::BufferPtr stagingBuffer(new sgl::vk::Buffer(
            renderer->getDevice(), voxelGridNumLineSegmentsBuffer->getSizeInBytes(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_TO_CPU));

    VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands();
    voxelGridNumLineSegmentsBuffer->copyDataTo(stagingBuffer, commandBuffer);
    renderer->getDevice()->endSingleTimeCommands(commandBuffer);

    void* mappedData = stagingBuffer->mapMemory();
    memcpy(voxelGridNumLineSegmentsArray, mappedData, voxelGridNumLineSegmentsBuffer->getSizeInBytes());
    stagingBuffer->unmapMemory();

    // Dilate the grid.
    for (int z = -1; z <= gridResolution.z; z++) {
        for (int y = -1; y <= gridResolution.y; y++) {
            for (int x = -1; x <= gridResolution.x; x++) {
                uint32_t isFilled = 0;
                for (int dz = -1; dz <= 1; dz++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (isVoxelFilled(voxelGridNumLineSegmentsArray, x + dx, y + dy, z + dz)) {
                                isFilled = 1;
                            }
                        }
                    }
                }
                voxelGridDilatedNumSegmentsArray[
                        (x + 1)
                        + (y + 1) * (gridResolution.x + 2)
                        + (z + 1) * (gridResolution.x + 2) * (gridResolution.y + 2)] = isFilled;
            }
        }
    }

    uint32_t indexOffset = 0;

    for (int z = -1; z <= gridResolution.z + 1; z++) {
        for (int y = -1; y <= gridResolution.y + 1; y++) {
            for (int x = -1; x <= gridResolution.x + 1; x++) {
                bool linesSetThis = isVoxelFilledDilated(voxelGridDilatedNumSegmentsArray, x, y, z);
                bool linesSetLeft = isVoxelFilledDilated(voxelGridDilatedNumSegmentsArray, x - 1, y, z);
                bool linesSetBottom = isVoxelFilledDilated(voxelGridDilatedNumSegmentsArray, x, y - 1, z);
                bool linesSetBehind = isVoxelFilledDilated(voxelGridDilatedNumSegmentsArray, x, y, z - 1);

                if (linesSetThis != linesSetLeft) {
                    lineHullVertices.emplace_back(x, y, z);
                    lineHullVertices.emplace_back(x, y + 1, z);
                    lineHullVertices.emplace_back(x, y + 1, z + 1);
                    lineHullVertices.emplace_back(x, y, z + 1);
                    lineHullIndices.push_back(indexOffset + 0);
                    lineHullIndices.push_back(indexOffset + 1);
                    lineHullIndices.push_back(indexOffset + 2);
                    lineHullIndices.push_back(indexOffset + 0);
                    lineHullIndices.push_back(indexOffset + 2);
                    lineHullIndices.push_back(indexOffset + 3);
                    indexOffset += 4;
                }
                if (linesSetThis != linesSetBottom) {
                    lineHullVertices.emplace_back(x, y, z);
                    lineHullVertices.emplace_back(x + 1, y, z);
                    lineHullVertices.emplace_back(x + 1, y, z + 1);
                    lineHullVertices.emplace_back(x, y, z + 1);
                    lineHullIndices.push_back(indexOffset + 0);
                    lineHullIndices.push_back(indexOffset + 1);
                    lineHullIndices.push_back(indexOffset + 2);
                    lineHullIndices.push_back(indexOffset + 0);
                    lineHullIndices.push_back(indexOffset + 2);
                    lineHullIndices.push_back(indexOffset + 3);
                    indexOffset += 4;
                }
                if (linesSetThis != linesSetBehind) {
                    lineHullVertices.emplace_back(x, y, z);
                    lineHullVertices.emplace_back(x, y + 1, z);
                    lineHullVertices.emplace_back(x + 1, y + 1, z);
                    lineHullVertices.emplace_back(x + 1, y, z);
                    lineHullIndices.push_back(indexOffset + 0);
                    lineHullIndices.push_back(indexOffset + 1);
                    lineHullIndices.push_back(indexOffset + 2);
                    lineHullIndices.push_back(indexOffset + 0);
                    lineHullIndices.push_back(indexOffset + 2);
                    lineHullIndices.push_back(indexOffset + 3);
                    indexOffset += 4;
                }
            }
        }
    }

    delete[] voxelGridNumLineSegmentsArray;
    delete[] voxelGridDilatedNumSegmentsArray;

    if (lineHullIndices.empty() || lineHullVertices.empty()) {
        return;
    }

    lineHullIndexBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t) * lineHullIndices.size(), lineHullIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    lineHullVertexBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(glm::vec3) * lineHullVertices.size(), lineHullVertices.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}
