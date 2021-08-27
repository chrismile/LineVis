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

#ifndef LINEVIS_VOXELDATA_HPP
#define LINEVIS_VOXELDATA_HPP

#include <vector>
#include <glm/glm.hpp>

#define PACK_LINES

struct Curve {
    std::vector<glm::vec3> points;
    std::vector<float> attributes;
    unsigned int lineID = 0;
};

struct LineSegment {
    LineSegment(const glm::vec3 &v1, float a1, const glm::vec3 &v2, float a2, unsigned int lineID)
            : v1(v1), a1(a1), v2(v2), a2(a2), lineID(lineID) {}
    LineSegment() : v1(0.0f), a1(0.0f), v2(0.0f), a2(0.0f), lineID(0) {}
    float length() const { return glm::length(v2 - v1); }

    glm::vec3 v1; // Vertex position
    float a1; // Vertex attribute
    glm::vec3 v2; // Vertex position
    float a2; // Vertex attribute
    unsigned int lineID;
    glm::vec3 padding{};
};

struct LineSegmentQuantized {
    uint8_t faceIndex1; // 3 bits
    uint8_t faceIndex2; // 3 bits
    uint32_t facePositionQuantized1; // 2*log2(QUANTIZATION_RESOLUTION) bits
    uint32_t facePositionQuantized2; // 2*log2(QUANTIZATION_RESOLUTION) bits
    unsigned int lineID; // 8 bits
    float a1; // Attribute 1
    float a2; // Attribute 2
};

struct LineSegmentCompressed {
    // Bit 0-2, 3-5: Face ID of start/end point.
    // For c = log2(QUANTIZATION_RESOLUTION^2) = 2*log2(QUANTIZATION_RESOLUTION):
    // Bit 6-(5+c), (6+c)-(5+2c): Quantized face position of start/end point.
    uint32_t linePosition;
    // Bit 11-15: Line ID (5 bits for bitmask of 2^5 bits = 32 bits).
    // Bit 16-23, 24-31: Attribute of start/end point (normalized to [0,1]).
    uint32_t attributes;
};

#endif //LINEVIS_VOXELDATA_HPP
