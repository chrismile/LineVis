/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2021, Christoph Neuhauser
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

-- Compute

#version 450 core

layout (local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout (std430, binding = 2) readonly buffer OpacityBufferIn {
    float opacityBufferIn[];
};

layout (std430, binding = 3) writeonly buffer OpacityBufferOut {
    float opacityBufferOut[];
};

layout (std430, binding = 4) readonly buffer LineSegmentVisibilityBuffer {
    uint lineSegmentVisibilityBuffer[];
};

layout (std430, binding = 5) readonly buffer LineSegmentConnectivityBuffer {
    // Indices of left and right neighbors (or index of element itself if no left/right neighbor).
    uvec2 lineSegmentConnectivityBuffer[];
};

uniform uint numLineSegments; ///< Number of entries in opacityBufferIn/opacityBufferOut.
uniform float relaxationConstant;

/**
 * Computes (see http://graphics.stanford.edu/courses/cs468-12-spring/LectureSlides/06_smoothing.pdf):
 * p_i = p_i + lamba * L(i)
 * L(i) = 0.5 * p_{i-1} - p_i + 0.5 * p_{i+1}
 */
void main() {
    if (gl_GlobalInvocationID.x >= numLineSegments) {
        return;
    }

    uvec2 neighborIndices = lineSegmentConnectivityBuffer[gl_GlobalInvocationID.x];
    float ownValue = opacityBufferIn[gl_GlobalInvocationID.x];
    float neighborLeft = opacityBufferIn[neighborIndices.x];
    float neighborRight = opacityBufferIn[neighborIndices.y];
    uint visibleLeft = lineSegmentVisibilityBuffer[neighborIndices.x];
    uint visibleRight = lineSegmentVisibilityBuffer[neighborIndices.y];
    if (visibleLeft == 0) {
        neighborLeft = ownValue;
    }
    if (visibleRight == 0) {
        neighborRight = ownValue;
    }

    float outputValue = -ownValue;
    outputValue += 0.5 * neighborLeft;
    outputValue += 0.5 * neighborRight;
    outputValue = ownValue + relaxationConstant * outputValue;
    opacityBufferOut[gl_GlobalInvocationID.x] = outputValue;
}
