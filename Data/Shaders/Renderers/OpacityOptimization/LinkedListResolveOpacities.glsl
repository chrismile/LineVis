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

-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "LinkedListHeaderOpacities.glsl"
#include "FloatPack.glsl"

uint lineSegmentIdList[MAX_NUM_FRAGS];
//float attributeList[MAX_NUM_FRAGS];
uint depthList[MAX_NUM_FRAGS];

#define DEPTH_TYPE_UINT
#define colorList lineSegmentIdList
#include "LinkedListSortOpacities.glsl"

#ifdef USE_QUICKSORT
#include "LinkedListQuicksortOpacities.glsl"
#endif

layout (std430, binding = 3) buffer OpacityBufferUint {
    uint lineSegmentOpacities[];
};

layout (std430, binding = 4) readonly buffer LineSegmentVisibilityBuffer {
    uint lineSegmentVisibilityBuffer[];
};

out vec4 fragColor;

const float p = 1.0f; ///< Normalization of parameters.
uniform float q = 2000.0f; ///< Overall opacity, q >= 0.
uniform float r = 20.0f; ///< Clearing of background, r >= 0.
//uniform int s = 15; ///< Iterations for smoothing.
uniform float lambda = 2.0f; ///< Relaxation constant for smoothing, lambda > 0.

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Get start offset from array
    uint fragOffset = startOffset[pixelIndex];

    // Collect all fragments for this pixel
    int numFrags = 0;
    LinkedListFragmentNode fragment;
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        if (fragOffset == -1) {
            // End of list reached
            break;
        }

        fragment = fragmentBuffer[fragOffset];
        fragOffset = fragment.next;

        lineSegmentIdList[i] = fragment.lineSegmentId;
        depthList[i] = fragment.depth;

        numFrags++;
    }

    if (numFrags == 0) {
        return;
    }

    sortingAlgorithm(numFrags);

    // Algorithm 1 in "Decoupled Opacity Optimization for Points, Lines and Surfaces" (Günther et al. 2017).
    float g_all = 0.0f;
    float g_f = 0.0f;
    for (int i = 0; i < numFrags; i++) {
        float g_i = unpackFloat10(depthList[i]); // attributeList[i]
        g_all = g_all + g_i * g_i;
    }
    for (int i = 0; i < numFrags; i++) {
        float g_i = unpackFloat10(depthList[i]); // attributeList[i]
        float g_b = g_all - g_i * g_i - g_f;
        float alpha_i = p / (p + pow(1 - g_i, 2.0 * lambda) * (r * g_f + q * g_b));
        g_f = g_f + g_i * g_i;

        // Update minimum segment opacity using atomic operations.
        // GLSL (without using extensions) only supports atomic operations for integer values.
        uint alpha_i_uint = convertNormalizedFloatToUint32(alpha_i);
        uint lineSegmentId = lineSegmentIdList[i];
        atomicMin(lineSegmentOpacities[lineSegmentId], alpha_i_uint);
        //atomicMin(lineSegmentOpacities[lineSegmentId], lineSegmentId % 2 == 0 ? 0x0u : 0xFFFFFFFFu);
        atomicExchange(lineSegmentVisibilityBuffer[lineSegmentId], 1u);
    }
}
