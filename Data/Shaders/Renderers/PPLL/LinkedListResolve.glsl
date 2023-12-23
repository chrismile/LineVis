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

#version 450 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = vec4(vertexPosition, 1.0);
}


-- Fragment

#version 450 core

#include "LinkedListHeader.glsl"

uint colorList[MAX_NUM_FRAGS];
float depthList[MAX_NUM_FRAGS];

#include "LinkedListSort.glsl"

#ifdef USE_QUICKSORT
#include "LinkedListQuicksort.glsl"
#endif

layout(location = 0) out vec4 fragColor;

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Get start offset from array
    uint fragOffset = startOffset[pixelIndex];

#ifdef INITIALIZE_ARRAY_POW2
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        colorList[i] = 0;
        depthList[i] = 0.0;
    }
#endif

    // Collect all fragments for this pixel
    int numFrags = 0;
    LinkedListFragmentNode fragment;
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        if (fragOffset == -1) {
            // End of list reached
            break;
        }

#if defined(FRAGMENT_BUFFER_REFERENCE_ARRAY)
        FragmentBufferEntry fbe = FragmentBufferEntry(
                fagmentBuffers[fragOffset / NUM_FRAGS_PER_BUFFER] + 12u * uint64_t(fragOffset % NUM_FRAGS_PER_BUFFER));
        fragment.color = fbe.color;
        fragment.depth = fbe.depth;
        fragment.next = fbe.next;
#elif defined(FRAGMENT_BUFFER_ARRAY)
        fragment = fragmentBuffers[nonuniformEXT(fragOffset / NUM_FRAGS_PER_BUFFER)].fragmentBuffer[fragOffset % NUM_FRAGS_PER_BUFFER];
#else
        fragment = fragmentBuffer[fragOffset];
#endif
        fragOffset = fragment.next;

        colorList[i] = fragment.color;
        depthList[i] = fragment.depth;

        numFrags++;
    }

    if (numFrags == 0) {
        discard;
    }

    fragColor = sortingAlgorithm(numFrags);
}
