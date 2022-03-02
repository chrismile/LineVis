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


#include "LinkedListHeader.glsl"

layout(location = 0) out vec4 fragColor;

void gatherFragment(vec4 color) {
    if (color.a < 0.001) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.depth = length(fragmentPositionWorld - cameraPosition);//gl_FragCoord.z;//
    frag.next = -1;

    uint insertIndex = atomicAdd(fragCounter, 1u);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    if (color.a < 0.001) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.depth = depth;
    frag.next = -1;

    uint insertIndex = atomicAdd(fragCounter, 1u);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}
