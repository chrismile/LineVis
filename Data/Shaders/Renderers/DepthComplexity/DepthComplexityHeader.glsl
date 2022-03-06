/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2022, Christoph Neuhauser
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

// Use early z-test to cull transparent fragments occluded by opaque fragments.
layout(early_fragment_tests) in;

in vec4 gl_FragCoord;

layout(binding = 0) uniform UniformDataBuffer {
    int viewportW;
    int padding0;

    // The number of fragments necessary to reach the maximal color opacity.
    uint numFragmentsMaxColor;
    uint padding1;

    // The color to shade the framents with.
    vec4 color;
};

// Stores the number of fragments using atomic operations.
layout(binding = 1) coherent buffer FragmentCounterBuffer {
    uint fragmentCounterBuffer[];
};

//#include "TiledAddress.glsl"
uint addrGen(uvec2 addr2D) {
    return addr2D.x + viewportW * addr2D.y;
}
