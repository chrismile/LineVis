/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2023, Christoph Neuhauser
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

/**
 * Uses the "Atomic Loop 64-bit" idea from:
 * "Order Independent Transparency In OpenGL 4.x", Christoph Kubisch (2014).
 * https://on-demand.gputechconf.com/gtc/2014/presentations/S4385-order-independent-transparency-opengl.pdf
 */

#extension GL_EXT_control_flow_attributes : require
#extension GL_ARB_gpu_shader_int64 : require
#extension GL_EXT_shader_atomic_int64 : require

// Use early z-test to cull transparent fragments occluded by opaque fragments.
layout(early_fragment_tests) in;

layout(std430, binding = 0) coherent buffer FragmentNodes {
    uint64_t nodes[];
};

layout(binding = 1) uniform UniformDataBuffer {
    // Size of the viewport in x direction (in pixels).
    int viewportW;
};

#include "TiledAddress.glsl"

layout(location = 0) out vec4 fragColor;

void gatherFragmentCustomDepth(vec4 color, float depth) {
    if (color.a < 0.001) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }

    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    uint64_t valueInsert = packUint2x32(uvec2(packUnorm4x8(color), floatBitsToUint(depth)));

    uint i;
    [[unroll]] for (i = 0; i < MAX_NUM_LAYERS; i++) {
        uint64_t valueArray = atomicMin(nodes[pixelIndex * MAX_NUM_LAYERS + i], valueInsert);
        if (valueArray == packUint2x32(uvec2(0xFFFFFFFFu, 0xFFFFFFFFu))) {
            break;
        }
        if (valueArray > valueInsert) {
            valueInsert = valueArray;
        }
    }

    vec4 finalColor = vec4(0.0);
    if (i == MAX_NUM_LAYERS) {
        finalColor = unpackUnorm4x8(unpackUint2x32(valueInsert).x);
        //finalColor.rgb *= finalColor.a;
    }
    fragColor = finalColor;
}

void gatherFragment(vec4 color) {
    gatherFragmentCustomDepth(color, gl_FragCoord.z);
}
