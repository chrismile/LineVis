/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2022, Christoph Neuhauser, Michael Kern
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

#if defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK) && !defined(RESOLVE_PASS)
// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_fragment_shader_interlock.txt
//#extension GL_ARB_fragment_shader_interlock : require // Set in code using __extensions.
#ifdef PIXEL_SYNC_UNORDERED
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_unordered) in;
#else
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_ordered) in;
#endif
#else
// Use early z-test to cull transparent fragments occluded by opaque fragments.
layout(early_fragment_tests) in;
#endif

#if defined(USE_SYNC_SPINLOCK)
// Viewport-sized spinlock buffer.
// 0 means pixel is unlocked, and 1 means pixel is locked by a fragment shader invocation.
layout (std430, binding = 0) coherent buffer SpinlockViewportBuffer {
    uint spinlockViewportBuffer[];
};
#endif

in vec4 gl_FragCoord;

layout(binding = 1) uniform UniformDataBuffer {
    // Size of the viewport in x direction (in pixels).
    int viewportW;
};

layout(binding = 2) uniform UniformBucketDataBuffer {
    // Range of logarithmic depth.
    float logDepthMin;
    float logDepthMax;

    float lowerBackBufferOpacity; // default 0.25
    float upperBackBufferOpacity; // default 0.98
};

struct MinDepthNode {
    float minDepth;
    float minOpaqueDepth;
};

layout (std430, binding = 3) coherent buffer MinDepthBuffer {
    MinDepthNode depthBuffer[];
};

#include "TiledAddress.glsl"

// Maps depth to range [0,1] with logarithmic scale.
float logDepthWarp(float z) {
    return (log(z) - logDepthMin) / (logDepthMax - logDepthMin);
    //return (z - exp(logmin)) / (exp(logmax) - exp(logmin));
}

void gatherFragment(vec4 color) {
    ivec2 fragPos2D = ivec2(int(gl_FragCoord.x), int(gl_FragCoord.y));
    uint pixelIndex = addrGen(uvec2(fragPos2D));

    float depthLog = logDepthWarp(-screenSpacePosition.z);

    MinDepthNode depthInfo = depthBuffer[pixelIndex];

    MinDepthNode newDepthInfo;
    newDepthInfo.minDepth = depthInfo.minDepth;
    newDepthInfo.minOpaqueDepth = depthInfo.minOpaqueDepth;

    // estimate boundary of front buffer
    if (color.a > lowerBackBufferOpacity && depthLog < depthInfo.minDepth) {
        newDepthInfo.minDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }

    // estimate boundary of back buffer
    if (color.a >= upperBackBufferOpacity && depthLog < depthInfo.minOpaqueDepth) {
        newDepthInfo.minOpaqueDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    ivec2 fragPos2D = ivec2(int(gl_FragCoord.x), int(gl_FragCoord.y));
    uint pixelIndex = addrGen(uvec2(fragPos2D));

    float depthLog = logDepthWarp(-depth);

    MinDepthNode depthInfo = depthBuffer[pixelIndex];

    MinDepthNode newDepthInfo;
    newDepthInfo.minDepth = depthInfo.minDepth;
    newDepthInfo.minOpaqueDepth = depthInfo.minOpaqueDepth;

    // estimate boundary of front buffer
    if (color.a > lowerBackBufferOpacity && depthLog < depthInfo.minDepth) {
        newDepthInfo.minDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }

    // estimate boundary of back buffer
    if (color.a >= upperBackBufferOpacity && depthLog < depthInfo.minOpaqueDepth) {
        newDepthInfo.minOpaqueDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }
}
