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

// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_shader_image_load_store.txt
#extension GL_ARB_shader_image_load_store : require

// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests) in;

in vec4 gl_FragCoord;

// A fragment node stores rendering information about one specific fragment
struct LinkedListFragmentNode {
    // RGBA color of the node
    uint color;
    // Depth value of the fragment (in view space)
    float depth;
    // The index of the next node in "nodes" array
    uint next;
};

layout(binding = 0) uniform UniformDataBuffer {
    // Number of fragments we can store in total.
    uint linkedListSize;
    // Size of the viewport in x direction (in pixels).
    int viewportW;
};

// Fragment-and-link buffer (linked list). Stores "nodesPerPixel" number of fragments.
layout(std430, binding = 1) buffer FragmentBuffer {
    LinkedListFragmentNode fragmentBuffer[];
};

// Start-offset buffer (mapping pixels to first pixel in the buffer) of size viewportSize.x * viewportSize.y.
layout(std430, binding = 2) coherent buffer StartOffsetBuffer {
    uint startOffset[];
};

// Position of the first free fragment node in the linked list.
layout(std430, binding = 3) buffer FragCounterBuffer {
    uint fragCounter;
};

#include "TiledAddress.glsl"
