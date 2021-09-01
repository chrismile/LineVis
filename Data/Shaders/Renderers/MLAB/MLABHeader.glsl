/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2021, Christoph Neuhauser
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

// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_fragment_shader_interlock.txt
#extension GL_ARB_fragment_shader_interlock : require

#pragma optionNV (unroll all)

#if defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK) && defined(INTERLOCK_UNORDERED)
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_unordered) in;
#elif defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK)
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_ordered) in;
#endif

#if defined(USE_SYNC_SPINLOCK)
layout(early_fragment_tests, pixel_interlock_ordered) in;
// Viewport-sized spinlock buffer.
// 0 means pixel is unlocked, and 1 means pixel is locked by a fragment shader invocation.
layout (std430, binding = 1) coherent buffer SpinlockViewportBuffer {
    uint spinlockViewportBuffer[];
};

#endif

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

uniform int viewportW;

// Distance of infinitely far away fragments (used for initialization)
#define DISTANCE_INFINITE (1E30)

// Data structure for MAX_NUM_LAYERS nodes, packed in vectors for faster access
#if MAX_NUM_LAYERS == 1
struct MLABFragmentNode_compressed {
    // Linear depth, i.e. distance to viewer
    float depth[1];
    // RGB color (3 bytes), opacity (1 byte)
    uint premulColor[1];
};
#elif MAX_NUM_LAYERS == 2
struct MLABFragmentNode_compressed {
    // Linear depth, i.e. distance to viewer
    vec2 depth;
    // RGB color (3 bytes), opacity (1 byte)
    uvec2 premulColor;
};
#elif MAX_NUM_LAYERS == 4
struct MLABFragmentNode_compressed {
    // Linear depth, i.e. distance to viewer
    vec4 depth;
    // RGB color (3 bytes), opacity (1 byte)
    uvec4 premulColor;
};
#elif MAX_NUM_LAYERS % 4 == 0
struct MLABFragmentNode_compressed {
    // Linear depth, i.e. distance to viewer
    vec4 depth[MAX_NUM_LAYERS/4];
    // RGB color (3 bytes), opacity (1 byte)
    uvec4 premulColor[MAX_NUM_LAYERS/4];
};
#else
struct MLABFragmentNode_compressed {
    // Linear depth, i.e. distance to viewer
    float depth[MAX_NUM_LAYERS];
    // RGB color (3 bytes), opacity (1 byte)
    uint premulColor[MAX_NUM_LAYERS];
};
#endif

struct MLABFragmentNode {
    // Linear depth, i.e. distance to viewer
    float depth;
    // RGB color (3 bytes), opacity (1 byte)
    uint premulColor;
};

// Stores viewportW * viewportH * MAX_NUM_LAYERS fragments.
// Access fragment i at screen position (x,y) using "nodes[w*npp*y + npp*x + i]".
layout (std430, binding = 0) coherent buffer FragmentNodes {
    MLABFragmentNode_compressed nodes[];
};


#ifdef MLAB_MIN_DEPTH_BUCKETS
struct MinDepthNode {
    float minDepth;
    float minOpaqueDepth;
};

layout (std430, binding = 1) coherent buffer MinDepthBuffer {
    MinDepthNode depthBuffer[];
};
#endif


// Load the fragments into "nodeArray"
void loadFragmentNodes(in uint pixelIndex, out MLABFragmentNode nodeArray[MAX_NUM_LAYERS+1]) {
    MLABFragmentNode_compressed fragmentNode = nodes[pixelIndex];

#if (MAX_NUM_LAYERS % 4 == 0) && (MAX_NUM_LAYERS > 4)
    for (int i = 0; i < MAX_NUM_LAYERS/4; i++) {
        for (int j = 0; j < 4; j++) {
            MLABFragmentNode node = { fragmentNode.depth[i][j], fragmentNode.premulColor[i][j] };
            nodeArray[i*4 + j] = node;
        }
    }
#else
    for (int i = 0; i < MAX_NUM_LAYERS; i++) {
        MLABFragmentNode node = { fragmentNode.depth[i], fragmentNode.premulColor[i] };
        nodeArray[i] = node;
    }
#endif

    // For merging to see if last node is unused
    nodeArray[MAX_NUM_LAYERS].depth = DISTANCE_INFINITE;
}

// Store the fragments from "nodeArray" into VRAM
void storeFragmentNodes(in uint pixelIndex, in MLABFragmentNode nodeArray[MAX_NUM_LAYERS+1]) {
    MLABFragmentNode_compressed fragmentNode;

#if (MAX_NUM_LAYERS % 4 == 0) && (MAX_NUM_LAYERS > 4)
    for (int i = 0; i < MAX_NUM_LAYERS/4; i++) {
        for(int j = 0; j < 4; j++) {
            fragmentNode.depth[i][j] = nodeArray[4*i + j].depth;
            fragmentNode.premulColor[i][j] = nodeArray[4*i + j].premulColor;
        }
    }
#else
    for (int i = 0; i < MAX_NUM_LAYERS; i++) {
        fragmentNode.depth[i] = nodeArray[i].depth;
        fragmentNode.premulColor[i] = nodeArray[i].premulColor;
    }
#endif

    nodes[pixelIndex] = fragmentNode;
}

// Reset the data for the passed fragment position
void clearPixel(uint pixelIndex) {
    // TODO: Compressed?
    MLABFragmentNode nodeArray[MAX_NUM_LAYERS+1];
    for (uint i = 0; i < MAX_NUM_LAYERS; i++) {
        nodeArray[i].depth = DISTANCE_INFINITE;
        nodeArray[i].premulColor = 0xFF000000u; // 100% transparency, i.e. 0% opacity
    }
    storeFragmentNodes(pixelIndex, nodeArray);

#ifdef MLAB_MIN_DEPTH_BUCKETS
    MinDepthNode depthInfo;
    depthInfo.minDepth = 1.0;
    depthInfo.minOpaqueDepth = 1.0;
    depthBuffer[pixelIndex] = depthInfo;
#endif
}
