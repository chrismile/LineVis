/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2021, Christoph Neuhauser, Michael Kern
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

#include "MLABBucketHeader.glsl"
#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"
#include "ColorPack.glsl"
#include "TiledAddress.glsl"

out vec4 fragColor;

// Adapted version of "Multi-Layer Alpha Blending" [Salvi and Vaidyanathan 2014]
void multiLayerAlphaBlendingOffset(in MLABBucketFragmentNode frag, inout MLABBucketFragmentNode list[BUFFER_SIZE+1]) {
    MLABBucketFragmentNode temp, merge;
    // Use bubble sort to insert new fragment node (single pass)
    for (int i = 1; i < BUFFER_SIZE+1; i++) {
        if (frag.depth <= list[i].depth) {
            temp = list[i];
            list[i] = frag;
            frag = temp;
        }
    }

    // Merge last two nodes if necessary
    if (list[BUFFER_SIZE].depth != DISTANCE_INFINITE) {
        vec4 src = unpackUnorm4x8(list[BUFFER_SIZE-1].premulColor);
        vec4 dst = unpackUnorm4x8(list[BUFFER_SIZE].premulColor);
        vec4 mergedColor;
        mergedColor.rgb = src.rgb + dst.rgb * src.a;
        mergedColor.a = src.a * dst.a; // Transmittance
        merge.premulColor = packUnorm4x8(mergedColor);
        merge.depth = list[BUFFER_SIZE-1].depth;
        list[BUFFER_SIZE-1] = merge;
    }
}


void multiLayerAlphaBlendingMergeFront(
        in MLABBucketFragmentNode frag, inout MLABBucketFragmentNode list[BUFFER_SIZE+1]) {
    MLABBucketFragmentNode temp, merge;
    if (frag.depth <= list[0].depth) {
        temp = list[0];
        list[0] = frag;
        frag = temp;
    }

    // Merge last two nodes if necessary
    if (frag.depth != DISTANCE_INFINITE) {
        vec4 src = unpackUnorm4x8(list[0].premulColor);
        vec4 dst = unpackUnorm4x8(frag.premulColor);
        vec4 mergedColor;
        mergedColor.rgb = src.rgb + dst.rgb * src.a;
        mergedColor.a = src.a * dst.a; // Transmittance
        merge.premulColor = packUnorm4x8(mergedColor);
        merge.depth = list[0].depth;
        list[0] = merge;
    }
}

void gatherFragment(vec4 color) {
    if (color.a < 0.001) {
        discard;
    }

    ivec2 fragPos2D = ivec2(int(gl_FragCoord.x), int(gl_FragCoord.y));
    uint pixelIndex = addrGen(uvec2(fragPos2D));

    MLABBucketFragmentNode frag;
    frag.depth = gl_FragCoord.z;
    frag.premulColor = packUnorm4x8(vec4(color.rgb * color.a, 1.0 - color.a));

    MLABBucketFragmentNode nodeArray[BUFFER_SIZE+1];
    loadFragmentNodes(pixelIndex, fragPos2D, nodeArray);
    float depthLog = logDepthWarp(-screenSpacePosition.z);
    frag.depth = depthLog;

    MinDepthNode depthInfo = depthBuffer[pixelIndex];

    if (depthLog <= depthInfo.minOpaqueDepth + 0.0001) {
        if (depthLog < depthInfo.minDepth) {
            // Merge new fragment with first one.
            multiLayerAlphaBlendingMergeFront(frag, nodeArray);
        } else {
            // Insert normally (with offset of one).
            multiLayerAlphaBlendingOffset(frag, nodeArray);
        }
        storeFragmentNodes(pixelIndex, fragPos2D, nodeArray);
    } else {
        discard;
    }

    fragColor = vec4(0.0, 0.0, 0.0, 0.0);
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    if (color.a < 0.001) {
        discard;
    }

    ivec2 fragPos2D = ivec2(int(gl_FragCoord.x), int(gl_FragCoord.y));
    uint pixelIndex = addrGen(uvec2(fragPos2D));

    MLABBucketFragmentNode frag;
    frag.depth = convertLinearDepthToDepthBufferValue(depth);
    frag.premulColor = packUnorm4x8(vec4(color.rgb * color.a, 1.0 - color.a));

    MLABBucketFragmentNode nodeArray[BUFFER_SIZE+1];
    loadFragmentNodes(pixelIndex, fragPos2D, nodeArray);
    float depthLog = logDepthWarp(-depth);
    frag.depth = depthLog;

    MinDepthNode depthInfo = depthBuffer[pixelIndex];

    if (depthLog <= depthInfo.minOpaqueDepth + 0.0001) {
        if (depthLog < depthInfo.minDepth) {
            // Merge new fragment with first one.
            multiLayerAlphaBlendingMergeFront(frag, nodeArray);
        } else {
            // Insert normally (with offset of one).
            multiLayerAlphaBlendingOffset(frag, nodeArray);
        }
        storeFragmentNodes(pixelIndex, fragPos2D, nodeArray);
    } else {
        discard;
    }

    fragColor = vec4(0.0, 0.0, 0.0, 0.0);
}
