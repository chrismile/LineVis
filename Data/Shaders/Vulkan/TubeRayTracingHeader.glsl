/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

layout(binding = 0) uniform CameraSettingsBuffer {
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
} camera;

layout(binding = 3) uniform RayTracerSettingsBuffer {
    vec3 cameraPosition;
    float paddingFlt;
    vec4 backgroundColor;
    vec4 foregroundColor;

    // The maximum number of transparent fragments to blend before stopping early.
    uint maxDepthComplexity;
    // How many rays should be shot per frame?
    uint numSamplesPerFrame;

    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;

    uint paddingUint;
};

#ifdef USE_MLAT

#ifndef NUM_NODES
#error If USE_MLAT is defined, then NUM_NODES must also be defined.
#endif

#if NUM_NODES >= 1 && NUM_NODES <= 4
//#define NODES_UNROLLED
#endif

struct MlatNode {
    vec4 color;
    float transmittance;
    float depth;
};

struct RayPayload {
#ifdef NODES_UNROLLED
    MlatNode node0;
#if NUM_NODES >= 2
    MlatNode node1;
#endif
#if NUM_NODES >= 3
    MlatNode node2;
#endif
#if NUM_NODES >= 4
    MlatNode node3;
#endif
#else
    MlatNode nodes[NUM_NODES];
#endif
    float depth2;
};

#else

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};

#endif

#ifdef RAY_GEN_SHADER
layout(location = 0) rayPayloadEXT RayPayload payload;
#else
layout(location = 0) rayPayloadInEXT RayPayload payload;
#endif
