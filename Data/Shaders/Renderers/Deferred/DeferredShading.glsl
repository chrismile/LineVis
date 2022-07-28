/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

layout(location = 0) in vec4 vertexPosition;

void main() {
    gl_Position = vertexPosition;
}


-- Fragment

#version 450 core
#extension GL_EXT_scalar_block_layout : require
// for DEBUG_MESHLETS
//#extension GL_NV_fragment_shader_barycentric : require

in vec4 gl_FragCoord;
layout(location = 0) out vec4 fragColor;

#define DEFERRED_SHADING
#ifndef FRAGMENT_SHADER
#define FRAGMENT_SHADER
#endif
#include "LineUniformData.glsl"
#include "BarycentricInterpolation.glsl"
#include "RayHitCommon.glsl"

/*
 * While storing only the primitive index, thus resulting in an actual visibility buffer [Burns and Hunt 2013], might
 * result in lower memory usage, storing also a depth buffer might result in performance benefits due to allowing for
 * early depth testing.
 *
 * Burns, Christopher A. and Hunt, Warren A. (2013). The Visibility Buffer: A Cache-Friendly Approach to Deferred
 * Shading. Journal of Computer Graphics Techniques, 2(2):55-69.
 */
layout(binding = 0) uniform usampler2D primitiveIndexBuffer; //< Clear value: 0xFFFFFFFFu
layout(binding = 1) uniform sampler2D depthBuffer;

layout(scalar, binding = 2) readonly buffer TriangleIndexBuffer {
    uvec3 indexBuffer[];
};

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to LinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};
layout(std430, binding = TUBE_TRIANGLE_MESH_VERTEX_BUFFER_BINDING) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

void main() {
    ivec2 fragCoord = ivec2(gl_FragCoord.xy);
    uint primitiveIndex = texelFetch(primitiveIndexBuffer, fragCoord, 0).x;

    // Clear value of primitive index, i.e., no primitive maps to this pixel?
    if (primitiveIndex == 0xFFFFFFFFu) {
        fragColor = backgroundColor;
        return;
    }

    float fragmentDepth = texelFetch(depthBuffer, fragCoord, 0).x;
    vec4 fragPosNdc = vec4(2.0 * gl_FragCoord.xy / vec2(viewportSize) - vec2(1.0), fragmentDepth, 1.0);
    vec4 fragPosView = inverseProjectionMatrix * fragPosNdc;
    vec4 fragPosWorld = inverseViewMatrix * fragPosView;

    vec3 fragmentPositionWorld = fragPosWorld.xyz / fragPosWorld.w;
#if defined(USE_DEPTH_CUES) || (defined(USE_AMBIENT_OCCLUSION) && !defined(STATIC_AMBIENT_OCCLUSION_PREBAKING))
    vec3 screenSpacePosition = fragPosView.xyz / fragPosView.w;
#endif

    uvec3 triangleIndices = indexBuffer[primitiveIndex];
    TubeTriangleVertexData vertexData0 = tubeTriangleVertexDataBuffer[triangleIndices.x];
    TubeTriangleVertexData vertexData1 = tubeTriangleVertexDataBuffer[triangleIndices.y];
    TubeTriangleVertexData vertexData2 = tubeTriangleVertexDataBuffer[triangleIndices.z];

    vec3 d20 = vertexData2.vertexPosition - vertexData0.vertexPosition;
    vec3 d21 = vertexData2.vertexPosition - vertexData1.vertexPosition;
    float totalArea = length(cross(d20, d21));
    float u = length(cross(d21, fragmentPositionWorld - vertexData1.vertexPosition)) / totalArea;
    float v = length(cross(fragmentPositionWorld - vertexData0.vertexPosition, d20)) / totalArea;

    const vec3 barycentricCoordinates = vec3(u, v, 1.0 - u - v);

#include "LineAttributesBarycentric.glsl"
}
