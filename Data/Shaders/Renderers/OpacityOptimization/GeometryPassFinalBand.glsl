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

-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in vec3 vertexOffsetLeft;
layout(location = 5) in vec3 vertexOffsetRight;
layout(location = 6) in float vertexOpacity;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
layout(location = 7) in uint vertexPrincipalStressIndex;
#endif

layout(location = 0) out vec3 linePosition;
layout(location = 1) out float lineAttribute;
layout(location = 2) out vec3 lineNormal;
layout(location = 3) out vec3 lineTangent;
layout(location = 4) out vec3 lineOffsetLeft;
layout(location = 5) out vec3 lineOffsetRight;
layout(location = 6) out float lineOpacity;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
layout(location = 7) out uint linePrincipalStressIndex;
#endif

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineNormal = vertexNormal;
    lineTangent = vertexTangent;
    lineOffsetLeft = vertexOffsetLeft;
    lineOffsetRight = vertexOffsetRight;
    lineOpacity = vertexOpacity;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
    linePrincipalStressIndex = vertexPrincipalStressIndex;
#endif
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float lineWidth;
uniform float bandWidth;
uniform ivec3 psUseBands;

layout(location = 0) in vec3 linePosition[];
layout(location = 1) in float lineAttribute[];
layout(location = 2) in vec3 lineNormal[];
layout(location = 3) in vec3 lineTangent[];
layout(location = 4) in vec3 lineOffsetLeft[];
layout(location = 5) in vec3 lineOffsetRight[];
layout(location = 6) in float lineOpacity[];
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
layout(location = 7) in uint linePrincipalStressIndex[];
#endif

layout(location = 0) out vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
layout(location = 1) out vec3 screenSpacePosition;
#endif
layout(location = 2) out float fragmentAttribute;
layout(location = 3) out float fragmentOpacity;
layout(location = 4) out float fragmentNormalFloat; // Between -1 and 1
layout(location = 5) out vec3 normal0;
layout(location = 6) out vec3 normal1;
layout(location = 7) flat out int useBand;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
layout(location = 8) flat out uint fragmentPrincipalStressIndex;
#endif

void main() {
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    vec3 tangent0 = normalize(v_in[0].lineTangent);
    vec3 tangent1 = normalize(v_in[1].lineTangent);

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
    useBand = psUseBands[v_in[0].linePrincipalStressIndex];
#else
    useBand = 1;
#endif

    vec3 offsetDirectionLeft0;
    vec3 offsetDirectionRight0;
    vec3 offsetDirectionLeft1;
    vec3 offsetDirectionRight1;
    if (useBand != 0) {
        offsetDirectionLeft0 = v_in[0].lineOffsetLeft;
        offsetDirectionRight0 = v_in[0].lineOffsetRight;
        offsetDirectionLeft1 = v_in[1].lineOffsetLeft;
        offsetDirectionRight1 = v_in[1].lineOffsetRight;
    } else {
        vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
        vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
        offsetDirectionRight0 = normalize(cross(tangent0, viewDirection0));
        offsetDirectionLeft0 = -offsetDirectionRight0;
        offsetDirectionRight1 = normalize(cross(tangent1, viewDirection1));
        offsetDirectionLeft1 = -offsetDirectionRight1;
    }

    vec3 vertexPosition;

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    // Vertex 0
    fragmentAttribute = v_in[0].lineAttribute;
    fragmentOpacity = v_in[0].lineOpacity;
    if (useBand != 0) {
        normal0 = v_in[0].lineNormal;
        normal1 = v_in[0].lineNormal;
    } else {
        normal0 = normalize(cross(tangent0, offsetDirectionRight0));
        normal1 = offsetDirectionRight0;
    }
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[0].linePrincipalStressIndex;
#endif

    vertexPosition = linePosition0 + lineRadius * offsetDirectionLeft0;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = v_in[0].lineAttribute;
    fragmentOpacity = v_in[0].lineOpacity;
    if (useBand != 0) {
        normal0 = v_in[0].lineNormal;
        normal1 = v_in[0].lineNormal;
    } else {
        normal0 = normalize(cross(tangent0, offsetDirectionRight0));
        normal1 = offsetDirectionRight0;
    }
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[0].linePrincipalStressIndex;
#endif

    vertexPosition = linePosition0 + lineRadius * offsetDirectionRight0;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = 1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    // Vertex 1
    fragmentAttribute = v_in[1].lineAttribute;
    fragmentOpacity = v_in[1].lineOpacity;
    if (useBand != 0) {
        normal0 = v_in[1].lineNormal;
        normal1 = v_in[1].lineNormal;
    } else {
        normal0 = normalize(cross(tangent1, offsetDirectionRight1));
        normal1 = offsetDirectionRight1;
    }
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
#endif

    vertexPosition = linePosition1 + lineRadius * offsetDirectionLeft1;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = v_in[1].lineAttribute;
    fragmentOpacity = v_in[1].lineOpacity;
    if (useBand != 0) {
        normal0 = v_in[1].lineNormal;
        normal1 = v_in[1].lineNormal;
    } else {
        normal0 = normalize(cross(tangent1, offsetDirectionRight1));
        normal1 = offsetDirectionRight1;
    }
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
#endif

    vertexPosition = linePosition1 + lineRadius * offsetDirectionRight1;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = 1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

layout(location = 0) in vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
layout(location = 1) in vec3 screenSpacePosition;
#endif
layout(location = 2) in float fragmentAttribute;
layout(location = 3) in float fragmentOpacity;
layout(location = 4) in float fragmentNormalFloat;
layout(location = 5) in vec3 normal0;
layout(location = 6) in vec3 normal1;
layout(location = 7) flat in int useBand;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
layout(location = 8) flat in uint fragmentPrincipalStressIndex;
#endif

#ifdef USE_COVERAGE_MASK
in int gl_SampleMaskIn[];
#endif

#if defined(DIRECT_BLIT_GATHER)
layout(location = 0) out vec4 fragColor;
#endif

uniform vec3 cameraPosition;
uniform float lineWidth;
uniform float bandWidth;
uniform vec3 backgroundColor;
uniform vec3 foregroundColor;

#define M_PI 3.14159265358979323846

#include "TransferFunction.glsl"

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"
#include "FloatPack.glsl"
#include "Lighting.glsl"
#include "LinkedListHeaderFinal.glsl"

void gatherFragmentCustomDepth(vec4 color, float fragmentDepth) {
    if (color.a < 0.001) {
        discard;
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.next = -1;

    //float depthNormalized = gl_FragCoord.z;
    float depthNormalized = convertLinearDepthToDepthBufferValue(
            convertDepthBufferValueToLinearDepth(gl_FragCoord.z) + fragmentDepth - length(fragmentPositionWorld - cameraPosition) - 0.0001);

#ifdef USE_COVERAGE_MASK
    //packFloat24Uint8(frag.depth, gl_FragCoord.z, gl_SampleMaskIn[0]);
    float coverageRatio = float(bitCount(gl_SampleMaskIn[0])) / float(gl_NumSamples);
    packFloat24Float8(frag.depth, depthNormalized, coverageRatio);
#else
    frag.depth = convertNormalizedFloatToUint32(depthNormalized);
#endif

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}

void main() {
    vec3 fragmentNormal;
    if (useBand != 0) {
        fragmentNormal = normal0;
    } else {
        // Compute the normal of the billboard tube for shading.
        float interpolationFactor = fragmentNormalFloat;
        vec3 normalCos = normalize(normal0);
        vec3 normalSin = normalize(normal1);
        if (interpolationFactor < 0.0) {
            normalSin = -normalSin;
            interpolationFactor = -interpolationFactor;
        }
        float angle = interpolationFactor * M_PI * 0.5;
        fragmentNormal = cos(angle) * normalCos + sin(angle) * normalSin;
    }

#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    vec4 fragmentColor = transferFunction(fragmentAttribute, fragmentPrincipalStressIndex);
#else
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#endif
    fragmentColor.a = 1.0; // Ignore transparency mapping.
    fragmentColor = blinnPhongShading(fragmentColor, fragmentNormal);

    float absCoords = abs(fragmentNormalFloat);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth * 0.0015 / (useBand != 0 ? bandWidth : lineWidth), 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(fragmentNormalFloat));
    vec4 colorOut = vec4(mix(fragmentColor.rgb, foregroundColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, absCoords)),
            fragmentColor.a * coverage);

    colorOut.a *= fragmentOpacity;
    gatherFragmentCustomDepth(colorOut, fragmentDepth);
}
