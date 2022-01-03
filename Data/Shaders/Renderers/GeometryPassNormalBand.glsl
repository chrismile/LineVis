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

-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in vec3 vertexOffsetLeft;
layout(location = 5) in vec3 vertexOffsetRight;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 6) in uint vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 7) in float vertexLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 8) in uint vertexLineAppearanceOrder;
#endif

out VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineNormal;
    vec3 lineTangent;
    vec3 lineOffsetLeft;
    vec3 lineOffsetRight;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    float lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    uint lineLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
    uint lineVertexId;
#endif
};

#include "TransferFunction.glsl"

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineNormal = vertexNormal;
    lineTangent = vertexTangent;
    lineOffsetLeft = vertexOffsetLeft;
    lineOffsetRight = vertexOffsetRight;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    linePrincipalStressIndex = vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    lineLineHierarchyLevel = vertexLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    lineLineAppearanceOrder = vertexLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
    lineVertexId = uint(gl_VertexID);
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
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
uniform ivec3 psUseBands;
#endif

out vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
out vec3 screenSpacePosition;
#endif
out float fragmentAttribute;
out vec3 fragmentTangent;
out float fragmentNormalFloat; // Between -1 and 1
out vec3 normal0;
out vec3 normal1;
flat out int useBand;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
flat out uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
flat out float fragmentLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
flat out uint fragmentLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
out float fragmentVertexId;
#endif

in VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineNormal;
    vec3 lineTangent;
    vec3 lineOffsetLeft;
    vec3 lineOffsetRight;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    float lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    uint lineLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
    uint lineVertexId;
#endif
} v_in[];

void main() {
    vec3 linePosition0 = (mMatrix * vec4(v_in[0].linePosition, 1.0)).xyz;
    vec3 linePosition1 = (mMatrix * vec4(v_in[1].linePosition, 1.0)).xyz;
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
#ifdef BAND_RENDERING_THICK
    vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
    vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
    offsetDirectionRight0 = normalize(cross(tangent0, viewDirection0));
    offsetDirectionRight1 = normalize(cross(tangent1, viewDirection1));

    offsetDirectionLeft0 = -offsetDirectionRight0;
    offsetDirectionLeft1 = -offsetDirectionRight1;
    if (useBand != 0) {
        vec3 helperVec0 = offsetDirectionRight0;
        vec3 viewDirectionNew0 = normalize(cross(helperVec0, tangent0));

        vec3 helperVec1 = offsetDirectionRight1;
        vec3 viewDirectionNew1 = normalize(cross(helperVec1, tangent1));

        float thickness0 = max(abs(dot(v_in[0].lineNormal, viewDirectionNew0)), MIN_THICKNESS);
        float thickness1 = max(abs(dot(v_in[1].lineNormal, viewDirectionNew1)), MIN_THICKNESS);
        offsetDirectionRight0 *= thickness0 * length(v_in[0].lineOffsetLeft);
        offsetDirectionLeft0 *= thickness0 * length(v_in[0].lineOffsetRight);
        offsetDirectionRight1 *= thickness1 * length(v_in[1].lineOffsetLeft);
        offsetDirectionLeft1 *= thickness1 * length(v_in[1].lineOffsetRight);
    }
#else
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
#endif

    vec3 vertexPosition;

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    // Vertex 0
    fragmentAttribute = v_in[0].lineAttribute;
    fragmentTangent = tangent0;
    if (useBand != 0) {
        normal0 = v_in[0].lineNormal;
        normal1 = v_in[0].lineNormal;
    } else {
        normal0 = normalize(cross(tangent0, offsetDirectionRight0));
        normal1 = offsetDirectionRight0;
    }
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    fragmentPrincipalStressIndex = v_in[0].linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = v_in[0].lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = v_in[0].lineLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
    fragmentVertexId = float(v_in[0].lineVertexId);
#endif

    vertexPosition = linePosition0 + lineRadius * offsetDirectionLeft0;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

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
    fragmentTangent = tangent1;
    if (useBand != 0) {
        normal0 = v_in[1].lineNormal;
        normal1 = v_in[1].lineNormal;
    } else {
        normal0 = normalize(cross(tangent1, offsetDirectionRight1));
        normal1 = offsetDirectionRight1;
    }
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = v_in[1].lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = v_in[1].lineLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
    fragmentVertexId = float(v_in[1].lineVertexId);
#endif

    vertexPosition = linePosition1 + lineRadius * offsetDirectionLeft1;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

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

#version 450 core

in vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
in vec3 screenSpacePosition;
#endif
in float fragmentAttribute;
in vec3 fragmentTangent;
in float fragmentNormalFloat;
in vec3 normal0;
in vec3 normal1;
flat in int useBand;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
flat in uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
flat in float fragmentLineHierarchyLevel;
#ifdef USE_TRANSPARENCY
//uniform vec3 lineHierarchySliderLower;
//uniform vec3 lineHierarchySliderUpper;
uniform sampler1DArray lineHierarchyImportanceMap;
#else
uniform vec3 lineHierarchySlider;
#endif
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
flat in uint fragmentLineAppearanceOrder;
uniform int currentSeedIdx;
#endif
#ifdef USE_AMBIENT_OCCLUSION
in float fragmentVertexId;
float phi;
#endif

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;
uniform float lineWidth;
uniform float bandWidth;
uniform vec3 backgroundColor;
uniform vec3 foregroundColor;

#define M_PI 3.14159265358979323846

#include "TransferFunction.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"
#include "Lighting.glsl"
#include "Antialiasing.glsl"

void main() {
#if defined(USE_LINE_HIERARCHY_LEVEL) && !defined(USE_TRANSPARENCY)
    float slider = lineHierarchySlider[fragmentPrincipalStressIndex];
    if (slider > fragmentLineHierarchyLevel) {
        discard;
    }
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    if (int(fragmentLineAppearanceOrder) > currentSeedIdx) {
        discard;
    }
#endif

    vec3 fragmentNormal;
    if (useBand != 0) {
        fragmentNormal = normal0;
#ifdef USE_AMBIENT_OCCLUSION
        float interpolationFactor = fragmentNormalFloat;
        if (interpolationFactor < 0.0) {
            interpolationFactor = -interpolationFactor;
        }
        phi = interpolationFactor * M_PI * 0.5;
#endif
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
#ifdef USE_AMBIENT_OCCLUSION
        phi = angle;
#endif
    }


#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    vec4 fragmentColor = transferFunction(fragmentAttribute, fragmentPrincipalStressIndex);
#else
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#endif

#if defined(USE_LINE_HIERARCHY_LEVEL) && defined(USE_TRANSPARENCY)
    //float lower = lineHierarchySliderLower[fragmentPrincipalStressIndex];
    //float upper = lineHierarchySliderUpper[fragmentPrincipalStressIndex];
    //fragmentColor.a *= (upper - lower) * fragmentLineHierarchyLevel + lower;
    fragmentColor.a *= texture(
            lineHierarchyImportanceMap, vec2(fragmentLineHierarchyLevel, float(fragmentPrincipalStressIndex))).r;
#endif

    fragmentColor = blinnPhongShadingTube(fragmentColor, fragmentNormal, fragmentTangent);

    float absCoords = abs(fragmentNormalFloat);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(getAntialiasingFactor(fragmentDepth / (useBand != 0 ? bandWidth : lineWidth) * 2.0), 0.0, 0.49);
    float EPSILON_WHITE = fwidth(absCoords);
    float coverage = 1.0 - smoothstep(1.0 - EPSILON, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(fragmentNormalFloat));
    vec4 colorOut = vec4(mix(fragmentColor.rgb, foregroundColor,
            smoothstep(WHITE_THRESHOLD - EPSILON_WHITE, WHITE_THRESHOLD + EPSILON_WHITE, absCoords)),
            fragmentColor.a * coverage);

#if defined(DIRECT_BLIT_GATHER)
    // To counteract depth fighting with overlay wireframe.
    float depthOffset = -0.00001;
    if (absCoords >= WHITE_THRESHOLD - EPSILON_WHITE) {
        depthOffset = 0.002;
    }
    //gl_FragDepth = clamp(gl_FragCoord.z + depthOffset, 0.0, 0.999);
    gl_FragDepth = convertLinearDepthToDepthBufferValue(
            convertDepthBufferValueToLinearDepth(gl_FragCoord.z) + fragmentDepth
            - length(fragmentPositionWorld - cameraPosition) - 0.0001);
    if (colorOut.a < 0.01) {
        discard;
    }
    colorOut.a = 1.0;
    fragColor = colorOut;
#elif defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK)
    // Area of mutual exclusion for fragments mapping to the same pixel
    beginInvocationInterlockARB();
    gatherFragment(colorOut);
    endInvocationInterlockARB();
#elif defined(USE_SYNC_SPINLOCK)
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));
    /**
     * Spinlock code below based on code in:
     * Brüll, Felix. (2018). Order-Independent Transparency Acceleration. 10.13140/RG.2.2.17568.84485.
     */
    if (!gl_HelperInvocation) {
        bool keepWaiting = true;
        while (keepWaiting) {
            if (atomicCompSwap(spinlockViewportBuffer[pixelIndex], 0, 1) == 0) {
                gatherFragment(colorOut);
                memoryBarrier();
                atomicExchange(spinlockViewportBuffer[pixelIndex], 0);
                keepWaiting = false;
            }
        }
    }
#else
    gatherFragment(colorOut);
#endif
}
