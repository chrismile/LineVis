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

#version 450 core

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

layout(location = 0) out vec3 linePosition;
layout(location = 1) out float lineAttribute;
layout(location = 2) out vec3 lineNormal;
layout(location = 3) out vec3 lineTangent;
layout(location = 4) out vec3 lineOffsetLeft;
layout(location = 5) out vec3 lineOffsetRight;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 6) out uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 7) out float lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 8) out uint lineLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
layout(location = 9) out uint lineVertexId;
#endif

#include "LineUniformData.glsl"
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
    lineVertexId = uint(gl_VertexIndex);
#endif
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 450 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

#include "LineUniformData.glsl"

layout(location = 0) in vec3 linePosition[];
layout(location = 1) in float lineAttribute[];
layout(location = 2) in vec3 lineNormal[];
layout(location = 3) in vec3 lineTangent[];
layout(location = 4) in vec3 lineOffsetLeft[];
layout(location = 5) in vec3 lineOffsetRight[];
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 6) in uint linePrincipalStressIndex[];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 7) in float lineLineHierarchyLevel[];
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 8) in uint lineLineAppearanceOrder[];
#endif
#ifdef USE_AMBIENT_OCCLUSION
layout(location = 9) in uint lineVertexId[];
#endif

layout(location = 0) out vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
layout(location = 1) out vec3 screenSpacePosition;
#endif
layout(location = 2) out float fragmentAttribute;
layout(location = 3) out vec3 fragmentTangent;
layout(location = 4) out float fragmentNormalFloat; // Between -1 and 1
layout(location = 5) out vec3 normal0;
layout(location = 6) out vec3 normal1;
layout(location = 7) flat out int useBand;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 8) flat out uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 9) flat out float fragmentLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 10) flat out uint fragmentLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
layout(location = 11) out float fragmentVertexId;
#endif

void main() {
    vec3 linePosition0 = (mMatrix * vec4(linePosition[0], 1.0)).xyz;
    vec3 linePosition1 = (mMatrix * vec4(linePosition[1], 1.0)).xyz;
    vec3 tangent0 = normalize(lineTangent[0]);
    vec3 tangent1 = normalize(lineTangent[1]);

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
    int useBandValue = psUseBands[linePrincipalStressIndex[0]];
#else
    int useBandValue = 1;
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
    if (useBandValue != 0) {
        vec3 helperVec0 = offsetDirectionRight0;
        vec3 viewDirectionNew0 = normalize(cross(helperVec0, tangent0));

        vec3 helperVec1 = offsetDirectionRight1;
        vec3 viewDirectionNew1 = normalize(cross(helperVec1, tangent1));

        float thickness0 = max(abs(dot(lineNormal[0], viewDirectionNew0)), MIN_THICKNESS);
        float thickness1 = max(abs(dot(lineNormal[1], viewDirectionNew1)), MIN_THICKNESS);
        offsetDirectionRight0 *= thickness0 * length(lineOffsetLeft[0]);
        offsetDirectionLeft0 *= thickness0 * length(lineOffsetRight[0]);
        offsetDirectionRight1 *= thickness1 * length(lineOffsetLeft[1]);
        offsetDirectionLeft1 *= thickness1 * length(lineOffsetRight[1]);
    }
#else
    if (useBandValue != 0) {
        offsetDirectionLeft0 = lineOffsetLeft[0];
        offsetDirectionRight0 = lineOffsetRight[0];
        offsetDirectionLeft1 = lineOffsetLeft[1];
        offsetDirectionRight1 = lineOffsetRight[1];
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

    const float lineRadius = (useBandValue != 0 ? bandWidth : lineWidth) * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    // Vertex 0
    fragmentAttribute = lineAttribute[0];
    fragmentTangent = tangent0;
    if (useBandValue != 0) {
        normal0 = lineNormal[0];
        normal1 = lineNormal[0];
    } else {
        normal0 = normalize(cross(tangent0, offsetDirectionRight0));
        normal1 = offsetDirectionRight0;
    }
    useBand = useBandValue;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    fragmentPrincipalStressIndex = linePrincipalStressIndex[0];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = lineLineAppearanceOrder[0];
#endif
#ifdef USE_AMBIENT_OCCLUSION
    fragmentVertexId = float(lineVertexId[0]);
#endif

    vertexPosition = linePosition0 + lineRadius * offsetDirectionLeft0;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = lineAttribute[0];
    fragmentTangent = tangent0;
    if (useBandValue != 0) {
        normal0 = lineNormal[0];
        normal1 = lineNormal[0];
    } else {
        normal0 = normalize(cross(tangent0, offsetDirectionRight0));
        normal1 = offsetDirectionRight0;
    }
    useBand = useBandValue;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    fragmentPrincipalStressIndex = linePrincipalStressIndex[0];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = lineLineAppearanceOrder[0];
#endif
#ifdef USE_AMBIENT_OCCLUSION
    fragmentVertexId = float(lineVertexId[0]);
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
    fragmentAttribute = lineAttribute[1];
    fragmentTangent = tangent1;
    if (useBandValue != 0) {
        normal0 = lineNormal[1];
        normal1 = lineNormal[1];
    } else {
        normal0 = normalize(cross(tangent1, offsetDirectionRight1));
        normal1 = offsetDirectionRight1;
    }
    useBand = useBandValue;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    fragmentPrincipalStressIndex = linePrincipalStressIndex[1];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[1];
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = lineLineAppearanceOrder[1];
#endif
#ifdef USE_AMBIENT_OCCLUSION
    fragmentVertexId = float(lineVertexId[1]);
#endif

    vertexPosition = linePosition1 + lineRadius * offsetDirectionLeft1;
    fragmentPositionWorld = vertexPosition;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
    fragmentNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = lineAttribute[1];
    fragmentTangent = tangent1;
    if (useBandValue != 0) {
        normal0 = lineNormal[1];
        normal1 = lineNormal[1];
    } else {
        normal0 = normalize(cross(tangent1, offsetDirectionRight1));
        normal1 = offsetDirectionRight1;
    }
    useBand = useBandValue;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    fragmentPrincipalStressIndex = linePrincipalStressIndex[1];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[1];
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = lineLineAppearanceOrder[1];
#endif
#ifdef USE_AMBIENT_OCCLUSION
    fragmentVertexId = float(lineVertexId[1]);
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

#version 450 core

#include "LineUniformData.glsl"

layout(location = 0) in vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
layout(location = 1) in vec3 screenSpacePosition;
#endif
layout(location = 2) in float fragmentAttribute;
layout(location = 3) in vec3 fragmentTangent;
layout(location = 4) in float fragmentNormalFloat;
layout(location = 5) in vec3 normal0;
layout(location = 6) in vec3 normal1;
layout(location = 7) flat in int useBand;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 8) flat in uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 9) flat in float fragmentLineHierarchyLevel;
#ifdef USE_TRANSPARENCY
layout(binding = LINE_HIERARCHY_IMPORTANCE_MAP_BINDING) uniform sampler1DArray lineHierarchyImportanceMap;
#endif
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 10) flat in uint fragmentLineAppearanceOrder;
#endif
#ifdef USE_AMBIENT_OCCLUSION
layout(location = 11) in float fragmentVertexId;
float phi;
#endif

#if defined(DIRECT_BLIT_GATHER)
layout(location = 0) out vec4 fragColor;
#endif

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
        phi = asin(interpolationFactor);
#endif
    } else {
        // Compute the normal of the billboard tube for shading.
        float interpolationFactor = fragmentNormalFloat;
        vec3 normalCos = normalize(normal0);
        vec3 normalSin = normalize(normal1);
        //if (interpolationFactor < 0.0) {
        //    normalSin = -normalSin;
        //    interpolationFactor = -interpolationFactor;
        //}
        float angle = asin(interpolationFactor);
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
    vec4 colorOut = vec4(mix(fragmentColor.rgb, foregroundColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON_WHITE, WHITE_THRESHOLD + EPSILON_WHITE, absCoords)),
            fragmentColor.a * coverage);

#include "LinePassGather.glsl"
}
