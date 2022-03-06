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

#version 450 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in vec3 vertexOffsetLeft;
layout(location = 5) in vec3 vertexOffsetRight;
layout(location = 6) in uint vertexLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 7) in uint vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 8) in float vertexLineHierarchyLevel;
#endif

layout(location = 0) out vec3 linePosition;
layout(location = 1) out float lineAttribute;
layout(location = 2) out vec3 lineTangent;
layout(location = 3) out vec3 lineOffsetLeft;
layout(location = 4) out vec3 lineOffsetRight;
layout(location = 5) out uint lineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 6) out uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 7) out float lineLineHierarchyLevel;
#endif

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
    lineOffsetLeft = vertexOffsetLeft;
    lineOffsetRight = vertexOffsetRight;
    lineSegmentId = vertexLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    linePrincipalStressIndex = vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    lineLineHierarchyLevel = vertexLineHierarchyLevel;
#endif

    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 450 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

layout(location = 0) in vec3 linePosition[];
layout(location = 1) in float lineAttribute[];
layout(location = 2) in vec3 lineTangent[];
layout(location = 3) in vec3 lineOffsetLeft[];
layout(location = 4) in vec3 lineOffsetRight[];
layout(location = 5) in uint lineSegmentId[];
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 6) in uint linePrincipalStressIndex[];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 7) in float lineLineHierarchyLevel[];
#endif

layout(location = 0) out vec3 fragmentPositionWorld;
layout(location = 1) out float fragmentAttribute;
layout(location = 2) flat out uint fragmentLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 3) flat out uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 4) flat out float fragmentLineHierarchyLevel;
#endif

#include "LineUniformData.glsl"

void main() {
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    int useBand = psUseBands[v_in[0].linePrincipalStressIndex];
#else
    int useBand = 1;
#endif

    vec3 offsetDirectionLeft0;
    vec3 offsetDirectionRight0;
    vec3 offsetDirectionLeft1;
    vec3 offsetDirectionRight1;
    if (useBand != 0) {
        offsetDirectionLeft0 = lineOffsetLeft[0];
        offsetDirectionRight0 = lineOffsetRight[0];
        offsetDirectionLeft1 = lineOffsetLeft[1];
        offsetDirectionRight1 = lineOffsetRight[1];
    } else {
        vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
        vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
        offsetDirectionRight0 = normalize(cross(lineTangent[0], viewDirection0));
        offsetDirectionLeft0 = -offsetDirectionRight0;
        offsetDirectionRight1 = normalize(cross(lineTangent[1], viewDirection1));
        offsetDirectionLeft1 = -offsetDirectionRight1;
    }

    vec3 vertexPosition;

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    // Vertex 0
    fragmentAttribute = lineAttribute[0];
    fragmentLineSegmentId = lineSegmentId[0];
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = linePrincipalStressIndex[0];
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
    vertexPosition = linePosition0 + lineRadius * offsetDirectionLeft0;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = lineAttribute[0];
    fragmentLineSegmentId = lineSegmentId[0];
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = linePrincipalStressIndex[0];
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
    vertexPosition = linePosition0 + lineRadius * offsetDirectionRight0;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    // Vertex 1
    fragmentAttribute = v_in[1].lineAttribute;
    fragmentLineSegmentId = v_in[1].lineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
    fragmentLineHierarchyLevel = v_in[1].lineLineHierarchyLevel;
#endif
    vertexPosition = linePosition1 + lineRadius * offsetDirectionLeft1;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = v_in[1].lineAttribute;
    fragmentLineSegmentId = v_in[1].lineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
    fragmentLineHierarchyLevel = v_in[1].lineLineHierarchyLevel;
#endif
    vertexPosition = linePosition1 + lineRadius * offsetDirectionRight1;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 450 core

layout(location = 0) in vec3 fragmentPositionWorld;
layout(location = 1) in float fragmentAttribute;
layout(location = 2) flat in uint fragmentLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 3) flat in uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 4) flat in float fragmentLineHierarchyLevel;
layout(binding = LINE_HIERARCHY_IMPORTANCE_MAP_BINDING) uniform sampler1DArray lineHierarchyImportanceMap;
#endif

#ifndef USE_LINE_HIERARCHY_LEVEL
layout(binding = 7) uniform AttributeRangeUniformDataBuffer {
    float minAttrValue; // = 0.0f
    float maxAttrValue; // = 1.0f
};
#endif

#include "LineUniformData.glsl"
#include "FloatPack.glsl"
#include "LinkedListHeaderOpacities.glsl"

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.lineSegmentId = fragmentLineSegmentId;
    frag.next = -1;
#ifdef USE_LINE_HIERARCHY_LEVEL
    //float lower = lineHierarchySliderLower[fragmentPrincipalStressIndex];
    //float upper = lineHierarchySliderUpper[fragmentPrincipalStressIndex];
    //float fragmentAttributeHierarchy = (upper - lower) * fragmentLineHierarchyLevel + lower;
    float fragmentAttributeHierarchy = texture(
            lineHierarchyImportanceMap, vec2(fragmentLineHierarchyLevel, float(fragmentPrincipalStressIndex))).r;
    packFloat22Float10(frag.depth, gl_FragCoord.z, fragmentAttributeHierarchy);
#else
    packFloat22Float10(frag.depth, gl_FragCoord.z, (fragmentAttribute - minAttrValue) / (maxAttrValue - minAttrValue));
#endif

    uint insertIndex = atomicAdd(fragCounter, 1u);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}
