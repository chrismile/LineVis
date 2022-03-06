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
layout(location = 2) in vec3 vertexTangent;
layout(location = 3) in uint vertexLineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 4) in uint vertexPrincipalStressIndex;
layout(location = 5) in float vertexLineHierarchyLevel;
#endif

layout(location = 0) out vec3 linePosition;
layout(location = 1) out float lineAttribute;
layout(location = 2) out vec3 lineTangent;
layout(location = 3) out uint lineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 4) out uint linePrincipalStressIndex;
layout(location = 5) out float lineLineHierarchyLevel;
#endif

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
    lineSegmentId = vertexLineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
    linePrincipalStressIndex = vertexPrincipalStressIndex;
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
layout(location = 3) in uint lineSegmentId[];
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 4) in uint linePrincipalStressIndex[];
layout(location = 5) in float lineLineHierarchyLevel[];
#endif

layout(location = 0) out vec3 fragmentPositionWorld;
layout(location = 1) out float fragmentAttribute;
layout(location = 2) flat out uint fragmentLineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 3) flat out uint fragmentPrincipalStressIndex;
layout(location = 4) flat out float fragmentLineHierarchyLevel;
#endif

#include "LineUniformData.glsl"

void main() {
    vec3 linePosition0 = linePosition[0];
    vec3 linePosition1 = linePosition[1];
    float lineAttribute1 = lineAttribute[1];

    vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
    vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
    vec3 offsetDirection0 = normalize(cross(normalize(lineTangent[0]), viewDirection0));
    vec3 offsetDirection1 = normalize(cross(normalize(lineTangent[1]), viewDirection1));
    vec3 vertexPosition;

    const float lineRadius = lineWidth * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    // Vertex 0
    fragmentAttribute = lineAttribute[0];
    fragmentLineSegmentId = lineSegmentId[0];
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = linePrincipalStressIndex[0];
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
    vertexPosition = linePosition0 - lineRadius * offsetDirection0;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = lineAttribute[0];
    fragmentLineSegmentId = lineSegmentId[0];
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = linePrincipalStressIndex[0];
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
    vertexPosition = linePosition0 + lineRadius * offsetDirection0;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    // Vertex 1
    fragmentAttribute = lineAttribute1;
    fragmentLineSegmentId = lineSegmentId[1];
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = linePrincipalStressIndex[1];
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[1];
#endif
    vertexPosition = linePosition1 - lineRadius * offsetDirection1;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    fragmentAttribute = lineAttribute1;
    fragmentLineSegmentId = lineSegmentId[1];
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = linePrincipalStressIndex[1];
    fragmentLineHierarchyLevel = lineLineHierarchyLevel[1];
#endif
    vertexPosition = linePosition1 + lineRadius * offsetDirection1;
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
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 3) flat in uint fragmentPrincipalStressIndex;
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
