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

struct NodeAabb {
    vec3 worldSpaceAabbMin;
    float normalizedHierarchyLevel;
    vec3 worldSpaceAabbMax;
    uint passIdx;
};
layout(std430, binding = 0) readonly buffer NodeAabbBuffer {
    NodeAabb nodeAabbs[];
};

layout(location = 0) in vec3 vertexPosition;

layout(location = 0) out vec3 fragmentPosition;
layout(location = 1) out vec3 fragmentPositionNorm;
layout(location = 2) flat out uint fragmentInstanceIndex;

void main() {
    NodeAabb nodeAabb = nodeAabbs[gl_InstanceIndex];
    vec3 vertexPositionAabb =
            vertexPosition * (nodeAabb.worldSpaceAabbMax - nodeAabb.worldSpaceAabbMin) + nodeAabb.worldSpaceAabbMin;
    fragmentPosition = vertexPositionAabb;
    fragmentPositionNorm = vertexPosition;
    fragmentInstanceIndex = gl_InstanceIndex;
    vec4 positionProj = mvpMatrix * vec4(vertexPositionAabb, 1.0);
    positionProj.z += nodeAabb.normalizedHierarchyLevel * 1e-5;
    gl_Position = positionProj;
}


-- Fragment

#version 450 core

struct NodeAabb {
    vec3 worldSpaceAabbMin;
    float normalizedHierarchyLevel;
    vec3 worldSpaceAabbMax;
    uint passIdx;
};
layout(std430, binding = 0) readonly buffer NodeAabbBuffer {
    NodeAabb nodeAabbs[];
};

layout(binding = 1) uniform UniformDataBuffer {
    vec3 cameraPosition;
    float fieldOfViewY;
    vec2 viewportSize;
    vec2 viewportSizeVirtual;
    vec3 paddingUniformData;
    float lineWidthBase;
};

#ifdef SCREEN_SPACE_LINE_THICKNESS
#define viewportSize viewportSizeVirtual
#include "Antialiasing.glsl"
#undef viewportSize
#endif

layout(binding = 2) uniform sampler1D hierarchyLevelColorMap;

layout(location = 0) in vec3 fragmentPosition;
layout(location = 1) in vec3 fragmentPositionNorm;
layout(location = 2) flat in uint fragmentInstanceIndex;

layout(location = 0) out vec4 fragmentColor;

void main() {
    NodeAabb nodeAabb = nodeAabbs[fragmentInstanceIndex];

    vec3 cornerPosNorm = 0.5 * (vec3(1.0) - abs(2.0 * fragmentPositionNorm - vec3(1.0)));
    vec3 fragmentPositionCorner = cornerPosNorm * (nodeAabb.worldSpaceAabbMax - nodeAabb.worldSpaceAabbMin);

    float edgeDistance;
    if (cornerPosNorm.z < cornerPosNorm.y && cornerPosNorm.z < cornerPosNorm.x) {
        edgeDistance = min(fragmentPositionCorner.x, fragmentPositionCorner.y);
    } else if (cornerPosNorm.y < cornerPosNorm.x) {
        edgeDistance = min(fragmentPositionCorner.x, fragmentPositionCorner.z);
    } else {
        edgeDistance = min(fragmentPositionCorner.y, fragmentPositionCorner.z);
    }

    float lineWidth = lineWidthBase;
#ifdef SCREEN_SPACE_LINE_THICKNESS
    float fragmentDistance = length(fragmentPosition - cameraPosition);
    float distFactor = getAntialiasingFactor(fragmentDistance);
    lineWidth = lineWidth * distFactor * 1500;
#endif
    edgeDistance = edgeDistance / lineWidth;
    if (edgeDistance > 1.0) {
        discard;
    }

    //fragmentColor = vec4(vec3(nodeAabb.normalizedHierarchyLevel), 1.0);
    fragmentColor = texture(hierarchyLevelColorMap, nodeAabb.normalizedHierarchyLevel);
}
