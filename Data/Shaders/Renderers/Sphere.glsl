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

-- Vertex

#version 450 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;

layout(location = 0) out vec3 fragmentPositionWorld;
layout(location = 1) out vec3 screenSpacePosition;
layout(location = 2) out vec3 fragmentNormal;

layout(binding = 0) uniform UniformDataBuffer {
    vec3 cameraPosition;
    float padding0;
    vec3 spherePosition;
    float sphereRadius;
    vec4 sphereColor;
    vec3 backgroundColor;
    float padding1;
    vec3 foregroundColor;
    float padding2;
};

void main() {
    vec4 sphereVertexPosition = vec4(vertexPosition * sphereRadius + spherePosition, 1.0);
    fragmentPositionWorld = (mMatrix * sphereVertexPosition).xyz;
    screenSpacePosition = (vMatrix * vec4(fragmentPositionWorld, 1.0)).xyz;
    fragmentNormal = vertexNormal;
    gl_Position = mvpMatrix * sphereVertexPosition;
}


-- Fragment

#version 450 core

layout(location = 0) in vec3 fragmentPositionWorld;
layout(location = 1) in vec3 screenSpacePosition;
layout(location = 2) in vec3 fragmentNormal;

layout(location = 0) out vec4 fragColor;

layout(binding = 0) uniform UniformDataBuffer {
    vec3 cameraPosition;
    float padding0;
    vec3 spherePosition;
    float sphereRadius;
    vec4 sphereColor;
    vec3 backgroundColor;
    float padding1;
    vec3 foregroundColor;
    float padding2;
};

#include "Lighting.glsl"

void main() {
    // Draw an outline.
    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    vec3 crossProdVn = cross(v, n);
    float ribbonPosition = length(crossProdVn);

    vec4 fragmentColor = blinnPhongShading(sphereColor, fragmentNormal);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth * 0.0005 / sphereRadius, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, ribbonPosition);
    vec4 colorOut = vec4(mix(fragmentColor.rgb, foregroundColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, ribbonPosition)),
            fragmentColor.a * coverage);

    fragColor = colorOut;
}


-- Vertex.Textured

#version 450 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexTexCoords;

layout(location = 0) out vec2 fragmentTexCoords;

void main() {
    fragmentTexCoords = vertexTexCoords;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment.Textured

#version 450 core

layout(location = 0) in vec2 fragmentTexCoords;

layout(location = 0) out vec4 fragColor;

layout(binding = 0) uniform sampler2D mollweideMapImage;

void main() {
    fragColor = texture(mollweideMapImage, fragmentTexCoords);
}
