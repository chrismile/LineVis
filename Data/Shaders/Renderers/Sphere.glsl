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

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;

out vec3 fragmentPositionWorld;
out vec3 screenSpacePosition;
out vec3 fragmentNormal;

uniform vec3 spherePosition;
uniform float sphereRadius;

void main() {
    vec4 sphereVertexPosition = vec4(vertexPosition * sphereRadius + spherePosition, 1.0);
    fragmentPositionWorld = (mMatrix * sphereVertexPosition).xyz;
    screenSpacePosition = (vMatrix * vec4(fragmentPositionWorld, 1.0)).xyz;
    fragmentNormal = vertexNormal;
    gl_Position = mvpMatrix * sphereVertexPosition;
}


-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in vec3 screenSpacePosition;
in vec3 fragmentNormal;

out vec4 fragColor;

uniform float sphereRadius;
uniform vec4 sphereColor;
uniform vec3 cameraPosition;
uniform vec3 backgroundColor;
uniform vec3 foregroundColor;

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
    vec4 colorOut = vec4(mix(fragmentColor.rgb, foregroundColor,
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, ribbonPosition)),
            fragmentColor.a * coverage);

    fragColor = colorOut;
}
