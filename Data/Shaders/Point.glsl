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
layout(location = 0) out vec3 pointPosition;

void main() {
    pointPosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Geometry

#version 430 core

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

layout(binding = 0) uniform UniformDataBuffer {
    vec3 cameraPosition;
    float pointWidth;
    vec4 pointColor;
    vec3 foregroundColor;
    float padding;
};

layout(location = 0) in vec3 pointPosition[];

layout(location = 0) out vec3 fragmentPositionWorld;
layout(location = 1) out vec2 quadCoords; // Between -1 and 1

void main() {
    vec3 pointPosition = pointPosition[0];

    vec3 quadNormal = normalize(cameraPosition - pointPosition);
    vec3 vertexPosition;

    vec3 right = cross(quadNormal, vec3(0, 1, 0));
    vec3 top = cross(quadNormal, right);

    vertexPosition = pointPosition + pointWidth / 2.0 * (right - top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(1, -1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + pointWidth / 2.0 * (right + top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(1, 1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + pointWidth / 2.0 * (-right - top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(-1, -1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + pointWidth / 2.0 * (-right + top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(-1, 1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

layout(location = 0) in vec3 fragmentPositionWorld;
layout(location = 1) in vec2 quadCoords; // Between -1 and 1
layout(location = 0) out vec4 fragColor;

layout(binding = 0) uniform RenderSettingsBuffer {
    vec3 cameraPosition;
    float pointWidth;
    vec4 pointColor;
    vec3 foregroundColor;
    float padding;
};

void main() {
    float lengthCoords = length(quadCoords);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth / 2.0, 0.0, 0.49);
    //float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lengthCoords);
    float coverage = 1.0 - step(1.0, lengthCoords);

    if (coverage < 0.999) {
        discard;
    }

    fragColor = vec4(
            mix(
                pointColor.rgb, foregroundColor,
                smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lengthCoords)),
            pointColor.a * coverage);
}

