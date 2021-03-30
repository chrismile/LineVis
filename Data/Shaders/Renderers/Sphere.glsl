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
