-- Compute

#version 450

layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0) uniform CameraSettingsBuffer {
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
} camera;

layout(binding = 1) uniform RenderSettingsBuffer {
    vec4 backgroundColor;
    vec3 minBoundingBox; float padding0;
    vec3 maxBoundingBox; float padding1;
};

layout (binding = 2, rgba8) uniform image2D outputImage;
layout (binding = 3) uniform sampler3D lineDensityField;

#include "RayIntersectionTests.glsl"
#include "Blending.glsl"

void main() {
    ivec2 outputImageSize = imageSize(outputImage);
    ivec2 imageCoords = ivec2(gl_GlobalInvocationID.xy);
    if (imageCoords.x >= outputImageSize.x || imageCoords.y >= outputImageSize.y) {
        return;
    }

    vec3 rayOrigin = (camera.inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec2 fragNdc = 2.0 * ((vec2(gl_GlobalInvocationID.xy) + vec2(0.5)) / vec2(outputImageSize)) - 1.0;
    vec3 rayTarget = (camera.inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (camera.inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;

    vec4 outputColor;
    vec3 entrancePoint;
    vec3 exitPoint;
    if (rayBoxIntersection(rayOrigin, rayDirection, minBoundingBox, maxBoundingBox, entrancePoint, exitPoint)) {
        float opticalDepth = length(exitPoint - entrancePoint);
        opticalDepth = clamp(opticalDepth, 0.0, 1.0);
        outputColor = vec4(opticalDepth, 0.0, 0.0, opticalDepth);
        blend(backgroundColor, outputColor);
    } else {
        outputColor = backgroundColor;
    }
    imageStore(outputImage, imageCoords, outputColor);
}
