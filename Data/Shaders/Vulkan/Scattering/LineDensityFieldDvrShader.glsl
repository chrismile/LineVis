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

vec3 lerp(vec3 point_a, vec3 t, vec3 point_b) {
    return point_a + t * (point_b - point_a);
}

vec3 unlerp(vec3 source_range_start, vec3 source_point, vec3 source_range_end) {
    return    (source_point - source_range_start) /
           /*--------------------------------------*/
            (source_range_end - source_range_start);
}

vec3 remap(vec3 source_range_start, vec3 source_point, vec3 source_range_end,
           vec3 dest_range_start, vec3 dest_range_end)
{
    return lerp(dest_range_start,
                unlerp(source_range_start, source_point, source_range_end),
                dest_range_end);
}


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

    float step_width = 0.005;

    if (rayBoxIntersection(rayOrigin, rayDirection, minBoundingBox, maxBoundingBox, entrancePoint, exitPoint)) {

        vec3 iter_point = entrancePoint + 0.01*rayDirection;
        float density = 0;

        while (boxContainsPoint(iter_point, minBoundingBox, maxBoundingBox)) {
            // vec3 tex_coods = remap(minBoundingBox, iter_point, maxBoundingBox,
                                   // vec3(0), vec3(1));
            // tex_coods = clamp(tex_coods, vec3(0), vec3(1));

            vec3 tex_coods = vec3(0.5,0.5,0.5); // DEBUG

            float local_density = texture(lineDensityField, tex_coods).r;
            if (isinf(local_density)){
                imageStore(outputImage, imageCoords, vec4(1,1,0,1));
                return;
            }
            if (isnan(local_density)) {
                imageStore(outputImage, imageCoords, vec4(1,0,1,1));
                return;
            }
            density += clamp(local_density, 0, 1);
            density +=  0.003; // DEBUG
            iter_point += rayDirection * step_width;
        }
        density = clamp(density, 0, 1);


        float opticalDepth = length(exitPoint - entrancePoint);
        opticalDepth = clamp(opticalDepth, 0.0, 1.0);
        outputColor = vec4(1,0,0, density);
        // outputColor = vec4(1,0,0, opticalDepth);
        blend(backgroundColor, outputColor);
    } else {
        outputColor = backgroundColor;
    }
    imageStore(outputImage, imageCoords, outputColor);
}
