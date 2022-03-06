-- Compute

#version 450 core

layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0) uniform RenderSettingsBuffer {
    vec3 minBoundingBox;
    float attenuationCoefficient;
    vec3 maxBoundingBox;
    float voxelSize;
};

layout (binding = 1, rgba8) uniform image2D outputImage;
layout (binding = 2) uniform sampler3D lineDensityField;

#include "RayIntersectionTests.glsl"
#include "Blending.glsl"
#include "LineUniformData.glsl"
#include "TransferFunction.glsl"

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

    vec3 rayOrigin = (inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec2 fragNdc = 2.0 * ((vec2(gl_GlobalInvocationID.xy) + vec2(0.5)) / vec2(outputImageSize)) - 1.0;
    vec3 rayTarget = (inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;

    vec4 outputColor;
    vec3 entrancePoint;
    vec3 exitPoint;

    float step_size = voxelSize / 10;


    float tNear, tFar;

    if (rayBoxIntersectionRayCoords(rayOrigin, rayDirection, minBoundingBox, maxBoundingBox, tNear, tFar))
    {
        vec3 iter_point = rayOrigin + rayDirection * tNear;
        entrancePoint = iter_point;
        exitPoint = rayOrigin + rayDirection * tFar;

        if (tNear < 0) {
            iter_point = rayOrigin;
        }


        outputColor = vec4(0);

        while (length(iter_point - entrancePoint) < length(exitPoint - entrancePoint)) {
            vec3 tex_coods = remap(minBoundingBox, iter_point, maxBoundingBox,
                                   vec3(0), vec3(1));

            // tex_coods = vec3(0.5,0.5,0.5);
            // float att_coeff = 200;
            float local_density = texture(lineDensityField, tex_coods).r;
            vec4 t_fun_color = transferFunction(local_density);
            float alpha = 1 - exp(-t_fun_color.a * step_size * attenuationCoefficient);
            // float alpha = 0.01;

            vec4 color = vec4(t_fun_color.rgb, alpha);

            if (blend(color, outputColor)) {
                break;
            }


            iter_point += rayDirection * step_size;
        }
        // density = clamp(density, 0, 1);


        // float opticalDepth = length(exitPoint - entrancePoint);
        // opticalDepth = clamp(opticalDepth, 0.0, 1.0);
        // outputColor = vec4(1,0,0, density);
        // outputColor = vec4(1,0,0, opticalDepth);
        blend(backgroundColor, outputColor);

        outputColor = vec4(outputColor.rgb / outputColor.a, outputColor.a);
    } else {
        outputColor = backgroundColor;
    }
    imageStore(outputImage, imageCoords, outputColor);
}
