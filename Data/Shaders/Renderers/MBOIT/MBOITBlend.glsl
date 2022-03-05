/**
 * This file is part of an Vulkan GLSL port of the HLSL code accompanying the paper "Moment-Based Order-Independent
 * Transparency" by Münstermann, Krumpen, Klein, and Peters (http://momentsingraphics.de/?page_id=210).
 * The original code was released in accordance to CC0 (https://creativecommons.org/publicdomain/zero/1.0/).
 *
 * This port is released under the terms of BSD 2-Clause License. For more details please see the LICENSE file in the
 * root directory of this project.
 *
 * Changes for the Vulkan GLSL port: Copyright 2018-2022 Christoph Neuhauser
 */

-- Vertex

#version 450 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = vec4(vertexPosition, 1.0);
}


-- Fragment

#version 450 core

layout(location = 0) in vec4 gl_FragCoord;
layout(location = 0) out vec4 fragColor;

layout(binding = 5) uniform UniformDataBuffer {
    // Size of viewport in x direction (in pixels).
    int viewportW;

    // Range of logarithmic depth.
    float logDepthMin;
    float logDepthMax;
};

#include "TiledAddress.glsl"

layout (binding = 0, r32f) coherent uniform image2DArray zeroth_moment; // float
#if SINGLE_PRECISION
#if NUM_MOMENTS == 6
layout (binding = 1, rg32f) coherent uniform image2DArray moments; // vec2
#if USE_R_RG_RGBA_FOR_MBOIT6
layout (binding = 2, rgba32f) coherent uniform image2DArray extra_moments; // vec4
#endif
#else
layout (binding = 1, rgba32f) coherent uniform image2DArray moments; // vec4
#endif
#else
#if NUM_MOMENTS == 6
layout (binding = 1, rg16) coherent uniform image2DArray moments;
#if USE_R_RG_RGBA_FOR_MBOIT6
layout (binding = 2, rgba16) coherent uniform image2DArray extra_moments;
#endif
#else
layout (binding = 1, rgba16) coherent uniform image2DArray moments;
#endif
#endif

layout(binding = 3) uniform sampler2D transparentSurfaceAccumulator;

void clearMoments(ivec3 idx0) {
    ivec3 idx1 = ivec3(idx0.xy, 1);
    ivec3 idx2 = ivec3(idx0.xy, 2);

    imageStore(zeroth_moment, idx0, vec4(0.0));
    imageStore(moments, idx0, vec4(0.0));
#if NUM_MOMENTS == 6
#if USE_R_RG_RGBA_FOR_MBOIT6
    imageStore(extra_moments, idx0, vec4(0.0));
#else
    imageStore(moments, idx1, vec4(0.0));
    imageStore(moments, idx2, vec4(0.0));
#endif
#elif NUM_MOMENTS == 8
    imageStore(moments, idx1, vec4(0.0));
#endif
}

void main() {
    //ivec2 addr2D = addrGen2D(ivec2(gl_FragCoord.xy));
    ivec2 addr2D = ivec2(gl_FragCoord.xy);
    ivec3 idx0Tiled = ivec3(addr2D, 0);
    ivec3 idx0 = ivec3(ivec2(gl_FragCoord.xy), 0);
    vec4 color = texelFetch(transparentSurfaceAccumulator, idx0.xy, 0);
    float b_0 = imageLoad(zeroth_moment, idx0Tiled).x;
    if (b_0 < 0.00100050033f) {
        discard;
    }
    float total_transmittance = exp(-b_0);
    if (isinf(b_0)) {
        total_transmittance = 1e7;
    }

    // Make sure data is cleared for next rendering pass
    clearMoments(idx0Tiled);

    //color_blend = exp(-b_0) * L_n + (1 - exp(-b_0)) * weighted_color
    fragColor = vec4(color.rgb / color.a, 1.0 - total_transmittance);
}

