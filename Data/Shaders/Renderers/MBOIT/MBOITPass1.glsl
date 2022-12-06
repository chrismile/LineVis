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

#define MOMENT_GENERATION 1
#define MOMENT_BASED_OIT

#include "MBOITHeader.glsl"
#include "MomentOIT.glsl"
#include "TiledAddress.glsl"

#if ROV

layout(location = 0) out vec4 fragColor;

#else

layout(location = 0) out float b_0;
#if NUM_MOMENTS == 4
layout(location = 1) out vec4 b;
#elif NUM_MOMENTS == 6
#if USE_R_RG_RGBA_FOR_MBOIT6
layout(location = 1) out vec2 b_12;
layout(location = 2) out vec4 b_3456;
#else
layout(location = 1) out vec2 b_12;
layout(location = 2) out vec2 b_34;
layout(location = 3) out vec2 b_56;
#endif
#elif NUM_MOMENTS == 8
layout(location = 1) out vec4 b_even;
layout(location = 2) out vec4 b_odd;
#endif

#endif

void gatherFragment(vec4 color) {
    float depth = logDepthWarp(-screenSpacePosition.z, logDepthMin, logDepthMax); // gl_FragCoord.z
    //float depth = gl_FragCoord.z * 2.0 - 1.0;
    /*if (depth < -0.5f || depth > 0.5f) {
        discard;
    }*/

    //float depth = gl_FragCoord.z * 2.0 - 1.0;
    float transmittance = 1.0 - color.a;
    //ivec2 addr2D = addrGen2D(ivec2(gl_FragCoord.xy));
    ivec2 addr2D = ivec2(gl_FragCoord.xy);

#if ROV
    memoryBarrierImage();
    generateMoments(depth, transmittance, addr2D, MomentOIT.wrapping_zone_parameters);
    fragColor = vec4(color);
#else
    generateMoments(
            depth, transmittance, MomentOIT.wrapping_zone_parameters,
#if NUM_MOMENTS == 4
            b_0, b
#elif NUM_MOMENTS == 6
#if USE_R_RG_RGBA_FOR_MBOIT6
            b_0, b_12, b_3456
#else
            b_0, b_12, b_34, b_56
#endif
#elif NUM_MOMENTS == 8
            b_0, b_even, b_odd
#endif
    );
#endif
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    float depthLog = logDepthWarp(-depth, logDepthMin, logDepthMax);

    /*if (depth < -0.5f || depth > 0.5f) {
        discard;
    }*/

    //float depth = gl_FragCoord.z * 2.0 - 1.0;
    float transmittance = 1.0 - color.a;
    //ivec2 addr2D = addrGen2D(ivec2(gl_FragCoord.xy));
    ivec2 addr2D = ivec2(gl_FragCoord.xy);

#if ROV
    memoryBarrierImage();
    generateMoments(depthLog, transmittance, addr2D, MomentOIT.wrapping_zone_parameters);
    fragColor = vec4(color);
#else
    generateMoments(
            depthLog, transmittance, MomentOIT.wrapping_zone_parameters,
#if NUM_MOMENTS == 4
            b_0, b
#elif NUM_MOMENTS == 6
#if USE_R_RG_RGBA_FOR_MBOIT6
            b_0, b_12, b_3456
#else
            b_0, b_12, b_34, b_56
#endif
#elif NUM_MOMENTS == 8
            b_0, b_even, b_odd
#endif
);
#endif
}
