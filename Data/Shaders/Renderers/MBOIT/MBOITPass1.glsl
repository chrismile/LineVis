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

layout(location = 0) out vec4 fragColor;

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

    memoryBarrierImage();
    generateMoments(depth, transmittance, addr2D, MomentOIT.wrapping_zone_parameters);

    fragColor = vec4(color);
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

    memoryBarrierImage();
    generateMoments(depthLog, transmittance, addr2D, MomentOIT.wrapping_zone_parameters);

    fragColor = vec4(color);
}
