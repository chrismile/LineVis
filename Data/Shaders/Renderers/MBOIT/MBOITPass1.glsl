
#define MOMENT_GENERATION 1
#define MOMENT_BASED_OIT

uniform int viewportW;

#include "MBOITHeader.glsl"
#include "MomentOIT.glsl"
#include "TiledAddress.glsl"

out vec4 fragColor;

void gatherFragment(vec4 color) {
    float depth = logDepthWarp(-screenSpacePosition.z, logDepthMin, logDepthMax); // gl_FragCoord.z

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
