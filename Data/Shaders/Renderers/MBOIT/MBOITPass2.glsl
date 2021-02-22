
#define MOMENT_GENERATION 0
#define MOMENT_BASED_OIT

uniform int viewportW;

#include "MBOITHeader.glsl"
#include "MomentOIT.glsl"
#include "TiledAddress.glsl"

out vec4 fragColor;

void gatherFragment(vec4 color) {
    float depth = logDepthWarp(-screenSpacePosition.z, logDepthMin, logDepthMax);
    //float depth = gl_FragCoord.z * 2.0 - 1.0;
    //ivec2 addr2D = addrGen2D(ivec2(gl_FragCoord.xy));
    ivec2 addr2D = ivec2(gl_FragCoord.xy);
    float transmittance_at_depth = 1.0;
    float total_transmittance = 1.0;  // exp(-b_0)
    resolveMoments(transmittance_at_depth, total_transmittance, depth, addr2D);

    // Normal back-to-front blending: c_out = c_src * alpha_src + c_dest * (1 - alpha_src)
    // Blended Color = exp(-b_0) * L_n + (1 - exp(-b_0))
    //      / (Sum from l=0 to n-1:       alpha_l * T(z_f, b, beta))
    //      * (Sum from l=0 to n-1: L_l * alpha_l * T(z_f, b, beta))
    // => Set alpha_src = (1 - exp(-b_0)),
    //    Premultiply c_src with 1 / (Sum from l=0 to n-1: alpha_l * T(z_f, b, beta))
    fragColor = vec4(color.rgb * color.a * transmittance_at_depth, color.a * transmittance_at_depth);
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    float depthLog = logDepthWarp(-depth, logDepthMin, logDepthMax);
    //float depth = gl_FragCoord.z * 2.0 - 1.0;
    //ivec2 addr2D = addrGen2D(ivec2(gl_FragCoord.xy));
    ivec2 addr2D = ivec2(gl_FragCoord.xy);
    float transmittance_at_depth = 1.0;
    float total_transmittance = 1.0;  // exp(-b_0)
    resolveMoments(transmittance_at_depth, total_transmittance, depthLog, addr2D);

    // Normal back-to-front blending: c_out = c_src * alpha_src + c_dest * (1 - alpha_src)
    // Blended Color = exp(-b_0) * L_n + (1 - exp(-b_0))
    //      / (Sum from l=0 to n-1:       alpha_l * T(z_f, b, beta))
    //      * (Sum from l=0 to n-1: L_l * alpha_l * T(z_f, b, beta))
    // => Set alpha_src = (1 - exp(-b_0)),
    //    Premultiply c_src with 1 / (Sum from l=0 to n-1: alpha_l * T(z_f, b, beta))
    fragColor = vec4(color.rgb * color.a * transmittance_at_depth, color.a * transmittance_at_depth);
}
