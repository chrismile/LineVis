/**
 * Based on: Morgan McGuire and Louis Bavoil, Weighted Blended Order-Independent Transparency, Journal of Computer
 * Graphics Techniques (JCGT), vol. 2, no. 2, 122-141, 2013.
 *
 * For more details regarding the implementation see:
 * http://casual-effects.blogspot.com/2015/03/implemented-weighted-blended-order.html
 */

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"

in vec4 gl_FragCoord;

layout(location = 0) out vec4 accumulatedColor;
layout(location = 1) out float revealage;

void gatherFragment(vec4 color) {
    vec4 premultipliedColor = vec4(color.rgb * color.a, color.a);
    //vec3 transmit = vec3(0.0);
    //premultipliedColor.a *= 1.0 - clamp((transmit.r + transmit.g + transmit.b) * (1.0 / 3.0), 0, 1);
    float a = min(1.0, premultipliedColor.a) * 8.0 + 0.01;
    float b = -gl_FragCoord.z * 0.95 + 1.0;
    float w = clamp(a * a * a * 1e8 * b * b * b, 1e-2, 3e2);
    accumulatedColor = premultipliedColor * w;
    revealage = premultipliedColor.a;
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    vec4 premultipliedColor = vec4(color.rgb * color.a, color.a);
    //vec3 transmit = vec3(0.0);
    //premultipliedColor.a *= 1.0 - clamp((transmit.r + transmit.g + transmit.b) * (1.0 / 3.0), 0, 1);
    float a = min(1.0, premultipliedColor.a) * 8.0 + 0.01;
    float b = -convertLinearDepthToDepthBufferValue(depth) * 0.95 + 1.0;
    float w = clamp(a * a * a * 1e8 * b * b * b, 1e-2, 3e2);
    accumulatedColor = premultipliedColor * w;
    revealage = premultipliedColor.a;
}
