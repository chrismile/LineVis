#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"

uniform sampler2D depthReadTexture;
uniform int iteration;

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

out vec4 fragColor;

void gatherFragment(vec4 color) {
    float previousDepthValue = texelFetch(depthReadTexture, ivec2(gl_FragCoord.xy), 0).x;
    if (gl_FragCoord.z <= previousDepthValue && iteration != 0) {
        discard;
    }

    fragColor = vec4(color.rgb * color.a, color.a);
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    float previousDepthValue = texelFetch(depthReadTexture, ivec2(gl_FragCoord.xy), 0).x;
    if (convertLinearDepthToDepthBufferValue(depth) <= previousDepthValue && iteration != 0) {
        discard;
    }

    fragColor = vec4(color.rgb * color.a, color.a);
}
