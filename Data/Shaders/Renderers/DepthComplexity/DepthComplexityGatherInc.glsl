
#include "DepthComplexityHeader.glsl"

void gatherFragment(vec4 color) {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));
    atomicAdd(fragmentCounterBuffer[pixelIndex], 1u);
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));
    atomicAdd(fragmentCounterBuffer[pixelIndex], 1u);
}
