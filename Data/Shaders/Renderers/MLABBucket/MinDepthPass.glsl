// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_fragment_shader_interlock.txt
#extension GL_ARB_fragment_shader_interlock : require

#ifdef PIXEL_SYNC_UNORDERED
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_unordered) in;
#else
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_ordered) in;
#endif

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

uniform int viewportW;

uniform float logDepthMin;
uniform float logDepthMax;

// Maps depth to range [0,1] with logarithmic scale
float logDepthWarp(float z) {
    return (log(z) - logDepthMin) / (logDepthMax - logDepthMin);
    //return (z - exp(logmin)) / (exp(logmax) - exp(logmin));
}

struct MinDepthNode {
    float minDepth;
    float minOpaqueDepth;
};

layout (std430, binding = 1) coherent buffer MinDepthBuffer {
    MinDepthNode depthBuffer[];
};

#include "TiledAddress.glsl"

//#define OPACITY_THRESHOLD 0.3
//#define OPACITY_OPAQUE_THRESHOLD 0.98
uniform float lowerBackBufferOpacity; // default 0.25
uniform float upperBackBufferOpacity; // default 0.98


void gatherFragment(vec4 color) {
    ivec2 fragPos2D = ivec2(int(gl_FragCoord.x), int(gl_FragCoord.y));
    uint pixelIndex = addrGen(uvec2(fragPos2D));

    float depthLog = logDepthWarp(-screenSpacePosition.z);

    MinDepthNode depthInfo = depthBuffer[pixelIndex];

    MinDepthNode newDepthInfo;
    newDepthInfo.minDepth = depthInfo.minDepth;
    newDepthInfo.minOpaqueDepth = depthInfo.minOpaqueDepth;

    // estimate boundary of front buffer
    if (color.a > lowerBackBufferOpacity && depthLog < depthInfo.minDepth) {
        newDepthInfo.minDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }

    // estimate boundary of back buffer
    if (color.a >= upperBackBufferOpacity && depthLog < depthInfo.minOpaqueDepth) {
        newDepthInfo.minOpaqueDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    ivec2 fragPos2D = ivec2(int(gl_FragCoord.x), int(gl_FragCoord.y));
    uint pixelIndex = addrGen(uvec2(fragPos2D));

    float depthLog = logDepthWarp(-depth);

    MinDepthNode depthInfo = depthBuffer[pixelIndex];

    MinDepthNode newDepthInfo;
    newDepthInfo.minDepth = depthInfo.minDepth;
    newDepthInfo.minOpaqueDepth = depthInfo.minOpaqueDepth;

    // estimate boundary of front buffer
    if (color.a > lowerBackBufferOpacity && depthLog < depthInfo.minDepth) {
        newDepthInfo.minDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }

    // estimate boundary of back buffer
    if (color.a >= upperBackBufferOpacity && depthLog < depthInfo.minOpaqueDepth) {
        newDepthInfo.minOpaqueDepth = depthLog;
        depthBuffer[pixelIndex] = newDepthInfo;
    }
}
