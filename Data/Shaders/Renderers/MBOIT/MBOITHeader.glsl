/**
 * This file is part of an OpenGL GLSL port of the HLSL code accompanying the paper "Moment-Based Order-Independent
 * Transparency" by Münstermann, Krumpen, Klein, and Peters (http://momentsingraphics.de/?page_id=210).
 * The original code was released in accordance to CC0 (https://creativecommons.org/publicdomain/zero/1.0/).
 *
 * This port is released under the terms of BSD 2-Clause License. For more details please see the LICENSE file in the
 * root directory of this project.
 *
 * Changes for the OpenGL port: Copyright 2018 - 2019 Christoph Neuhauser
 */

#if defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK) && !defined(RESOLVE_PASS)
// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_fragment_shader_interlock.txt
//#extension GL_ARB_fragment_shader_interlock : require // Set in code using __extensions.
#ifdef PIXEL_SYNC_UNORDERED
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_unordered) in;
#else
// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests, pixel_interlock_ordered) in;
#endif
#else
// Use early z-test to cull transparent fragments occluded by opaque fragments.
layout(early_fragment_tests) in;
#endif

in vec4 gl_FragCoord;

layout(binding = 0) uniform UniformDataBuffer {
    // Size of viewport in x direction (in pixels).
    int viewportW;

    // Range of logarithmic depth.
    float logDepthMin;
    float logDepthMax;
};

// Maps depth to range [-1,1] with logarithmic scale
float logDepthWarp(float z, float logmin, float logmax) {
    return (log(z) - logmin) / (logmax - logmin) * 2.0 - 1.0;
    //return (z - exp(logmin)) / (exp(logmax) - exp(logmin)) * 2.0 - 1.0;
}
