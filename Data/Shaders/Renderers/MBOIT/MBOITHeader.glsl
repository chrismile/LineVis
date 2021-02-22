// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_shader_image_load_store.txt
#extension GL_ARB_shader_image_load_store : require

// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_fragment_shader_interlock.txt
#extension GL_ARB_fragment_shader_interlock : require

#pragma optionNV (unroll all)

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


uniform float logDepthMin;
uniform float logDepthMax;

// Maps depth to range [-1,1] with logarithmic scale
float logDepthWarp(float z, float logmin, float logmax)
{
    return (log(z) - logmin) / (logmax - logmin) * 2.0 - 1.0;
    //return (z - exp(logmin)) / (exp(logmax) - exp(logmin)) * 2.0 - 1.0;
}
