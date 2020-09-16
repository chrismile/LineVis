// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_shader_image_load_store.txt
#extension GL_ARB_shader_image_load_store : require

// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests) in;

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

// Stores the number of fragments using atomic operations.
layout(binding = 0) coherent buffer FragmentCounterBuffer {
    uint fragmentCounterBuffer[];
};

uniform int viewportW;
//uniform int viewportH; // Not needed

#include "TiledAddress.glsl"
