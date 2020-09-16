// See https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_shader_image_load_store.txt
#extension GL_ARB_shader_image_load_store : require

// Use early z-test to cull transparent fragments occluded by opaque fragments.
// Additionaly, use fragment interlock.
layout(early_fragment_tests) in;

// gl_FragCoord will be used for pixel centers at integer coordinates.
// See https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/gl_FragCoord.xhtml
layout(pixel_center_integer) in vec4 gl_FragCoord;

// A fragment node stores rendering information about one specific fragment
struct LinkedListFragmentNode {
    // RGBA color of the node
    uint color;
    // If no multisampling used: Depth value of the fragment (32-bit).
    // If multisampling used: Depth value of the fragment (24-bit) and coverage (8-bit).
    uint depth;
    // The index of the next node in "nodes" array
    uint next;
};

// fragment-and-link buffer and a start-offset buffer

// Fragment-and-link buffer (linked list). Stores "nodesPerPixel" number of fragments.
layout (std430, binding = 0) coherent buffer FragmentBuffer {
    LinkedListFragmentNode fragmentBuffer[];
};

// Start-offset buffer (mapping pixels to first pixel in the buffer) of size viewportW*viewportH.
layout (std430, binding = 1) coherent buffer StartOffsetBuffer {
    uint startOffset[];
};

// Position of the first free fragment node in the linked list
layout(binding = 0, offset = 0) uniform atomic_uint fragCounter;

// Number of fragments we can store in total
uniform uint linkedListSize;

uniform int viewportW;
//uniform int viewportH; // Not needed

#include "TiledAddress.glsl"
