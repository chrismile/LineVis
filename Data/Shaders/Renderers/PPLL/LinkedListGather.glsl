
#include "LinkedListHeader.glsl"

out vec4 fragColor;

void gatherFragment(vec4 color) {
    if (color.a < 0.001) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.depth = length(fragmentPositionWorld - cameraPosition);//gl_FragCoord.z;//
    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    if (color.a < 0.001) {
#ifndef GATHER_NO_DISCARD
        discard;
#else
        return;
#endif
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.depth = depth;
    frag.next = -1;

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}
