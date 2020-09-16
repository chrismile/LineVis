-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "FloatPack.glsl"
#include "LinkedListHeaderFinal.glsl"

uint colorList[MAX_NUM_FRAGS];
uint depthList[MAX_NUM_FRAGS];

#define DEPTH_TYPE_UINT
#include "LinkedListSort.glsl"

#ifdef USE_QUICKSORT
#include "LinkedListQuicksort.glsl"
#endif

out vec4 fragColor;

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Get start offset from array
    uint fragOffset = startOffset[pixelIndex];

    // Collect all fragments for this pixel
    int numFrags = 0;
    LinkedListFragmentNode fragment;
    for (int i = 0; i < MAX_NUM_FRAGS; i++) {
        if (fragOffset == -1) {
            // End of list reached
            break;
        }

        fragment = fragmentBuffer[fragOffset];
        fragOffset = fragment.next;

        colorList[i] = fragment.color;
        depthList[i] = fragment.depth;

        numFrags++;
    }

    if (numFrags == 0) {
        discard;
    }

    fragColor = sortingAlgorithm(numFrags);
}
