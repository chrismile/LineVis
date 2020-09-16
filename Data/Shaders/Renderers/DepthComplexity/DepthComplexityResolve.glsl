-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "DepthComplexityHeader.glsl"

// The number of fragments necessary to reach the maximal color opacity.
uniform uint numFragmentsMaxColor;

// The color to shade the framents with
uniform vec4 color;

out vec4 fragColor;

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));
    
    uint numFragments = fragmentCounterBuffer[pixelIndex];
    float percentage = clamp(float(numFragments)/float(min(numFragmentsMaxColor, 512)), 0.0, 1.0)*color.a;
    
    vec4 color = vec4(color.rgb, percentage);
    fragColor = color;
}
