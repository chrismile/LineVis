-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main()
{
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "MLABHeader.glsl"
#include "ColorPack.glsl"
#include "TiledAddress.glsl"

out vec4 fragColor;

void main() {
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    memoryBarrierBuffer();

    // Read data from SSBO
    MLABFragmentNode nodeArray[MAX_NUM_LAYERS+1];
    loadFragmentNodes(pixelIndex, nodeArray);

    // Read data from SSBO
    vec3 color = vec3(0.0, 0.0, 0.0);
    float transmittance = 1.0;
    for (uint i = 0; i < MAX_NUM_LAYERS; i++) {
        // Blend the accumulated color with the color of the fragment node
        vec4 colorSrc = unpackUnorm4x8(nodeArray[i].premulColor);
        color.rgb = color.rgb + transmittance * colorSrc.rgb;
        transmittance *= colorSrc.a;
    }

    // Make sure data is cleared for next rendering pass
    clearPixel(pixelIndex);

    float alphaOut = 1.0 - transmittance;
    fragColor = vec4(color.rgb / alphaOut, alphaOut);
    //fragColor = vec4(vec3(alphaOut), 1.0); // Output opacity
}
