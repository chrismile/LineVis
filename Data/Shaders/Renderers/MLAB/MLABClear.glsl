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

void main()
{
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    clearPixel(addrGen(uvec2(x,y)));
}
