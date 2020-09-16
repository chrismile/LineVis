-- Vertex

#version 430 core

in vec4 position;
in vec2 texcoord;
out vec2 fragTexCoord;

void main()
{
    fragTexCoord = texcoord;
    gl_Position = mvpMatrix * position;
}

-- Fragment

#version 430 core

uniform sampler2DMS texture;
uniform int numSamples;
in vec2 fragTexCoord;
out vec4 fragColor;

void main()
{
    ivec2 size = textureSize(texture);
    ivec2 iCoords = ivec2(int(fragTexCoord.x*size.x), int(fragTexCoord.y*size.y));
    vec3 color = vec3(0, 0, 0); 
    for (int currSample = 0; currSample < numSamples; currSample++) {
        color += texelFetch(texture, iCoords, currSample).rgb;
    }
    fragColor = vec4(color / numSamples, 1);
}
