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
    vec4 color = vec4(0.0);
    vec4 totalSum = vec4(0.0);
    for (int currSample = 0; currSample < numSamples; currSample++) {
        vec4 fetchedColor = texelFetch(texture, iCoords, currSample);
        totalSum += fetchedColor;
        color.rgb += fetchedColor.rgb * fetchedColor.a;
        color.a += fetchedColor.a;
    }
    color /= float(numSamples);
    //fragColor = vec4(color.rgb, color.a);
    if (color.a > 1.0 / 256.0) {
        fragColor = vec4(color.rgb / color.a, color.a);
    } else {
        fragColor = totalSum / float(numSamples);
    }
}
