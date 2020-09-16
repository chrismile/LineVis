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

uniform sampler2D texture;
in vec2 fragTexCoord;
out vec4 fragColorOut;

// Values for perfect weights and offsets that utilize bilinear texture filtering
// are from http://rastergrid.com/blog/2010/09/efficient-gaussian-blur-with-linear-sampling/
uniform float offsets[3] = float[](0.0, 1.3846153846, 3.2307692308);
uniform float weights[3] = float[](0.2270270270, 0.3162162162, 0.0702702703);
uniform bool horzBlur;
uniform vec2 texSize;

void main()
{
    vec4 fragColor = texture2D(texture, fragTexCoord) * weights[0];
    for (int i = 1; i < 3; i++) {
        vec2 offset;
        if (horzBlur) {
            offset = vec2(offsets[i] / texSize.x, 0.0) ;
        } else {
            offset = vec2(0.0, offsets[i] / texSize.y);
        }
        fragColor += texture2D(texture, fragTexCoord+offset) * weights[i];
        fragColor += texture2D(texture, fragTexCoord-offset) * weights[i];
    }
    fragColorOut = vec4(fragColor.xyz, 1.0);
}
