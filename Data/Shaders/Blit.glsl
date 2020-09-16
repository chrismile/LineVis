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
out vec4 fragColor;

void main()
{
    fragColor = texture2D(texture, fragTexCoord);
}
