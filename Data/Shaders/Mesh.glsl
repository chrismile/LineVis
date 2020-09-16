-- Vertex.Plain

#version 430 core

in vec4 vertexPosition;

void main()
{
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.Plain

#version 430 core

uniform vec4 color;
out vec4 fragColor;

void main()
{
    fragColor = color;
}



-- Vertex.Textured

#version 430 core

in vec4 vertexPosition;
in vec2 texcoord;
out vec2 fragTexCoord;

void main()
{
    fragTexCoord = texcoord;
    gl_Position = mvpMatrix * vertexPosition;
}

-- Fragment.Textured

#version 430 core

uniform sampler2D texture;
uniform vec4 color;
in vec2 fragTexCoord;
out vec4 fragColor;

void main()
{
    fragColor = color * texture2D(texture, fragTexCoord);
}
