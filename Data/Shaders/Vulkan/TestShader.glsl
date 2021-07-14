-- Vertex

#version 450

layout(location = 0) in vec3 vertexPosition;

void main() {
    gl_Position =  vec4(vertexPosition, 1.0);
}

-- Fragment

#version 450

layout(location = 0) out vec4 outColor;

void main() {
    outColor = vec4(1.0, 0.0, 0.0, 1.0);
}
