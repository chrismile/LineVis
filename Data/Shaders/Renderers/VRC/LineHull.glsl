-- Vertex

#version 430 core

in vec3 vertexPosition;

uniform mat4 voxelSpaceToWorldSpace;

void main() {
    gl_Position = mvpMatrix * voxelSpaceToWorldSpace * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

out vec4 fragColor;

void main() {
    fragColor = vec4(0.0, 0.0, 0.0, 1.0);
}
