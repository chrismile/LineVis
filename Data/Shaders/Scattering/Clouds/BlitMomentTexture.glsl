-- Vertex

#version 450

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexTexCoord;
layout(location = 0) out vec2 fragTexCoord;

void main() {
    fragTexCoord = vertexTexCoord;
    gl_Position = vec4(vertexPosition, 1.0);
}

-- Fragment

#version 450

layout(binding = 0) uniform sampler2DArray inputTexture;
layout(location = 0) in vec2 fragTexCoord;
layout(location = 0) out vec4 fragColor;

layout(push_constant) uniform PushConstants {
    int momentIndex;
};

void main() {
    float momentValue = texture(inputTexture, vec3(fragTexCoord, float(momentIndex))).r;
    if (momentIndex != 0) {
        momentValue /= max(texture(inputTexture, vec3(fragTexCoord, 0.0)).r, 1e-6);
        momentValue = 1.0 - momentValue;
    } else {
        momentValue = exp(-momentValue);
    }
    fragColor = vec4(vec3(momentValue), 1.0);
}
