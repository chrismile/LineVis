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
#extension GL_EXT_scalar_block_layout : require

/**
 * An image denoiser based on the following paper.
 *
 * H. Dammertz, D. Sewtz, J. Hanika, and H. P. A. Lensch. Edge-avoiding À-trous wavelet transform for fast global
 * illumination filtering. In Proceedings of the Conference on High Performance Graphics, HPG '10, page 67-75,
 * Goslar, DEU, 2010. Eurographics Association.
 *
 * For more details, please also refer to:
 * https://www.highperformancegraphics.org/previous/www_2010/media/RayTracing_I/HPG2010_RayTracing_I_Dammertz.pdf
 */

layout(push_constant) uniform PushConstants {
    float stepWidth;
};

layout(binding = 0) uniform UniformBuffer {
    float phiColor;
    float phiPosition;
    float phiNormal;
    float paddingFloat;
    uint useColor;
    uint usePosition;
    uint useNormal;
    uint paddingUint;
};

layout(binding = 1, scalar) uniform KernelBuffer {
    float kernel[25];
};

layout(binding = 2, scalar) uniform OffsetBuffer {
    vec2 offset[25];
};

layout(binding = 3) uniform sampler2D colorTexture;
layout(binding = 4) uniform sampler2D positionTexture;
layout(binding = 5) uniform sampler2D normalTexture;

layout(location = 0) in vec2 fragTexCoord;
layout(location = 0) out vec4 fragColor;

void main() {
    ivec2 size = textureSize(colorTexture, 0);
    vec2 step = vec2(1.0) / vec2(size);

    vec4 centerColor = texture(colorTexture, fragTexCoord);
    vec4 centerPosition = texture(positionTexture, fragTexCoord);
    vec4 centerNormal = texture(normalTexture, fragTexCoord);

    vec4 sum = vec4(0.0);
    float accumW = 0.0;

    for (int i = 0; i < 25; i++) {
        vec2 offsetTexCoord = fragTexCoord + offset[i] * step * stepWidth;

        vec4 offsetColor = texture(colorTexture, offsetTexCoord);
        vec4 diffColor = centerColor - offsetColor;
        float distColor = dot(diffColor, diffColor);
        float weightColor = min(exp(-distColor / phiColor), 1.0);

        vec4 offsetPosition = texture(positionTexture, offsetTexCoord);
        vec4 diffPosition = centerPosition - offsetPosition;
        float distPosition = dot(diffPosition, diffPosition);
        float weightPosition = min(exp(-distPosition / phiPosition), 1.0);

        vec4 offsetNormal = texture(normalTexture, offsetTexCoord);
        vec4 diffNormal = centerNormal - offsetNormal;
        float distNormal = dot(diffNormal, diffNormal);
        float weightNormal = min(exp(-distNormal / phiNormal), 1.0);

        float kernelValue = kernel[i];
        float weight = 1.0;
        if (useColor > 0) {
            weight *= weightColor;
        }
        if (usePosition > 0) {
            weight *= weightPosition;
        }
        if (useNormal > 0) {
            weight *= weightNormal;
        }
        sum += offsetColor * weight * kernelValue;
        accumW += weight * kernelValue;
    }

    fragColor = sum / accumW;
}
