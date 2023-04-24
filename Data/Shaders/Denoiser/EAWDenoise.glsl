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
    int stepWidth;
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
#ifdef USE_POSITION_TEXTURE
layout(binding = 4) uniform sampler2D positionTexture;
#endif
#ifdef USE_NORMAL_TEXTURE
layout(binding = 5) uniform sampler2D normalTexture;
#endif

layout(location = 0) in vec2 fragTexCoord;
layout(location = 0) out vec4 fragColor;

void main() {
    ivec2 size = textureSize(colorTexture, 0);
    vec2 step = vec2(float(stepWidth)) / vec2(size);

    vec4 centerColor = texture(colorTexture, fragTexCoord);
#ifdef USE_POSITION_TEXTURE
    vec4 centerPosition = texture(positionTexture, fragTexCoord);
#endif
#ifdef USE_NORMAL_TEXTURE
    vec4 centerNormal = texture(normalTexture, fragTexCoord);
#endif

    vec4 sum = vec4(0.0);
    float accumW = 0.0;

    for (int i = 0; i < 25; i++) {
        /*
         * Computing the offset on-the-fly was faster on NVIDIA hardware due to lower amount of memory accesses and
         * consequentially a higher throughput due to low computational complexity of the shader code.
         */
        //vec2 offsetTexCoord = fragTexCoord + offset[i] * step;
        //float kernelValue = kernel[i];
        float x = float(i % 5 - 2);
        float y = float(i / 5 - 2);
        float kernelValue = exp(-float(x * x + y * y) / 2.0);
        vec2 offset = vec2(x, y);
        vec2 offsetTexCoord = fragTexCoord + offset * step;

        vec4 offsetColor = texture(colorTexture, offsetTexCoord);
        vec4 diffColor = centerColor - offsetColor;
        float distColor = dot(diffColor, diffColor);
        float weightColor = min(exp(-distColor / phiColor), 1.0);

#ifdef USE_POSITION_TEXTURE
        vec4 offsetPosition = texture(positionTexture, offsetTexCoord);
        vec4 diffPosition = centerPosition - offsetPosition;
        float distPosition = dot(diffPosition, diffPosition);
        float weightPosition = min(exp(-distPosition / phiPosition), 1.0);
#endif

#ifdef USE_NORMAL_TEXTURE
        vec4 offsetNormal = texture(normalTexture, offsetTexCoord);
        vec4 diffNormal = centerNormal - offsetNormal;
        float distNormal = dot(diffNormal, diffNormal);
        float weightNormal = min(exp(-distNormal / phiNormal), 1.0);
#endif

        float weight = 1.0;
        if (useColor > 0) {
            weight *= weightColor;
        }
#ifdef USE_POSITION_TEXTURE
        if (usePosition > 0) {
            weight *= weightPosition;
        }
#endif
#ifdef USE_NORMAL_TEXTURE
        if (useNormal > 0) {
            weight *= weightNormal;
        }
#endif
        sum += offsetColor * weight * kernelValue;
        accumW += weight * kernelValue;
    }

    // fragColor = sum / accumW;
    fragColor = vec4(0);
}


-- Compute

#version 450
#extension GL_EXT_scalar_block_layout : require

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;

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
    int stepWidth;
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


layout(binding = 3) uniform sampler2D colorTexture;
layout(binding = 4) uniform sampler2D positionTexture;
layout(binding = 5) uniform sampler2D normalTexture;

layout(binding = 6, rgba32f) uniform writeonly image2D outputImage;

void main() {

    ivec2 size = textureSize(colorTexture, 0);
    ivec2 i_pos = ivec2(gl_GlobalInvocationID.xy);


    if (i_pos.x >= size.x || i_pos.y >= size.y) {
        return;
    }

    vec4 centerColor    = texelFetch(colorTexture, i_pos, 0);
    vec4 centerPosition = texelFetch(positionTexture, i_pos, 0);
    vec4 centerNormal   = texelFetch(normalTexture, i_pos, 0);

    float kernel_values[3] = {1.0, 2.0 / 3.0, 1.0 / 6.0};

    float accumW = kernel_values[0] * kernel_values[0];
    vec4  sum    = centerColor * accumW;

    for (int y = -2; y <= 2; ++y) {
        for (int x = -2; x <= 2; ++x) {
            vec2  offset = vec2(x, y);
            ivec2 i_pos_offset = i_pos + ivec2(x, y) * stepWidth;
            const bool inside =
                i_pos_offset.x >= 0     && i_pos_offset.y >= 0 &&
                i_pos_offset.x < size.x && i_pos_offset.y < size.y;

            if (!inside || (x == 0 && y == 0))
                continue; // center pixel is already accumulated

            vec4  offset_color  = texelFetch(colorTexture,    i_pos_offset, 0);
            vec4  offset_pos    = texelFetch(positionTexture, i_pos_offset, 0);
            vec4  offset_normal = texelFetch(normalTexture,   i_pos_offset, 0);

            float kernelValue = kernel_values[abs(x)] * kernel_values[abs(y)];

            // color
            // vec4  diffColor        = centerColor - offset_color;
            // float distColor        = dot(diffColor, diffColor);
            // float color_weight_exp = distColor*stepWidth / (phiColor);

            // pos
            vec4  diffPosition   = centerPosition - offset_pos;
            float distPosition   = dot(diffPosition, diffPosition);
            float pos_weight_exp = distPosition / phiPosition;

            // normal
            vec4  diffNormal        = centerNormal - offset_normal;
            float distNormal        = dot(diffNormal, diffNormal);
            float normal_weight_exp = distNormal / phiNormal;

            float weight = exp(0.0
                               // -color_weight_exp
                               -pos_weight_exp
                               -normal_weight_exp
                              );

            sum += offset_color * weight * kernelValue;
            accumW += weight * kernelValue;
        }
    }

    imageStore(outputImage, i_pos, sum / accumW);
}
