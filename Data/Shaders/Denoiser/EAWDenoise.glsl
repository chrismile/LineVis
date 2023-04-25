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

layout(binding = 6, rgba32f) uniform writeonly image2D outputImage;

#ifdef USE_COLOR_TEXTURE
shared vec4 sharedMemColors[BLOCK_SIZE * BLOCK_SIZE];
#endif
#ifdef USE_POSITION_TEXTURE
shared vec4 sharedMemPositions[BLOCK_SIZE * BLOCK_SIZE];
#endif
#ifdef USE_NORMAL_TEXTURE
shared vec4 sharedMemNormals[BLOCK_SIZE * BLOCK_SIZE];
#endif

void main() {
    ivec2 size = textureSize(colorTexture, 0);
    ivec2 localIdx = ivec2(gl_LocalInvocationID.xy);
    ivec2 globalIdx = ivec2(gl_GlobalInvocationID.xy);

    if (globalIdx.x >= size.x || globalIdx.y >= size.y) {
        return;
    }
    
    vec4 centerColor = texelFetch(colorTexture, globalIdx, 0);
#ifdef USE_COLOR_TEXTURE
    sharedMemColors[localIdx.x + BLOCK_SIZE * localIdx.y] = centerColor;
#endif
#ifdef USE_POSITION_TEXTURE
    vec4 centerPosition = texelFetch(positionTexture, globalIdx, 0);
    sharedMemPositions[localIdx.x + BLOCK_SIZE * localIdx.y] = centerPosition;
#endif
#ifdef USE_NORMAL_TEXTURE
    vec4 centerNormal = texelFetch(normalTexture, globalIdx, 0);
    sharedMemNormals[localIdx.x + BLOCK_SIZE * localIdx.y] = centerNormal;
#endif
    memoryBarrierShared();
    barrier();

    const float kernel_values[3] = {1.0, 2.0 / 3.0, 1.0 / 6.0};

    float accumW = kernel_values[0] * kernel_values[0];
    vec4  sum    = centerColor * accumW;

    for (int y = -2; y <= 2; ++y) {
        for (int x = -2; x <= 2; ++x) {
            vec2  offset = vec2(x, y);
            ivec2 globalIdxOffset = globalIdx + ivec2(x, y) * stepWidth;
            const bool inside =
                globalIdxOffset.x >= 0     && globalIdxOffset.y >= 0 &&
                globalIdxOffset.x < size.x && globalIdxOffset.y < size.y;

            if (!inside || (x == 0 && y == 0))
                continue; // center pixel is already accumulated

            ivec2 localReadIdx = ivec2(localIdx.x + x * stepWidth, localIdx.y + y * stepWidth);
            int sharedMemIdx = localReadIdx.x + BLOCK_SIZE * localReadIdx.y;
            bool isInSharedMemory =
                    localReadIdx.x >= 0 && localReadIdx.y >= 0
                    && localReadIdx.x < BLOCK_SIZE && localReadIdx.y < BLOCK_SIZE;
            
            float kernelValue = kernel_values[abs(x)] * kernel_values[abs(y)];

#ifdef USE_COLOR_TEXTURE
            vec4 offsetColor;
            if (isInSharedMemory) {
                offsetColor = sharedMemColors[sharedMemIdx];
            } else {
                offsetColor = texelFetch(colorTexture, globalIdxOffset, 0);
            }
            vec4  diffColor        = centerColor - offsetColor;
            float distColor        = dot(diffColor, diffColor);
            float color_weight_exp = distColor*stepWidth / (phiColor);
#endif

#ifdef USE_POSITION_TEXTURE
            vec4 offsetPosition;
            if (isInSharedMemory) {
                offsetPosition = sharedMemPositions[sharedMemIdx];
            } else {
                offsetPosition = texelFetch(positionTexture, globalIdxOffset, 0);
            }
            vec4  diffPosition   = centerPosition - offsetPosition;
            float distPosition   = dot(diffPosition, diffPosition);
            float pos_weight_exp = distPosition / phiPosition;
#endif

#ifdef USE_NORMAL_TEXTURE
            vec4 offsetNormal;
            if (isInSharedMemory) {
                offsetNormal = sharedMemNormals[sharedMemIdx];
            } else {
                offsetNormal = texelFetch(normalTexture, globalIdxOffset, 0);
            }
            vec4  diffNormal        = centerNormal - offsetNormal;
            float distNormal        = dot(diffNormal, diffNormal);
            float normal_weight_exp = distNormal / phiNormal;
#endif

            float weight = exp(0.0
#ifdef USE_COLOR_TEXTURE
                               -color_weight_exp
#endif
#ifdef USE_NORMAL_TEXTURE
                               -pos_weight_exp
#endif
#ifdef USE_NORMAL_TEXTURE
                               -normal_weight_exp
#endif
            );

            sum += offsetColor * weight * kernelValue;
            accumW += weight * kernelValue;
        }
    }

    imageStore(outputImage, globalIdx, sum / accumW);
}
