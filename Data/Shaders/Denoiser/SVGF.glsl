-- Compute

#version 450
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_debug_printf : enable

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;


layout(push_constant) uniform PushConstants {
    int iteration;
    float z_multiplier;
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

// this frame
layout(binding = 3) uniform sampler2D color_texture;
layout(binding = 4) uniform sampler2D normal_texture;
layout(binding = 5) uniform sampler2D depth_texture;
layout(binding = 6) uniform sampler2D motion_texture;
// TODO(Felix): meshid??

// last frame
layout(binding = 8)  uniform sampler2D color_history_texture;
layout(binding = 9)  uniform sampler2D variance_history_texture;
layout(binding = 10) uniform sampler2D depth_history_texture;
// layout(binding = 11) uniform sampler2D normals_history_texture;
// TODO(Felix): meshid??


// output
layout(binding = 7, rgba32f) uniform writeonly image2D outputImage;

void main() {
    int stepWidth = 1 << iteration;
    ivec2 size = textureSize(color_texture, 0);
    vec2 step = vec2(float(stepWidth)) / vec2(size);
    ivec2 localIdx = ivec2(gl_LocalInvocationID.xy);
    ivec2 globalIdx = ivec2(gl_GlobalInvocationID.xy);
    vec2 fragTexCoord = (vec2(gl_GlobalInvocationID.xy) + vec2(0.5, 0.5)) / vec2(size);

    vec4 centerColor = texture(color_texture, fragTexCoord);
    vec3 center_normal = texture(normal_texture, fragTexCoord).rgb;
    float center_z = texture(depth_texture, fragTexCoord).r;


    float kernel_values[5] = {1.0/16.0, 1.0/4.0, 3.0/8.0, 1.0/4.0, 1.0/16.0};

    // memoryBarrierShared();
    // barrier();

    if (globalIdx.x >= size.x || globalIdx.y >= size.y) {
        return;
    }

    vec4 sum = vec4(0.0);
    float accumW = 0.0;

    if (iteration == 4) {
        // debugPrintfEXT("frame");
    }

    for (int i = 0; i < 25; i++) {
        /*
         * Computing the offset on-the-fly was faster on NVIDIA hardware due to lower amount of memory accesses and
         * consequentially a higher throughput due to low computational complexity of the shader code.
         */
        //vec2 offsetTexCoord = fragTexCoord + offset[i] * step;
        //float kernelValue = kernel[i];
        int x = i % 5 - 2;
        int y = i / 5 - 2;

        float kernelValue = kernel_values[x+2] * kernel_values[y+2];

        vec2 offset = vec2(x, y);
        vec2  offsetTexCoord = fragTexCoord + offset * step;
        vec4  offset_color = texture(color_texture, offsetTexCoord);
        vec3  offset_normal = texture(normal_texture, offsetTexCoord).rgb;
        float offset_z   = texture(depth_texture, offsetTexCoord).x;
        float offset_z_x = texture(depth_texture, offsetTexCoord+vec2(step.x, 0)).x;
        float offset_z_y = texture(depth_texture, offsetTexCoord+vec2(0, step.y)).x;

        // sigmas
        float sigma_n = 128;
        float sigma_z = 128;
        float sigma_l = 4;

        float weight_n = 1;
        {
            // weight_n = pow(max(0, dot(center_normal, offset_normal)), sigma_n);

            // vec3 diff_normal = center_normal - offset_normal;
            // float distNormal = dot(diff_normal, diff_normal);
            // float weight_n   = min(exp(-distNormal / 0.1), 1.0);


            // if (iteration == 4 && globalIdx == ivec2(1167, 243)) {
            //     // debugPrintfEXT("weight_n: %f", weight_n);
            // }

        }

        float weight_z = 1;
        {
            // vec2 nabla_z = vec2(offset_z_x - offset_z, offset_z_y - offset_z);

            // weight_z = exp(-abs(offset_z - center_z)
            //                / // ----------------------
            //                (sigma_z* abs(dot(nabla_z, )) + 0.0001));

            weight_z = exp(- abs(offset_z - center_z) * z_multiplier
                           /
                           (max(5e-3, 1e-8) * stepWidth * sqrt(dot(vec2(x, y), vec2(x, y)))));
        }

        float weight_l = 1;
        {

        }


        float weight = min((weight_n * weight_z * weight_l), 0.00001); // NOTE(Felix): so we cant devide by 0

        sum += offset_color * weight * kernelValue;
        accumW += weight * kernelValue;

    }


    imageStore(outputImage, globalIdx, sum / accumW);
}
