// --------------------------------------
//           Reprojection
// --------------------------------------
-- Compute-Reproject
#version 450
#extension GL_EXT_scalar_block_layout : require
layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;

// this frame
layout(binding = 1) uniform sampler2D noisy_texture;
layout(binding = 2) uniform sampler2D color_history; // TODO(Felix): should we accum onto the fully denoised??

layout(binding = 3) uniform sampler2D motion_texture;

layout(binding = 4) uniform sampler2D normal_texture;
layout(binding = 5) uniform sampler2D depth_texture;

layout(binding = 6) uniform sampler2D normal_history_texture;
layout(binding = 7) uniform sampler2D depth_history_texture;

layout(binding = 8, rgba32f) uniform writeonly image2D temp_accum_texture;

layout(push_constant) uniform PushConstants {
    float allowed_z_dist;
    float allowed_normal_dist;
};


void main() {

    ivec2 size = textureSize(noisy_texture, 0);
    ivec2 globalIdx = ivec2(gl_GlobalInvocationID.xy);
    vec2 fragTexCoord = (globalIdx + vec2(0.5, 0.5)) / vec2(size);

    vec4 color                   = texture(noisy_texture, fragTexCoord);
    // NOTE(Felix): +0.5 because motion texture stores -.5 on no motion
    vec2 fragTexCoord_last_frame = fragTexCoord - ((texture(motion_texture, fragTexCoord).xy + vec2(0.5)) / vec2(size));

    float depth  = texture(depth_texture, fragTexCoord).x;
    vec3  normal = texture(normal_texture, fragTexCoord).xyz;

    float depth_last_frame  = texture(depth_history_texture, fragTexCoord_last_frame).x;
    vec3  normal_last_frame = texture(normal_history_texture, fragTexCoord_last_frame).xyz;

    vec4 color_last_frame = texture(color_history, fragTexCoord_last_frame);

    if (abs(depth - depth_last_frame) > allowed_z_dist ||
        distance(normal, normal_last_frame) > allowed_normal_dist)
    {
        color_last_frame = color;
    }

    float alpha = .2;


    // if (distance(normal, normal_last_frame) > 0.001)
    // {
        // imageStore(temp_accum_texture, globalIdx, vec4(0));
    // } else
        imageStore(temp_accum_texture, globalIdx, alpha*color + (1-alpha)*color_last_frame);
}

// --------------------------------------
//           À-Trous-Pass
// --------------------------------------
-- Compute-ATrous

#version 450
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_debug_printf : enable
// #extension GL_NV_compute_shader_derivatives : enable

#include "svgf_common.glsl"

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;


layout(push_constant) uniform PushConstants {
    int iteration;
    float z_multiplier;
    float fwidth_h;
    float nabla_max;
};

// this frame
layout(binding = 3) uniform sampler2D color_texture;
layout(binding = 4) uniform sampler2D normal_texture;
layout(binding = 5) uniform sampler2D depth_texture;
layout(binding = 6) uniform sampler2D motion_texture;

// last frame
layout(binding = 8,  rgba32f) uniform image2D color_history_texture;
layout(binding = 9,  rgba32f) uniform image2D variance_history_texture;
layout(binding = 10, rgba32f) uniform image2D depth_history_texture;
layout(binding = 11, rgba32f) uniform image2D normals_history_texture;

// output
layout(binding = 7, rgba32f) uniform writeonly image2D outputImage;

void main() {
    int stepWidth = 1 << iteration;
    ivec2 size = textureSize(color_texture, 0);
    vec2 pixel_step = vec2(1.0) / vec2(size);
    vec2 step = vec2(float(stepWidth)) / vec2(size);
    ivec2 globalIdx = ivec2(gl_GlobalInvocationID.xy);
    vec2 fragTexCoord = (vec2(gl_GlobalInvocationID.xy) + vec2(0.5, 0.5)) / vec2(size);

    vec4 centerColor = texture(color_texture, fragTexCoord);
    vec3 center_normal = texture(normal_texture, fragTexCoord).rgb;
    float center_z = texture(depth_texture, fragTexCoord).r;


    float kernel_values[3] = {1.0, 2.0 / 3.0, 1.0 / 6.0};

    // memoryBarrierShared();
    // barrier();

    if (globalIdx.x >= size.x || globalIdx.y >= size.y) {
        return;
    }

    float accumW = kernel_values[0] * kernel_values[0];
    vec4 sum = centerColor * accumW;
    // float accumW = 0;
    // vec4 sum = vec4(0);

    for (int i = 0; i < 25; i++) {
        int x = i % 5 - 2;
        int y = i / 5 - 2;

        if (x == 0 && y == 0)
            continue; // center pixel is already accumulated

        float kernelValue = kernel_values[abs(x)] * kernel_values[abs(y)];

        vec2  offset = vec2(x, y);
        vec2  offsetTexCoord = fragTexCoord + offset * step;
        vec4  offset_color  = texture(color_texture,  offsetTexCoord);
        vec3  offset_normal = texture(normal_texture, offsetTexCoord).rgb;
        float offset_z      = texture(depth_texture,  offsetTexCoord).x;

        float w_n_e = weight_n(center_normal, offset_normal);

        float w_z_e = weight_z_exp(center_z, offset_z, x, y, stepWidth,
                                   depth_texture, fragTexCoord,
                                   pixel_step, fwidth_h, z_multiplier, nabla_max);
        float w_l_e = weight_l_exp();


        float weight = (exp(w_n_e + w_z_e) * w_l_e); 

        sum += offset_color * (weight * kernelValue);
        accumW += (weight * kernelValue);

    }

    if (iteration == 0)
        imageStore(color_history_texture, globalIdx, sum / accumW);

    imageStore(outputImage, globalIdx, sum / accumW);
    // imageStore(outputImage, globalIdx, centerColor);
}
