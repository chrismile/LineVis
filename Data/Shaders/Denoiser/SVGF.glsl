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
    vec2 fragTexCoord_last_frame = fragTexCoord - (texture(motion_texture, fragTexCoord).xy / vec2(size));

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

    float alpha = .05;

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

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;


layout(push_constant) uniform PushConstants {
    int iteration;
    float z_multiplier;
    float fwidth_h;

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


    float kernel_values[3] = {6.0/16.0, 4.0/16.0, 1.0/16.0};

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
        vec4  offset_color = texture(color_texture, offsetTexCoord);
        vec3  offset_normal = texture(normal_texture, offsetTexCoord).rgb;
        float offset_z   = texture(depth_texture, offsetTexCoord).x;

        // sigmas
        float sigma_n = 128;
        float sigma_z = 1;
        float sigma_l = 4;

        float weight_n = 1;
        {
            // NOTE(Felix): paper
            weight_n = pow(max(0, dot(center_normal, offset_normal)), sigma_n);

            // NOTE(Felix): empirical
            // vec3 diff_normal = center_normal - offset_normal;
            // float distNormal = dot(diff_normal, diff_normal);
            // float weight_n   = min(exp(-distNormal / 0.1), 1.0);


            // if (iteration == 4 && globalIdx == ivec2(1167, 243)) {
            //     // debugPrintfEXT("weight_n: %f", weight_n);
            // }

        }

        float weight_z = 1;
        {

            // // NOTE(Felix): paper
            float h = fwidth_h;
            float offset_z_x =
                (texture(depth_texture,   fragTexCoord+vec2(pixel_step.x*h, 0)).x
                 - texture(depth_texture, fragTexCoord+vec2(0,              0)).x)

                / (h*pixel_step.x);

            float offset_z_y =
                (texture(depth_texture,   fragTexCoord+vec2(0, pixel_step.y*h)).x
                 - texture(depth_texture, fragTexCoord+vec2(0,              0)).x)

                / (h*pixel_step.y);

            vec2 nabla = vec2(offset_z_x, offset_z_y);
            float f_width = abs(offset_z_x) + abs(offset_z_y);
            // NOTE(Felix): paper
            weight_z = exp(-abs(offset_z - center_z) * z_multiplier /
                           max(1e-8, abs(dot(nabla, vec2(x, y)))));

            // NOTE(Felix): falcor
            // weight_z = exp(-abs(offset_z - center_z) * z_multiplier /
            //                (max(f_width, 1e-8) * stepWidth * length(vec2(x,y))));

            // // NOTE(Felix): empirical
            // weight_z = exp(-abs(offset_z - center_z) * z_multiplier /
                           // (5e-3 * stepWidth * length(vec2(x, y))));
        }

        float weight_l = 1;
        {

        }


        float weight = max((weight_n * weight_z * weight_l), 0.00001); // NOTE(Felix): so we cant devide by 0

        sum += offset_color * weight * kernelValue;
        accumW += weight * kernelValue;

    }

    if (iteration == 0)
        imageStore(color_history_texture, globalIdx, sum / accumW);

    imageStore(outputImage, globalIdx, sum / accumW);
    // imageStore(outputImage, globalIdx, centerColor);
}
