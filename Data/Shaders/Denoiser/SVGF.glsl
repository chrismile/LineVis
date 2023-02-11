// --------------------------------------
//           Reprojection
// --------------------------------------
-- Compute-Reproject
#version 450
#extension GL_EXT_scalar_block_layout : require
layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;

// this frame
layout(binding = 1) uniform sampler2D noisy_texture;
layout(binding = 2) uniform sampler2D color_history;

layout(binding = 3) uniform sampler2D motion_texture;

layout(binding = 4) uniform sampler2D normal_texture;
layout(binding = 5) uniform sampler2D depth_texture;

layout(binding = 6) uniform sampler2D normal_history_texture;
layout(binding = 7) uniform sampler2D depth_history_texture;

// layout(binding = 8) uniform sampler2D depth_fwidth_texture;

layout(binding = 15, rgba32f) uniform writeonly image2D temp_accum_texture;

layout(push_constant) uniform PushConstants {
    float allowed_z_dist;
    float allowed_normal_dist;
};

// source: https://github.com/NVIDIAGameWorks/Falcor/blob/master/Source/RenderPasses/SVGFPass/SVGFReproject.ps.slang#L62
// bool isReprjValid(int2 coord, float Z, float Zprev, float fwidthZ, vec3 normal, vec3 normalPrev, float fwidthNormal)
// {
    // const int2 imageDim = textureSize(gColor, 0);

    // check whether reprojected pixel is inside of the screen
    // if (any(coord < int2(1,1)) || any(coord > imageDim - int2(1,1))) return false;

    // check if deviation of depths is acceptable
    // if (abs(Zprev - Z) / (fwidthZ + 1e-2f) > 10.f) return false;

    // check normals for compatibility
    // if (distance(normal, normalPrev) / (fwidthNormal + 1e-2) > 16.0) return false;

    // return true;
// }

bool is_reprj_valid(ivec2 coord, float z, float z_prev, vec3 normal, vec3 normal_prev) {
    ivec2 im_size = textureSize(noisy_texture, 0);

    // check whether reprojected pixel is inside of the screen
    if (coord.x < 0 || coord.y < 0 ||
        coord.x >= im_size.x || coord.y >= im_size.y )
    {
        return false;
    }

    if (abs(z_prev - z)               > allowed_z_dist)      return false;
    if (distance(normal_prev, normal) > allowed_normal_dist) return false;

    return true;
}

bool try_2x2_tap(vec2 pos_prev_minus_half, ivec2 i_pos_prev_minus_half, float depth, vec3 normal, out vec4 color_last_frame) {
    const ivec2 _2x2_offsets[4] = {ivec2(0,0), ivec2(0,1), ivec2(1,0), ivec2(1,1) };
    bool valid_found = false;
    bool valids[4];

    for (int i = 0; i < 4; ++i) {
        ivec2 i_offset_pos = i_pos_prev_minus_half + _2x2_offsets[i];

        vec3  offset_normal = texelFetch(normal_history_texture, i_offset_pos, 0).xyz;
        float offset_depth  = texelFetch(depth_history_texture, i_offset_pos, 0).x;

        valids[i] = is_reprj_valid(i_offset_pos, depth, offset_depth, normal, offset_normal);
        valid_found = valid_found || valids[i];
    }

    // 2x2
    if (valid_found) {
        float x = fract(pos_prev_minus_half.x);
        float y = fract(pos_prev_minus_half.y);
        const float w[4] =
            { (1 - x) * (1 - y), x  * (1 - y),
              (1 - x) *      y,  x  *      y };

        vec3 color_bilinear = vec3(0);
        float sum_w = 0;

        for (int i = 0; i < 4; ++i) {
            ivec2 i_offset_pos = i_pos_prev_minus_half + _2x2_offsets[i];
            vec3 offset_color = texelFetch(color_history, i_offset_pos, 0).xyz;

            if (valids[i]) {
                color_bilinear += w[i] * offset_color;
                sum_w          += w[i];
            }
        }

        // NOTE(Felix): some bilinear weights can be 0, some pixels are valid
        //   but have 0 weight. only pixels with weight 0 are valid, we have a
        //   problem.
        valid_found = (sum_w >= 0.001);

        if (valid_found)
            color_last_frame = vec4(color_bilinear / sum_w, 0);
    }

    return valid_found;
}

bool try_3x3_bilat(ivec2 i_pos_prev_minus_half, float depth, vec3 normal, out vec4 color_last_frame) {
    // 3x3
    // source: https://github.com/NVIDIAGameWorks/Falcor/blob/master/Source/RenderPasses/SVGFPass/SVGFReproject.ps.slang#L144
    float nValid = 0.0;
    vec4 filtered_color = vec4(0);
    // this code performs a binary descision for each tap of the cross-bilateral filter
    int radius = 1;
    for (int yy = -radius; yy <= radius; yy++) {
        for (int xx = -radius; xx <= radius; xx++) {
            ivec2 i_offset_pos = i_pos_prev_minus_half + ivec2(xx, yy);
            vec3  offset_normal = texelFetch(normal_history_texture, i_offset_pos, 0).xyz;
            float offset_depth  = texelFetch(depth_history_texture, i_offset_pos, 0).x;

            if (is_reprj_valid(i_offset_pos, depth, offset_depth, normal, offset_normal)) {
                filtered_color += texelFetch(color_history, i_offset_pos, 0).rgba;
                // prevMoments += gPrevMoments[p].xy;
                nValid += 1.0;
            }
        }
    }
    if (nValid > 0) {
        color_last_frame = filtered_color / nValid;
        // prevMoments /= nValid;
        return true;
    }
    return false;
}

void main() {
    ivec2 i_pos = ivec2(gl_GlobalInvocationID.xy);

    // vec2  pos_prev   = vec2(0.5,0.5) + i_pos - imageLoad(motion_texture, i_pos).xy;
    // see paper why -0.5
    vec2  pos_prev_minus_half   = vec2(0.001) + i_pos - texelFetch(motion_texture, i_pos, 0).xy;
    ivec2 i_pos_prev_minus_half = ivec2(pos_prev_minus_half);

    vec4  color  = texelFetch(noisy_texture,  i_pos, 0).rgba;
    float depth  = texelFetch(depth_texture,  i_pos, 0).x;
    vec3  normal = texelFetch(normal_texture, i_pos, 0).xyz;

    vec4 color_last_frame;
    bool success = try_2x2_tap(pos_prev_minus_half, i_pos_prev_minus_half, depth, normal, color_last_frame);
    if (!success) success = try_3x3_bilat(i_pos_prev_minus_half, depth, normal, color_last_frame);
    if (!success) color_last_frame = texelFetch(noisy_texture, i_pos, 0);

    float alpha = .2;

    imageStore(temp_accum_texture, i_pos, alpha*color + (1-alpha)*color_last_frame);
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
layout(binding = 3)  uniform sampler2D color_texture;
layout(binding = 4)  uniform sampler2D normal_texture;
layout(binding = 5)  uniform sampler2D depth_texture;
layout(binding = 12) uniform sampler2D depth_fwidth_texture;
layout(binding = 13) uniform sampler2D depth_nabla_texture;


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

    if (globalIdx.x >= size.x || globalIdx.y >= size.y) {
        return;
    }

    float accumW = kernel_values[0] * kernel_values[0];
    vec4 sum = centerColor * accumW;

    for (int y = -2; y <= 2; ++y) {
        for (int x = -2; x <= 2; ++x) {
            if (x == 0 && y == 0)
                continue; // center pixel is already accumulated

            float kernelValue = kernel_values[abs(x)] * kernel_values[abs(y)];

            vec2  offset = vec2(x, y);
            vec2  offsetTexCoord = fragTexCoord + offset * step;
            vec4  offset_color  = texture(color_texture,  offsetTexCoord);
            vec3  offset_normal = texture(normal_texture, offsetTexCoord).rgb;
            float offset_z      = texture(depth_texture,  offsetTexCoord).x;

            float w_n = weight_n(center_normal, offset_normal);

            // float w_z_e = weight_z_exp(center_z, offset_z, x, y, stepWidth,
            // depth_texture, fragTexCoord,
            // pixel_step, fwidth_h, z_multiplier, nabla_max);

            // float w_z_e =
                // -abs(center_z - offset_z) * z_multiplier
                // /
                // ((abs(dot(texture(depth_nabla_texture, fragTexCoord).xy,  normalize(vec2(x, y))))) + 0.001);

            // float w_z_e =
            // -abs(center_z - offset_z) * z_multiplier
            // /
            // ((abs(dot(texture(depth_nabla_texture, fragTexCoord).xy,  vec2(x, y) * stepWidth))) + 0.0001);

            float w_z_e = -abs(center_z - offset_z) * z_multiplier
                /
                ((abs(dot(texture(depth_fwidth_texture, fragTexCoord).xy,  vec2(x, y)))) + 0.0001);

            // eaw
            // vec3 diffNormal = center_normal - offset_normal;
            // float distNormal = dot(diffNormal, diffNormal);
            // float w_n = min(exp(-distNormal / 1), 1.0);
            // float w_z_e= -abs(center_z - offset_z) * z_multiplier;

            float w_l_e = weight_l_exp();


            float weight = (exp(w_l_e + w_z_e) * w_n);


            sum += offset_color * (weight * kernelValue);
            accumW += (weight * kernelValue);

        }
    }

    if (iteration == 0)
        imageStore(color_history_texture, globalIdx, sum / accumW);

    imageStore(outputImage, globalIdx, sum / accumW);
    // imageStore(outputImage, globalIdx, centerColor);
}
