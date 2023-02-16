// --------------------------------------
//           Reprojection
// --------------------------------------
-- Compute-Reproject
#version 450
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_debug_printf : enable

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;

// this frame
layout(binding = 1) uniform sampler2D noisy_texture;
layout(binding = 2) uniform sampler2D color_history;

layout(binding = 3) uniform sampler2D motion_texture;

layout(binding = 4) uniform sampler2D normal_texture;
layout(binding = 5) uniform sampler2D depth_texture;

layout(binding = 6) uniform sampler2D normal_history_texture;
layout(binding = 7) uniform sampler2D depth_history_texture;

layout(binding = 8) uniform sampler2D moments_history_texture;

// layout(binding = 8) uniform sampler2D depth_fwidth_texture;

layout(binding = 14, rgba32f) uniform writeonly image2D accum_moments_texture;
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
    if (coord.x < 1           || coord.y < 1 ||
        coord.x > im_size.x-1 || coord.y > im_size.y-1)
    {
        return false;
    }

    if (abs(z_prev - z)               > allowed_z_dist)      return false;
    if (distance(normal_prev, normal) > allowed_normal_dist) return false;

    return true;
}

bool try_2x2_tap(vec2 pos_prev_minus_half, ivec2 i_pos_prev_minus_half, float depth, vec3 normal,
                 out vec3 color_last_frame, out vec2 prev_moments)
{
    const ivec2 _2x2_offsets[4] = {ivec2(0,0), ivec2(0,1), ivec2(1,0), ivec2(1,1) };
    bool valid_found = false;
    bool valids[4];

    for (int i = 0; i < 4; ++i) {
        ivec2 i_offset_pos = i_pos_prev_minus_half + _2x2_offsets[i];

        vec3  offset_normal = texelFetch(normal_history_texture, i_offset_pos, 0).xyz;
        float offset_depth  = texelFetch(depth_history_texture,  i_offset_pos, 0).x;

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
        vec2 moments_bilinear = vec2(0);
        float sum_w = 0;

        for (int i = 0; i < 4; ++i) {
            if (valids[i]) {
                ivec2 i_offset_pos = i_pos_prev_minus_half + _2x2_offsets[i];
                vec3 offset_color   = texelFetch(color_history, i_offset_pos, 0).xyz;
                vec2 offset_moments = texelFetch(moments_history_texture, i_offset_pos, 0).xy;

                moments_bilinear += w[i] * offset_moments;
                color_bilinear   += w[i] * offset_color;
                sum_w            += w[i];
            }
        }

        // NOTE(Felix): some bilinear weights can be 0, some pixels are valid
        //   but have 0 weight. only pixels with weight 0 are valid, we have a
        //   problem.
        valid_found = (sum_w >= 0.001);

        if (valid_found) {
            color_last_frame = color_bilinear   / sum_w;
            prev_moments     = moments_bilinear / sum_w;
        }
    }

    return valid_found;
}

bool try_3x3_bilat(ivec2 i_pos_prev_minus_half, float depth, vec3 normal,
                   out vec3 color_last_frame, out vec2 prev_moments)
{
    // 3x3
    // source: https://github.com/NVIDIAGameWorks/Falcor/blob/master/Source/RenderPasses/SVGFPass/SVGFReproject.ps.slang#L144
    float nValid = 0.0;
    vec3 filtered_color   = vec3(0);
    vec2 filtered_moments = vec2(0);
    // this code performs a binary descision for each tap of the cross-bilateral filter
    int radius = 1;
    for (int yy = -radius; yy <= radius; yy++) {
        for (int xx = -radius; xx <= radius; xx++) {

            ivec2 i_offset_pos = i_pos_prev_minus_half + ivec2(xx, yy);

            {
                // HACK(Felix): this check should not be necessary since
                //   is_reprj_valid should reject the i_offset_pos, however it
                //   does not always do that?? and we end up with invalid
                //   filtered moments
                ivec2 im_size = textureSize(noisy_texture, 0);
                if (i_offset_pos.x < 1 || i_offset_pos.y < 1 ||
                    i_offset_pos.x >= im_size.x || i_offset_pos.y >=im_size.y)
                {
                    continue;
                }
            }

            vec3  offset_normal = texelFetch(normal_history_texture, i_offset_pos, 0).xyz;
            float offset_depth  = texelFetch(depth_history_texture,  i_offset_pos, 0).x;

            if (is_reprj_valid(i_offset_pos, depth, offset_depth, normal, offset_normal)) {
                filtered_color   += texelFetch(color_history,           i_offset_pos, 0).rgb;
                filtered_moments += texelFetch(moments_history_texture, i_offset_pos, 0).rg;
                nValid           += 1.0;
            }
        }
    }
    if (nValid > 0) {
        color_last_frame = filtered_color / nValid;
        prev_moments     = filtered_moments / nValid;
        return true;
    }
    return false;
}

bool load_moments_and_history_length(const ivec2 i_pos_prev, out vec2 prev_moments, out float history_length) {
    ivec2 tex_size = textureSize(noisy_texture, 0);
    if (i_pos_prev.x < 0 || i_pos_prev.y < 0 ||
        i_pos_prev.x >= tex_size.x || i_pos_prev.y >= tex_size.y)
        return false;

    vec4 read = texelFetch(moments_history_texture, i_pos_prev, 0);
    prev_moments   = read.xy;
    history_length = read.z;

    return true;
}

void main() {
    ivec2 i_pos = ivec2(gl_GlobalInvocationID.xy);
    ivec2 size = textureSize(noisy_texture, 0);
    if (i_pos.x >= size.x || i_pos.y >= size.y) {
        return;
    }


    vec2  prev_moments;
    float history_length;
    ivec2 i_pos_prev  = ivec2(vec2(0.5,0.5) + i_pos - texelFetch(motion_texture, i_pos, 0).xy);
    bool  success = load_moments_and_history_length(i_pos_prev, prev_moments, history_length);

    vec3 color  = texelFetch(noisy_texture,  i_pos, 0).rgb;

    vec3 color_last_frame = texelFetch(color_history, i_pos, 0).rgb;
    if (success) {
        // see paper why -0.5
        vec2  pos_prev_minus_half   = vec2(0.001) + i_pos - texelFetch(motion_texture, i_pos, 0).xy;
        ivec2 i_pos_prev_minus_half = ivec2(pos_prev_minus_half);

        float depth  = texelFetch(depth_texture,  i_pos, 0).x;
        vec3  normal = texelFetch(normal_texture, i_pos, 0).xyz;

        success = try_2x2_tap(pos_prev_minus_half, i_pos_prev_minus_half, depth, normal,
                              color_last_frame, prev_moments);
        if (!success)
            success = try_3x3_bilat(i_pos_prev_minus_half, depth, normal,
                                    color_last_frame, prev_moments);
    }


    history_length = min(success ? history_length+1 : 1, 32);

    float alpha_color   = success ? max(0.05, 1.0f/history_length) : 1.0f;
    float alpha_moments = success ? max(0.2 , 1.0f/history_length) : 1.0f;

    vec4 moments;
    moments.r = color.r;
    moments.g = moments.r * moments.r;
    moments.b = history_length;

    moments.rg = mix(prev_moments.rg, moments.rg, alpha_moments);

    // if (isnan(moments.r))
        // debugPrintfEXT("oh no!");

    float variance = max(0.f, moments.g - moments.r * moments.r);

    imageStore(accum_moments_texture, i_pos, moments);
    imageStore(temp_accum_texture,    i_pos, vec4(mix(color_last_frame, color, alpha_color), variance));
}

// --------------------------------------
//        Filter-Moments-Pass
// --------------------------------------
-- Compute-Filter-Moments
#version 450
layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;

layout(binding = 2)  uniform sampler2D accum_moments_texture;
layout(binding = 4)  uniform sampler2D normal_texture;
layout(binding = 5)  uniform sampler2D depth_texture;
layout(binding = 12) uniform sampler2D depth_fwidth_texture;
layout(binding = 13) uniform sampler2D depth_nabla_texture;

layout(binding = 3, rgba32f) uniform image2D temp_accum_texture;

#include "svgf_common.glsl"

void main() {
    ivec2 i_pos = ivec2(gl_GlobalInvocationID.xy);

    ivec2 size = textureSize(accum_moments_texture, 0);
    if (i_pos.x >= size.x || i_pos.y >= size.y) {
        return;
    }
    float history_length = texelFetch(accum_moments_texture, i_pos, 0).z;

    if (history_length >= 4)
        return;


    float sum_weight     = 0;
    vec3  sum_color   = vec3(0);
    vec2  sum_moments = vec2(0);

    vec4  center_color   = imageLoad(temp_accum_texture, i_pos).rgba;
    float center_depth   = texelFetch(depth_texture, i_pos, 0).r;
    vec2  center_z_nabla = texelFetch(depth_nabla_texture, i_pos, 0).xy;
    vec3  center_normal  = texelFetch(normal_texture, i_pos, 0).rgb;

    // NOTE(Felix): source: https://github.com/NVIDIAGameWorks/Falcor/blob/master/Source/RenderPasses/SVGFPass/SVGFFilterMoments.ps.slang#L75
    const int radius = 3;
    for (int yy = -radius; yy <= radius; yy++) {
        for (int xx = -radius; xx <= radius; xx++) {
            ivec2 offset_pos = i_pos + ivec2(xx, yy);
            bool inside =
                offset_pos.x  >= 0     && offset_pos.y >= 0 &&
                offset_pos.x  < size.x && offset_pos.y  < size.y;

            if (inside) {
                vec3  offset_color   = imageLoad(temp_accum_texture, offset_pos).rgb;
                vec2  offset_moments = texelFetch(accum_moments_texture, offset_pos, 0).rg;
                float offset_depth   = texelFetch(depth_texture, offset_pos, 0).r;
                vec3  offset_normal  = texelFetch(normal_texture, offset_pos, 0).xyz;

                float weight = compute_weight(center_depth, offset_depth, ((abs(dot(center_z_nabla,  vec2(xx, yy)))) + 0.0001),
                                              center_normal, offset_normal,
                                              center_color.r, offset_color.r, 10);

                sum_weight  += weight;
                sum_color   += weight * offset_color;
                sum_moments += weight * offset_moments;
            }
        }
    }

    sum_weight = max(sum_weight, 1e-6);

    sum_color   /= sum_weight;
    sum_moments /= sum_weight;

    float variance = sum_moments.g - sum_moments.r * sum_moments.r;

    // NOTE(Felix): falcor also does this to "givee variance a boost for the first frames"
    variance *= 4.0 / history_length;

    imageStore(temp_accum_texture, i_pos, vec4(sum_color, variance));
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

// output
layout(binding = 7, rgba32f) uniform writeonly image2D outputImage;
layout(binding = 8, rgba32f) uniform writeonly image2D color_history_texture;

float filter_variance(ivec2 i_pos) {
    // 3x3 gaussian

    float sum = 0.f;

    float kernel[2][2] = {
        { 1.0 / 4.0, 1.0 / 8.0  },
        { 1.0 / 8.0, 1.0 / 16.0 }
    };

    int radius = 1;
    for (int yy = -radius; yy <= radius; yy++) {
        for (int xx = -radius; xx <= radius; xx++) {
            ivec2 p = i_pos + ivec2(xx, yy);
            float k = kernel[abs(xx)][abs(yy)];
            sum += texelFetch(color_texture, p, 0).a * k;
        }
    }

    return sum;
}

void main() {
    int   step_width = 1 << iteration;
    ivec2 i_pos      = ivec2(gl_GlobalInvocationID.xy);

    vec4  center_color      = texelFetch(color_texture, i_pos, 0);
    float filtered_variance = filter_variance(i_pos);
    vec3  center_normal     = texelFetch(normal_texture, i_pos, 0).rgb;
    float center_z          = texelFetch(depth_texture, i_pos, 0).r;
    vec2  center_z_nabla    = texelFetch(depth_nabla_texture, i_pos, 0).xy;

    float phi_color = sqrt(max(0.0, 1e-10 + filtered_variance));

    float kernel_values[3] = {1.0, 2.0 / 3.0, 1.0 / 6.0};

    ivec2 size = textureSize(color_texture, 0);
    if (i_pos.x >= size.x || i_pos.y >= size.y) {
        return;
    }

    float accumW = kernel_values[0] * kernel_values[0];
    vec4  sum    = center_color * accumW;

    for (int y = -2; y <= 2; ++y) {
        for (int x = -2; x <= 2; ++x) {
            vec2  offset = vec2(x, y);
            ivec2 i_pos_offset = i_pos + ivec2(x, y) * step_width;
            const bool inside =
                i_pos_offset.x >= 0     && i_pos_offset.y >= 0 &&
                i_pos_offset.x < size.x && i_pos_offset.y < size.y;

            if (!inside || x == 0 && y == 0)
                continue; // center pixel is already accumulated

            float kernelValue = kernel_values[abs(x)] * kernel_values[abs(y)];

            vec4  offset_color  = texelFetch(color_texture,  i_pos_offset, 0);
            vec3  offset_normal = texelFetch(normal_texture, i_pos_offset, 0).rgb;
            float offset_z      = texelFetch(depth_texture,  i_pos_offset, 0).x;

            // float w_n = weight_n(center_normal, offset_normal);

            // float w_z_e = weight_z_exp(center_z, offset_z, x, y, step_width,
            // depth_texture, fragTexCoord,
            // pixel_step, fwidth_h, z_multiplier, nabla_max);

            // float w_z_e =
                // -abs(center_z - offset_z) * z_multiplier
                // /
                // ((abs(dot(texture(depth_nabla_texture, fragTexCoord).xy,  normalize(vec2(x, y))))) + 0.001);

            // float w_z_e =
            // -abs(center_z - offset_z) * z_multiplier
            // /
            // ((abs(dot(center_z_nabla,  vec2(x, y) * step_width))) + 0.0001);

            // float w_z_e = -abs(center_z - offset_z) * z_multiplier
            //     /
            //     ((abs(dot(texture(depth_fwidth_texture, fragTexCoord).xy,  vec2(x, y)))) + 0.0001);

            // eaw
            // vec3 diffNormal = center_normal - offset_normal;
            // float distNormal = dot(diffNormal, diffNormal);
            // float w_n = min(exp(-distNormal / 1), 1.0);
            // float w_z_e= -abs(center_z - offset_z) * z_multiplier;

            // float w_l_e = weight_l_exp(center_color, offset_color, filtered_variance);

            // float weight = exp(w_l_e + w_z_e) * w_n * kernelValue;
            float weight = compute_weight(center_z, offset_z, ((abs(dot(center_z_nabla,  vec2(x, y) * step_width))) + 0.0001),
                                          center_normal, offset_normal,
                                          center_color.x, offset_color.x, phi_color) * kernelValue;

            // variance gets squared weight
            sum += vec4(weight,weight,weight, weight*weight) * offset_color;
            accumW += weight;

        }
    }

    // variance gets squared weight
    vec4 result = sum / vec4(accumW,accumW,accumW, accumW*accumW);
    if (iteration == 0)
        imageStore(color_history_texture, i_pos, result);

    imageStore(outputImage, i_pos, result);
}
