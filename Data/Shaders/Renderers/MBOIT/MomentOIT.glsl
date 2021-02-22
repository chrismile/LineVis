/**
 * This file is part of an OpenGL GLSL port of the HLSL code accompanying the paper "Moment-Based Order-Independent
 * Transparency" by Münstermann, Krumpen, Klein, and Peters (http://momentsingraphics.de/?page_id=210).
 * The original code was released in accordance to CC0 (https://creativecommons.org/publicdomain/zero/1.0/).
 *
 * This port is released under the terms of BSD 2-Clause License. For more details please see the LICENSE file in the
 * root directory of this project.
 *
 * Changes for the OpenGL port: Copyright 2018-2019 Christoph Neuhauser
 */

/*! \file
    This header provides the functionality to create the vectors of moments and 
    to blend surfaces together with an appropriately reconstructed 
    transmittance. It is needed for both additive passes of moment-based OIT.
*/
#ifndef MOMENT_OIT_GLSL
#define MOMENT_OIT_GLSL

// https://www.khronos.org/opengl/wiki/Interface_Block_(GLSL)#Block_buffer_binding
#ifndef SHADOW_MAPPING_MOMENTS_GENERATION
layout(std140, binding = 1) uniform MomentOITUniformData
{
    vec4 wrapping_zone_parameters;
    float overestimation;
    float moment_bias;
} MomentOIT;
#else
layout(std140, binding = 2) uniform MomentOITUniformData
{
    vec4 wrapping_zone_parameters;
    float overestimation;
    float moment_bias;
} MomentOIT;
#endif

#include "MomentMath.glsl"

#if MOMENT_GENERATION

const float ABSORBANCE_MAX_VALUE = 10.0;

/*! Generation of moments in case that rasterizer ordered views are used. 
    This includes the case if moments are stored in 16 bits. */
#if ROV

#ifndef SHADOW_MAPPING_MOMENTS_GENERATION

//RasterizerOrderedTexture2DArray<float> b0 : register(u0);
layout (binding = 0, r32f) coherent uniform image2DArray b0; // float
#if SINGLE_PRECISION
#if NUM_MOMENTS == 6
//RasterizerOrderedTexture2DArray<vec2> b : register(u1);
layout (binding = 1, rg32f) coherent uniform image2DArray b; // vec2
#if USE_R_RG_RGBA_FOR_MBOIT6
//RasterizerOrderedTexture2DArray<vec4> b_extra : register(u2);
layout (binding = 2, rgba32f) coherent uniform image2DArray b_extra; // vec4
#endif
#else
//RasterizerOrderedTexture2DArray<vec4> b : register(u1);
layout (binding = 1, rgba32f) coherent uniform image2DArray b; // vec4
#endif
#else
#if NUM_MOMENTS == 6
//RasterizerOrderedTexture2DArray<unorm vec2> b : register(u1);
layout (binding = 1, rg16) coherent uniform image2DArray b;
#if USE_R_RG_RGBA_FOR_MBOIT6
//RasterizerOrderedTexture2DArray<unorm vec4> b_extra : register(u2);
layout (binding = 2, rgba16) coherent uniform image2DArray b_extra;
#endif
#else
//RasterizerOrderedTexture2DArray<unorm vec4> b : register(u1);
layout (binding = 1, rgba16) coherent uniform image2DArray b;
#endif
#endif

#else

layout (binding = 3, r32f) coherent uniform image2DArray b0; // float
#if SINGLE_PRECISION
#if NUM_MOMENTS == 6
layout (binding = 4, rg32f) coherent uniform image2DArray b; // vec2
#if USE_R_RG_RGBA_FOR_MBOIT6
layout (binding = 5, rgba32f) coherent uniform image2DArray b_extra; // vec4
#endif
#else
layout (binding = 4, rgba32f) coherent uniform image2DArray b; // vec4
#endif
#else
#if NUM_MOMENTS == 6
layout (binding = 4, rg16) coherent uniform image2DArray b;
#if USE_R_RG_RGBA_FOR_MBOIT6
layout (binding = 5, rgba16) coherent uniform image2DArray b_extra;
#endif
#else
layout (binding = 4, rgba16) coherent uniform image2DArray b;
#endif
#endif

#endif



#if !TRIGONOMETRIC

/*! This function handles the actual computation of the new vector of power 
    moments.*/
void generatePowerMoments(inout float b_0,
#if NUM_MOMENTS == 4
    inout vec2 b_even, inout vec2 b_odd,
#elif NUM_MOMENTS == 6
    inout vec3 b_even, inout vec3 b_odd,
#elif NUM_MOMENTS == 8
    inout vec4 b_even, inout vec4 b_odd,
#endif
    float depth, float transmittance)
{
    // Absorbance would be infinite for zero transmittance. Thus, make sure transittance is never close to zero.
    float absorbance = -log(transmittance);
    if (absorbance > ABSORBANCE_MAX_VALUE) {
        absorbance = ABSORBANCE_MAX_VALUE;
    }

    float depth_pow2 = depth * depth;
    float depth_pow4 = depth_pow2 * depth_pow2;

#if SINGLE_PRECISION
    b_0 += absorbance;
#if NUM_MOMENTS == 4
    b_even += vec2(depth_pow2, depth_pow4) * absorbance;
    b_odd += vec2(depth, depth_pow2 * depth) * absorbance;
#elif NUM_MOMENTS == 6
    b_even += vec3(depth_pow2, depth_pow4, depth_pow4 * depth_pow2) * absorbance;
    b_odd += vec3(depth, depth_pow2 * depth, depth_pow4 * depth) * absorbance;
#elif NUM_MOMENTS == 8
    float depth_pow6 = depth_pow4 * depth_pow2;
    b_even += vec4(depth_pow2, depth_pow4, depth_pow6, depth_pow6 * depth_pow2) * absorbance;
    b_odd += vec4(depth, depth_pow2 * depth, depth_pow4 * depth, depth_pow6 * depth) * absorbance;
#endif
#else // Quantized
    offsetMoments(b_even, b_odd, -1.0);
    b_even *= b_0;
    b_odd *= b_0;

    //  New Moments
#if NUM_MOMENTS == 4
    vec2 b_even_new = vec2(depth_pow2, depth_pow4);
    vec2 b_odd_new = vec2(depth, depth_pow2 * depth);
    vec2 b_even_new_q, b_odd_new_q;
#elif NUM_MOMENTS == 6
    vec3 b_even_new = vec3(depth_pow2, depth_pow4, depth_pow4 * depth_pow2);
    vec3 b_odd_new = vec3(depth, depth_pow2 * depth, depth_pow4 * depth);
    vec3 b_even_new_q, b_odd_new_q;
#elif NUM_MOMENTS == 8
    float depth_pow6 = depth_pow4 * depth_pow2;
    vec4 b_even_new = vec4(depth_pow2, depth_pow4, depth_pow6, depth_pow6 * depth_pow2);
    vec4 b_odd_new = vec4(depth, depth_pow2 * depth, depth_pow4 * depth, depth_pow6 * depth);
    vec4 b_even_new_q, b_odd_new_q;
#endif
    quantizeMoments(b_even_new_q, b_odd_new_q, b_even_new, b_odd_new);

    // Combine Moments
    b_0 += absorbance;
    b_even += b_even_new_q * absorbance;
    b_odd += b_odd_new_q * absorbance;

    // Go back to interval [0, 1]
    b_even /= b_0;
    b_odd /= b_0;
    offsetMoments(b_even, b_odd, 1.0);
#endif
}

#else

/*! This function handles the actual computation of the new vector of 
    trigonometric moments.*/
void generateTrigonometricMoments(inout float b_0,
#if NUM_MOMENTS == 4
    inout vec4 b_12,
#elif NUM_MOMENTS == 6
    inout vec2 b_1, inout vec2 b_2, inout vec2 b_3,
#elif NUM_MOMENTS == 8
    inout vec4 b_even, inout vec4 b_odd,
#endif
    float depth, float transmittance, vec4 wrapping_zone_parameters)
{
    // Absorbance would be infinite for zero transmittance. Thus, make sure transittance is never close to zero.
    float absorbance = -log(transmittance);
    if (absorbance > ABSORBANCE_MAX_VALUE) {
        absorbance = ABSORBANCE_MAX_VALUE;
    }

    float phase = fma(depth, wrapping_zone_parameters.y, wrapping_zone_parameters.y);
    vec2 circle_point = vec2(cos(phase), sin(phase));
    //sincos(phase, circle_point.y, circle_point.x);
    vec2 circle_point_pow2 = Multiply(circle_point, circle_point);

#if NUM_MOMENTS == 4
    vec4 b_12_new = vec4(circle_point, circle_point_pow2) * absorbance;
#if SINGLE_PRECISION
    b_0 += absorbance;
    b_12 += b_12_new;
#else
    b_12 = fma(b_12, vec4(2.0), vec4(-1.0)) * b_0;

    b_0 += absorbance;
    b_12 += b_12_new;

    b_12 /= b_0;
    b_12 = fma(b_12, vec4(0.5), vec4(0.5));
#endif
#elif NUM_MOMENTS == 6
    vec2 b_1_new = circle_point * absorbance;
    vec2 b_2_new = circle_point_pow2 * absorbance;
    vec2 b_3_new = Multiply(circle_point, circle_point_pow2) * absorbance;
#if SINGLE_PRECISION
    b_0 += absorbance;
    b_1 += b_1_new;
    b_2 += b_2_new;
    b_3 += b_3_new;
#else
    b_1 = fma(b_1, vec2(2.0), vec2(-1.0)) * b_0;
    b_2 = fma(b_2, vec2(2.0), vec2(-1.0)) * b_0;
    b_3 = fma(b_3, vec2(2.0), vec2(-1.0)) * b_0;

    b_0 += absorbance;
    b_1 += b_1_new;
    b_2 += b_2_new;
    b_3 += b_3_new;

    b_1 /= b_0;
    b_2 /= b_0;
    b_3 /= b_0;
    b_1 = fma(b_1, vec2(0.5), vec2(0.5));
    b_2 = fma(b_2, vec2(0.5), vec2(0.5));
    b_3 = fma(b_3, vec2(0.5), vec2(0.5));
#endif
#elif NUM_MOMENTS == 8
    vec4 b_even_new = vec4(circle_point_pow2, Multiply(circle_point_pow2, circle_point_pow2)) * absorbance;
    vec4 b_odd_new = vec4(circle_point, Multiply(circle_point, circle_point_pow2)) * absorbance;
#if SINGLE_PRECISION
    b_0 += absorbance;
    b_even += b_even_new;
    b_odd += b_odd_new;
#else
    b_even = fma(b_even, vec4(2.0), vec4(-1.0)) * b_0;
    b_odd = fma(b_odd, vec4(2.0), vec4(-1.0)) * b_0;

    b_0 += absorbance;
    b_even += b_even_new;
    b_odd += b_odd_new;

    b_even /= b_0;
    b_odd /= b_0;
    b_even = fma(b_even, vec4(0.5), vec4(0.5));
    b_odd = fma(b_odd, vec4(0.5), vec4(0.5));
#endif
#endif
}
#endif

/*! This function reads the stored moments from the rasterizer ordered view, 
    calls the appropriate moment-generating function and writes the new moments 
    back to the rasterizer ordered view.*/
void generateMoments(float depth, float transmittance, ivec2 sv_pos, vec4 wrapping_zone_parameters)
{
    ivec3 idx0 = ivec3(sv_pos, 0);
    ivec3 idx1 = ivec3(idx0.xy, 1);

    // Return early if the surface is fully transparent
    //clip(0.9999999f - transmittance);
    if (transmittance > 0.9999999f) {
        discard;
    }

#if NUM_MOMENTS == 4
    float b_0 = imageLoad(b0, idx0).x;
    vec4 b_raw = imageLoad(b, idx0);

#if TRIGONOMETRIC
    generateTrigonometricMoments(b_0, b_raw, depth, transmittance, wrapping_zone_parameters);
    imageStore(b, idx0, b_raw);
#else
    vec2 b_even = b_raw.yw;
    vec2 b_odd = b_raw.xz;

    generatePowerMoments(b_0, b_even, b_odd, depth, transmittance);

    imageStore(b, idx0, vec4(b_odd.x, b_even.x, b_odd.y, b_even.y));
#endif
    imageStore(b0, idx0, vec4(b_0, 0.0, 0.0, 1.0));
#elif NUM_MOMENTS == 6
    ivec3 idx2 = ivec3(idx0.xy, 2);

    float b_0 = imageLoad(b0, idx0).x;
    vec2 b_raw[3];
    b_raw[0] = imageLoad(b, idx0).xy;
#if USE_R_RG_RGBA_FOR_MBOIT6
    vec4 tmp = imageLoad(b_extra, idx0);
    b_raw[1] = tmp.xy;
    b_raw[2] = tmp.zw;
#else
    b_raw[1] = imageLoad(b, idx1).xy;
    b_raw[2] = imageLoad(b, idx2).xy;
#endif

#if TRIGONOMETRIC
    generateTrigonometricMoments(b_0, b_raw[0], b_raw[1], b_raw[2], depth, transmittance, wrapping_zone_parameters);
#else
    vec3 b_even = vec3(b_raw[0].y, b_raw[1].y, b_raw[2].y);
    vec3 b_odd = vec3(b_raw[0].x, b_raw[1].x, b_raw[2].x);

    generatePowerMoments(b_0, b_even, b_odd, depth, transmittance);

    b_raw[0] = vec2(b_odd.x, b_even.x);
    b_raw[1] = vec2(b_odd.y, b_even.y);
    b_raw[2] = vec2(b_odd.z, b_even.z);
#endif

    imageStore(b0, idx0, vec4(b_0, 0.0, 0.0, 1.0));
    imageStore(b, idx0, vec4(b_raw[0], 0.0, 1.0));
#if USE_R_RG_RGBA_FOR_MBOIT6
    imageStore(b_extra, idx0, vec4(b_raw[1], b_raw[2]));
#else
    imageStore(b, idx1, vec4(b_raw[1], 0.0, 1.0));
    imageStore(b, idx2, vec4(b_raw[2], 0.0, 1.0));
#endif
#elif NUM_MOMENTS == 8
    float b_0 = imageLoad(b0, idx0).x;
    vec4 b_even = imageLoad(b, idx0);
    vec4 b_odd = imageLoad(b, idx1);

#if TRIGONOMETRIC
    generateTrigonometricMoments(b_0, b_even, b_odd, depth, transmittance, wrapping_zone_parameters);
#else
    generatePowerMoments(b_0, b_even, b_odd, depth, transmittance);
#endif

    imageStore(b0, idx0, vec4(b_0, 0.0, 0.0, 1.0));
    imageStore(b, idx0, b_even);
    imageStore(b, idx1, b_odd);
#endif

}

#else // NO ROVs, therefore only single precision

/*! This functions relies on fixed function additive blending to compute the 
    vector of moments.moment vector. The shader that calls this function must 
    provide the required render targets.*/
#if NUM_MOMENTS == 4
void generateMoments(float depth, float transmittance, vec4 wrapping_zone_parameters, out float b_0, out vec4 b)
#elif NUM_MOMENTS == 6
#if USE_R_RG_RGBA_FOR_MBOIT6
void generateMoments(float depth, float transmittance, vec4 wrapping_zone_parameters, out float b_0, out vec2 b_12, out vec4 b_3456)
#else
void generateMoments(float depth, float transmittance, vec4 wrapping_zone_parameters, out float b_0, out vec2 b_12, out vec2 b_34, out vec2 b_56)
#endif
#elif NUM_MOMENTS == 8
void generateMoments(float depth, float transmittance, vec4 wrapping_zone_parameters, out float b_0, out vec4 b_even, out vec4 b_odd)
#endif
{
    float absorbance = -log(transmittance);

    b_0 = absorbance;
#if TRIGONOMETRIC
    float phase = fma(depth, wrapping_zone_parameters.y, wrapping_zone_parameters.y);
    vec2 circle_point = vec2(cos(phase), sin(phase));
    //sincos(phase, circle_point.y, circle_point.x);
    vec2 circle_point_pow2 = Multiply(circle_point, circle_point);
#if NUM_MOMENTS == 4
    b = vec4(circle_point, circle_point_pow2) * absorbance;
#elif NUM_MOMENTS == 6
    b_12 = circle_point * absorbance;
#if USE_R_RG_RGBA_FOR_MBOIT6
    b_3456 = vec4(circle_point_pow2, Multiply(circle_point, circle_point_pow2)) * absorbance;
#else
    b_34 = circle_point_pow2 * absorbance;
    b_56 = Multiply(circle_point, circle_point_pow2) * absorbance;
#endif
#elif NUM_MOMENTS == 8
    b_even = vec4(circle_point_pow2, Multiply(circle_point_pow2, circle_point_pow2)) * absorbance;
    b_odd = vec4(circle_point, Multiply(circle_point, circle_point_pow2)) * absorbance;
#endif
#else
    float depth_pow2 = depth * depth;
    float depth_pow4 = depth_pow2 * depth_pow2;
#if NUM_MOMENTS == 4
    b = vec4(depth, depth_pow2, depth_pow2 * depth, depth_pow4) * absorbance;
#elif NUM_MOMENTS == 6
    b_12 = vec2(depth, depth_pow2) * absorbance;
#if USE_R_RG_RGBA_FOR_MBOIT6
    b_3456 = vec4(depth_pow2 * depth, depth_pow4, depth_pow4 * depth, depth_pow4 * depth_pow2) * absorbance;
#else
    b_34 = vec2(depth_pow2 * depth, depth_pow4) * absorbance;
    b_56 = vec2(depth_pow4 * depth, depth_pow4 * depth_pow2) * absorbance;
#endif
#elif NUM_MOMENTS == 8
    float depth_pow6 = depth_pow4 * depth_pow2;
    b_even = vec4(depth_pow2, depth_pow4, depth_pow6, depth_pow6 * depth_pow2) * absorbance;
    b_odd = vec4(depth, depth_pow2 * depth, depth_pow4 * depth, depth_pow6 * depth) * absorbance;
#endif
#endif
}
#endif

#else //MOMENT_GENERATION is disabled

/*Texture2DArray moments;
Texture2DArray zeroth_moment;
#if USE_R_RG_RGBA_FOR_MBOIT6
Texture2DArray extra_moments;
#endif*/

layout (binding = 0, r32f) coherent uniform image2DArray zeroth_moment; // float
#if SINGLE_PRECISION
#if NUM_MOMENTS == 6
layout (binding = 1, rg32f) coherent uniform image2DArray moments; // vec2
#if USE_R_RG_RGBA_FOR_MBOIT6
layout (binding = 2, rgba32f) coherent uniform image2DArray extra_moments; // vec4
#endif
#else
layout (binding = 1, rgba32f) coherent uniform image2DArray moments; // vec4
#endif
#else
#if NUM_MOMENTS == 6
layout (binding = 1, rg16) coherent uniform image2DArray moments;
#if USE_R_RG_RGBA_FOR_MBOIT6
layout (binding = 2, rgba16) coherent uniform image2DArray extra_moments;
#endif
#else
layout (binding = 1, rgba16) coherent uniform image2DArray moments;
#endif
#endif


/*! This function is to be called from the shader that composites the 
    transparent fragments. It reads the moments and calls the appropriate 
    function to reconstruct the transmittance at the specified depth.*/
void resolveMoments(out float transmittance_at_depth, out float total_transmittance, float depth, ivec2 sv_pos)
{
    ivec3 idx0 = ivec3(sv_pos, 0);
    ivec3 idx1 = ivec3(idx0.xy, 1);

    transmittance_at_depth = 1;
    total_transmittance = 1;
    
    float b_0 = imageLoad(zeroth_moment, idx0).x;
    if (b_0 < 0.00100050033f) {
        discard;
    }
    total_transmittance = exp(-b_0);

#if NUM_MOMENTS == 4
#if TRIGONOMETRIC
    vec4 b_tmp = imageLoad(moments, idx0);
    vec2 trig_b[2];
    trig_b[0] = b_tmp.xy;
    trig_b[1] = b_tmp.zw;
#if SINGLE_PRECISION
    trig_b[0] /= b_0;
    trig_b[1] /= b_0;
#else
    trig_b[0] = fma(trig_b[0], vec2(2.0), vec2(-1.0));
    trig_b[1] = fma(trig_b[1], vec2(2.0), vec2(-1.0));
#endif
    transmittance_at_depth = computeTransmittanceAtDepthFrom2TrigonometricMoments(b_0, trig_b, depth,
            MomentOIT.moment_bias, MomentOIT.overestimation, MomentOIT.wrapping_zone_parameters);
#else
    vec4 b_1234 = imageLoad(moments, idx0).xyzw;
#if SINGLE_PRECISION
    vec2 b_even = b_1234.yw;
    vec2 b_odd = b_1234.xz;

    b_even /= b_0;
    b_odd /= b_0;

    const vec4 bias_vector = vec4(0, 0.375, 0, 0.375);
#else
    vec2 b_even_q = b_1234.yw;
    vec2 b_odd_q = b_1234.xz;

    // Dequantize the moments
    vec2 b_even;
    vec2 b_odd;
    offsetAndDequantizeMoments(b_even, b_odd, b_even_q, b_odd_q);
    const vec4 bias_vector = vec4(0, 0.628, 0, 0.628);
#endif
    transmittance_at_depth = computeTransmittanceAtDepthFrom4PowerMoments(b_0, b_even, b_odd, depth,
            MomentOIT.moment_bias, MomentOIT.overestimation, bias_vector);
#endif
#elif NUM_MOMENTS == 6
    ivec3 idx2 = ivec3(idx0.xy, 2);
#if TRIGONOMETRIC
    vec2 trig_b[3];
    trig_b[0] = imageLoad(moments, idx0).xy;
#if USE_R_RG_RGBA_FOR_MBOIT6
    vec4 tmp = imageLoad(extra_moments, idx0);
    trig_b[1] = tmp.xy;
    trig_b[2] = tmp.zw;
#else
    trig_b[1] = imageLoad(moments, idx1).xy;
    trig_b[2] = imageLoad(moments, idx2).xy;
#endif
#if SINGLE_PRECISION
    trig_b[0] /= b_0;
    trig_b[1] /= b_0;
    trig_b[2] /= b_0;
#else
    trig_b[0] = fma(trig_b[0], vec2(2.0), vec2(-1.0));
    trig_b[1] = fma(trig_b[1], vec2(2.0), vec2(-1.0));
    trig_b[2] = fma(trig_b[2], vec2(2.0), vec2(-1.0));
#endif
    transmittance_at_depth = computeTransmittanceAtDepthFrom3TrigonometricMoments(b_0, trig_b, depth,
            MomentOIT.moment_bias, MomentOIT.overestimation, MomentOIT.wrapping_zone_parameters);
#else
    vec2 b_12 = imageLoad(moments, idx0).xy;
#if USE_R_RG_RGBA_FOR_MBOIT6
    vec4 tmp = imageLoad(extra_moments, idx0);
    vec2 b_34 = tmp.xy;
    vec2 b_56 = tmp.zw;
#else
    vec2 b_34 = imageLoad(moments, idx1).xy;
    vec2 b_56 = imageLoad(moments, idx2).xy;
#endif
#if SINGLE_PRECISION
    vec3 b_even = vec3(b_12.y, b_34.y, b_56.y);
    vec3 b_odd = vec3(b_12.x, b_34.x, b_56.x);

    b_even /= b_0;
    b_odd /= b_0;

    const float bias_vector[6] = { 0, 0.48, 0, 0.451, 0, 0.45 };
#else
    vec3 b_even_q = vec3(b_12.y, b_34.y, b_56.y);
    vec3 b_odd_q = vec3(b_12.x, b_34.x, b_56.x);
    // Dequantize b_0 and the other moments
    vec3 b_even;
    vec3 b_odd;
    offsetAndDequantizeMoments(b_even, b_odd, b_even_q, b_odd_q);

    const float bias_vector[6] = { 0, 0.5566, 0, 0.489, 0, 0.47869382 };
#endif
    transmittance_at_depth = computeTransmittanceAtDepthFrom6PowerMoments(b_0, b_even, b_odd, depth,
            MomentOIT.moment_bias, MomentOIT.overestimation, bias_vector);
#endif
#elif NUM_MOMENTS == 8
#if TRIGONOMETRIC
    vec4 b_tmp = imageLoad(moments, idx0);
    vec4 b_tmp2 = imageLoad(moments, idx1);
#if SINGLE_PRECISION
    vec2 trig_b[4] = {
        b_tmp2.xy / b_0,
        b_tmp.xy / b_0,
        b_tmp2.zw / b_0,
        b_tmp.zw / b_0
    };
#else
    vec2 trig_b[4] = {
        fma(b_tmp2.xy, vec2(2.0), vec2(-1.0)),
        fma(b_tmp.xy, vec2(2.0), vec2(-1.0)),
        fma(b_tmp2.zw, vec2(2.0), vec2(-1.0)),
        fma(b_tmp.zw, vec2(2.0), vec2(-1.0))
    };
#endif
    transmittance_at_depth = computeTransmittanceAtDepthFrom4TrigonometricMoments(b_0, trig_b, depth,
            MomentOIT.moment_bias, MomentOIT.overestimation, MomentOIT.wrapping_zone_parameters);
#else
#if SINGLE_PRECISION
    vec4 b_even = imageLoad(moments, idx0);
    vec4 b_odd = imageLoad(moments, idx1);

    b_even /= b_0;
    b_odd /= b_0;
    const float bias_vector[8] = { 0, 0.75, 0, 0.67666666666666664, 0, 0.63, 0, 0.60030303030303034 };
#else
    vec4 b_even_q = imageLoad(moments, idx0);
    vec4 b_odd_q = imageLoad(moments, idx1);

    // Dequantize the moments
    vec4 b_even;
    vec4 b_odd;
    offsetAndDequantizeMoments(b_even, b_odd, b_even_q, b_odd_q);
    const float bias_vector[8] = { 0, 0.42474916387959866, 0, 0.22407802675585284, 0, 0.15369230769230768, 0, 0.12900440529089119 };
#endif
    transmittance_at_depth = computeTransmittanceAtDepthFrom8PowerMoments(b_0, b_even, b_odd, depth,
            MomentOIT.moment_bias, MomentOIT.overestimation, bias_vector);
#endif
#endif

}
#endif

#endif