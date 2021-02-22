/**
 * This file is part of an OpenGL port of the HLSL code accompanying the paper "Moment-Based Order-Independent
 * Transparency" by Münstermann, Krumpen, Klein, and Peters (http://momentsingraphics.de/?page_id=210).
 * The original code was released in accordance to CC0 (https://creativecommons.org/publicdomain/zero/1.0/).
 *
 * This port is released under the terms of BSD 2-Clause License. For more details please see the LICENSE file in the
 * root directory of this project.
 *
 * Changes for the OpenGL port: Copyright 2018-2020 Christoph Neuhauser
 */

#ifndef PIXELSYNCOIT_OIT_MBOIT_UTILS_HPP
#define PIXELSYNCOIT_OIT_MBOIT_UTILS_HPP

#include <glm/glm.hpp>

struct MomentOITUniformData
{
    glm::vec4 wrapping_zone_parameters;
    float overestimation;
    float moment_bias;
};

enum MBOITPixelFormat {
    MBOIT_PIXEL_FORMAT_FLOAT_32, MBOIT_PIXEL_FORMAT_UNORM_16
};

// Circle constant.
#ifndef M_PI
#define M_PI 3.14159265358979323f
#endif

/**
 * This utility function turns an angle from 0 to 2*pi into a parameter that grows monotonically as function of the
 * input. It is designed to be efficiently computable from a point on the unit circle and must match the function in
 * TrigonometricMomentMath.glsl.
 * @param pOutMaxParameter Set to the maximal possible output value.
 */
float circleToParameter(float angle, float* pOutMaxParameter = nullptr);

/**
 * Given an angle in radians providing the size of the wrapping zone, this function computes all constants required by
 * the shader.
 */
void computeWrappingZoneParameters(glm::vec4 &p_out_wrapping_zone_parameters,
        float new_wrapping_zone_angle = 0.1f * M_PI);

#endif //PIXELSYNCOIT_OIT_MBOIT_UTILS_HPP
