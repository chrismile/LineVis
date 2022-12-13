/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2021, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DEPTH_HELPER_GLSL
#define DEPTH_HELPER_GLSL

#ifdef VULKAN

#ifdef DEPTH_HELPER_USE_PROJECTION_MATRIX

float convertDepthBufferValueToLinearDepth(float z_ndc) {
    float z_eye = pMatrix[3][2] / (pMatrix[2][2] + z_ndc);
    return z_eye;
}

float convertLinearDepthToDepthBufferValue(float z_eye) {
    float z_ndc = (pMatrix[3][2] - pMatrix[2][2] * z_eye) / z_eye;
    return z_ndc;
}

#ifdef USE_INVERSE_PROJECTION_MATRIX

vec3 convertDepthBufferValueToViewSpacePosition(vec2 ndcPositionXY, float depth) {
    vec4 posViewSpaceHomog = inverseProjectionMatrix * vec4(ndcPositionXY.x, ndcPositionXY.y, depth, 1.0);
    vec3 posViewSpace = posViewSpaceHomog.xyz / posViewSpaceHomog.w;
    return posViewSpace;
}

#endif

#else

#ifdef REVERSE_DEPTH
// Near plane is at 1, far plane at 0.
float convertDepthBufferValueToLinearDepth(float z_ndc) {
    float z_eye = zNear * zFar / (zNear + z_ndc * (zFar - zNear));
    return z_eye;
}

float convertLinearDepthToDepthBufferValue(float z_eye) {
    float diff = zFar - zNear;
    float z_ndc = (zNear * zFar / diff - z_eye * zNear / diff) / z_eye;
    return z_ndc;
}
#else
// Near plane is at 0, far plane at 1.
float convertDepthBufferValueToLinearDepth(float z_ndc) {
    float z_eye = zNear * zFar / (zFar + z_ndc * (zNear - zFar));
    return z_eye;
}

float convertLinearDepthToDepthBufferValue(float z_eye) {
    float diff = zNear - zFar;
    float z_ndc = (zNear * zFar / diff - z_eye * zFar / diff) / z_eye;
    return z_ndc;
}
#endif

#endif

#else

/**
 * See: http://www.songho.ca/opengl/gl_projectionmatrix.html
 * We know (OpenGL matrices are column-major):
 * P_33 = M_proj[2][2] = -(f + n) / (f - n)
 * P_34 = M_proj[3][2] = -2fn / (f - n)
 * p_clip = M_proj * p_eye, p_ndc = p_clip.xyz / p_clip.w
 * z_ndc = z_clip / w_clip = (P_33 * z_e + P_34 * w_e) / (-z_e) = (P_33 * z_e + P_34) / (-z_e) =
 * <=> z_e = -P_34 / (z_ndc + P_33)
 *
 * However: z_e in looking direction is negative in OpenGL standard convention!
 * Thus: z_e = P_34 / (z_ndc + P_33), z_ndc = (-P_33 * z_e + P_34) / z_e
 * Furthermore: d is in range [0,1], but z_ndc is in range [-1,1].
 * d = (z_ndc + 1) / 2
 *
 * Can be validated with: https://stackoverflow.com/questions/11277501/how-to-recover-view-space-position-given-view-space-depth-value-and-ndc-xy/46118945#46118945
 * (the answer also contains infos to convert NDC to eye position).
 */
#ifdef DEPTH_HELPER_USE_PROJECTION_MATRIX

float convertDepthBufferValueToLinearDepth(float depth) {
    // The depth buffer stores values in [0,1], but OpenGL uses [-1,1] for NDC.
    float z_ndc = 2.0 * depth - 1.0;
    float z_eye = pMatrix[3][2] / (pMatrix[2][2] + z_ndc);
    return z_eye;
}

float convertLinearDepthToDepthBufferValue(float z_eye) {
    float z_ndc = (pMatrix[3][2] - pMatrix[2][2] * z_eye) / z_eye;
    // The depth buffer stores values in [0,1], but OpenGL uses [-1,1] for NDC.
    float depth = (z_ndc + 1.0) / 2.0;
    return depth;
}

#ifdef USE_INVERSE_PROJECTION_MATRIX

vec3 convertDepthBufferValueToViewSpacePosition(vec2 ndcPositionXY, float depth) {
    // The depth buffer stores values in [0,1], but OpenGL uses [-1,1] for NDC.
    vec4 posViewSpaceHomog = inverseProjectionMatrix * vec4(ndcPositionXY.x, ndcPositionXY.y, 2.0 * depth - 1.0, 1.0);
    vec3 posViewSpace = posViewSpaceHomog.xyz / posViewSpaceHomog.w;
    return posViewSpace;
}

#endif

#else

float convertDepthBufferValueToLinearDepth(float depth) {
    // The depth buffer stores values in [0,1], but OpenGL uses [-1,1] for NDC.
    float z_ndc = 2.0 * depth - 1.0;
    float z_eye = 2.0 * zNear * zFar / (zFar + zNear - z_ndc * (zFar - zNear));
    return z_eye;
}

float convertLinearDepthToDepthBufferValue(float z_eye) {
    float diff = zFar - zNear;
    float z_ndc = (-2.0 * zNear * zFar / diff + z_eye * (zNear + zFar) / diff) / z_eye;
    // The depth buffer stores values in [0,1], but OpenGL uses [-1,1] for NDC.
    float depth = (z_ndc + 1.0) / 2.0;
    return depth;
}

#endif

#endif

#endif // DEPTH_HELPER_GLSL
