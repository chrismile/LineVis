/**
 * See: http://www.songho.ca/opengl/gl_projectionmatrix.html
 * We know (OpenGL matrices are column-major):
 * P_33 = M_proj[2][2] = -(f + n) / (f - n)
 * P_34 = M_proj[3][2] = -2fn / (f - n)
 * p_clip = M_proj * p_eye, p_ndc = p_clip.xyz / p_clip.w
 * z_ndc = z_clip / w_clip = (P_33 * z_e + P_34 * w_e) / (-z_e) = (P_33 * z_e + P_34) / (-z_e) =
 * <=> z_e = -P_34 / (z_ndc + p_33)
 *
 * However: z_e in looking direction is negative in OpenGL standard convention!
 * Thus: z_e = P_34 / (z_ndc + p_33), z_ndc = (-P_33 * z_e + P_34) / z_e
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

#else

float convertDepthBufferValueToLinearDepth(float depth) {
    // The depth buffer stores values in [0,1], but OpenGL uses [-1,1] for NDC.
    float z_ndc = 2.0 * depth - 1.0;
    float z_eye = 2.0 * zNear * zFar / (zFar + zNear - z_ndc * (zFar - zNear));
    return z_eye;
}

float convertLinearDepthToDepthBufferValue(float z_eye) {
    float z_ndc = (-2.0 * zNear * zFar + z_eye * (zNear + zFar)) / z_eye;
    // The depth buffer stores values in [0,1], but OpenGL uses [-1,1] for NDC.
    float depth = (z_ndc + 1.0) / 2.0;
    return depth;
}

#endif