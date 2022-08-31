-- IntersectionEllipticTubeUtil

float computeRadius(float r1, float r2, float phi, float rho) {
    float sinphirho = sin(phi + rho);
    float cosphirho = cos(phi + rho);
    return r1 * r2 / sqrt(r1 * r1 * sinphirho * sinphirho + r2 * r2 * cosphirho * cosphirho);
}

float computeRadiusB(float r1, float r2, float phi, float rho) {
    float sinphirho = sin(phi + rho);
    float cosphirho = cos(phi + rho);
    return sqrt(r1 * r1 * cosphirho * cosphirho + r2 * r2 * sinphirho * sinphirho);
}

vec3 computePosition(float r1, float r2, float x, float phi, float rho) {
    float sinphi = sin(phi);
    float cosphi = cos(phi);
    float r = r1 * r2 / sqrt(r1 * r1 * sinphi * sinphi + r2 * r2 * cosphi * cosphi);
    return vec3(x, r * cos(phi + rho), r * sin(phi + rho));
}

vec3 computeNormal(float r1, float r2, float phi, float t, float l, float rho, float rho_r) {
    float sinphi = sin(phi + rho);
    float cosphi = cos(phi + rho);
    float sinphirho = sin(phi);
    float cosphirho = cos(phi);
    float rho_r_div_l = rho_r / l;

    float r1sq = r1 * r1;
    float r2sq = r2 * r2;
    float r1r2 = r1 * r2;
    float r_denom_sq = r1sq * sinphi * sinphi + r2sq * cosphi * cosphi;
    float r_denom = sqrt(r_denom_sq);
    float r = r1r2 / r_denom;
    float ddenom_dphi = (r1sq - r2sq) * sinphi * cosphi / r_denom;
    float dr_dphi = -r1r2 * ddenom_dphi / r_denom_sq;
    //return vec3(0.0, dr_dphi * sinphirho + r * cosphirho, r * sinphirho - dr_dphi * cosphirho);
    vec3 dx_dphi = vec3(0.0, dr_dphi * cosphirho - r * sinphirho, dr_dphi * sinphirho + r * cosphirho);
    //vec3 dx_dx = vec3(1.0, -r * sinphirho * rho_r_div_l, r * cosphirho * rho_r_div_l);
    vec3 dx_dx = vec3(1.0, 0.0, 0.0);
    return cross(dx_dphi, dx_dx);
}

/**
 * Code for vector axis-angle rotation from glm::rotate (mtrix_transform.inl).
 *
 * ================================================================================
 * The MIT License
 * --------------------------------------------------------------------------------
 * Copyright (c) 2005 - G-Truc Creation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
mat3 matrixAxisRotationCos(vec3 axis, float cosAngle) {
    float c = cosAngle;
    float s = sqrt(1.0 - cosAngle * cosAngle);

    axis = normalize(axis);
    vec3 temp = vec3((1.0 - c) * axis);

    mat3 Rotate;
    Rotate[0][0] = c + temp[0] * axis[0];
    Rotate[0][1] = temp[0] * axis[1] + s * axis[2];
    Rotate[0][2] = temp[0] * axis[2] - s * axis[1];

    Rotate[1][0] = temp[1] * axis[0] - s * axis[2];
    Rotate[1][1] = c + temp[1] * axis[1];
    Rotate[1][2] = temp[1] * axis[2] + s * axis[0];

    Rotate[2][0] = temp[2] * axis[0] + s * axis[1];
    Rotate[2][1] = temp[2] * axis[1] - s * axis[0];
    Rotate[2][2] = c + temp[2] * axis[2];

    return Rotate;
}
mat3 matrixAxisRotation(vec3 axis, float angle) {
    float c = cos(angle);
    float s = sin(angle);

    axis = normalize(axis);
    vec3 temp = vec3((1.0 - c) * axis);

    mat3 Rotate;
    Rotate[0][0] = c + temp[0] * axis[0];
    Rotate[0][1] = temp[0] * axis[1] + s * axis[2];
    Rotate[0][2] = temp[0] * axis[2] - s * axis[1];

    Rotate[1][0] = temp[1] * axis[0] - s * axis[2];
    Rotate[1][1] = c + temp[1] * axis[1];
    Rotate[1][2] = temp[1] * axis[2] + s * axis[0];

    Rotate[2][0] = temp[2] * axis[0] + s * axis[1];
    Rotate[2][1] = temp[2] * axis[1] - s * axis[0];
    Rotate[2][2] = c + temp[2] * axis[2];

    return Rotate;
}


-- IntersectionEllipticTube

/**
 * Uses analytic ray-tube intersection is performed using the sphere tracing technique introduced by:
 * Guido Reina, Katrin Bidmon, Frank Enders, Peter Hastreiter and Thomas Ertl.
 * "GPU-Based Hyperstreamlines for Diffusion Tensor Imaging".
 * EUROVIS - Eurographics/IEEE VGTC Symposium on Visualization, 2006.
 * https://doi.org/10.2312/VisSym/EuroVis06/035-042
 */

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "LineUniformData.glsl"
#include "LineDataSSBO.glsl"
#import ".IntersectionEllipticTubeUtil"

layout(std430, binding = 3) readonly buffer BoundingBoxLinePointIndexBuffer {
    uvec2 boundingBoxLinePointIndices[];
};

//struct TubeletData {
//    vec4 orientationQuat;
//    float rho_r;
//};
struct AABB3 {
    vec3 boxMin;
    vec3 boxMax;
};
layout(scalar, binding = 4) readonly buffer BoundingBoxBuffer {
    AABB3 aabbs[];
};

#define BIAS 1e-3

/**
 * Helper function for rayBoxIntersection (see below).
 */
bool rayBoxPlaneIntersection(
        float rayOriginX, float rayDirectionX, float lowerX, float upperX, inout float tNear, inout float tFar) {
    if (abs(rayDirectionX) < BIAS) {
        // Ray is parallel to the x planes
        if (rayOriginX < lowerX || rayOriginX > upperX) {
            return false;
        }
    } else {
        // Not parallel to the x planes. Compute the intersection distance to the planes.
        float t0 = (lowerX - rayOriginX) / rayDirectionX;
        float t1 = (upperX - rayOriginX) / rayDirectionX;
        if (t0 > t1) {
            // Since t0 intersection with near plane
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }

        if (t0 > tNear) {
            // We want the largest tNear
            tNear = t0;
        }
        if (t1 < tFar) {
            // We want the smallest tFar
            tFar = t1;
        }
        if (tNear > tFar) {
            // Box is missed
            return false;
        }
        if (tFar < 0) {
            // Box is behind ray
            return false;
        }
    }
    return true;
}

/**
 * Implementation of ray-box intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter3.htm
 */
bool rayBoxIntersectionRayCoords(
        vec3 rayOrigin, vec3 rayDirection, vec3 lower, vec3 upper, out float tNearOut, out float tFarOut) {
    float tNear = -1e7, tFar = 1e7; // Infinite values
    for (int i = 0; i < 3; i++) {
        if (!rayBoxPlaneIntersection(rayOrigin[i], rayDirection[i], lower[i], upper[i], tNear, tFar)) {
            return false;
        }
    }

    tNearOut = tNear;
    tFarOut = tFar;
    return true;
}


void main() {
    const float radius0 = bandWidth * 0.5 * minBandThickness;
    const float radius1 = bandWidth * 0.5;
    AABB3 aabb = aabbs[gl_PrimitiveID];
    float tNear = 0.0, tFar = 0.0;
    if (!rayBoxIntersectionRayCoords(gl_WorldRayOriginEXT, gl_WorldRayDirectionEXT, aabb.boxMin, aabb.boxMax, tNear, tFar)) {
        return;
    }
    vec3 startPoint = gl_WorldRayOriginEXT + tNear * gl_WorldRayDirectionEXT;

    uvec2 linePointIndices = boundingBoxLinePointIndices[gl_PrimitiveID];
    LinePointData linePointData0 = linePoints[linePointIndices.x];
    LinePointData linePointData1 = linePoints[linePointIndices.y];
    vec3 p0 = linePointData0.linePosition;
    vec3 p1 = linePointData1.linePosition;
    //TubeletData tubelet = tubelets[gl_PrimitiveID];
    float l = length(p1 - p0); // Tubelet length.
    vec3 xt = normalize(p1 - p0); // Local tubelet x axis.
    // Reina et al. use the center point as the origin, but we use p0 for simpler interpolation.
    //vec3 tubeletCenter = 0.5 * (p0 + p1);
    vec3 tubeletCenter = p0;

    // As specified by Reina et al., we need to rotate the normal and binormal by dot(n, xt) around the axis cross(n, xt).
    vec3 rotAxis = cross(linePointData0.lineNormal, xt);
    float rotCosAngle = dot(linePointData0.lineNormal, xt);
    mat3 rotationMatrix = mat3(1.0);
    if (abs(rotCosAngle) > 0.999) {
        rotationMatrix = matrixAxisRotationCos(rotAxis, rotCosAngle);
    }
    vec3 yt = rotationMatrix * linePointData0.lineNormal;
    vec3 zt = rotationMatrix * cross(linePointData0.lineTangent, linePointData0.lineNormal);

    // Get the ellipse rotation angle from the left to the right end of the tubelet.
    float rho_r = -atan(
            dot(cross(linePointData0.lineNormal, linePointData1.lineNormal), linePointData0.lineTangent),
            dot(linePointData0.lineNormal, linePointData1.lineNormal));

    // Left and right cutting planes.
    vec4 E_l;
    vec4 E_r;
    E_l.xyz = linePointData0.lineTangent;
    E_l.w = -dot(E_l.xyz, p0);
    E_r.xyz = -linePointData1.lineTangent;
    E_r.w = -dot(E_r.xyz, p1);

    // Transform to tubelet coordinate system.
    mat3 frameMatrix = mat3(xt, yt, zt);
    mat3 invFrameMatrix = transpose(frameMatrix);
    vec3 pStart = invFrameMatrix * (startPoint - tubeletCenter);
    vec3 p = pStart;
    vec3 d = invFrameMatrix * gl_WorldRayDirectionEXT;

    // Use sphere tracing.
    const int MAX_NUM_SPHERE_TRACING_ITERATIONS = 80;
    const float EPSILON_SPHERE_TRACING = 1e-4f;
    const float EPSILON_SPHERE_TRACING_TERMINATION = 1e-5f;
    float hitT = tNear; // Hit distance
    float d_tmp;
    for (int i = 0; i < MAX_NUM_SPHERE_TRACING_ITERATIONS; i++) {
        float t = clamp(p.x / l, 0.0, 1.0);
        float rho_x = t * rho_r;
        float phi = atan(p.z, p.y);
        float r = computeRadius(radius0, radius1, phi, rho_x);
        d_tmp = sqrt(p.y * p.y + p.z * p.z) - r;
        d_tmp *= 0.25;
        p += d * d_tmp;
        hitT += d_tmp;
        // TODO: Scale by (1 − cos(dot(N, s))) + 1 (?)
        if (d_tmp < EPSILON_SPHERE_TRACING_TERMINATION) {
            break;
        }
    }
    vec3 pointWorld = frameMatrix * p + tubeletCenter;
    // Cull points on the left and right of the clipping planes.
    float EPSILON_CLIPPING_PLANE_1 = abs(dot(E_l.xyz, normalize(cameraPosition - p0))) * 5e-5;
    float EPSILON_CLIPPING_PLANE_2 = abs(dot(E_r.xyz, normalize(cameraPosition - p1))) * 5e-5;
    bool isNotClippedLeft = dot(E_l.xyz, pointWorld) + E_l.w > -EPSILON_CLIPPING_PLANE_1;
    bool isNotClippedRight = dot(E_r.xyz, pointWorld) + E_r.w > -EPSILON_CLIPPING_PLANE_2;
    if (d_tmp < EPSILON_SPHERE_TRACING && hitT > 0.0 && isNotCulledLeft && isNotCulledRight) {
        reportIntersectionEXT(hitT, 0);
    }
}


-- ClosestHitEllipticTubeAnalytic

#version 460
#extension GL_EXT_ray_tracing : require

#define ANALYTIC_ELLIPTIC_TUBE_INTERSECTIONS

#define VULKAN_RAY_TRACING_SHADER
#include "RayHitCommon.glsl"
#import ".IntersectionEllipticTubeUtil"

layout(std430, binding = 3) readonly buffer BoundingBoxLinePointIndexBuffer {
    uvec2 boundingBoxLinePointIndices[];
};

void main() {
    uvec2 linePointIndices = boundingBoxLinePointIndices[gl_PrimitiveID];
    LinePointData linePointData0 = linePoints[linePointIndices.x];
    LinePointData linePointData1 = linePoints[linePointIndices.y];

    vec3 fragmentPositionWorld = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;

    vec3 p0 = linePointData0.linePosition;
    vec3 p1 = linePointData1.linePosition;
    float l = length(p1 - p0); // Tubelet length.
    vec3 xt = normalize(p1 - p0); // Local tubelet x axis.
    // Reina et al. use the center point as the origin, but we use p0 for simpler interpolation.
    //vec3 tubeletCenter = 0.5 * (p0 + p1);
    vec3 tubeletCenter = p0;

    // As specified by Reina et al., we need to rotate the normal and binormal by dot(n, xt) around the axis cross(n, xt).
    vec3 rotAxis = cross(linePointData0.lineNormal, xt);
    float rotCosAngle = dot(linePointData0.lineNormal, xt);
    mat3 rotationMatrix = mat3(1.0);
    if (abs(rotCosAngle) > 0.999) {
        rotationMatrix = matrixAxisRotationCos(rotAxis, rotCosAngle);
    }
    vec3 yt = rotationMatrix * linePointData0.lineNormal;
    vec3 zt = rotationMatrix * cross(linePointData0.lineTangent, linePointData0.lineNormal);

    // Get the ellipse rotation angle from the left to the right end of the tubelet.
    float rho_r = -atan(
            dot(cross(linePointData0.lineNormal, linePointData1.lineNormal), linePointData0.lineTangent),
            dot(linePointData0.lineNormal, linePointData1.lineNormal));

    // Transform to tubelet coordinate system.
    mat3 frameMatrix = mat3(xt, yt, zt);
    mat3 invFrameMatrix = transpose(frameMatrix);
    vec3 p = invFrameMatrix * (fragmentPositionWorld - tubeletCenter);
    vec3 d = invFrameMatrix * gl_WorldRayDirectionEXT;

    const float radius0 = bandWidth * 0.5 * minBandThickness;
    const float radius1 = bandWidth * 0.5;
    float t = clamp(p.x / l, 0.0, 1.0);
    float phi = atan(p.z, p.y);
    float rho = t * rho_r;
    vec3 fragmentNormal = frameMatrix * computeNormal(radius0, radius1, phi, t, l, rho, rho_r);
    fragmentNormal = normalize(fragmentNormal);


    // Attribute interpolation.
    float fragmentAttribute = (1.0 - t) * linePointData0.lineAttribute + t * linePointData1.lineAttribute;
    vec3 linePointInterpolated = (1.0 - t) * linePointData0.linePosition + t * linePointData1.linePosition;
    //vec3 fragmentTangent = (1.0 - t) * linePointData0.lineTangent + t * linePointData1.lineTangent;
    vec3 fragmentTangent = xt;

#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    /*vec3 rotAxisNormal = xt;
    float rotAngleNormal = rho;
    mat3 rotationMatrixNormal = mat3(1.0);
    if (abs(rotAngleNormal) > 1e-3) {
        rotationMatrixNormal = matrixAxisRotation(rotAxisNormal, rotAngleNormal);
    }
    vec3 lineNormal = rotationMatrixNormal * yt;*/
    vec3 lineNormal = (1.0 - t) * linePointData0.lineNormal + t * linePointData1.lineNormal;
    lineNormal = normalize(lineNormal);

    /**
     * There are two representations for the angle phi.
     * - Reina et al. use a parametrization p = (r(phi) * cos(phi), r(phi) * sin(phi))^T. Here, phi is a polar angle.
     *   r(phi) is the dilation of the unit circle into the polar direction given by phi, which is used to get an ellipse.
     *   r(phi) = r1 * r2 / sqrt(r1^2 * sin^2(phi) + r2^2 * cos^2(phi)) can be obtained by plugging p into the quadratic
     *   form x^2/r1^2 + y^2/r2^2 = 1.
     * - We use the parametrization p = (r1 * cos(phi), r2 * sin(phi))^T, which distributes more points close to the
     *   semi-major axis of the ellipse.
     * Thus, a conversion of one parametrization space to the other must be used below.
     */
    float sinphi = sin(phi + rho);
    float cosphi = cos(phi + rho);
    float phiDenomInv = 1.0 / sqrt(radius0 * radius0 * sinphi * sinphi + radius1 * radius1 * cosphi * cosphi);
    float sinPhiLine = radius0 * sinphi * phiDenomInv;
    float cosPhiLine = radius1 * cosphi * phiDenomInv;
    float phiLine = mod(atan(sinPhiLine, cosPhiLine) + 6.283185307, 6.283185307);
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING)
    float fragmentVertexId = (1.0 - t) * linePointIndices.x + t * linePointIndices.y;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
    float fragmentRotation =
            ((1.0 - t) * linePointData0.lineRotation + t * linePointData1.lineRotation) * helicityRotationFactor;
#endif

#ifdef STRESS_LINE_DATA
    StressLinePointData stressLinePointData0 = stressLinePoints[linePointIndices.x];
    uint principalStressIndex = stressLinePointData0.linePrincipalStressIndex;
    float lineAppearanceOrder = stressLinePointData0.lineLineAppearanceOrder;
#ifdef USE_PRINCIPAL_STRESSES
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData0 = principalStressLinePoints[linePointIndices.x];
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData1 = principalStressLinePoints[linePointIndices.y];
    float fragmentMajorStress =
            (1.0 - t) * stressLinePointPrincipalStressData0.lineMajorStress
            + t * stressLinePointPrincipalStressData1.lineMajorStress;
    float fragmentMediumStress =
            (1.0 - t) * stressLinePointPrincipalStressData0.lineMediumStress
            + t * stressLinePointPrincipalStressData1.lineMediumStress;
    float fragmentMinorStress =
            (1.0 - t) * stressLinePointPrincipalStressData0.lineMinorStress
            + t * stressLinePointPrincipalStressData1.lineMinorStress;
#endif
#endif

    computeFragmentColor(
            fragmentPositionWorld, fragmentNormal, fragmentTangent,
#ifdef USE_CAPPED_TUBES
            false,
#endif
#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
            phiLine,
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING)
            fragmentVertexId,
#endif
#ifdef USE_BANDS
            linePointInterpolated, lineNormal,
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
            fragmentRotation,
#endif
#ifdef STRESS_LINE_DATA
            principalStressIndex, lineAppearanceOrder,
#ifdef USE_PRINCIPAL_STRESSES
            fragmentMajorStress, fragmentMediumStress, fragmentMinorStress,
#endif
#endif
            fragmentAttribute
    );
}


-- AnyHitEllipticTubeAnalytic

#import ".ClosestHitEllipticTubeAnalytic"
