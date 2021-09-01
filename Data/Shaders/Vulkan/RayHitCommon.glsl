/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#include "TransferFunction.glsl"

layout (binding = 0) uniform CameraSettingsBuffer {
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
} camera;

layout (binding = 3) uniform RayTracerSettingsBuffer {
    vec3 cameraPosition;
    float paddingFlt;
    vec4 backgroundColor;
    vec4 foregroundColor;

    // The maximum number of transparent fragments to blend before stopping early.
    uint maxDepthComplexity;
    // How many rays should be shot per frame?
    uint numSamplesPerFrame;

    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;

    uint paddingUint;
};

layout (binding = 4) uniform LineRenderSettingsBuffer {
    float lineWidth;
    int hasHullMesh;
    float depthCueStrength;
    float ambientOcclusionStrength;

    // Ambient occlusion settings.
    uint numAoTubeSubdivisions;
    uint numLineVertices;
    uint numParametrizationVertices;
    uint paddingLineSettings;
};

#ifdef STRESS_LINE_DATA
layout (binding = 5) uniform StressLineRenderSettingsBuffer {
    vec3 lineHierarchySlider;
    float bandWidth;
    ivec3 psUseBands;
    int currentSeedIdx;
};
#endif

struct TubeLinePointData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    float lineHierarchyLevel; ///< Zero for flow lines.
    vec3 lineNormal;
    float lineAppearanceOrder; ///< Zero for flow lines.
    uvec3 padding;
    uint principalStressIndex; ///< Zero for flow lines.
};

layout(std430, binding = 8) readonly buffer TubeLinePointDataBuffer {
    TubeLinePointData tubeLinePointDataBuffer[];
};

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadInEXT RayPayload payload;

#define RAYTRACING
#include "Lighting.glsl"

#define M_PI 3.14159265358979323846

//#define USE_ORTHOGRAPHIC_TUBE_PROJECTION

#ifndef USE_ORTHOGRAPHIC_TUBE_PROJECTION
mat3 shearSymmetricMatrix(vec3 p) {
    return mat3(vec3(0.0, -p.z, p.y), vec3(p.z, 0.0, -p.x), vec3(-p.y, p.x, 0.0));
}
#endif

void computeFragmentColor(
        vec3 fragmentPositionWorld, vec3 fragmentNormal, vec3 fragmentTangent, float fragmentAttribute,
#ifdef USE_CAPPED_TUBES
        bool isCap,
#endif
#if defined (STRESS_LINE_DATA) || defined(USE_AMBIENT_OCCLUSION)
        float phi,
#endif
#ifdef USE_AMBIENT_OCCLUSION
        float fragmentVertexId,
#endif
#ifdef STRESS_LINE_DATA
        vec3 linePosition, vec3 lineNormal,
#endif
        TubeLinePointData linePointData0, TubeLinePointData linePointData1
) {
    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 t = normalize(fragmentTangent);

#ifdef STRESS_LINE_DATA
#ifdef ANALYTIC_TUBE_INTERSECTIONS
    bool useBand = false; // Bands not supported (yet) for analytic tube intersections.
#else
    bool useBand = psUseBands[linePointData0.principalStressIndex] > 0;
#endif
    const float thickness = useBand ? 0.15 : 1.0; // hard-coded
#endif

    float ribbonPosition;

#ifdef USE_CAPPED_TUBES
    if (isCap) {
        vec3 crossProdVn = cross(v, n);
        ribbonPosition = length(crossProdVn);

        // Project v into plane perpendicular to t to get newV.
        vec3 helperVec = normalize(cross(t, v));
        vec3 newV = normalize(cross(helperVec, t));

        // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
        // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
        vec3 crossProdVn2 = cross(newV, n);
        float ribbonPosition2 = length(crossProdVn2);

        // Side note: We can also use the code below, as for a, b with length 1:
        // sqrt(1 - dot^2(a,b)) = len(cross(a,b))
        // Due to:
        // - dot(a,b) = ||a|| ||b|| cos(phi)
        // - len(cross(a,b)) = ||a|| ||b|| |sin(phi)|
        // - sin^2(phi) + cos^2(phi) = 1
        //ribbonPosition = dot(newV, n);
        //ribbonPosition = sqrt(1 - ribbonPosition * ribbonPosition);

        // Get the winding of newV relative to n, taking into account that t is the normal of the plane both vectors lie in.
        // NOTE: dot(a, cross(b, c)) = det(a, b, c), which is the signed volume of the parallelepiped spanned by a, b, c.
        if (dot(t, crossProdVn) < 0.0) {
            ribbonPosition2 = -ribbonPosition2;
        }
        // Normalize the ribbon position: [-1, 1] -> [0, 1].
        //ribbonPosition = ribbonPosition / 2.0 + 0.5;
        ribbonPosition2 = clamp(ribbonPosition2, -1.0, 1.0);

        ribbonPosition = min(ribbonPosition, abs(ribbonPosition2));
    } else {
#endif

#ifdef STRESS_LINE_DATA
        vec3 lineN = normalize(lineNormal);
        vec3 lineB = cross(t, lineN);
        mat3 tangentFrameMatrix = mat3(lineN, lineB, t);

#ifdef USE_ORTHOGRAPHIC_TUBE_PROJECTION
        vec2 localV = normalize((transpose(tangentFrameMatrix) * newV).xy);
        vec2 p = vec2(thickness * cos(phi), sin(phi));
        float d = length(p);
        p = normalize(p);
        float alpha = acos(dot(localV, p));

        float phiMax = atan(localV.x, -thickness * localV.y);

        vec2 pointMax0 = vec2(thickness * cos(phiMax), sin(phiMax));
        vec2 pointMax1 = vec2(thickness * cos(phiMax + M_PI), sin(phiMax + M_PI));
        vec2 planeDir = pointMax1 - pointMax0;
        float totalDist = length(planeDir);
        planeDir = normalize(planeDir);

        float beta = acos(dot(planeDir, localV));

        float x = d / sin(beta) * sin(alpha);
        ribbonPosition = x / totalDist * 2.0;
#else
        // Project onto the tangent plane.
        const vec3 cNorm = cameraPosition - linePosition;
        const float dist = dot(cNorm, fragmentTangent);
        const vec3 cHat = transpose(tangentFrameMatrix) * (cNorm - dist * fragmentTangent);
        const float lineRadius = (useBand ? bandWidth : lineWidth) * 0.5;

        // Homogeneous, normalized coordinates of the camera position in the tangent plane.
        const vec3 c = vec3(cHat.xy / lineRadius, 1.0);

        // Primal conic section matrix.
        //const mat3 A = mat3(
        //        1.0 / (thickness*thickness), 0.0, 0.0,
        //        0.0, 1.0, 0.0,
        //        0.0, 0.0, -1.0
        //);

        // Polar of c.
        //const vec3 l = A * c;

        // Polar of c.
        const float a = 1.0 / (thickness*thickness);
        const vec3 l = vec3(a * c.x, c.y, -1.0);

        const mat3 M_l = shearSymmetricMatrix(l);
        //const mat3 B = transpose(M_l) * A * M_l;

        const mat3 B = mat3(
        l.z*l.z - l.y*l.y, l.x*l.y, -l.x*l.z,
        l.x*l.y, a*l.z*l.z - l.x*l.x, -a*l.y*l.z,
        -l.x*l.z, -a*l.y*l.z, a*l.y*l.y + l.x*l.x
        );

        const float EPSILON = 1e-4;
        float alpha = 0.0;
        float discr = 0.0;
        if (abs(l.z) > EPSILON) {
            discr = -B[0][0] * B[1][1] + B[0][1] * B[1][0];
            alpha = sqrt(discr) / l.z;
        } else if (abs(l.y) > EPSILON) {
            discr = -B[0][0] * B[2][2] + B[0][2] * B[2][0];
            alpha = sqrt(discr) / l.y;
        } else if (abs(l.x) > EPSILON) {
            discr = -B[1][1] * B[2][2] + B[1][2] * B[2][1];
            alpha = sqrt(discr) / l.x;
        }

        mat3 C = B + alpha * M_l;

        vec2 pointMax0 = vec2(0.0);
        vec2 pointMax1 = vec2(0.0);
        for (int i = 0; i < 2; ++i) {
            if (abs(C[i][i]) > EPSILON) {
                pointMax0 = C[i].xy / C[i].z; // column vector
                pointMax1 = vec2(C[0][i], C[1][i]) / C[2][i]; // row vector
            }
        }

        vec2 p = vec2(thickness * cos(phi), sin(phi));

        vec3 pLineHomogeneous = cross(l, cross(c, vec3(p, 1.0)));
        vec2 pLine = pLineHomogeneous.xy / pLineHomogeneous.z;

        ribbonPosition = length(pLine - pointMax0) / length(pointMax1 - pointMax0) * 2.0 - 1.0;
#endif

#else
        // Project v into plane perpendicular to t to get newV.
        vec3 helperVec = normalize(cross(t, v));
        vec3 newV = normalize(cross(helperVec, t));

        // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
        // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
        vec3 crossProdVn = cross(newV, n);
        ribbonPosition = length(crossProdVn);

        // Side note: We can also use the code below, as for a, b with length 1:
        // sqrt(1 - dot^2(a,b)) = len(cross(a,b))
        // Due to:
        // - dot(a,b) = ||a|| ||b|| cos(phi)
        // - len(cross(a,b)) = ||a|| ||b|| |sin(phi)|
        // - sin^2(phi) + cos^2(phi) = 1
        //ribbonPosition = dot(newV, n);
        //ribbonPosition = sqrt(1 - ribbonPosition * ribbonPosition);

        // Get the winding of newV relative to n, taking into account that t is the normal of the plane both vectors lie in.
        // NOTE: dot(a, cross(b, c)) = det(a, b, c), which is the signed volume of the parallelepiped spanned by a, b, c.
        if (dot(t, crossProdVn) < 0.0) {
            ribbonPosition = -ribbonPosition;
        }
        // Normalize the ribbon position: [-1, 1] -> [0, 1].
        //ribbonPosition = ribbonPosition / 2.0 + 0.5;
        ribbonPosition = clamp(ribbonPosition, -1.0, 1.0);
#endif

#ifdef USE_CAPPED_TUBES
    }
#endif

#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    vec4 fragmentColor = transferFunction(fragmentAttribute, linePointData0.principalStressIndex);
#else
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#endif

#ifdef USE_DEPTH_CUES
    vec3 screenSpacePosition = (camera.viewMatrix * vec4(fragmentPositionWorld, 1.0)).xyz;
#endif

    fragmentColor = blinnPhongShadingTube(
            fragmentColor, fragmentPositionWorld,
#ifdef USE_DEPTH_CUES
            screenSpacePosition,
#endif
#ifdef USE_AMBIENT_OCCLUSION
            fragmentVertexId, phi,
#endif
#ifdef STRESS_LINE_DATA
            useBand ? 1 : 0,
#endif
            n, t);

    float absCoords = abs(ribbonPosition);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
#ifdef STRESS_LINE_DATA
    float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / (useBand ? bandWidth : lineWidth), 0.0, 0.49);
    float EPSILON_WHITE = 1e-5; // TODO
#else
    float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / lineWidth, 0.0, 0.49);
    float EPSILON_WHITE = 1e-5; // TODO
#endif
    float coverage = 1.0 - smoothstep(1.0 - EPSILON_OUTLINE, 1.0, absCoords);
    vec4 colorOut = vec4(
            mix(fragmentColor.rgb, foregroundColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON_WHITE, WHITE_THRESHOLD + EPSILON_WHITE, absCoords)),
            fragmentColor.a * coverage);

#ifdef USE_AMBIENT_OCCLUSION
    //colorOut = vec4(getAoFactor(fragmentVertexId, phi), 0.0, 0.0, 1.0);
#endif

    payload.hitColor = colorOut;
    payload.hitT = length(fragmentPositionWorld - cameraPosition);
    payload.hasHit = true;
}
