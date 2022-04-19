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

-- Compute

#version 460
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_ray_tracing : enable
#extension GL_EXT_ray_query : enable

#include "RayTracingUtilities.glsl"

/**
 * This shader computes ambient occlusion coefficients for points on tubes.
 * The work groups iterate over all lines and over a number of subdivisions of the tubes.
 */

layout(local_size_x = 256) in;

layout(binding = 0) uniform UniformsBuffer {
    // The radius of the lines.
    float lineRadius;
    // The radius of the bands.
    float bandRadius;
    // The minimum band thickness to use when rendering bands.
    float minBandThickness;
    // What is the radius to take into account for ambient occlusion?
    float ambientOcclusionRadius;

    // How many line points exist in total (i.e., the number of entries in the buffer "LineGeometry").
    uint numLinePoints;
    // How many line points exist in total (i.e., the number of entries in the buffer "LineGeometry").
    uint numParametrizationVertices;
    // How often should the tube be subdivided in the normal plane?
    uint numTubeSubdivisions;
    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;

    // How many rays should the shader shoot?
    uint numAmbientOcclusionSamples;
    // Should the distance of the AO hits be used?
    uint useDistance;

    uint aoUniformBufferPadding0;
    uint aoUniformBufferPadding1;
};

#ifdef STRESS_LINE_DATA
layout (binding = 1) uniform StressLineUniformDataBuffer {
    vec3 lineHierarchySlider;
    float paddingStressLineSettings;
    ivec3 psUseBands;
    int currentSeedIdx;
};
#endif

layout(binding = 2) uniform accelerationStructureEXT topLevelAS;

layout(std430, binding = 3) readonly buffer SamplingLocationsBuffer {
    float samplingLocations[];
};

layout(std430, binding = 4) buffer AmbientOcclusionFactorsBuffer {
    float ambientOcclusionFactors[];
};

struct LinePoint {
    vec3 position;
    vec3 tangent;
    vec3 normal;
    vec3 binormal;
#ifdef STRESS_LINE_DATA
    uint principalStressIndex;
#ifdef USE_PRINCIPAL_STRESSES
    float majorStress;
    float mediumStress;
    float minorStress;
#endif
#endif
};

#define LINE_POINTS_BUFFER_BINDING 5
#define STRESS_LINE_POINTS_BUFFER_BINDING 6
#define STRESS_LINE_POINTS_PRINCIPAL_STRESS_BUFFER_BINDING 7
#include "LineDataSSBO.glsl"

LinePoint getInterpolatedLinePoint(uint lineSamplingIdx) {
    float samplingLocation = samplingLocations[lineSamplingIdx];
    uint lowerIdx = uint(samplingLocation);
    uint upperIdx = min(lowerIdx + 1u, numLinePoints - 1u);
    float interpolationFactor = fract(samplingLocation);

    LinePointData lowerLinePoint = linePoints[lowerIdx];
    LinePointData upperLinePoint = linePoints[upperIdx];

    vec3 binormalLower = cross(lowerLinePoint.lineTangent.xyz, lowerLinePoint.lineNormal.xyz);
    vec3 binormalUpper = cross(upperLinePoint.lineTangent.xyz, upperLinePoint.lineNormal.xyz);

    LinePoint interpolatedLinePoint;
    interpolatedLinePoint.position = mix(
            lowerLinePoint.linePosition.xyz, upperLinePoint.linePosition.xyz, interpolationFactor);
    interpolatedLinePoint.tangent =
            normalize(mix(lowerLinePoint.lineTangent.xyz, upperLinePoint.lineTangent.xyz, interpolationFactor));
    interpolatedLinePoint.normal =
            normalize(mix(lowerLinePoint.lineNormal.xyz, upperLinePoint.lineNormal.xyz, interpolationFactor));
    interpolatedLinePoint.binormal =
            normalize(mix(binormalLower, binormalUpper, interpolationFactor));
#ifdef STRESS_LINE_DATA
    StressLinePointData lowerStressLinePoint = stressLinePoints[lowerIdx];
    interpolatedLinePoint.principalStressIndex = lowerStressLinePoint.linePrincipalStressIndex;
#ifdef USE_PRINCIPAL_STRESSES
    StressLinePointPrincipalStressData lowerStressLinePointPrincipalStressData = principalStressLinePoints[lowerIdx];
    StressLinePointPrincipalStressData upperStressLinePointPrincipalStressData = principalStressLinePoints[upperIdx];
    interpolatedLinePoint.majorStress = mix(
            lowerStressLinePointPrincipalStressData.lineMajorStress,
            upperStressLinePointPrincipalStressData.lineMajorStress,
            interpolationFactor);
    interpolatedLinePoint.mediumStress = mix(
            lowerStressLinePointPrincipalStressData.lineMediumStress,
            upperStressLinePointPrincipalStressData.lineMediumStress,
            interpolationFactor);
    interpolatedLinePoint.minorStress = mix(
            lowerStressLinePointPrincipalStressData.lineMinorStress,
            upperStressLinePointPrincipalStressData.lineMinorStress,
            interpolationFactor);
#endif
#endif
    return interpolatedLinePoint;
}

#define M_PI 3.14159265358979323846

/**
 * Uniformly samples a direction on the upper hemisphere for the surface normal vector n = (0, 0, 1)^T.
 * For more details see:
 * https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations
 * @param xi Two random numbers uniformly sampled in the range [0, 1).
 */
vec3 sampleHemisphere(vec2 xi) {
    float theta = acos(xi.x);
    float phi = 2.0 * M_PI * xi.y;
    float r = sqrt(1 - xi.x * xi.x);
    return vec3(cos(phi) * r, sin(phi) * r, xi.x);
}

float traceAoRay(rayQueryEXT rayQuery, vec3 rayOrigin, vec3 rayDirection) {
    uint flags = gl_RayFlagsOpaqueEXT;
    if (useDistance == 0) {
        flags |= gl_RayFlagsTerminateOnFirstHitEXT;
    }

    rayQueryInitializeEXT(rayQuery, topLevelAS, flags, 0xFF, rayOrigin, 0.0, rayDirection, ambientOcclusionRadius);
    while(rayQueryProceedEXT(rayQuery)) {}

    if (rayQueryGetIntersectionTypeEXT(rayQuery, true) != gl_RayQueryCommittedIntersectionNoneEXT) {
        if (useDistance == 0) {
            return 0.0;
        }
        return rayQueryGetIntersectionTEXT(rayQuery, true) / ambientOcclusionRadius;
    }

    return 1.0;
}

void main() {
    if (gl_GlobalInvocationID.x >= numParametrizationVertices) {
        return;
    }

    uint lineSamplingIdx = gl_GlobalInvocationID.x;
    uint seed = tea(lineSamplingIdx, frameNumber);

    LinePoint linePoint = getInterpolatedLinePoint(lineSamplingIdx);

#ifdef USE_BANDS
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
    float stressX;
    float stressZ;
    if (linePoint.principalStressIndex == 0) {
        stressX = linePoint.mediumStress;
        stressZ = linePoint.minorStress;
    } else if (linePoint.principalStressIndex == 1) {
        stressX = linePoint.minorStress;
        stressZ = linePoint.majorStress;
    } else {
        stressX = linePoint.mediumStress;
        stressZ = linePoint.majorStress;
    }
#endif

#ifdef STRESS_LINE_DATA
    bool useBand = psUseBands[linePoint.principalStressIndex] > 0;
#else
    bool useBand = true;
#endif

#if defined(USE_NORMAL_STRESS_RATIO_TUBES)
    float factorX = clamp(abs(stressX / stressZ), 0.0, 1.0f);
    float factorZ = clamp(abs(stressZ / stressX), 0.0, 1.0f);
    const float thickness0 = useBand ? factorX : 1.0;
    const float thickness1 = useBand ? factorZ : 1.0;
#elif defined(USE_HYPERSTREAMLINES)
    stressX = abs(stressX);
    stressZ = abs(stressZ);
    const float thickness0 = useBand ? stressX : 1.0;
    const float thickness1 = useBand ? stressZ : 1.0;
#else
    // Bands with minimum thickness.
    const float thickness = useBand ? minBandThickness : 1.0;
#endif
#endif

    for (uint tubeSudivIdx = 0; tubeSudivIdx < numTubeSubdivisions; tubeSudivIdx++) {
        float angle = float(tubeSudivIdx) / float(numTubeSubdivisions) * 2.0 * M_PI;
        float sinAngle = sin(angle);
        float cosAngle = cos(angle);

        // Get the ray origin on the tube surface (pushed out by a small epsilon to avoid self intersections).
#ifdef USE_BANDS
        const float tubeRadius = (useBand ? bandRadius : lineRadius);
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        vec3 surfaceNormal =
                normalize(thickness1 * cosAngle * linePoint.normal + thickness0 * sinAngle * linePoint.binormal);
        vec3 rayOrigin = linePoint.position + (tubeRadius + 1e-3) *
                (thickness0 * cosAngle * linePoint.normal + thickness1 * sinAngle * linePoint.binormal);
#else
        vec3 surfaceNormal =
                normalize(cosAngle * linePoint.normal + thickness * sinAngle * linePoint.binormal);
        vec3 rayOrigin = linePoint.position + (tubeRadius + 1e-3) *
                (thickness * cosAngle * linePoint.normal + sinAngle * linePoint.binormal);
#endif
#else
        vec3 surfaceNormal = cosAngle * linePoint.normal + sinAngle * linePoint.binormal;
        vec3 rayOrigin = linePoint.position + (lineRadius + 1e-6) * surfaceNormal;
#endif

        vec3 surfaceBitangent = cross(surfaceNormal, linePoint.tangent);
        mat3 frame = mat3(linePoint.tangent, surfaceBitangent, surfaceNormal);

        float occlusionFactorAccumulated = 0.0;
        for (uint rayIdx = 0; rayIdx < numAmbientOcclusionSamples; rayIdx++) {
            vec2 xi = vec2(rnd(seed), rnd(seed));
            vec3 rayDirection = normalize(frame * sampleHemisphere(xi));

            rayQueryEXT rayQuery;
            float occlusionFactor = traceAoRay(rayQuery, rayOrigin, rayDirection);
            occlusionFactorAccumulated += occlusionFactor;
        }
        occlusionFactorAccumulated /= float(numAmbientOcclusionSamples);

        if (frameNumber != 0) {
            float oldAoFactor = ambientOcclusionFactors[tubeSudivIdx + numTubeSubdivisions * lineSamplingIdx];
            occlusionFactorAccumulated = mix(oldAoFactor, occlusionFactorAccumulated, 1.0 / float(frameNumber + 1));
        }
        ambientOcclusionFactors[tubeSudivIdx + numTubeSubdivisions * lineSamplingIdx] = occlusionFactorAccumulated;
    }
}
