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
    int useDistance;
};

#ifdef STRESS_LINE_DATA
layout (binding = 1) uniform StressLineRenderSettingsBuffer {
    vec3 lineHierarchySlider;
    float bandWidth;
    ivec3 psUseBands;
    int currentSeedIdx;
};
#endif

struct LinePoint {
    vec3 position;
    vec3 tangent;
    vec3 normal;
    vec3 binormal;
#ifdef STRESS_LINE_DATA
    uint principalStressIndex;
#endif
};

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

layout(std430, binding = 2) readonly buffer TubeLinePointDataBuffer {
    TubeLinePointData linePoints[];
};

layout(std430, binding = 3) readonly buffer SamplingLocationsBuffer {
    float samplingLocations[];
};

layout(std430, binding = 4) buffer AmbientOcclusionFactorsBuffer {
    float ambientOcclusionFactors[];
};

layout(binding = 5) uniform accelerationStructureEXT topLevelAS;

LinePoint getInterpolatedLinePoint(uint lineSamplingIdx) {
    float samplingLocation = samplingLocations[lineSamplingIdx];
    uint lowerIdx = uint(samplingLocation);
    uint upperIdx = min(lowerIdx + 1u, numLinePoints - 1u);
    float interpolationFactor = fract(samplingLocation);

    TubeLinePointData lowerLinePoint = linePoints[lowerIdx];
    TubeLinePointData upperLinePoint = linePoints[upperIdx];
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
    interpolatedLinePoint.principalStressIndex = lowerLinePoint.principalStressIndex;
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

    for (uint tubeSudivIdx = 0; tubeSudivIdx < numTubeSubdivisions; tubeSudivIdx++) {
        float angle = float(tubeSudivIdx) / float(numTubeSubdivisions) * 2.0 * M_PI;
        float sinAngle = sin(angle);
        float cosAngle = cos(angle);

        // Get the ray origin on the tube surface (pushed out by a small epsilon to avoid self intersections).
#ifdef STRESS_LINE_DATA
        bool useBand = psUseBands[linePoint.principalStressIndex] > 0;
        const float thickness = useBand ? 0.15 : 1.0;
        const float tubeRadius = (useBand ? bandWidth * 0.5 : lineRadius);
        vec3 surfaceNormal = normalize(cosAngle * linePoint.normal + thickness * sinAngle * linePoint.binormal);
        vec3 rayOrigin = linePoint.position
                + (tubeRadius + 1e-3) * (thickness * cosAngle * linePoint.normal + sinAngle * linePoint.binormal);
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
