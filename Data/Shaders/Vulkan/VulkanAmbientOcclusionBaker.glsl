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

layout(binding = 0) uniform Uniforms {
    // The radius of the lines.
    float lineRadius;

    // How many line points exist in total (i.e., the number of entries in the buffer "LineGeometry").
    uint numLinePoints;
    // How often should the tube be subdivided in the normal plane?
    uint numTubeSubdivisions;
    // The number of this frame (used for accumulation of samples accross frames).
    uint frameNumber;

    // How many rays should the shader shoot?
    uint numAmbientOcclusionSamples;
    // What is the radius to take into account for ambient occlusion?
    float ambientOcclusionRadius;
    // Should the distance of the AO hits be used?
    int useDistance;

    int padding;
};

struct LinePoint {
    vec3 position;
    vec3 tangent;
    vec3 normal;
    vec3 binormal;
};

struct LinePointInput {
    vec4 position;
    vec4 tangent;
    vec4 normal;
};

layout(std430, binding = 1) readonly buffer LineGeometry {
    LinePointInput linePoints[];
};

layout(std430, binding = 2) readonly buffer SamplingLocations {
    float samplingLocations[];
};

layout(std430, binding = 3) buffer AmbientOcclusionFactors {
    float ambientOcclusionFactors[];
};

layout(binding = 4) uniform accelerationStructureEXT topLevelAS;

LinePoint getInterpolatedLinePoint(uint lineSamplingIdx) {
    float samplingLocation = samplingLocations[lineSamplingIdx];
    uint lowerIdx = uint(floor(lineSamplingIdx));
    uint upperIdx = min(lowerIdx + 1u, numLinePoints - 1u);
    float interpolationFactor = fract(samplingLocation);

    LinePointInput lowerLinePoint = linePoints[lowerIdx];
    LinePointInput upperLinePoint = linePoints[upperIdx];
    vec3 binormalLower = cross(lowerLinePoint.tangent.xyz, lowerLinePoint.normal.xyz);
    vec3 binormalUpper = cross(upperLinePoint.tangent.xyz, upperLinePoint.normal.xyz);

    LinePoint interpolatedLinePoint;
    interpolatedLinePoint.position = mix(lowerLinePoint.position.xyz, upperLinePoint.position.xyz, interpolationFactor);
    interpolatedLinePoint.tangent =
            normalize(mix(lowerLinePoint.tangent.xyz, upperLinePoint.tangent.xyz, interpolationFactor));
    interpolatedLinePoint.normal =
            normalize(mix(lowerLinePoint.normal.xyz, upperLinePoint.normal.xyz, interpolationFactor));
    interpolatedLinePoint.binormal =
            normalize(mix(binormalLower, binormalUpper, interpolationFactor));
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
        flags = gl_RayFlagsTerminateOnFirstHitEXT;
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
    if (gl_GlobalInvocationID.x >= numLinePoints) {
        return;
    }

    uint lineSamplingIdx = gl_GlobalInvocationID.x;
    uint seed = tea(lineSamplingIdx, frameNumber);

    LinePoint linePoint = getInterpolatedLinePoint(lineSamplingIdx);

    for (uint tubeSudivIdx = 0; tubeSudivIdx < numTubeSubdivisions; tubeSudivIdx++) {
        float angle = float(tubeSudivIdx) / float(numTubeSubdivisions) * 2.0 * M_PI;
        vec3 surfaceNormal = cos(angle) * linePoint.normal + sin(angle) * linePoint.binormal;
        vec3 surfaceBitangent = cross(surfaceNormal, linePoint.tangent);
        mat3 frame = mat3(linePoint.tangent, surfaceBitangent, surfaceNormal);

        // Get the ray origin on the tube surface (pushed out by a small epsilon to avoid self intersections).
        vec3 rayOrigin = linePoint.position + (lineRadius + 1e-6) * surfaceNormal;

        bool didHitAny = false;
        float occlusionFactorAccumulated = 0.0;
        for (uint rayIdx = 0; rayIdx < numAmbientOcclusionSamples; rayIdx++) {
            vec2 xi = vec2(rnd(seed), rnd(seed));
            vec3 rayDirection = normalize(frame * sampleHemisphere(xi));

            rayQueryEXT rayQuery;
            float occlusionFactor = traceAoRay(rayQuery, rayOrigin, rayDirection);
            occlusionFactorAccumulated += occlusionFactor;
            if (occlusionFactorAccumulated > 0.0) {
                didHitAny = true;
            }
        }
        occlusionFactorAccumulated /= float(numAmbientOcclusionSamples);

        if (frameNumber != 0) {
            float oldAoFactor = ambientOcclusionFactors[tubeSudivIdx + numTubeSubdivisions * lineSamplingIdx];
            occlusionFactorAccumulated = mix(oldAoFactor, occlusionFactorAccumulated, 1.0 / float(frameNumber + 1));
        }
        //ambientOcclusionFactors[tubeSudivIdx + numTubeSubdivisions * lineSamplingIdx] = didHitAny ?1.0:0.0;
        ambientOcclusionFactors[tubeSudivIdx + numTubeSubdivisions * lineSamplingIdx] = occlusionFactorAccumulated;
        //ambientOcclusionFactors[tubeSudivIdx + numTubeSubdivisions * lineSamplingIdx] = mod(lineSamplingIdx / 255.0, 1.0);
    }
}
