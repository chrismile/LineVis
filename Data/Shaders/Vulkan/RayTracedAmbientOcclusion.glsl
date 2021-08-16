-- Compute

#version 460
#extension GL_EXT_ray_query : require

layout(local_size_x = 256) in;

layout(location = 0) in vec3 vertexPosition;

layout(binding = 0) uniform Uniforms {
    uint numAmbientOcclusionRays;
    uint numTubeSubdivisions;
    uint numLinePoints;
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

layout(std430, binding = 0) readonly buffer LineGeometry {
    LinePointInput linePoints[];
};

layout(std430, binding = 1) readonly buffer SamplingLocations {
    float samplingLocations[];
};

layout(std430, binding = 2) writeonly buffer AmbientOcclusionFactors {
    float ambientOcclusionFactors[];
};

//layout(binding = 1, r8) uniform image2D image;

layout(binding = 2) uniform accelerationStructureEXT topLevelAS;

layout(location = 0) rayPayloadEXT float occlusionFactor;

LinePoint getInterpolatedLinePoint(uint lineSamplingIdx) {
    float samplingLocation = samplingLocations[lineSamplingIdx];
    uint lowerIdx = uint(floor(lineSamplingIdx));
    uint upperIdx = min(lowerIdx + 1u, numLinePoints - 1u);
    float interpolationFactor = fract(samplingLocation);

    LinePointInput lowerLinePoint = linePoints[lowerIdx];
    LinePointInput upperLinePoint = linePoints[upperIdx];
    vec4 binormalLower = cross(lowerLinePoint.tangent, lowerLinePoint.normal);
    vec4 binormalUpper = cross(upperLinePoint.tangent, upperLinePoint.normal);

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
    vec3 v = vec3(cos(phi) * r, sin(phi) * r, xi.x);
}

void main() {
    uint lineSamplingIdx = gl_GlobalInvocationID.x;
    uint seed = seed(lineSamplingIdx);

    LinePoint linePoint = getInterpolatedLinePoint(lineSamplingIdx);

    for (uint tubeSudivIdx = 0; tubeSudivIdx < numTubeSubdivisions; tubeSudivIdx++) {
        float angle = float(tubeSudivIdx) / float(numTubeSubdivisions) * 2.0 * M_PI;
        float cosAngle = cos(t);
        float sinAngle = sin(t);

        vec3 surfaceNormal = cos(angle) * linePoint.normal + sin(angle) * linePoint.binormal;
        vec3 surfaceBitangent = cross(surfaceNormal, linePoint.tangent);
        mat3 frame = mat3(linePoint.tangent, surfaceBitangent, surfaceNormal);

        float occlusionFactorAccumulated = 0.0f;
        for (uint rayIdx = 0; rayIdx < numAmbientOcclusionRays; rayIdx++) {
            vec2 xi;
            vec3 rayDirection = normalize(frame * sampleHemisphere(xi));

            occlusionFactor = 1.0;
            traceRayEXT(
                    topLevelAS,
                    gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsOpaqueEXT | gl_RayFlagsSkipClosestHitShaderEXT,
                    0xFF, 0, 0, 1, origin, tMin, rayDirection, tMax, 1);
            occlusionFactorAccumulated += occlusionFactor;
        }
        occlusionFactorAccumulated /= float(numAmbientOcclusionRays);
        ambientOcclusionFactors[tubeSudivIdx + numTubeSubdivisions * lineSamplingIdx] = occlusionFactorAccumulated;
    }
}


-- Miss

#version 460
#extension GL_EXT_ray_tracing : require

layout(location = 0) rayPayloadInEXT float occlusionFactor;

void main() {
    occlusionFactor = 1.0;
}
