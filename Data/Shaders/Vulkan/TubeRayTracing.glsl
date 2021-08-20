-- RayGen

#version 460
#extension GL_EXT_ray_tracing : require

#include "LineRenderSettings.glsl"

layout (binding = 0) uniform CameraSettings {
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
} camera;

layout(binding = 5, rgba8) uniform image2D outputImage;

layout(binding = 6) uniform accelerationStructureEXT topLevelAS;

layout(location = 0) rayPayloadEXT vec4 hitColor;
#ifdef USE_TRANSPARENCY
layout(location = 1) rayPayloadEXT float hitT;
layout(location = 2) rayPayloadEXT bool hasHit;
#endif

// Minimum distance between two consecutive hits.
const float HIT_DISTANCE_EPSILON = 1e-6;

void main() {
    vec2 fragNdc = 2.0 * ((vec2(gl_LaunchIDEXT.xy) + vec2(0.5)) / vec2(gl_LaunchSizeEXT.xy)) - 1.0;

    vec3 rayOrigin = (camera.inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec3 rayTarget = (camera.inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (camera.inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;

    float tMin = 0.0001f;
    float tMax = 1000.0f;

    vec4 fragmentColor = vec4(0.0);

#ifdef USE_TRANSPARENCY
    for (uint hitIdx = 0; hitIdx < maxDepthComplexity; hitIdx++) {
        traceRayEXT(
                topLevelAS, 0, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0);

        if (!hasHit) {
            break;
        }
        tMin = hitT + HIT_DISTANCE_EPSILON;

        // Front-to-back blending (hitColor uses post-multiplied, fragmentColor uses pre-multiplied alpha).
        fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.a * hitColor.rgb;
        fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
    }
#else
    traceRayEXT(
            topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0);
    fragmentColor = hitColor;
#endif

    ivec2 writePos = ivec2(gl_LaunchIDEXT.xy);
    writePos.y = imageSize(outputImage).y - writePos.y - 1;
    imageStore(outputImage, writePos, fragmentColor);
}


-- Miss

#version 460
#extension GL_EXT_ray_tracing : require

#include "LineRenderSettings.glsl"

layout(location = 0) rayPayloadInEXT vec4 hitColor;
#ifdef USE_TRANSPARENCY
layout(location = 1) rayPayloadInEXT float hitT;
layout(location = 2) rayPayloadInEXT bool hasHit;
#endif

void main() {
    hitColor = backgroundColor;
#ifdef USE_TRANSPARENCY
    hitT = 0.0f;
    hasHit = false;
#endif
}


-- ClosestHit

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "LineRenderSettings.glsl"

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to TubeTriangleLinePointData entry.
    vec3 vertexNormal;
    float padding;
};

struct TubeTriangleLinePointData {
    vec3 lineTangent;
    float lineAttribute;
    vec3 lineNormal;
    float lineHierarchyLevel; ///< Zero for flow lines.
    float lineAppearanceOrder; ///< Zero for flow lines.
    uint principalStressIndex; ///< Zero for flow lines.
    float padding0, padding1;
};

layout(scalar, binding = 2) readonly buffer IndexBuffer {
    uvec3 indexBuffer[];
};

layout(std430, binding = 3) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

layout(std430, binding = 4) readonly buffer TubeTriangleLinePointDataBuffer {
    TubeTriangleLinePointData tubeTriangleLinePointDataBuffer[];
};

layout(location = 0) rayPayloadInEXT vec4 hitColor;
#ifdef USE_TRANSPARENCY
layout(location = 1) rayPayloadInEXT float hitT;
layout(location = 2) rayPayloadInEXT bool hasHit;
#endif

hitAttributeEXT vec2 attribs;

float interpolateFloat(float v0, float v1, float v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}
vec3 interpolateVec3(vec3 v0, vec3 v1, vec3 v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}

void main() {
    uvec3 triangleIndices = indexBuffer[gl_PrimitiveID];
    const vec3 barycentricCoordinates = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    TubeTriangleVertexData vertexData0 = tubeTriangleVertexDataBuffer[triangleIndices.x];
    TubeTriangleVertexData vertexData1 = tubeTriangleVertexDataBuffer[triangleIndices.y];
    TubeTriangleVertexData vertexData2 = tubeTriangleVertexDataBuffer[triangleIndices.z];

    TubeTriangleLinePointData linePointData0 =  tubeTriangleLinePointDataBuffer[vertexData0.vertexLinePointIndex];
    TubeTriangleLinePointData linePointData1 =  tubeTriangleLinePointDataBuffer[vertexData1.vertexLinePointIndex];
    TubeTriangleLinePointData linePointData2 =  tubeTriangleLinePointDataBuffer[vertexData2.vertexLinePointIndex];

    vec3 fragmentWorldPos = interpolateVec3(
            vertexData0.vertexPosition, vertexData1.vertexPosition, vertexData2.vertexPosition, barycentricCoordinates);
    vec3 fragmentNormal = interpolateVec3(
            vertexData0.vertexNormal, vertexData1.vertexNormal, vertexData2.vertexNormal, barycentricCoordinates);
    fragmentNormal = normalize(fragmentNormal);
    vec3 fragmentTangent = interpolateVec3(
            linePointData0.lineTangent, linePointData1.lineTangent, linePointData2.lineTangent, barycentricCoordinates);
    fragmentTangent = normalize(fragmentTangent);
    float fragmentAttribute = interpolateFloat(
            linePointData0.lineAttribute, linePointData1.lineAttribute, linePointData2.lineAttribute, barycentricCoordinates);

    // TODO: Stress lines.

    //hitColor = vec4(vec3(1.0) - backgroundColor.xyz, 1.0);
    float val = clamp(fragmentAttribute, 0.0, 1.0);
    hitColor = vec4(val, 0.0, 0.0, 1.0);
#ifdef USE_TRANSPARENCY
    hitT = length(fragmentWorldPos - cameraPosition);
    hasHit = true;
#endif
}
