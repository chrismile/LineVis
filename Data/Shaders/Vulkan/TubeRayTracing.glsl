-- RayGen

#version 460
#extension GL_EXT_ray_tracing : require

#ifdef USE_JITTERED_RAYS
#include "RayTracingUtilities.glsl"
#endif

layout (binding = 0) uniform CameraSettingsBuffer {
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
} camera;

layout(binding = 1, rgba8) uniform image2D outputImage;

layout(binding = 2) uniform accelerationStructureEXT topLevelAS;

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

layout (binding = 9) uniform HullRenderSettingsBuffer {
    vec4 color;
    ivec3 padding;
    int useShading;
} hullRenderSettings;

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadEXT RayPayload payload;

// Minimum distance between two consecutive hits.
const float HIT_DISTANCE_EPSILON = 1e-4;

#define USE_TRANSPARENCY

vec4 traceRayOpaque(vec3 rayOrigin, vec3 rayDirection) {
    float tMin = 0.0001f;
    float tMax = 1000.0f;

    traceRayEXT(topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0);

    return payload.hitColor;
}

vec4 traceRayTransparent(vec3 rayOrigin, vec3 rayDirection) {
    vec4 fragmentColor = vec4(0.0);

    float tMin = 0.0001f;
    float tMax = 1000.0f;

    for (uint hitIdx = 0; hitIdx < maxDepthComplexity; hitIdx++) {
        traceRayEXT(topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0); // gl_RayFlagsNoneEXT

        tMin = payload.hitT + HIT_DISTANCE_EPSILON;

        // Front-to-back blending (hitColor uses post-multiplied, fragmentColor uses pre-multiplied alpha).
        fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * payload.hitColor.a * payload.hitColor.rgb;
        fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * payload.hitColor.a;

        if (!payload.hasHit || fragmentColor.a > 0.99) {
            break;
        }
    }

    return fragmentColor;
}

void main() {
    ivec2 outputImageSize = imageSize(outputImage);
    vec4 fragmentColor = vec4(0.0);

    vec3 rayOrigin = (camera.inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;

#ifdef USE_JITTERED_RAYS
    for (int sampleIdx = 0; sampleIdx < numSamplesPerFrame; sampleIdx++) {

    uint seed = tea(
            gl_LaunchIDEXT.x + gl_LaunchIDEXT.y * outputImageSize.x,
            frameNumber * numSamplesPerFrame + sampleIdx);
    vec2 xi = vec2(rnd(seed), rnd(seed));
    vec2 fragNdc = 2.0 * ((vec2(gl_LaunchIDEXT.xy) + xi) / vec2(gl_LaunchSizeEXT.xy)) - 1.0;
#else
    vec2 fragNdc = 2.0 * ((vec2(gl_LaunchIDEXT.xy) + vec2(0.5)) / vec2(gl_LaunchSizeEXT.xy)) - 1.0;
#endif

    vec3 rayTarget = (camera.inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (camera.inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;

#ifdef USE_TRANSPARENCY
    fragmentColor += traceRayTransparent(rayOrigin, rayDirection);
#else
    if (hasHullMesh == 1) {
        fragmentColor += traceRayTransparent(rayOrigin, rayDirection);
    } else {
        fragmentColor += traceRayOpaque(rayOrigin, rayDirection);
    }
#endif

#ifdef USE_JITTERED_RAYS
    }

    fragmentColor /= float(numSamplesPerFrame);
#endif

    ivec2 writePos = ivec2(gl_LaunchIDEXT.xy);
    writePos.y = outputImageSize.y - writePos.y - 1;
    if (frameNumber != 0) {
        vec4 fragmentColorPrev = imageLoad(outputImage, writePos);
        fragmentColor = mix(fragmentColorPrev, fragmentColor, 1.0 / float(frameNumber + 1));
    }
    imageStore(outputImage, writePos, fragmentColor);
}


-- Miss

#version 460
#extension GL_EXT_ray_tracing : require

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

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadInEXT RayPayload payload;

void main() {
    payload.hitColor = backgroundColor;
    payload.hitT = 0.0f;
    payload.hasHit = false;
}


-- ClosestHitTubeTriangles

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "BarycentricInterpolation.glsl"
#include "RayHitCommon.glsl"

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to TubeLinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};

layout(scalar, binding = 6) readonly buffer TubeIndexBuffer {
    uvec3 indexBuffer[];
};

layout(std430, binding = 7) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

void main() {
    uvec3 triangleIndices = indexBuffer[gl_PrimitiveID];
    const vec3 barycentricCoordinates = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    TubeTriangleVertexData vertexData0 = tubeTriangleVertexDataBuffer[triangleIndices.x];
    TubeTriangleVertexData vertexData1 = tubeTriangleVertexDataBuffer[triangleIndices.y];
    TubeTriangleVertexData vertexData2 = tubeTriangleVertexDataBuffer[triangleIndices.z];

#ifdef USE_CAPPED_TUBES
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex & 0x7FFFFFFFu;
    TubeLinePointData linePointData0 =  tubeLinePointDataBuffer[vertexLinePointIndex0];
    TubeLinePointData linePointData1 =  tubeLinePointDataBuffer[vertexLinePointIndex1];
    TubeLinePointData linePointData2 =  tubeLinePointDataBuffer[vertexLinePointIndex2];
    bool isCap =
            bitfieldExtract(vertexData0.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData1.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData2.vertexLinePointIndex, 31, 1) > 0u;
#else
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex;
    TubeLinePointData linePointData0 =  tubeLinePointDataBuffer[vertexLinePointIndex0];
    TubeLinePointData linePointData1 =  tubeLinePointDataBuffer[vertexLinePointIndex1];
    TubeLinePointData linePointData2 =  tubeLinePointDataBuffer[vertexLinePointIndex2];
#endif

    vec3 fragmentPositionWorld = interpolateVec3(
            vertexData0.vertexPosition, vertexData1.vertexPosition, vertexData2.vertexPosition, barycentricCoordinates);
    vec3 fragmentNormal = interpolateVec3(
            vertexData0.vertexNormal, vertexData1.vertexNormal, vertexData2.vertexNormal, barycentricCoordinates);
    fragmentNormal = normalize(fragmentNormal);
    vec3 fragmentTangent = interpolateVec3(
            linePointData0.lineTangent, linePointData1.lineTangent, linePointData2.lineTangent, barycentricCoordinates);
    fragmentTangent = normalize(fragmentTangent);
    float fragmentAttribute = interpolateFloat(
            linePointData0.lineAttribute, linePointData1.lineAttribute, linePointData2.lineAttribute,
            barycentricCoordinates);

    computeFragmentColor(
            fragmentPositionWorld, fragmentNormal, fragmentTangent, fragmentAttribute, linePointData0, linePointData1);

}


-- ClosestHitHull

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "BarycentricInterpolation.glsl"

struct HullVertex {
    vec3 vertexPosition;
    float padding0;
    vec3 vertexNormal;
    float padding1;
};

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

layout (binding = 9) uniform HullRenderSettingsBuffer {
    vec4 color;
    ivec3 padding;
    int useShading;
};

layout(scalar, binding = 10) readonly buffer HullIndexBuffer {
    uvec3 hullIndexBuffer[];
};

layout(std430, binding = 11) readonly buffer HullTriangleVertexDataBuffer {
    HullVertex hullVertices[];
};

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadInEXT RayPayload payload;

#define RAYTRACING
#ifdef USE_DEPTH_CUES
#undef USE_DEPTH_CUES
#endif
#ifdef USE_AMBIENT_OCCLUSION
#undef USE_AMBIENT_OCCLUSION
#endif
#include "Lighting.glsl"

void main() {
    uvec3 triangleIndices = hullIndexBuffer[gl_PrimitiveID];
    const vec3 barycentricCoordinates = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    HullVertex vertex0 = hullVertices[triangleIndices.x];
    HullVertex vertex1 = hullVertices[triangleIndices.y];
    HullVertex vertex2 = hullVertices[triangleIndices.z];

    vec3 fragmentPositionWorld = interpolateVec3(
            vertex0.vertexPosition, vertex1.vertexPosition, vertex2.vertexPosition, barycentricCoordinates);
    vec3 fragmentNormal = interpolateVec3(
            vertex0.vertexNormal, vertex1.vertexNormal, vertex2.vertexNormal, barycentricCoordinates);
    fragmentNormal = normalize(fragmentNormal);

    vec4 phongColor;
    if (useShading == 1) {
        phongColor = blinnPhongShading(color, fragmentPositionWorld, fragmentNormal);
    } else {
        phongColor = color;
    }

    vec3 viewDir = normalize(cameraPosition - fragmentPositionWorld);
    float cosAngle = dot(fragmentNormal, viewDir);
    if (cosAngle < 0.0f) {
        cosAngle *= -1.0f;
    }
    phongColor.a *= 1.0f - cosAngle;

    payload.hitColor = phongColor;
    payload.hitT = length(fragmentPositionWorld - cameraPosition);
    payload.hasHit = true;
}


-- IntersectionTube

#version 460
#extension GL_EXT_ray_tracing : require

#include "RayIntersectionTestsVulkan.glsl"

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

layout(std430, binding = 17) readonly buffer BoundingBoxLinePointIndexBuffer {
    uvec2 boundingBoxLinePointIndices[];
};

layout(std430, binding = 8) readonly buffer TubeLinePointDataBuffer {
    TubeLinePointData tubeLinePointDataBuffer[];
};

void main() {
    uvec2 linePointIndices = boundingBoxLinePointIndices[gl_PrimitiveID];
    vec3 lineSegmentPoint0 = tubeLinePointDataBuffer[linePointIndices.x].linePosition;
    vec3 lineSegmentPoint1 = tubeLinePointDataBuffer[linePointIndices.y].linePosition;

    bool hasIntersection = false;
    float hitT = 1e7;
    int hitKind = 0;

    bool hasTubeIntersection, hasSphereIntersection0 = false, hasSphereIntersection1 = false;
    float tubeT, sphere0T, sphere1T;
    hasTubeIntersection = rayTubeIntersection(
            rayOrigin, rayDirection, lineSegmentPoint0, lineSegmentPoint1, lineRadius, tubeT);
    if (hasTubeIntersection) {
        hitT = tubeT;
        hasIntersection = true;
        hitKind = 0;
    } else {
        hasSphereIntersection0 = raySphereIntersection(
                rayOrigin, rayDirection, lineSegmentPoint0, lineRadius, sphere0T);
        hasSphereIntersection1 = raySphereIntersection(
                rayOrigin, rayDirection, lineSegmentPoint1, lineRadius, sphere1T);

        if (hasSphereIntersection0 && sphere0T < hitT) {
            hasIntersection = true;
            hitT = sphere0T;
            hitKind = 1;
        }
        if (hasSphereIntersection1 && sphere1T < hitT) {
            hasIntersection = true;
            hitT = sphere1T;
            hitKind = 2;
        }
    }

    if (hasIntersection) {
        reportIntersectionEXT(hitT, hitKind);
    }
}


-- ClosestHitTubeAnalytic

#version 460
#extension GL_EXT_ray_tracing : require

#include "RayHitCommon.glsl"
#include "RayIntersectionTestsVulkan.glsl"

layout(std430, binding = 17) readonly buffer BoundingBoxLinePointIndexBuffer {
    uvec2 boundingBoxLinePointIndices[];
};

void main() {
    uvec2 linePointIndices = boundingBoxLinePointIndices[gl_PrimitiveID];
    TubeLinePointData linePointData0 = tubeLinePointDataBuffer[linePointIndices.x];
    TubeLinePointData linePointData1 = tubeLinePointDataBuffer[linePointIndices.y];

    vec3 fragmentPositionWorld = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;

    vec3 linePointInterpolated;
    float fragmentAttribute;

    vec3 v = linePointData1.linePosition - linePointData0.linePosition;
    if (gl_HitKindEXT == 0) {
        // Tube
        vec3 u = fragmentPositionWorld - linePointData0.linePosition;
        float t = dot(v, u) / dot(v, v);
        linePointInterpolated = tubePoint1 + t * v;
        fragmentAttribute = (1.0 - t) * linePointData0.lineAttribute + t * linePointData1.lineAttribute;
    } else {
        if (gl_HitKindEXT == 1) {
            linePointInterpolated = linePointData0.linePosition;
            fragmentAttribute = linePointData0.lineAttribute;
        } else {
            linePointInterpolated = linePointData1.linePosition;
            fragmentAttribute = linePointData1.lineAttribute;
        }
    }
    vec3 fragmentTangent = normalize(v);
    vec3 fragmentNormal = normalize(fragmentPositionWorld - linePointInterpolated);

    computeFragmentColor(
            fragmentPositionWorld, fragmentNormal, fragmentTangent, fragmentAttribute, linePointData0, linePointData1);
}
