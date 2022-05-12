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

-- RayGen

#version 460
#extension GL_EXT_ray_tracing : require

#define RAY_GEN_SHADER

#ifdef USE_JITTERED_RAYS
#include "RayTracingUtilities.glsl"
#endif

#include "TubeRayTracingHeader.glsl"

layout(binding = 1, rgba8) uniform image2D outputImage;

layout(binding = 2) uniform accelerationStructureEXT topLevelAS;

// Minimum distance between two consecutive hits.
const float HIT_DISTANCE_EPSILON = 1e-5;

#define USE_TRANSPARENCY

#ifndef USE_MLAT
vec4 traceRayOpaque(vec3 rayOrigin, vec3 rayDirection) {
    float tMin = 0.0001;
    float tMax = 1000.0;

    traceRayEXT(topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0);

    return payload.hitColor;
}

vec4 traceRayTransparent(vec3 rayOrigin, vec3 rayDirection) {
    vec4 fragmentColor = vec4(0.0);

    float tMin = 0.0001;
    float tMax = 1000.0;

    for (uint hitIdx = 0; hitIdx < maxDepthComplexity; hitIdx++) {
        traceRayEXT(topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0);

        tMin = payload.hitT + max(payload.hitT * HIT_DISTANCE_EPSILON, 1e-7);

        // Front-to-back blending (hitColor uses post-multiplied, fragmentColor uses pre-multiplied alpha).
        fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * payload.hitColor.a * payload.hitColor.rgb;
        fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * payload.hitColor.a;

        if (!payload.hasHit || fragmentColor.a > 0.99) {
            break;
        }
    }

    return fragmentColor;
}

#else

vec4 traceRayMlat(vec3 rayOrigin, vec3 rayDirection) {
    vec4 fragmentColor = vec4(0.0);

    // Clear the payload data.
#ifdef NODES_UNROLLED
    payload.node0.color = vec4(0.0);
    payload.node0.transmittance = 1.0;
    payload.node0.depth = 0.0;
#if NUM_NODES >= 2
    payload.node1.color = vec4(0.0);
    payload.node1.transmittance = 1.0;
    payload.node1.depth = 0.0;
#endif
#if NUM_NODES >= 3
    payload.node2.color = vec4(0.0);
    payload.node2.transmittance = 1.0;
    payload.node2.depth = 0.0;
#endif
#if NUM_NODES >= 4
    payload.node3.color = vec4(0.0);
    payload.node3.transmittance = 1.0;
    payload.node3.depth = 0.0;
#endif
#if NUM_NODES >= 5
    payload.node4.color = vec4(0.0);
    payload.node4.transmittance = 1.0;
    payload.node4.depth = 0.0;
#endif
#if NUM_NODES >= 6
    payload.node5.color = vec4(0.0);
    payload.node5.transmittance = 1.0;
    payload.node5.depth = 0.0;
#endif
#if NUM_NODES >= 7
    payload.node6.color = vec4(0.0);
    payload.node6.transmittance = 1.0;
    payload.node6.depth = 0.0;
#endif
#if NUM_NODES >= 8
    payload.node7.color = vec4(0.0);
    payload.node7.transmittance = 1.0;
    payload.node7.depth = 0.0;
#endif
#else
    [[unroll]] for (int i = 0; i < NUM_NODES; i++) {
        payload.nodes[i].color = vec4(0.0);
        payload.nodes[i].transmittance = 1.0;
        payload.nodes[i].depth = 0.0;
    }
#endif
    payload.depth2 = 0.0;

    float tMin = 0.0001;
    float tMax = 1000.0;
    traceRayEXT(topLevelAS, gl_RayFlagsNoOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0);

    // Front-to-back blending (hitColor and fragmentColor use pre-multiplied alpha).
    vec4 hitColor;
#ifdef NODES_UNROLLED
    hitColor = payload.node0.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#if NUM_NODES >= 2
    hitColor = payload.node1.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#endif
#if NUM_NODES >= 3
    hitColor = payload.node2.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#endif
#if NUM_NODES >= 4
    hitColor = payload.node3.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#endif
#if NUM_NODES >= 5
    hitColor = payload.node4.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#endif
#if NUM_NODES >= 6
    hitColor = payload.node5.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#endif
#if NUM_NODES >= 7
    hitColor = payload.node6.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#endif
#if NUM_NODES >= 8
    hitColor = payload.node7.color;
    fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
    fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
#endif
#else
    for (int i = 0; i < NUM_NODES; ++i) {
        hitColor = payload.nodes[i].color;
        fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * hitColor.rgb;
        fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * hitColor.a;
    }
#endif

    return fragmentColor;
}
#endif

void main() {
    ivec2 outputImageSize = imageSize(outputImage);
    vec4 fragmentColor = vec4(0.0);

    vec3 rayOrigin = (inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;

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

    vec3 rayTarget = (inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;

#if defined(USE_MLAT)
    fragmentColor += traceRayMlat(rayOrigin, rayDirection);
#elif defined(USE_TRANSPARENCY)
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
    if (frameNumber != 0) {
        vec4 fragmentColorPrev = imageLoad(outputImage, writePos);
        fragmentColor = mix(fragmentColorPrev, fragmentColor, 1.0 / float(frameNumber + 1));
    }
    imageStore(outputImage, writePos, fragmentColor);
}


-- Miss

#version 460
#extension GL_EXT_ray_tracing : require

#define MISS_SHADER

#include "TubeRayTracingHeader.glsl"

#ifdef USE_MLAT
#include "MlatInsert.glsl"
#endif

void main() {
#ifdef USE_MLAT
    insertNodeMlat(backgroundColor);
#else
    payload.hitColor = backgroundColor;
    payload.hitT = 0.0;
    payload.hasHit = false;
#endif
}


-- ClosestHitTubeTriangles

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "BarycentricInterpolation.glsl"
#include "RayHitCommon.glsl"

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to LinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};

layout(scalar, binding = 3) readonly buffer TubeIndexBuffer {
    uvec3 indexBuffer[];
};

layout(std430, binding = 4) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

#ifdef USE_INSTANCE_TRIANGLE_INDEX_OFFSET
layout(std430, binding = INSTANCE_INDEX_OFFSET_BUFFER_BINDING) readonly buffer InstanceTriangleIndexOffsetBuffer {
    uint instanceTriangleIndexOffsets[];
};
#endif

void main() {
#ifdef USE_INSTANCE_TRIANGLE_INDEX_OFFSET
    // gl_InstanceID and gl_InstanceCustomIndexEXT should be the same, as hull instances are always specified last.
    uint instanceTriangleIndexOffset = instanceTriangleIndexOffsets[gl_InstanceID];
    uvec3 triangleIndices = indexBuffer[0 + gl_PrimitiveID];
#else
    uvec3 triangleIndices = indexBuffer[gl_PrimitiveID];
#endif

    const vec3 barycentricCoordinates = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    TubeTriangleVertexData vertexData0 = tubeTriangleVertexDataBuffer[triangleIndices.x];
    TubeTriangleVertexData vertexData1 = tubeTriangleVertexDataBuffer[triangleIndices.y];
    TubeTriangleVertexData vertexData2 = tubeTriangleVertexDataBuffer[triangleIndices.z];

#ifdef USE_CAPPED_TUBES
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex & 0x7FFFFFFFu;
    LinePointData linePointData0 = linePoints[vertexLinePointIndex0];
    LinePointData linePointData1 = linePoints[vertexLinePointIndex1];
    LinePointData linePointData2 = linePoints[vertexLinePointIndex2];
    bool isCap =
            bitfieldExtract(vertexData0.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData1.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData2.vertexLinePointIndex, 31, 1) > 0u;
#else
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex;
    LinePointData linePointData0 = linePoints[vertexLinePointIndex0];
    LinePointData linePointData1 = linePoints[vertexLinePointIndex1];
    LinePointData linePointData2 = linePoints[vertexLinePointIndex2];
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

#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    float phi = interpolateAngle(
            vertexData0.phi, vertexData1.phi, vertexData2.phi, barycentricCoordinates);
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING)
    float fragmentVertexId = interpolateFloat(
            float(vertexLinePointIndex0), float(vertexLinePointIndex1), float(vertexLinePointIndex2),
            barycentricCoordinates);
#endif

#ifdef USE_BANDS
    vec3 linePosition = interpolateVec3(
            linePointData0.linePosition, linePointData1.linePosition, linePointData2.linePosition, barycentricCoordinates);
    vec3 lineNormal = interpolateVec3(
            linePointData0.lineNormal, linePointData1.lineNormal, linePointData2.lineNormal, barycentricCoordinates);
#endif

#ifdef USE_ROTATING_HELICITY_BANDS
    float fragmentRotation = interpolateFloat(
            linePointData0.lineRotation, linePointData1.lineRotation, linePointData2.lineRotation,
            barycentricCoordinates);
#endif

#ifdef STRESS_LINE_DATA
    StressLinePointData stressLinePointData0 = stressLinePoints[vertexLinePointIndex0];
    uint principalStressIndex = stressLinePointData0.linePrincipalStressIndex;
    float lineAppearanceOrder = stressLinePointData0.lineLineAppearanceOrder;
#ifdef USE_PRINCIPAL_STRESSES
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData0 = principalStressLinePoints[vertexLinePointIndex0];
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData1 = principalStressLinePoints[vertexLinePointIndex1];
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData2 = principalStressLinePoints[vertexLinePointIndex2];
    float fragmentMajorStress = interpolateFloat(
            stressLinePointPrincipalStressData0.lineMajorStress,
            stressLinePointPrincipalStressData1.lineMajorStress,
            stressLinePointPrincipalStressData2.lineMajorStress,
            barycentricCoordinates);
    float fragmentMediumStress = interpolateFloat(
            stressLinePointPrincipalStressData0.lineMediumStress,
            stressLinePointPrincipalStressData1.lineMediumStress,
            stressLinePointPrincipalStressData2.lineMediumStress,
            barycentricCoordinates);
    float fragmentMinorStress = interpolateFloat(
            stressLinePointPrincipalStressData0.lineMinorStress,
            stressLinePointPrincipalStressData1.lineMinorStress,
            stressLinePointPrincipalStressData2.lineMinorStress,
            barycentricCoordinates);
#endif
#endif

    computeFragmentColor(
            fragmentPositionWorld, fragmentNormal, fragmentTangent,
#ifdef USE_CAPPED_TUBES
            isCap,
#endif
#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
            phi,
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING)
            fragmentVertexId,
#endif
#ifdef USE_BANDS
            linePosition, lineNormal,
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


-- AnyHitTubeTriangles

#import ".ClosestHitTubeTriangles"


-- ClosestHitHull

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "BarycentricInterpolation.glsl"
#include "TubeRayTracingHeader.glsl"

#ifdef USE_MLAT
#include "MlatInsert.glsl"
#endif

struct HullVertex {
    vec3 vertexPosition;
    float padding0;
    vec3 vertexNormal;
    float padding1;
};

layout(scalar, binding = 6) readonly buffer HullIndexBuffer {
    uvec3 hullIndexBuffer[];
};

layout(std430, binding = 7) readonly buffer HullTriangleVertexDataBuffer {
    HullVertex hullVertices[];
};

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
    if (hullUseShading == 1) {
        phongColor = blinnPhongShading(hullColor, fragmentPositionWorld, fragmentNormal);
    } else {
        phongColor = hullColor;
    }

    vec3 viewDir = normalize(cameraPosition - fragmentPositionWorld);
    float cosAngle = dot(fragmentNormal, viewDir);
    if (cosAngle < 0.0) {
        cosAngle *= -1.0;
    }
    phongColor.a *= 1.0 - cosAngle;

#ifdef USE_MLAT
    insertNodeMlat(phongColor);
#else
    payload.hitColor = phongColor;
    payload.hitT = length(fragmentPositionWorld - cameraPosition);
    payload.hasHit = true;
#endif
}


-- AnyHitHull

#import ".ClosestHitHull"


-- IntersectionTube

#version 460
#extension GL_EXT_ray_tracing : require

#include "LineUniformData.glsl"
#include "LineDataSSBO.glsl"
#include "RayIntersectionTestsVulkan.glsl"

layout(std430, binding = 3) readonly buffer BoundingBoxLinePointIndexBuffer {
    uvec2 boundingBoxLinePointIndices[];
};

void main() {
    const float lineRadius = lineWidth / 2.0;

    uvec2 linePointIndices = boundingBoxLinePointIndices[gl_PrimitiveID];
    vec3 lineSegmentPoint0 = linePoints[linePointIndices.x].linePosition;
    vec3 lineSegmentPoint1 = linePoints[linePointIndices.y].linePosition;

    bool hasIntersection = false;
    float hitT = 1e7;
    int hitKind = 0;

    bool hasTubeIntersection, hasSphereIntersection0 = false, hasSphereIntersection1 = false;
    float tubeT, sphere0T, sphere1T;
    hasTubeIntersection = rayTubeIntersection(
            gl_WorldRayOriginEXT, gl_WorldRayDirectionEXT, lineSegmentPoint0, lineSegmentPoint1, lineRadius, tubeT);
    if (hasTubeIntersection) {
        hitT = tubeT;
        hasIntersection = true;
        hitKind = 0;
    }

#ifdef USE_CAPPED_TUBES
    hasSphereIntersection0 = raySphereIntersection(
            gl_WorldRayOriginEXT, gl_WorldRayDirectionEXT, lineSegmentPoint0, lineRadius, sphere0T);
    hasSphereIntersection1 = raySphereIntersection(
            gl_WorldRayOriginEXT, gl_WorldRayDirectionEXT, lineSegmentPoint1, lineRadius, sphere1T);

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
#endif

    if (hasIntersection) {
        reportIntersectionEXT(hitT, hitKind);
    }
}


-- ClosestHitTubeAnalytic

#version 460
#extension GL_EXT_ray_tracing : require

#define ANALYTIC_TUBE_INTERSECTIONS

#include "RayHitCommon.glsl"
#include "RayIntersectionTestsVulkan.glsl"

layout(std430, binding = 3) readonly buffer BoundingBoxLinePointIndexBuffer {
    uvec2 boundingBoxLinePointIndices[];
};

void main() {
    uvec2 linePointIndices = boundingBoxLinePointIndices[gl_PrimitiveID];
    LinePointData linePointData0 = linePoints[linePointIndices.x];
    LinePointData linePointData1 = linePoints[linePointIndices.y];

    vec3 fragmentPositionWorld = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;

    vec3 linePointInterpolated;
    float fragmentAttribute;

    float t;
    vec3 v = linePointData1.linePosition - linePointData0.linePosition;
    if (gl_HitKindEXT == 0) {
        // Tube
        vec3 u = fragmentPositionWorld - linePointData0.linePosition;
        t = dot(v, u) / dot(v, v);
        linePointInterpolated = linePointData0.linePosition + t * v;
        fragmentAttribute = (1.0 - t) * linePointData0.lineAttribute + t * linePointData1.lineAttribute;
    }
#ifdef USE_CAPPED_TUBES
    else {
        if (gl_HitKindEXT == 1) {
            linePointInterpolated = linePointData0.linePosition;
            fragmentAttribute = linePointData0.lineAttribute;
            t = 0.0;
        } else {
            linePointInterpolated = linePointData1.linePosition;
            fragmentAttribute = linePointData1.lineAttribute;
            t = 1.0;
        }
    }
#endif
    vec3 fragmentTangent = normalize(v);
    vec3 fragmentNormal = normalize(fragmentPositionWorld - linePointInterpolated);

#ifdef USE_CAPPED_TUBES
    bool isCap = gl_HitKindEXT != 0;
#endif

#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    vec3 lineNormal = (1.0 - t) * linePointData0.lineNormal + t * linePointData1.lineNormal;

    // Compute the angle between the fragment and line normal to get phi.
    float phi = acos(dot(fragmentNormal, lineNormal));
    float val = dot(lineNormal, cross(fragmentNormal, fragmentTangent));
    if (val < 0.0) {
        phi = 2.0 * float(M_PI) - phi;
    }
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING)
    float fragmentVertexId = (1.0 - t) * linePointIndices.x + t * linePointIndices.y;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
    float fragmentRotation = (1.0 - t) * linePointData0.lineRotation + t * linePointData1.lineRotation;
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
            isCap,
#endif
#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
            phi,
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


-- AnyHitTubeAnalytic

#import ".ClosestHitTubeAnalytic"

