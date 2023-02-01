/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#define COMPUTE_SHADER

#include "RayTracingUtilities.glsl"
#include "BarycentricInterpolation.glsl"

layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 1, rgba32f) uniform image2D outputImage;

#ifdef WRITE_NORMAL_MAP
layout(binding = 2, rgba32f) uniform image2D normalMap;
#endif

#ifdef WRITE_DEPTH_MAP
layout(binding = 3, r32f) uniform image2D depthMap;
#endif

#ifdef WRITE_POSITION_MAP
layout(binding = 4, rgba32f) uniform image2D positionMap;
#endif

#ifdef WRITE_FLOW_MAP
layout(binding = 5, rg32f) uniform image2D flowMap;
#endif

layout(binding = 6) uniform accelerationStructureEXT topLevelAS;

layout(binding = 7) uniform UniformsBuffer {
    mat4 viewMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
    mat4 inverseTransposedViewMatrix;
    mat4 lastFrameViewProjectionMatrix;

    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;
    // The number of samples accumulated in one rendering pass.
    uint numSamplesPerFrame;
    // Should the distance of the AO hits be used?
    uint useDistance;

    // Either equivalent to frameNumber or a global frame ID not reset together with accumulation.
    uint globalFrameNumber;

    // What is the radius to take into account for ambient occlusion?
    float ambientOcclusionRadius;

    // A factor which should be used for offsetting secondary rays.
    float subdivisionCorrectionFactor;

    float padding1, padding2;
};

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to TubeLinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};

layout(scalar, binding = 8) readonly buffer TubeIndexBuffer {
    uvec3 indexBuffer[];
};

layout(std430, binding = 9) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

#ifdef USE_INSTANCE_TRIANGLE_INDEX_OFFSET
layout(std430, binding = 10) readonly buffer InstanceTriangleIndexOffsetBuffer {
    uint instanceTriangleIndexOffsets[];
};
#endif

#ifndef GENERAL_TRIANGLE_MESH
struct LinePointData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    float lineRotation;
    vec3 lineNormal;
    uint lineStartIndex;
};

layout(std430, binding = 11) readonly buffer LinePointDataBuffer {
    LinePointData linePoints[];
};
#endif


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
    ivec2 outputImageSize = imageSize(outputImage);
    if (gl_GlobalInvocationID.x >= outputImageSize.x || gl_GlobalInvocationID.y >= outputImageSize.y) {
        return;
    }

    // 1. Trace primary ray to tube and get the closest hit.
    vec3 cameraPosition = (inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec3 rayOrigin = cameraPosition;

    uint seed = tea(gl_GlobalInvocationID.x + gl_GlobalInvocationID.y * outputImageSize.x, globalFrameNumber);

    vec2 xi = vec2(rnd(seed), rnd(seed));
    vec2 fragNdc = 2.0 * ((vec2(gl_GlobalInvocationID.xy) + xi) / vec2(outputImageSize)) - 1.0;

    vec3 rayTarget = (inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;

    rayQueryEXT rayQueryPrimary;
    rayQueryInitializeEXT(
            rayQueryPrimary, topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF,
            rayOrigin, 0.0001, rayDirection, 1000.0);
    while(rayQueryProceedEXT(rayQueryPrimary)) {}

#ifdef WRITE_FLOW_MAP
#endif

    vec3 surfaceNormal = vec3(0.0);
    vec3 vertexPositionWorld = vec3(0.0);
    float aoFactor = 1.0;
    bool hasHitSurface =
            rayQueryGetIntersectionTypeEXT(rayQueryPrimary, true) != gl_RayQueryCommittedIntersectionNoneEXT;
    if (hasHitSurface) {
        // 2. Get the surface normal of the hit tube.
        int primitiveId = rayQueryGetIntersectionPrimitiveIndexEXT(rayQueryPrimary, true);
#ifdef USE_INSTANCE_TRIANGLE_INDEX_OFFSET
        // rayQueryGetIntersectionInstanceCustomIndexEXT and rayQueryGetIntersectionInstanceIdEXT should be the same,
        // as hull instances are always specified last.
        int instanceId = rayQueryGetIntersectionInstanceIdEXT(rayQueryPrimary, true);
        uint instanceTriangleIndexOffset = instanceTriangleIndexOffsets[instanceId];
        uvec3 triangleIndices = indexBuffer[instanceTriangleIndexOffset + primitiveId];
#else
        uvec3 triangleIndices = indexBuffer[primitiveId];
#endif
        vec2 attribs = rayQueryGetIntersectionBarycentricsEXT(rayQueryPrimary, true);
        const vec3 barycentricCoordinates = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

        TubeTriangleVertexData vertexData0 = tubeTriangleVertexDataBuffer[triangleIndices.x];
        TubeTriangleVertexData vertexData1 = tubeTriangleVertexDataBuffer[triangleIndices.y];
        TubeTriangleVertexData vertexData2 = tubeTriangleVertexDataBuffer[triangleIndices.z];

#ifndef GENERAL_TRIANGLE_MESH
        //#ifdef USE_CAPPED_TUBES
        uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex & 0x7FFFFFFFu;
        uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex & 0x7FFFFFFFu;
        uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex & 0x7FFFFFFFu;
        //#else
        //    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex;
        //    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex;
        //    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex;
        //#endif
        LinePointData linePointData0 = linePoints[vertexLinePointIndex0];
        LinePointData linePointData1 = linePoints[vertexLinePointIndex1];
        LinePointData linePointData2 = linePoints[vertexLinePointIndex2];
#endif

        vertexPositionWorld = interpolateVec3(
                vertexData0.vertexPosition, vertexData1.vertexPosition, vertexData2.vertexPosition, barycentricCoordinates);
        surfaceNormal = interpolateVec3(
                vertexData0.vertexNormal, vertexData1.vertexNormal, vertexData2.vertexNormal, barycentricCoordinates);
        surfaceNormal = normalize(surfaceNormal);

#ifndef GENERAL_TRIANGLE_MESH
        vec3 linePosition = interpolateVec3(
                linePointData0.linePosition, linePointData1.linePosition, linePointData2.linePosition, barycentricCoordinates);
        vec3 surfaceTangent = interpolateVec3(
                linePointData0.lineTangent, linePointData1.lineTangent, linePointData2.lineTangent, barycentricCoordinates);
        surfaceTangent = normalize(surfaceTangent);
        vec3 surfaceBitangent = cross(surfaceNormal, surfaceTangent);
#else
        vec3 v0 = vertexData0.vertexPosition - vertexData1.vertexPosition;
        vec3 v1 = vertexData0.vertexPosition - vertexData2.vertexPosition;
        vec3 surfaceNormalFlat = normalize(cross(v0, v1));
        if (dot(surfaceNormalFlat, rayDirection) > 0) {
            surfaceNormal = -surfaceNormal;
        }
        vec3 surfaceTangent;
        vec3 surfaceBitangent;
        ComputeDefaultBasis(surfaceNormal, surfaceTangent, surfaceBitangent);
#endif

        mat3 frame = mat3(surfaceTangent, surfaceBitangent, surfaceNormal);

#ifndef GENERAL_TRIANGLE_MESH
        const float offsetFactor = length(linePosition - vertexPositionWorld) / subdivisionCorrectionFactor;
#else
        const float offsetFactor = 1e-4;
#endif


        // 3. Trace the requested number of samples in the hemisphere around the hit point.
        aoFactor = 0.0;
        for (int sampleIdx = 0; sampleIdx < numSamplesPerFrame; sampleIdx++) {
            uint seed = tea(
                    gl_GlobalInvocationID.x + gl_GlobalInvocationID.y * outputImageSize.x,
                    globalFrameNumber * numSamplesPerFrame + sampleIdx);
            vec2 xi = vec2(rnd(seed), rnd(seed));

            vec3 rayDirection = normalize(frame * sampleHemisphere(xi));

            rayQueryEXT rayQuery;
            //const float offsetFactor = length(cameraPosition - vertexPositionWorld) * 1e-3;
#ifndef GENERAL_TRIANGLE_MESH
            float occlusionFactor = traceAoRay(rayQuery, vertexPositionWorld + rayDirection * offsetFactor, rayDirection);
#else
            float occlusionFactor = traceAoRay(rayQuery, vertexPositionWorld + surfaceNormal * offsetFactor, rayDirection);
#endif
            aoFactor += occlusionFactor;
        }

        aoFactor /= float(numSamplesPerFrame);
    }


    // 4. Write the AO factor to the output image.
    ivec2 writePos = ivec2(gl_GlobalInvocationID.xy);
    //writePos.y = outputImageSize.y - writePos.y - 1;
#ifndef DISABLE_ACCUMULATION
    if (frameNumber != 0) {
        float aoFactorPrev = imageLoad(outputImage, writePos).x;
        aoFactor = mix(aoFactorPrev, aoFactor, 1.0 / float(frameNumber + 1));
    }
#endif
    imageStore(outputImage, writePos, vec4(aoFactor, aoFactor, aoFactor, 1.0));

#ifdef WRITE_NORMAL_MAP
    // Convert to camera space. Necessary according to:
    // https://raytracing-docs.nvidia.com/optix7/guide/index.html#ai_denoiser#structure-and-use-of-image-buffers
    vec3 camNormal = (inverseTransposedViewMatrix * vec4(surfaceNormal, 0.0)).xyz;
#ifndef DISABLE_ACCUMULATION
    if (frameNumber != 0) {
        vec3 normalOld = imageLoad(normalMap, writePos).xyz;
        camNormal = mix(normalOld, camNormal, 1.0 / float(frameNumber + 1));
        camNormal = normalize(camNormal);
    }
#endif
    imageStore(normalMap, writePos, vec4(camNormal, 0.0));
#endif

#if defined(WRITE_DEPTH_MAP) || defined(WRITE_POSITION_MAP)
    vec3 positionViewSpace = (viewMatrix * vec4(vertexPositionWorld, 1.0)).xyz;
#endif

#ifdef WRITE_DEPTH_MAP
    float depth = -positionViewSpace.z;
#ifndef DISABLE_ACCUMULATION
    if (frameNumber != 0) {
        float depthOld = imageLoad(depthMap, writePos).x;
        depth = mix(depthOld, depth, 1.0 / float(frameNumber + 1));
    }
#endif
    imageStore(depthMap, writePos, vec4(depth));
#endif

#ifdef WRITE_POSITION_MAP
#ifndef DISABLE_ACCUMULATION
    if (frameNumber != 0) {
        vec3 positionViewSpaceOld = imageLoad(positionMap, writePos).xyz;
        positionViewSpace = mix(positionViewSpaceOld, positionViewSpace, 1.0 / float(frameNumber + 1));
    }
#endif
    imageStore(positionMap, writePos, vec4(positionViewSpace, 1.0));
#endif

#ifdef WRITE_FLOW_MAP
    vec2 flowVector = vec2(0.0);
    if (hasHitSurface) {
        vec4 lastFramePositionNdc = lastFrameViewProjectionMatrix * vec4(vertexPositionWorld, 1.0);
        lastFramePositionNdc.xyz /= lastFramePositionNdc.w;
        vec2 pixelPositionLastFrame = (0.5 * lastFramePositionNdc.xy + vec2(0.5)) * vec2(outputImageSize);
        flowVector = vec2(writePos) - pixelPositionLastFrame;
    }
#ifndef DISABLE_ACCUMULATION
    if (frameNumber != 0) {
        vec2 flowVectorOld = imageLoad(flowMap, writePos).xy;
        flowVector = mix(flowVectorOld, flowVector, 1.0 / float(frameNumber + 1));
    }
#endif
    imageStore(flowMap, writePos, vec4(flowVector, 0.0, 0.0));
#endif
}
