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

#version 450 core

layout(local_size_x = WORKGROUP_SIZE_X, local_size_y = WORKGROUP_SIZE_Y) in;

#extension GL_EXT_scalar_block_layout : require
//#extension GL_EXT_nonuniform_qualifier : require
//#extension GL_EXT_debug_printf : enable

#include "LineUniformData.glsl"

#define COMPUTE_SHADER
#include "BarycentricInterpolation.glsl"

struct Node {
    vec3 worldSpaceAabbMin;
    uint indexCount;
    vec3 worldSpaceAabbMax;
    uint firstChildOrPrimitiveIndex;
};

layout(std430, binding = 0) readonly buffer NodeBuffer {
    Node nodes[];
};

layout(scalar, binding = 1) readonly buffer TriangleIndexBuffer {
    uvec3 indexBuffer[];
};

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to LinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};
layout(std430, binding = TUBE_TRIANGLE_MESH_VERTEX_BUFFER_BINDING) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

layout(binding = 2, rgba8) uniform image2D outputImage;

#define SQR(x) ((x)*(x))
#define BIAS 1e-5

float squareVec(vec3 v) {
    return SQR(v.x) + SQR(v.y) + SQR(v.z);
}

/**
 * Helper function for rayBoxIntersection (see below).
 */
bool rayBoxPlaneIntersection(float rayOriginX, float rayDirectionX, float lowerX, float upperX) {
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

        float tNear = -1e7, tFar = 1e7; // Infinite values
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
bool testRayBoxIntersection(vec3 rayOrigin, vec3 rayDirection, vec3 lower, vec3 upper) {
    for (int i = 0; i < 3; i++) {
        if (!rayBoxPlaneIntersection(rayOrigin[i], rayDirection[i], lower[i], upper[i])) {
            return false;
        }
    }
    return true;
}



#define kEpsilon 1e-7

// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
bool testRayTriangleIntersection(vec3 orig, vec3 dir, vec3 v0, vec3 v1, vec3 v2, out float t) {
    // compute plane's normal
    vec3 v0v1 = v1 - v0;
    vec3 v0v2 = v2 - v0;
    // no need to normalize
    vec3 N = cross(v0v1, v0v2);  //N
    float area2 = length(N);

    // Step 1: finding P

    // check if ray and plane are parallel.
    float NdotRayDirection = dot(N, dir);
    if (abs(NdotRayDirection) < kEpsilon)  //almost 0
        return false;  //they are parallel so they don't intersect !

    // compute d parameter using equation 2
    float d = -dot(N, v0);

    // compute t (equation 3)
    t = -(dot(N, orig) + d) / NdotRayDirection;

    // check if the triangle is in behind the ray
    if (t < 0) return false;  //the triangle is behind

    // compute the intersection point using equation 1
    vec3 P = orig + t * dir;

    // Step 2: inside-outside test
    vec3 C;  //vector perpendicular to triangle's plane

    // edge 0
    vec3 edge0 = v1 - v0;
    vec3 vp0 = P - v0;
    C = cross(edge0, vp0);
    if (dot(N, C) < 0) return false;  //P is on the right side

    // edge 1
    vec3 edge1 = v2 - v1;
    vec3 vp1 = P - v1;
    C = cross(edge1, vp1);
    if (dot(N, C) < 0) return false;  //P is on the right side

    // edge 2
    vec3 edge2 = v0 - v2;
    vec3 vp2 = P - v2;
    C = cross(edge2, vp2);
    if (dot(N, C) < 0) return false;  //P is on the right side;

    return true;  //this ray hits the triangle
}

vec4 getColorAtRayHit(
        vec3 rayOrigin, vec3 rayDirection, float t, uint primitiveIndex) {
    uvec3 triangleIndices = indexBuffer[primitiveIndex];
    TubeTriangleVertexData vertexData0 = tubeTriangleVertexDataBuffer[triangleIndices.x];
    TubeTriangleVertexData vertexData1 = tubeTriangleVertexDataBuffer[triangleIndices.y];
    TubeTriangleVertexData vertexData2 = tubeTriangleVertexDataBuffer[triangleIndices.z];

    vec3 d20 = vertexData2.vertexPosition - vertexData0.vertexPosition;
    vec3 d21 = vertexData2.vertexPosition - vertexData1.vertexPosition;
    float totalArea = length(cross(d20, d21));
    vec3 fragmentPositionWorld = rayOrigin + t * rayDirection;
    float u = length(cross(d21, fragmentPositionWorld - vertexData1.vertexPosition)) / totalArea;
    float v = length(cross(fragmentPositionWorld - vertexData0.vertexPosition, d20)) / totalArea;

    const vec3 barycentricCoordinates = vec3(u, v, 1.0 - u - v);

    //return vec4(0.0, 1.0, 0.0, 1.0);
    vec3 fragmentNormal = interpolateVec3(
            vertexData0.vertexNormal, vertexData1.vertexNormal, vertexData2.vertexNormal, barycentricCoordinates);
    fragmentNormal = normalize(fragmentNormal);
    return vec4(fragmentNormal * 0.5 + vec3(0.5), 1.0);
}

void main() {
    if (gl_GlobalInvocationID.x >= viewportSize.x || gl_GlobalInvocationID.y >= viewportSize.y) {
        return;
    }

    vec3 rayOrigin = (inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec2 fragNdc = 2.0 * ((vec2(gl_GlobalInvocationID.xy) + vec2(0.5)) / vec2(imageSize(outputImage))) - 1.0;
    vec3 rayTarget = (inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;

    vec4 fragmentColor = backgroundColor;
    float t;
    float closestHitT = 1e9;
    uint closestHitPrimitiveIndex = 0;

    uint nodeIndexStack[MAX_STACK_SIZE];
    uint stackSize = 1;
    nodeIndexStack[0] = 0;
    while (stackSize > 0) {
        stackSize--;
        Node currNode = nodes[nodeIndexStack[stackSize]];

        if (!testRayBoxIntersection(rayOrigin, rayDirection, currNode.worldSpaceAabbMin, currNode.worldSpaceAabbMax)) {
            continue;
        }

        if (currNode.indexCount != 0) {
            uint primitiveIndexBegin = currNode.firstChildOrPrimitiveIndex / 3;
            uint primitiveIndexEnd = primitiveIndexBegin + currNode.indexCount / 3;
            for (uint primitiveIndex = primitiveIndexBegin; primitiveIndex < primitiveIndexEnd; primitiveIndex++) {
                // Check ray triangle intersection
                uvec3 triangleIndices = indexBuffer[primitiveIndex];
                vec3 v0 = tubeTriangleVertexDataBuffer[triangleIndices.x].vertexPosition;
                vec3 v1 = tubeTriangleVertexDataBuffer[triangleIndices.y].vertexPosition;
                vec3 v2 = tubeTriangleVertexDataBuffer[triangleIndices.z].vertexPosition;
                if (testRayTriangleIntersection(rayOrigin, rayDirection, v0, v1, v2, t)) {
                    if (t < closestHitT) {
                        closestHitT = t;
                        closestHitPrimitiveIndex = primitiveIndex;
                    }
                }
            }
        } else {
            uint childIdx0 = currNode.firstChildOrPrimitiveIndex;
            uint childIdx1 = childIdx0 % 2 == 1 ? childIdx0 + 1 : childIdx0 - 1;
            nodeIndexStack[stackSize] = childIdx0;
            nodeIndexStack[stackSize + 1] = childIdx1;
            stackSize += 2;
        }
    }

    if (closestHitT < 1e9) {
        fragmentColor = getColorAtRayHit(rayOrigin, rayDirection, closestHitT, closestHitPrimitiveIndex);
    }

    ivec2 imageCoord = ivec2(gl_GlobalInvocationID.xy);
    imageStore(outputImage, imageCoord, fragmentColor);
}
