/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2021, Christoph Neuhauser
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

#version 430

layout (local_size_x = 256) in;

struct LineSegment {
    vec3 v0; // Vertex position
    float a0; // Vertex attribute
    vec3 v1; // Vertex position
    float a1; // Vertex attribute
    uint lineID;
};

layout (std430, binding = 0) readonly buffer UniformData {
    ivec3 gridResolution;
    int numLines;
};

struct LinePoint {
    vec3 linePoint;
    float lineAttribute;
};

layout (std430, binding = 1) readonly buffer LinePointBuffer {
    LinePoint linePoints[];
};

layout (std430, binding = 2) readonly buffer LineOffsetBuffer {
    uint lineOffsets[];
};

layout (std430, binding = 3) coherent buffer SpinlockBuffer {
    uint spinlockViewportBuffer[];
};

layout (binding = 4) coherent image3D lineDensityFieldImage;

uint getVoxelIndex1D(ivec3 voxelIndex) {
    return voxelIndex.x + voxelIndex.y*gridResolution.x + voxelIndex.z*gridResolution.x*gridResolution.y;
}



#define BIAS 0.001

/**
 * Helper function for rayBoxIntersection (see below).
 */
bool rayBoxPlaneIntersection(
        float rayOriginX, float rayDirectionX, float lowerX, float upperX, inout float tNear, inout float tFar) {
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
        if (tFar < 0.0) {
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
int rayBoxIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 lower, vec3 upper, inout float tNear, inout float tFar) {
    tNear = -1e7;
    tFar = 1e7;
    for (int i = 0; i < 3; i++) {
        if (!rayBoxPlaneIntersection(rayOrigin[i], rayDirection[i], lower[i], upper[i], tNear, tFar)) {
            return 0;
        }
    }

    return (tNear >= 0.0f && tNear <= 1.0f ? 1 : 0) + (tFar >= 0.0f && tFar <= 1.0f ? 1 : 0);
}


void addLineSegment(ivec3 voxelIndex, LineSegment lineSegment) {
    float lineSegmentLength = length(lineSegment.v1 - lineSegment.v0);
    uint voxelIndex1D = getVoxelIndex1D(voxelIndex);

    // Use a spinlock to synchronize access to lineDensityFieldImage.
    bool keepWaiting = true;
    while (keepWaiting) {
        if (atomicCompSwap(spinlockBuffer[voxelIndex1D], 0, 1) == 0) {
            float newDensity = imageLoad(lineDensityFieldImage, voxelIndex).x + lineSegmentLength;
            imageStore(lineDensityFieldImage, voxelIndex, vec4(newDensity));
            memoryBarrier();
            atomicExchange(spinlockViewportBuffer[pixelIndex], 0);
            keepWaiting = false;
        }
    }
}



/**
 * Code inspired by "A Fast Voxel Traversal Algorithm for Ray Tracing" written by John Amanatides, Andrew Woo.
 * http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.42.3443&rep=rep0&type=pdf
 * Value initialization code adapted from: https://stackoverflow.com/questions/12367071/how-do-i-initialize-the-t-
 * variables-in-a-fast-voxel-traversal-algorithm-for-ray
 *
 * Traverses the voxel grid from "startPoint" until "endPoint".
 * Calls "nextVoxel" each time a voxel is entered.
 * Returns the accumulated color using voxel raytracing.
 */
void traverseVoxelGrid(uint lineID, vec3 startPoint, float startAttribute, vec3 endPoint, float endAttribute,
        inout ivec3 currentVoxel, inout int currentVoxelNumIntersections, inout vec3 currentVoxelIntersection,
        inout float currentVoxelIntersectionAttribute, inout bool noIntersectionForLineYet)
{
    startPoint = clamp(startPoint, vec3(0, 0, 0), vec3(gridResolution) - vec3(1e-4));
    endPoint = clamp(endPoint, vec3(0, 0, 0), vec3(gridResolution) - vec3(1e-4));
    ivec3 startVoxel = ivec3(startPoint);
    ivec3 endVoxel = ivec3(endPoint);

    float tMaxX, tMaxY, tMaxZ, tDeltaX, tDeltaY, tDeltaZ;
    ivec3 voxelIndex;

    int stepX = int(sign(endPoint.x - startPoint.x));
    if (stepX != 0)
        tDeltaX = min(stepX / (endPoint.x - startPoint.x), 1e7);
    else
        tDeltaX = 1e7; // inf
    if (stepX > 0)
        tMaxX = tDeltaX * (1.0 - fract(startPoint.x));
    else
        tMaxX = tDeltaX * fract(startPoint.x);
    voxelIndex.x = int(floor(startPoint.x));

    int stepY = int(sign(endPoint.y - startPoint.y));
    if (stepY != 0)
        tDeltaY = min(stepY / (endPoint.y - startPoint.y), 1e7);
    else
        tDeltaY = 1e7; // inf
    if (stepY > 0)
        tMaxY = tDeltaY * (1.0 - fract(startPoint.y));
    else
        tMaxY = tDeltaY * fract(startPoint.y);
    voxelIndex.y = int(floor(startPoint.y));

    int stepZ = int(sign(endPoint.z - startPoint.z));
    if (stepZ != 0)
        tDeltaZ = min(stepZ / (endPoint.z - startPoint.z), 1e7);
    else
        tDeltaZ = 1e7; // inf
    if (stepZ > 0)
        tMaxZ = tDeltaZ * (1.0 - fract(startPoint.z));
    else
        tMaxZ = tDeltaZ * fract(startPoint.z);
    voxelIndex.z = int(floor(startPoint.z));

    // Clamp to avoid imprecisions at boundaries.
    voxelIndex = clamp(voxelIndex, ivec3(0, 0, 0), gridResolution - ivec3(1));

    if (stepX == 0 && stepY == 0 && stepZ == 0) {
        return;
    }
    ivec3 step = ivec3(stepX, stepY, stepZ);
    vec3 tMax = vec3(tMaxX, tMaxY, tMaxZ);
    vec3 tDelta = vec3(tDeltaX, tDeltaY, tDeltaZ);


    vec3 rayDirection = endPoint - startPoint; // Not normalized -> t needs to be in [0.0, 1.0].
    int numIntersectionsNew;
    LineSegment lineSegment;

    while (all(greaterThanEqual(voxelIndex, ivec3(0))) && all(lessThan(voxelIndex, gridResolution))) {
        float tNear = -1e9, tFar = 1e9;
        numIntersectionsNew = rayBoxIntersection(startPoint, rayDirection,
        vec3(voxelIndex), vec3(voxelIndex) + vec3(1.0), tNear, tFar);

        if (numIntersectionsNew > 0 && noIntersectionForLineYet) {
            // Skip segment until first intersection.
            noIntersectionForLineYet = false;
            continue;
        }

        if (numIntersectionsNew == 2 || (numIntersectionsNew == 1 && currentVoxelNumIntersections == 1
                && currentVoxel == voxelIndex)) {
            lineSegment.lineID = lineID;
            if (numIntersectionsNew == 2) {
                lineSegment.v0 = startPoint + tNear * (endPoint - startPoint);
                lineSegment.a0 = startAttribute + tNear * (endAttribute - startAttribute);
            } else {
                lineSegment.v0 = currentVoxelIntersection;
                lineSegment.a0 = currentVoxelIntersectionAttribute;
            }
            lineSegment.v1 = startPoint + tFar * (endPoint - startPoint);
            lineSegment.a1 = startAttribute + tFar * (endAttribute - startAttribute);
            addLineSegment(voxelIndex, lineSegment);
            currentVoxelNumIntersections = 0;
        } else if (numIntersectionsNew == 1) {
            currentVoxel = voxelIndex;
            currentVoxelIntersection = startPoint + tNear * (endPoint - startPoint);
            currentVoxelIntersectionAttribute = startAttribute + tNear * (endAttribute - startAttribute);
            currentVoxelNumIntersections = 1;
        }

        if (voxelIndex == endVoxel) {
            // Break on last voxel.
            break;
        }

        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                voxelIndex.x += stepX;
                tMaxX += tDeltaX;
            } else {
                voxelIndex.z += stepZ;
                tMaxZ += tDeltaZ;
            }
        } else {
            if (tMaxY < tMaxZ) {
                voxelIndex.y += stepY;
                tMaxY += tDeltaY;
            } else {
                voxelIndex.z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
    }
}

void main() {
    uint lineNumber = gl_GlobalInvocationID.x;
    if (lineNumber >= uint(numLines)) {
        return;
    }

    uint lineOffset = lineOffsets[lineNumber];
    uint numLinePoints = lineOffsets[lineNumber + 1] - lineOffset;

    // Voxel for which we save intersetions that do not yet form a full line segment.
    ivec3 currentVoxel = ivec3(-1, -1, -1);
    int currentVoxelNumIntersections = 0;
    vec3 currentVoxelIntersection = vec3(1e6, 1e6, 1e6); // Carry-over-field across line segments.
    float currentVoxelIntersectionAttribute = 0.0; // Carry-over-field across line segments.
    vec3 tangent;

    bool noIntersectionForLineYet = true;
    for (int i = 0; i < numLinePoints - 1; i++) {
        LinePoint p0 = linePoints[lineOffset + i];
        LinePoint p1 = linePoints[lineOffset + i + 1];

        // Remove invalid line points (large values are used in some scientific datasets to indicate invalid lines).
        const float MAX_VAL = 1e10;
        if (abs(p0.linePoint.x) > MAX_VAL || abs(p0.linePoint.y) > MAX_VAL || abs(p0.linePoint.z) > MAX_VAL
                || abs(p1.linePoint.x) > MAX_VAL || abs(p1.linePoint.y) > MAX_VAL || abs(p1.linePoint.z) > MAX_VAL) {
            continue;
        }

        if (i == 0) {
            // First node
            tangent = p1.linePoint - p0.linePoint;
        } else if (i == numLinePoints - 1) {
            // Last node
            tangent = p0.linePoint - linePoints[lineOffset + i - 1].linePoint;
        } else {
            // Node with two neighbors - use both tangents.
            tangent = p1.linePoint - p0.linePoint;
        }
        if (length(tangent) < 0.0001) {
            // In case the two vertices are almost identical, just skip this path line segment.
            continue;
        }
        tangent = normalize(tangent);

        // DDA algorithm
        traverseVoxelGrid(
                lineNumber, p0.linePoint, p0.lineAttribute, p1.linePoint, p1.lineAttribute,
                currentVoxel, currentVoxelNumIntersections, currentVoxelIntersection, currentVoxelIntersectionAttribute,
                noIntersectionForLineYet);
    }
}
