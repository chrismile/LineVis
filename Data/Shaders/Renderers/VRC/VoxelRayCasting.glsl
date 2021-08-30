-- Vertex

#version 430 core

in vec4 vertexPosition;

void main()
{
    gl_Position = mvpMatrix * vertexPosition;
}


-- Fragment

#version 430

layout(pixel_center_integer) in vec4 gl_FragCoord;
out vec4 fragColorOut;

// Size of the rendering viewport (/window)
uniform ivec2 viewportSize;

// Camera data
uniform vec3 cameraPositionWorld;
uniform float aspectRatio;
uniform float fov;

// Convert points in world space to voxel space (voxel grid at range (0, 0, 0) to (rx, ry, rz)).
uniform mat4 worldSpaceToVoxelSpace;
uniform mat4 voxelSpaceToWorldSpace;
uniform mat4 viewMatrix;
uniform mat4 inverseViewMatrix;
uniform mat4 ndcToVoxelSpace;

uniform vec4 clearColor = vec4(1.0);
uniform vec3 foregroundColor = vec3(0.0);

#ifdef COMPUTE_NEAREST_FURTHEST_HIT_USING_HULL
uniform sampler2D nearestLineHullHitDepthTexture;
uniform sampler2D furthestLineHullHitDepthTexture;
#endif

#include "RayIntersectionTests.glsl"
#include "TransferFunction.glsl"
#include "Blending.glsl"
#include "VoxelData.glsl"
#include "ProcessVoxel.glsl"
#include "TraverseGrid.glsl"

void main() {
    ivec2 fragCoord = ivec2(gl_FragCoord.xy);
    vec4 fragColor = clearColor;

    vec3 rayOrigin = (worldSpaceToVoxelSpace * inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec2 ndcPosition = 2.0 * (vec2(fragCoord) + vec2(0.5)) / vec2(viewportSize) - vec2(1.0);
    float scale = tan(fov * 0.5);
    vec2 rayDirCameraSpace = vec2(ndcPosition.x * aspectRatio * scale, ndcPosition.y * scale);
    vec3 rayDirection = normalize((worldSpaceToVoxelSpace * inverseViewMatrix * vec4(rayDirCameraSpace, -1.0, 0.0)).xyz);

#ifdef COMPUTE_NEAREST_FURTHEST_HIT_USING_HULL
    float nearestLineHullHitDepth = texelFetch(nearestLineHullHitDepthTexture, fragCoord, 0).x;
    float furthestLineHullHitDepth = texelFetch(furthestLineHullHitDepthTexture, fragCoord, 0).x;

    vec4 entrancePointHomogeneous =
            ndcToVoxelSpace * vec4(ndcPosition.x, ndcPosition.y, 2.0 * nearestLineHullHitDepth - 1.0, 1.0);
    vec4 exitPointHomogeneous =
            ndcToVoxelSpace * vec4(ndcPosition.x, ndcPosition.y, 2.0 * furthestLineHullHitDepth - 1.0, 1.0);
    vec3 entrancePoint = entrancePointHomogeneous.xyz / entrancePointHomogeneous.w;
    vec3 exitPoint = exitPointHomogeneous.xyz / exitPointHomogeneous.w;

    // Get the intersections with the voxel grid.
    vec3 voxelGridLower = vec3(-1.0 + 1e-4);
    vec3 voxelGridUpper = vec3(gridResolution) + vec3(1.0 - 1e-4);
    float tNear, tFar;
    if (nearestLineHullHitDepth < furthestLineHullHitDepth) {
        rayBoxIntersectionRayCoords(rayOrigin, rayDirection, voxelGridLower, voxelGridUpper, tNear, tFar);
        if (tNear < 0.0) {
            entrancePoint = rayOrigin; // TODO
        }

        fragColor = traverseVoxelGrid(rayOrigin, rayDirection, entrancePoint, exitPoint);
        //blend(clearColor, fragColor);
        blend(vec4(1.0, 0.5, 0.0, 1.0), fragColor);
        fragColor = vec4(fragColor.rgb / fragColor.a, fragColor.a);
    }
#else
    // Get the intersections with the voxel grid.
    vec3 voxelGridLower = vec3(-1.0 + 1e-4);
    vec3 voxelGridUpper = vec3(gridResolution) + vec3(1.0 - 1e-4);
    float tNear, tFar;
    if (rayBoxIntersectionRayCoords(rayOrigin, rayDirection, voxelGridLower, voxelGridUpper, tNear, tFar)) {
        // First intersection point behind camera ray origin?
        vec3 entrancePoint = rayOrigin + tNear * rayDirection;
        vec3 exitPoint = rayOrigin + tFar * rayDirection;
        if (tNear < 0.0) {
            entrancePoint = rayOrigin;
        }

        fragColor = traverseVoxelGrid(rayOrigin, rayDirection, entrancePoint, exitPoint);
        blend(clearColor, fragColor);
        fragColor = vec4(fragColor.rgb / fragColor.a, fragColor.a);
    }
#endif

    fragColorOut = fragColor;
}
