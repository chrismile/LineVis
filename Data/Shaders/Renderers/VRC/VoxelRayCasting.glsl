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
uniform mat4 inverseViewMatrix;

uniform vec4 clearColor = vec4(0.0);

#define FAST_NEIGHBOR_SEARCH
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
    vec2 rayDirCameraSpace;
    float scale = tan(fov * 0.5);
    rayDirCameraSpace.x = (2.0 * (float(fragCoord.x) + 0.5) / float(viewportSize.x) - 1) * aspectRatio * scale;
    rayDirCameraSpace.y = (2.0 * (float(fragCoord.y) + 0.5) / float(viewportSize.y) - 1) * scale;
    vec3 rayDirection = normalize((worldSpaceToVoxelSpace * inverseViewMatrix * vec4(rayDirCameraSpace, -1.0, 0.0)).xyz);

    // Get the intersections with the voxel grid
    vec3 voxelGridLower = vec3(0.0);
    vec3 voxelGridUpper = vec3(gridResolution);
    float tNear, tFar;
    if (rayBoxIntersectionRayCoords(rayOrigin, rayDirection, voxelGridLower, voxelGridUpper, tNear, tFar)) {
        // First intersection point behind camera ray origin?
        vec3 entrancePoint = rayOrigin + tNear * rayDirection + rayDirection*0.01;
        vec3 exitPoint = rayOrigin + tFar * rayDirection - rayDirection*0.01;
        if (tNear < 0.0) {
            entrancePoint = rayOrigin;
        }
        fragColor = traverseVoxelGrid(rayOrigin, rayDirection, entrancePoint, exitPoint);
        blend(clearColor, fragColor);
        fragColor = vec4(fragColor.rgb / fragColor.a, fragColor.a);
    }

    fragColorOut = fragColor;
}
