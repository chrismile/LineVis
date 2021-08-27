-- Compute

#version 430

layout (local_size_x = 64, local_size_y = 4, local_size_z = 1) in;

#include "VRCComputeHeaderVoxel.glsl"
#include "TransferFunction.glsl"

layout (binding = 0, r32f) uniform image3D densityImage;

void main() {
    ivec3 voxelIndex = ivec3(gl_GlobalInvocationID.xyz);
    if (any(greaterThanEqual(voxelIndex, gridResolution))) {
        return;
    }
    uint voxelIndex1D = getVoxelIndex1D(voxelIndex);
    uint numLinePoints = max(numSegments[voxelIndex1D], MAX_NUM_LINES_PER_VOXEL);

    float density = 0.0;
    LineSegment lineSegment;
    for (uint i = 0; i < numLinePoints; i++) {
        decompressLine(vec3(voxelIndex), lineSegments[voxelIndex1D*MAX_NUM_LINES_PER_VOXEL+i], lineSegment);
        float lineLength = length(lineSegment.v2 - lineSegment.v1);
        density += lineLength * (transferFunction(lineSegment.a1).a + transferFunction(lineSegment.a2).a) / 2.0f;
    }
    imageStore(densityImage, voxelIndex, vec4(density));
}
