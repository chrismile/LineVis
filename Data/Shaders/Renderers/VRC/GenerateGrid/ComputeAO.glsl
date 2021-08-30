-- Compute

#version 430

layout (local_size_x = 64, local_size_y = 4, local_size_z = 1) in;

uniform sampler3D densityTexture;
layout (binding = 0, r32f) uniform image3D aoImage;

layout (binding = 6, std430) buffer GaussianFilterWeightBuffer {
    float blurKernel[];
};

float generateVoxelAOFactorFromDensity(ivec3 voxelIndex) {
    const int FILTER_SIZE = 7;
    const int FILTER_EXTENT = (FILTER_SIZE - 1) / 2;
    const int FILTER_NUM_FIELDS = FILTER_SIZE*FILTER_SIZE*FILTER_SIZE;

    // Filter the densities
    float aoFactor = 0.0f;

    for (int offsetZ = -FILTER_EXTENT; offsetZ <= FILTER_EXTENT; offsetZ++) {
        for (int offsetY = -FILTER_EXTENT; offsetY <= FILTER_EXTENT; offsetY++) {
            for (int offsetX = -FILTER_EXTENT; offsetX <= FILTER_EXTENT; offsetX++) {
                ivec3 readIndex = voxelIndex + ivec3(offsetX, offsetY, offsetZ);
                if (readIndex.x >= 0 && readIndex.y >= 0 && readIndex.z >= 0 && readIndex.x < gridResolution.x
                        && readIndex.y < gridResolution.y && readIndex.z < gridResolution.z) {
                    int filterIdx = (offsetZ+FILTER_EXTENT)*FILTER_SIZE*FILTER_SIZE
                            + (offsetY+FILTER_EXTENT)*FILTER_SIZE + (offsetX+FILTER_EXTENT);
                    aoFactor += texelFetch(densityTexture, readIndex, 0).x * blurKernel[filterIdx];
                }
            }
        }
    }

    return aoFactor;
}

void main() {
    ivec3 voxelIndex = ivec3(gl_GlobalInvocationID.xyz);
    if (any(greaterThanEqual(voxelIndex, gridResolution))) {
        return;
    }

    float aoFactor = generateVoxelAOFactorFromDensity(voxelIndex);
    imageStore(aoImage, voxelIndex, vec4(aoFactor));
}
