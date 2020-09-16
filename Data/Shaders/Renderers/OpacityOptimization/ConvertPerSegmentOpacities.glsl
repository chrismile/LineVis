-- Compute

#version 430

layout (local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout (std430, binding = 2) writeonly buffer OpacityBufferFloat {
    float opacityBufferFloat[];
};

layout (std430, binding = 3) readonly buffer OpacityBufferUint {
    uint opacityBufferUint[];
};

uniform uint numLineSegments; ///< Number of entries in opacityBufferIn/opacityBufferOut.

#include "FloatPack.glsl"

void main() {
    if (gl_GlobalInvocationID.x >= numLineSegments) {
        return;
    }

    uint opacityUint = opacityBufferUint[gl_GlobalInvocationID.x];
    float opacityFloat = convertUint32ToNormalizedFloat(opacityUint);
    opacityBufferFloat[gl_GlobalInvocationID.x] = opacityFloat;
}
