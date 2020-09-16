-- Compute

#version 430

layout (local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout (std430, binding = 2) buffer OpacityBufferPerVertex {
    float opacityBufferPerVertex[];
};

layout (std430, binding = 3) readonly buffer OpacityBufferPerSegment {
    float opacityBufferPerSegment[];
};

layout (std430, binding = 4) readonly buffer LineSegmentVisibilityBuffer {
    uint lineSegmentVisibilityBuffer[];
};

layout (std430, binding = 5) readonly buffer BlendingWeightParametrizationBuffer {
    float blendingWeightParametrizationBuffer[]; // per-vertex
};

uniform uint numLineVertices;
uniform float temporalSmoothingFactor = 0.1;

#include "FloatPack.glsl"

void main() {
    if (gl_GlobalInvocationID.x >= numLineVertices) {
        return;
    }

    float opacityOld = opacityBufferPerVertex[gl_GlobalInvocationID.x];
    float w = blendingWeightParametrizationBuffer[gl_GlobalInvocationID.x];

    /*const float EPSILON = 1e-5;
    bool shallInterpolate = abs(w - round(w)) > EPSILON;
    uint i;
    if (shallInterpolate) {
        i = uint(max(floor(w), 0.0));
    } else {
        i = uint(round(w));
    }
    float alpha = opacityBufferPerSegment[i];
    if (shallInterpolate) {
        float alpha_ip1 = opacityBufferPerSegment[i+1];
        alpha = mix(alpha, alpha_ip1, fract(w));
    }*/

    uint i = uint(floor(w));
    float alpha_i = opacityBufferPerSegment[i];
    float alpha_ip1 = opacityBufferPerSegment[i+1];
    uint visible_i = lineSegmentVisibilityBuffer[i];
    uint visible_ip1 = lineSegmentVisibilityBuffer[i+1];
    if (visible_i == 0) {
        alpha_i = opacityOld;
    }
    if (visible_ip1 == 0) {
        alpha_ip1 = opacityOld;
    }
    float alpha = mix(alpha_i, alpha_ip1, fract(w));

    float newValue = (1.0 - temporalSmoothingFactor) * opacityOld + temporalSmoothingFactor * alpha;
    opacityBufferPerVertex[gl_GlobalInvocationID.x] = newValue;
}
