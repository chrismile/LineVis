-- Compute

#version 430

layout (local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

layout (std430, binding = 2) readonly buffer OpacityBufferIn {
    float opacityBufferIn[];
};

layout (std430, binding = 3) writeonly buffer OpacityBufferOut {
    float opacityBufferOut[];
};

layout (std430, binding = 4) readonly buffer LineSegmentVisibilityBuffer {
    uint lineSegmentVisibilityBuffer[];
};

layout (std430, binding = 5) readonly buffer LineSegmentConnectivityBuffer {
    // Indices of left and right neighbors (or index of element itself if no left/right neighbor).
    uvec2 lineSegmentConnectivityBuffer[];
};

uniform uint numLineSegments; ///< Number of entries in opacityBufferIn/opacityBufferOut.
uniform float relaxationConstant;

/**
 * Computes (see http://graphics.stanford.edu/courses/cs468-12-spring/LectureSlides/06_smoothing.pdf):
 * p_i = p_i + lamba * L(i)
 * L(i) = 0.5 * p_{i-1} - p_i + 0.5 * p_{i+1}
 */
void main() {
    if (gl_GlobalInvocationID.x >= numLineSegments) {
        return;
    }

    uvec2 neighborIndices = lineSegmentConnectivityBuffer[gl_GlobalInvocationID.x];
    float ownValue = opacityBufferIn[gl_GlobalInvocationID.x];
    float neighborLeft = opacityBufferIn[neighborIndices.x];
    float neighborRight = opacityBufferIn[neighborIndices.y];
    uint visibleLeft = lineSegmentVisibilityBuffer[neighborIndices.x];
    uint visibleRight = lineSegmentVisibilityBuffer[neighborIndices.y];
    if (visibleLeft == 0) {
        neighborLeft = ownValue;
    }
    if (visibleRight == 0) {
        neighborRight = ownValue;
    }

    float outputValue = -ownValue;
    outputValue += 0.5 * neighborLeft;
    outputValue += 0.5 * neighborRight;
    outputValue = ownValue + relaxationConstant * outputValue;
    opacityBufferOut[gl_GlobalInvocationID.x] = outputValue;
}
