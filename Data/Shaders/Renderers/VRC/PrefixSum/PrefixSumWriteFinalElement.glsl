-- Compute

#version 430

layout(local_size_x = 1) in;

layout(std430, binding = 0) readonly buffer DataInBuffer {
    uint dataIn[];
};

layout(std430, binding = 1) buffer DataOutBuffer {
    uint dataOut[];
};

uniform uint N;

void main() {
    dataOut[N] = dataOut[N - 1] + dataIn[N - 1];
}
