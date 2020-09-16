-- Compute

#version 430

layout (local_size_x = WORK_GROUP_SIZE_1D, local_size_y = 1, local_size_z = 1) in;

layout (std430, binding = 2) buffer LineOffsetBuffer
{
    uint lineOffsets[];
};

layout (std430, binding = 3) buffer TubeIndexBuffer
{
    uint tubeIndices[];
};

uniform uint numLines;

void main() {
    uint globalID = gl_GlobalInvocationID.x;
    if (globalID.x >= numLines) {
        return;
    }

    uint lineOffset = lineOffsets[globalID];
    uint numVertexPts = lineOffsets[globalID+1] - lineOffset;
    uint numLineSegmentsBefore = lineOffset - globalID;
    uint indexBufferOffset = lineOffset*NUM_CIRCLE_SEGMENTS;

    uint indexBufferWriteIndex = numLineSegmentsBefore*NUM_CIRCLE_SEGMENTS*6;
    for (int i = 0; i < numVertexPts-1; i++) {
        for (int j = 0; j < NUM_CIRCLE_SEGMENTS; j++) {
            // Build two CCW triangles (one quad) for each side
            // Triangle 1
            tubeIndices[indexBufferWriteIndex++] = indexBufferOffset + (j + i*NUM_CIRCLE_SEGMENTS);
            tubeIndices[indexBufferWriteIndex++] = indexBufferOffset + ((j+1)%NUM_CIRCLE_SEGMENTS + i*NUM_CIRCLE_SEGMENTS);
            tubeIndices[indexBufferWriteIndex++] = indexBufferOffset + ((j+1)%NUM_CIRCLE_SEGMENTS + (i+1)*NUM_CIRCLE_SEGMENTS);

            // Triangle 2
            tubeIndices[indexBufferWriteIndex++] = indexBufferOffset + (j + i*NUM_CIRCLE_SEGMENTS);
            tubeIndices[indexBufferWriteIndex++] = indexBufferOffset + ((j+1)%NUM_CIRCLE_SEGMENTS + (i+1)*NUM_CIRCLE_SEGMENTS);
            tubeIndices[indexBufferWriteIndex++] = indexBufferOffset + (j + (i+1)*NUM_CIRCLE_SEGMENTS);
        }
    }
}
