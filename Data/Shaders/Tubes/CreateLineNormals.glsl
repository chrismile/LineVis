-- Compute

#version 430

layout (local_size_x = WORK_GROUP_SIZE_1D, local_size_y = 1, local_size_z = 1) in;

layout (std430, binding = 2) buffer LineOffsetBuffer
{
    uint lineOffsets[];
};

struct InputLinePoint {
    vec3 linePoint;
    float lineAttribute;
};
layout (std430, binding = 3) buffer InputLinePointBuffer
{
    InputLinePoint inputLinePoints[];
};

struct OutputLinePoint {
    vec3 linePoint;
    float lineAttribute;
    vec3 lineTangent;
    uint valid; // 0 or 1
    vec3 lineNormal;
    float padding2;
};
layout (std430, binding = 4) buffer OutputLinePointBuffer
{
    OutputLinePoint outputLinePoints[];
};

uniform uint numLines;


void computeLineNormal(in vec3 tangent, out vec3 normal, in vec3 lastNormal)
{
    vec3 helperAxis = lastNormal;
    if (length(cross(helperAxis, tangent)) < 0.01) {
        // If tangent == helperAxis
        helperAxis = vec3(0.0, 1.0, 0.0);
    }
    normal = normalize(helperAxis - tangent * dot(helperAxis, tangent)); // Gram-Schmidt
}

void main() {
    uint globalID = gl_GlobalInvocationID.x;
    if (globalID.x >= numLines) {
        return;
    }

    uint lineOffset = lineOffsets[globalID];
    uint numLinePoints = lineOffsets[globalID + 1] - lineOffset;

    OutputLinePoint outputLinePoint;
    vec3 lastNormal = vec3(1.0, 0.0, 0.0);
    for (int i = 0; i < numLinePoints; i++) {
        vec3 center = inputLinePoints[lineOffset + i].linePoint;

        // Remove invalid line points (used in many scientific datasets to indicate invalid lines).
        const float MAX_VAL = 1e10;
        if (abs(center.x) > MAX_VAL || abs(center.y) > MAX_VAL || abs(center.z) > MAX_VAL) {
            outputLinePoints[lineOffset + i].valid = 0;
            continue;
        }

        vec3 tangent;
        if (i == 0) {
            // First node
            tangent = inputLinePoints[lineOffset + i+1].linePoint - center;
        } else if (i == numLinePoints-1) {
            // Last node
            tangent = center - inputLinePoints[lineOffset + i-1].linePoint;
        } else {
            // Node with two neighbors - use both tangents.
            tangent = inputLinePoints[lineOffset + i+1].linePoint - center;
            //tangent += pathLineCenters.at(i) - pathLineCenters.at(i-1);
        }
        if (length(tangent) < 0.0001) {
            // In case the two vertices are almost identical, just skip this path line segment.
            outputLinePoints[lineOffset + i].valid = 0;
            continue;
        }
        tangent = normalize(tangent);

        vec3 normal;
        computeLineNormal(tangent, normal, lastNormal);
        lastNormal = normal;

        outputLinePoint.linePoint = center;
        outputLinePoint.lineTangent = tangent;
        outputLinePoint.lineNormal = normal;
        outputLinePoint.lineAttribute = inputLinePoints[lineOffset + i].lineAttribute;
        outputLinePoint.valid = 1;
        outputLinePoints[lineOffset + i] = outputLinePoint;
    }
}
