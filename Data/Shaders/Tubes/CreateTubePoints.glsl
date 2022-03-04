-- Compute

#version 450 core

layout (local_size_x = WORK_GROUP_SIZE_1D, local_size_y = 1, local_size_z = 1) in;

struct PathLinePoint {
    vec3 linePointPosition;
    float linePointAttribute;
    vec3 lineTangent;
    float padding1;
    vec3 lineNormal;
    float padding2;
};

struct TubeVertex {
    vec3 vertexPosition;
    float vertexAttribute;
    vec3 vertexNormal;
    float padding0;
    vec3 vertexTangent;
    float padding1;
};

layout (std430, binding = 2) buffer PathLinePoints
{
    PathLinePoint pathLinePoints[];
};

layout (std430, binding = 3) buffer TubeVertices
{
    TubeVertex tubeVertices[];
};

uniform uint numLinePoints;

void main() {
    uint globalID = gl_GlobalInvocationID.x;
    if (globalID.x >= numLinePoints) {
        return;
    }

    PathLinePoint pathLinePoint = pathLinePoints[globalID];

    vec3 lineBinormal = cross(pathLinePoint.lineTangent, pathLinePoint.lineNormal);
    mat3 tangentFrameMatrix = mat3(pathLinePoint.lineNormal, lineBinormal, pathLinePoint.lineTangent);

    const float theta = 2.0 * 3.1415926 / float(NUM_CIRCLE_SEGMENTS);
    const float tangetialFactor = tan(theta); // opposite / adjacent
    const float radialFactor = cos(theta); // adjacent / hypotenuse

    vec2 position = vec2(CIRCLE_RADIUS, 0.0);
    vec3 circlePoints[NUM_CIRCLE_SEGMENTS];
    vec3 vertexNormals[NUM_CIRCLE_SEGMENTS];
    for (int i = 0; i < NUM_CIRCLE_SEGMENTS; i++) {
        circlePoints[i] = tangentFrameMatrix * vec3(position, 0.0) + pathLinePoint.linePointPosition;
        vertexNormals[i] = normalize(circlePoints[i] - pathLinePoint.linePointPosition);

        // Add the tangent vector and correct the position using the radial factor.
        vec2 circleTangent = vec2(-position.y, position.x);
        position += tangetialFactor * circleTangent;
        position *= radialFactor;
    }

    TubeVertex tubeVertex;
    for (int i = 0; i < NUM_CIRCLE_SEGMENTS; i++) {
        tubeVertex.vertexPosition = circlePoints[i];
        tubeVertex.vertexAttribute = pathLinePoint.linePointAttribute;
        tubeVertex.vertexNormal = vertexNormals[i];
        tubeVertex.vertexTangent = pathLinePoint.lineTangent;
        tubeVertices[globalID*NUM_CIRCLE_SEGMENTS + i] = tubeVertex;
    }
}
