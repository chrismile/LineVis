-- Mesh

#version 450 core

#extension GL_NV_mesh_shader : require
//#extension GL_EXT_scalar_block_layout : require

#define WORKGROUP_SIZE 32
#define MESHLET_MAX_VERTICES 64
#define MESHLET_MAX_PRIMITIVES (2 * MESHLET_MAX_VERTICES - NUM_TUBE_SUBDIVISIONS)

layout(local_size_x = WORKGROUP_SIZE) in;
layout(triangles, max_vertices = MESHLET_MAX_VERTICES, max_primitives = MESHLET_MAX_PRIMITIVES) out;

#include "LineUniformData.glsl"
#include "LineDataSSBO.glsl"

struct MeshletData {
    uint linePointIndexStart;
    uint numLinePoints;
};

layout(std430, binding = MESHLET_INPUT_DATA_BUFFER_BINDING) readonly buffer MeshletDataBuffer {
    uvec4 meshlets[];
};

//layout (location = 0) out PerVertexData {
//    vec4 fragmentPositionWorld;
//} v_out[];
layout (location = 0) out vec3 fragmentPositionWorld[];

#define M_PI 3.14159265358979323846

void main() {
    uint meshletIdx = gl_WorkGroupID.x;
    uint threadIdx = gl_LocalInvocationID.x;

    uvec4 meshletVec = meshlets[meshletIdx];
    MeshletData meshletData;
    meshletData.linePointIndexStart = meshletVec.x;
    meshletData.numLinePoints = meshletVec.y;

#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
    int useBand = psUseBands[principalStressIndex];
#else
    int useBand = 1;
#endif

#if !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
    float thickness = useBand != 0 ? MIN_THICKNESS : 1.0f;
#else
    float thickness0Current[NUM_TUBE_SUBDIVISIONS];
    float thickness0Next[NUM_TUBE_SUBDIVISIONS];
    float thickness1Current[NUM_TUBE_SUBDIVISIONS];
    float thickness1Next[NUM_TUBE_SUBDIVISIONS];
#endif

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
#else
    const float lineRadius = lineWidth * 0.5;
#endif
    const mat4 vpMatrix = pMatrix * vMatrix;

    uint numVertices = meshletData.numLinePoints * NUM_TUBE_SUBDIVISIONS;
    for (uint vertexIdx = threadIdx; vertexIdx < numVertices; vertexIdx += WORKGROUP_SIZE) {
        uint localLinePointIdx = vertexIdx / NUM_TUBE_SUBDIVISIONS;
        uint globalLinePointIdx = meshletData.linePointIndexStart + localLinePointIdx;
        LinePointData linePointData = linePoints[globalLinePointIdx];
        uint circleIdx = vertexIdx % NUM_TUBE_SUBDIVISIONS;

        vec3 linePosition = (mMatrix * vec4(linePointData.vertexPosition, 1.0)).xyz;
        vec3 vertexBinormal = cross(linePointData.vertexTangent, linePointData.vertexNormal);
        mat3 tangentFrameMatrix = mat3(linePointData.vertexNormal, vertexBinormal, linePointData.vertexTangent);

        float t = float(circleIdx) / float(NUM_TUBE_SUBDIVISIONS) * 2.0 * M_PI;
        float cosAngle = cos(t);
        float sinAngle = sin(t);

#ifdef USE_BANDS
        // Bands with minimum thickness.
        vec3 localPosition = vec3(thickness * cosAngle, sinAngle, 0.0f);
        vec3 localNormal = vec3(cosAngle, thickness * sinAngle, 0.0f);
        vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + linePosition;
        vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
#else
        // Tubes with circular profile.
        vec3 localPosition = vec3(cosAngle, sinAngle, 0.0f);
        vec3 localNormal = vec3(cosAngle, sinAngle, 0.0f);
        vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + linePosition;
        vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
#endif

        gl_MeshVerticesNV[vertexIdx].gl_Position = vpMatrix * vec4(vertexPosition, 1.0);
        fragmentPositionWorld[vertexIdx] = vertexPosition;
    }

    for (uint localLinePointIdx = 0; localLinePointIdx < meshletData.numLinePoints; localLinePointIdx += WORKGROUP_SIZE) {
        for (int i = 0; i < NUM_TUBE_SUBDIVISIONS; i++) {
            gl_PrimitiveIndicesNV[i] = uint(0);
        }
    }

    uint numSegments = meshletData.numLinePoints - 1;
    uint numQuads = numSegments * NUM_TUBE_SUBDIVISIONS;
    gl_PrimitiveCountNV = 2 * numQuads;
    for (uint quadIdx = threadIdx; quadIdx < numQuads; quadIdx += WORKGROUP_SIZE) {
        uint writeIdx = 6 * quadIdx;
        uint j = quadIdx / NUM_TUBE_SUBDIVISIONS;
        uint k = quadIdx % NUM_TUBE_SUBDIVISIONS;
        uint kNext = (k + 1) % NUM_TUBE_SUBDIVISIONS;
        uint indexOffsetCurrent = (j) * NUM_TUBE_SUBDIVISIONS;
        uint indexOffsetNext = (j + 1) * NUM_TUBE_SUBDIVISIONS;

        gl_PrimitiveIndicesNV[writeIdx] = indexOffsetCurrent + k;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetCurrent + kNext;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetNext + k;

        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetNext + k;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetCurrent + kNext;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetNext + kNext;
    }
}


-- Fragment

#version 450 core

#include "LineUniformData.glsl"

layout(location = 0) in vec3 fragmentPositionWorld;

#if defined(DIRECT_BLIT_GATHER)
layout(location = 0) out vec4 fragColor;
#endif

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#define GEOMETRY_PASS_TUBE
#include "DepthHelper.glsl"

void main() {
    // TODO
    float ribbonPosition = 0.5;
    vec4 fragmentColor = vec4(0.0, 0.0, 0.0, 1.0);

    float absCoords = abs(ribbonPosition);

    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
#ifdef USE_ROTATING_HELICITY_BANDS
    const float WHITE_THRESHOLD = 0.8;
#else
    const float WHITE_THRESHOLD = 0.7;
#endif
    float EPSILON_OUTLINE = 0.0;
#ifdef USE_BANDS
    //float EPSILON_OUTLINE = clamp(getAntialiasingFactor(fragmentDistance / (useBand != 0 ? bandWidth : lineWidth) * 2.0), 0.0, 0.49);
    float EPSILON_WHITE = fwidth(ribbonPosition);
#else
    //float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / lineWidth, 0.0, 0.49);
    float EPSILON_WHITE = fwidth(ribbonPosition);
#endif
    float coverage = 1.0 - smoothstep(1.0 - EPSILON_OUTLINE, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(ribbonPosition));
    vec4 colorOut = vec4(
            mix(fragmentColor.rgb, foregroundColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON_WHITE, WHITE_THRESHOLD + EPSILON_WHITE, absCoords)),
            fragmentColor.a * coverage);

    // TODO
    colorOut = vec4(0.0, 0.0, 0.0, 1.0);

#include "LinePassGather.glsl"
}
