-- Mesh

#define MESHLET_MAX_VERTICES 64
#define MESHLET_MAX_PRIMITIVES (2 * MESHLET_MAX_VERTICES - 2 * NUM_TUBE_SUBDIVISIONS)

layout(local_size_x = WORKGROUP_SIZE) in;
layout(triangles, max_vertices = MESHLET_MAX_VERTICES, max_primitives = MESHLET_MAX_PRIMITIVES) out;

#include "LineUniformData.glsl"
#include "LineDataSSBO.glsl"

struct MeshletData {
    uint linePointIndexStart;
    uint numLinePoints;
};

layout(std430, binding = MESHLET_INPUT_DATA_BUFFER_BINDING) readonly buffer MeshletDataBuffer {
    uvec2 meshlets[];
};

layout(location = 0) out vec3 fragmentPositionWorld[];
#ifdef USE_SCREEN_SPACE_POSITION
layout(location = 1) out vec3 screenSpacePosition[];
#endif
layout(location = 2) out float fragmentAttribute[];
layout(location = 3) out vec3 fragmentNormal[];
layout(location = 4) out vec3 fragmentTangent[];

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 5) flat out uint fragmentPrincipalStressIndex[];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 6) flat out float fragmentLineHierarchyLevel[];
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 7) flat out uint fragmentLineAppearanceOrder[];
#endif

#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
layout(location = 8) out float interpolationFactorLine[];
layout(location = 9) flat out uint fragmentVertexIdUint[];
#endif
#ifdef USE_BANDS
layout(location = 10) flat out int useBand[];
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
layout(location = 11) out float thickness0[];
layout(location = 12) out float thickness1[];
#else
layout(location = 13) out float thickness[];
#endif
layout(location = 14) out vec3 lineNormal[];
layout(location = 15) out vec3 linePosition[];
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
layout(location = 16) out float fragmentRotation[];
#endif
#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
layout(location = 17) out float phi[];
layout(location = 18) flat out int interpolateWrap[];
#endif

//#define DEBUG_MESHLETS
#ifdef DEBUG_MESHLETS
layout(location = 20) flat out uint fragmentMeshletIdx[];
#endif

#define M_PI 3.14159265358979323846

void main() {
    uint meshletIdx = gl_WorkGroupID.x;
    uint threadIdx = gl_LocalInvocationIndex;

    uvec2 meshletVec = meshlets[meshletIdx];
    MeshletData meshletData;
    meshletData.linePointIndexStart = meshletVec.x;
    meshletData.numLinePoints = meshletVec.y;

    uint numVertices = meshletData.numLinePoints * NUM_TUBE_SUBDIVISIONS;
    uint numSegments = meshletData.numLinePoints - 1;
    uint numQuads = numSegments * NUM_TUBE_SUBDIVISIONS;
#ifdef VK_NV_mesh_shader
    gl_PrimitiveCountNV = 2 * numQuads;
#else
    SetMeshOutputsEXT(numVertices, 2 * numQuads);
#endif

    for (uint vertexIdx = threadIdx; vertexIdx < numVertices; vertexIdx += WORKGROUP_SIZE) {
        uint localLinePointIdx = vertexIdx / NUM_TUBE_SUBDIVISIONS;
        uint globalLinePointIdx = meshletData.linePointIndexStart + localLinePointIdx;
        LinePointData linePointData = linePoints[globalLinePointIdx];
        uint circleIdx = vertexIdx % NUM_TUBE_SUBDIVISIONS;

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(VISUALIZE_SEEDING_PROCESS)
        StressLinePointData stressLinePointData = stressLinePoints[globalLinePointIdx];
#endif

#ifdef USE_PRINCIPAL_STRESSES
        StressLinePointPrincipalStressData stressLinePointPrincipalStressData = principalStressLinePoints[globalLinePointIdx];
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
        uint principalStressIndex = stressLinePointData.linePrincipalStressIndex;
#endif

#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
        int useBandLocal = psUseBands[principalStressIndex];
#else
        int useBandLocal = 1;
#endif

#if !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
        float thicknessLocal = useBandLocal != 0 ? MIN_THICKNESS : 1.0;
#endif

        const float lineRadius = (useBandLocal != 0 ? bandWidth : lineWidth) * 0.5;
#else
        const float lineRadius = lineWidth * 0.5;
#endif

        vec3 lineCenterPosition = (mMatrix * vec4(linePointData.linePosition, 1.0)).xyz;
        vec3 normal = linePointData.lineNormal;
        vec3 tangent = linePointData.lineTangent;
        vec3 binormal = cross(linePointData.lineTangent, linePointData.lineNormal);
        mat3 tangentFrameMatrix = mat3(normal, binormal, tangent);

        float t = float(circleIdx) / float(NUM_TUBE_SUBDIVISIONS) * 2.0 * M_PI;
        float cosAngle = cos(t);
        float sinAngle = sin(t);

#ifdef USE_BANDS

#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        float stressX;
        float stressZ;
        if (principalStressIndex == 0) {
            stressX = stressLinePointPrincipalStressData.lineMediumStress;
            stressZ = stressLinePointPrincipalStressData.lineMinorStress;
        } else if (principalStressIndex == 1) {
            stressX = stressLinePointPrincipalStressData.lineMinorStress;
            stressZ = stressLinePointPrincipalStressData.lineMajorStress;
        } else {
            stressX = stressLinePointPrincipalStressData.lineMediumStress;
            stressZ = stressLinePointPrincipalStressData.lineMajorStress;
        }
    #endif

#if defined(USE_NORMAL_STRESS_RATIO_TUBES)
        float factorX = clamp(abs(stressX / stressZ), 0.0, 1.0);
        float factorZ = clamp(abs(stressZ / stressX), 0.0, 1.0);
        vec3 localPosition = vec3(cosAngle * factorX, sinAngle * factorZ, 0.0);
        vec3 localNormal = vec3(cosAngle * factorZ, sinAngle * factorX, 0.0);
        vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
        vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
        thickness0[vertexIdx] = factorX;
        thickness1[vertexIdx] = factorZ;
#elif defined(USE_HYPERSTREAMLINES)
        stressX = max(abs(stressX), minimumHyperstreamlineWidth);
        stressZ = max(abs(stressZ), minimumHyperstreamlineWidth);
        vec3 localPosition = vec3(cosAngle * stressX, sinAngle * stressZ, 0.0);
        vec3 localNormal = vec3(cosAngle * stressZ, sinAngle * stressX, 0.0);
        vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
        vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
        thickness0[vertexIdx] = useBand != 0 ? stressX : 1.0;
        thickness1[vertexIdx] = useBand != 0 ? stressZ : 1.0;
#else
        // Bands with minimum thickness.
        vec3 localPosition = vec3(thicknessLocal * cosAngle, sinAngle, 0.0);
        vec3 localNormal = vec3(cosAngle, thicknessLocal * sinAngle, 0.0);
        vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
        vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
        thickness[vertexIdx] = thicknessLocal;
#endif
#else
        vec3 localPosition = vec3(cosAngle, sinAngle, 0.0);
        vec3 localNormal = vec3(cosAngle, sinAngle, 0.0);
        vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
        vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
#endif


#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
        const float factor = 2.0 * M_PI / float(NUM_TUBE_SUBDIVISIONS);
#endif

#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
        phi[vertexIdx] = float(circleIdx) * factor;
        /*
         * https://www.khronos.org/registry/vulkan/specs/1.3-extensions/html/vkspec.html#drawing-triangle-lists
         * The provoking vertex is the first vertex of the triangle. In order to wrap when interpolating between the last
         * and the first vertex (from pi to 0), interpolateWrap needs to be set to 1 for the final circle index.
         */
        interpolateWrap[vertexIdx] = circleIdx == NUM_TUBE_SUBDIVISIONS - 1 ? 1 : 0;
#endif
#ifdef USE_BANDS
        useBand[vertexIdx] = useBandLocal;
        linePosition[vertexIdx] = lineCenterPosition;
        lineNormal[vertexIdx] = normal;
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
        fragmentPrincipalStressIndex[vertexIdx] = principalStressIndex;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
        fragmentLineAppearanceOrder[vertexIdx] = stressLinePointData.lineLineAppearanceOrder;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
        fragmentLineHierarchyLevel[vertexIdx] = stressLinePointData.lineLineHierarchyLevel;
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
        interpolationFactorLine[vertexIdx] = float(globalLinePointIdx - linePointData.lineStartIndex);
        fragmentVertexIdUint[vertexIdx] = linePointData.lineStartIndex;//globalLinePointIdx;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
        fragmentRotation[vertexIdx] = linePointData.lineRotation * helicityRotationFactor;
#endif
        fragmentAttribute[vertexIdx] = linePointData.lineAttribute;
        fragmentTangent[vertexIdx] = tangent;

#ifdef VK_NV_mesh_shader
        gl_MeshVerticesNV[vertexIdx].gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
#else
        gl_MeshVerticesEXT[vertexIdx].gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
#endif

        fragmentNormal[vertexIdx] = vertexNormal;
        fragmentPositionWorld[vertexIdx] = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition[vertexIdx] = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif

#ifdef DEBUG_MESHLETS
        fragmentMeshletIdx[vertexIdx] = meshletIdx;
#endif
    }

    for (uint quadIdx = threadIdx; quadIdx < numQuads; quadIdx += WORKGROUP_SIZE) {
        uint writeIdx = 6 * quadIdx;
        uint j = quadIdx / NUM_TUBE_SUBDIVISIONS;
        uint k = quadIdx % NUM_TUBE_SUBDIVISIONS;
        uint kNext = (k + 1) % NUM_TUBE_SUBDIVISIONS;
        uint indexOffsetCurrent = (j) * NUM_TUBE_SUBDIVISIONS;
        uint indexOffsetNext = (j + 1) * NUM_TUBE_SUBDIVISIONS;

#ifdef VK_NV_mesh_shader
        gl_PrimitiveIndicesNV[writeIdx] = indexOffsetCurrent + k;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetCurrent + kNext;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetNext + k;

        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetNext + k;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetCurrent + kNext;
        gl_PrimitiveIndicesNV[++writeIdx] = indexOffsetNext + kNext;
#else
        gl_PrimitiveTriangleIndicesEXT[writeIdx] = uvec3(
                indexOffsetCurrent + k, indexOffsetCurrent + kNext, indexOffsetNext + k);
        gl_PrimitiveTriangleIndicesEXT[writeIdx + 1] = uvec3(
                indexOffsetNext + k, indexOffsetCurrent + kNext, indexOffsetNext + kNext);
#endif
    }
}


-- MeshNV

#version 450 core

#extension GL_NV_mesh_shader : require

#define VK_NV_mesh_shader
#import ".Mesh"


-- MeshEXT

#version 450 core

#extension GL_EXT_mesh_shader : require

#define VK_EXT_mesh_shader
#import ".Mesh"
