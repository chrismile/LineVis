-- Vertex

#version 450 core

#include "LineUniformData.glsl"
#include "LineDataSSBO.glsl"

layout(location = 0) out vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
layout(location = 1) out vec3 screenSpacePosition;
#endif
layout(location = 2) out float fragmentAttribute;
layout(location = 3) out vec3 fragmentNormal;
layout(location = 4) out vec3 fragmentTangent;

/*
 * maxGeometryTotalOutputComponents is 1024 on NVIDIA hardware. When using stress line bands and ambient occlusion,
 * this value is exceeded for NUM_TUBE_SUBDIVISIONS = 8. Thus, we need to merge some attributes in this case.
 */
#if NUM_TUBE_SUBDIVISIONS >= 8 && defined(USE_AMBIENT_OCCLUSION) && defined(USE_BANDS)
#define COMPRESSED_GEOMETRY_OUTPUT_DATA
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 5) flat out uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 6) flat out float fragmentLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 7) flat out uint fragmentLineAppearanceOrder;
#endif

#ifdef USE_AMBIENT_OCCLUSION
layout(location = 8) out float interpolationFactorLine;
layout(location = 9) flat out uint fragmentVertexIdUint;
#endif
#ifdef USE_BANDS
layout(location = 10) flat out int useBand;
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
layout(location = 11) out float thickness0;
layout(location = 12) out float thickness1;
#else
layout(location = 13) out float thickness;
#endif
layout(location = 14) out vec3 lineNormal;
layout(location = 15) out vec3 linePosition;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
layout(location = 16) out float fragmentRotation;
#endif
#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
layout(location = 17) out float phi;
layout(location = 18) flat out int interpolateWrap;
#endif

#define M_PI 3.14159265358979323846

void main() {
    uint linePointIdx = gl_VertexIndex / NUM_TUBE_SUBDIVISIONS;
    uint circleIdx = gl_VertexIndex % NUM_TUBE_SUBDIVISIONS;
    LinePointData linePointData = linePoints[linePointIdx];

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(VISUALIZE_SEEDING_PROCESS)
    StressLinePointPrincipalStressData stressLinePointData = stressLinePoints[linePointIdx];
#endif

#ifdef USE_PRINCIPAL_STRESSES
    StressLinePointPrincipalStressDataBuffer stressLinePointPrincipalStressData = principalStressLinePoints[linePointIdx];
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
    uint principalStressIndex = stressLinePointData.vertexPrincipalStressIndex;
#endif

#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA)
    useBand = psUseBands[principalStressIndex];
#else
    useBand = 1;
#endif

#if !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
    thickness = useBand != 0 ? MIN_THICKNESS : 1.0f;
#endif

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
#else
    const float lineRadius = lineWidth * 0.5;
#endif
    const mat4 pvMatrix = pMatrix * vMatrix;

    vec3 lineCenterPosition = (mMatrix * vec4(linePointData.vertexPosition, 1.0)).xyz;
    vec3 normal = linePointData.vertexNormal;
    vec3 tangent = linePointData.vertexTangent;
    vec3 binormal = cross(linePointData.vertexTangent, linePointData.vertexNormal);
    mat3 tangentFrameMatrix = mat3(normal, binormal, tangent);

    float t = float(circleIdx) / float(NUM_TUBE_SUBDIVISIONS) * 2.0 * M_PI;
    float cosAngle = cos(t);
    float sinAngle = sin(t);

#ifdef USE_BANDS

#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
    float stressX;
    float stressZ;
    if (principalStressIndex == 0) {
        stressX = stressLinePointPrincipalStressData.vertexMediumStress;
        stressZ = stressLinePointPrincipalStressData.vertexMinorStress;
    } else if (principalStressIndex == 1) {
        stressX = stressLinePointPrincipalStressData.vertexMinorStress;
        stressZ = stressLinePointPrincipalStressData.vertexMajorStress;
    } else {
        stressX = stressLinePointPrincipalStressData.vertexMediumStress;
        stressZ = stressLinePointPrincipalStressData.vertexMajorStress;
    }
#endif

#if defined(USE_NORMAL_STRESS_RATIO_TUBES)
    float factorX = clamp(abs(stressX / stressZ), 0.0, 1.0f);
    float factorZ = clamp(abs(stressZ / stressX), 0.0, 1.0f);
    vec3 localPosition = vec3(cosAngle * factorX, sinAngle * factorZ, 0.0f);
    vec3 localNormal = vec3(cosAngle * factorZ, sinAngle * factorX, 0.0f);
    circlePoints = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
    vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
    thickness0 = factorX;
    thickness1 = factorZ;
#elif defined(USE_HYPERSTREAMLINES)
    stressX = abs(stressX);
    stressZ = abs(stressZ);
    vec3 localPosition = vec3(cosAngle * stressX, sinAngle * stressZ, 0.0f);
    vec3 localNormal = vec3(cosAngle * stressZ, sinAngle * stressX, 0.0f);
    vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
    vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
    thickness0 = stressX;
    thickness1 = stressZ;
#else
    // Bands with minimum thickness.
    vec3 localPosition = vec3(thickness * cosAngle, sinAngle, 0.0f);
    vec3 localNormal = vec3(cosAngle, thickness * sinAngle, 0.0f);
    vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
    vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
    thickness = useBand != 0 ? MIN_THICKNESS : 1.0f;
#endif
#else
    vec3 localPosition = vec3(cosAngle, sinAngle, 0.0f);
    vec3 localNormal = vec3(cosAngle, sinAngle, 0.0f);
    vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
    vec3 vertexNormal = normalize(tangentFrameMatrix * localNormal);
#endif

    
#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    const float factor = 2.0 * M_PI / float(NUM_TUBE_SUBDIVISIONS);
#endif

#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    phi = float(circleIdx) * factor;
    /*
     * https://www.khronos.org/registry/vulkan/specs/1.3-extensions/html/vkspec.html#drawing-triangle-lists
     * The provoking vertex is the first vertex of the triangle. In order to wrap when interpolating between the last
     * and the first vertex (from pi to 0), interpolateWrap needs to be set to 1 for the final circle index.
     */
    interpolateWrap = circleIdx == NUM_TUBE_SUBDIVISIONS - 1 ? 1 : 0;
#endif
#ifdef USE_BANDS
    linePosition = lineCenterPosition;
    lineNormal = normal;
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
    fragmentPrincipalStressIndex = principalStressIndex;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = stressLinePointData.vertexLineAppearanceOrder;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = stressLinePointData.vertexLineHierarchyLevel;
#endif
#ifdef USE_AMBIENT_OCCLUSION
    interpolationFactorLine = float(linePointIdx - linePointData.lineStartIndex);
    fragmentVertexIdUint = linePointData.lineStartIndex;//linePointIdx;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
    fragmentRotation = linePointData.vertexRotation;
#endif
    fragmentAttribute = linePointData.vertexAttribute;
    fragmentTangent = tangent;

    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
    fragmentNormal = vertexNormal;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
}
