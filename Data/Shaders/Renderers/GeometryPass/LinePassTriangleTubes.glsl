-- Vertex

#version 450 core

#include "LineUniformData.glsl"
#include "LineDataSSBO.glsl"

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to LinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};

layout(std430, binding = TUBE_TRIANGLE_MESH_VERTEX_BUFFER_BINDING) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

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

#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
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
#ifdef USE_CAPPED_TUBES
layout(location = 19) out float isCap;
#endif

#define M_PI 3.14159265358979323846

void main() {
    TubeTriangleVertexData vertexData = tubeTriangleVertexDataBuffer[gl_VertexIndex];
    vec3 vertexPosition = vertexData.vertexPosition;
    vec3 vertexNormal = vertexData.vertexNormal;
    uint circleIdx = uint(round(vertexData.phi / (2.0 * M_PI / float(NUM_TUBE_SUBDIVISIONS))));

#ifdef USE_CAPPED_TUBES
    uint linePointIdx = vertexData.vertexLinePointIndex & 0x7FFFFFFFu;
    isCap = bitfieldExtract(vertexData.vertexLinePointIndex, 31, 1) > 0u ? 1.0 : 0.0;
#else
    uint linePointIdx = vertexData.vertexLinePointIndex;
#endif
    LinePointData linePointData = linePoints[linePointIdx];

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(VISUALIZE_SEEDING_PROCESS)
    StressLinePointData stressLinePointData = stressLinePoints[linePointIdx];
#endif

#ifdef USE_PRINCIPAL_STRESSES
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData = principalStressLinePoints[linePointIdx];
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
    uint principalStressIndex = stressLinePointData.linePrincipalStressIndex;
#endif

#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
    useBand = psUseBands[principalStressIndex];
#else
    useBand = 1;
#endif

#if !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
    thickness = useBand != 0 ? MIN_THICKNESS : 1.0f;
#endif
#endif

    vec3 lineCenterPosition = (mMatrix * vec4(linePointData.linePosition, 1.0)).xyz;

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
    thickness0 = useBand != 0 ? factorX : 1.0;
    thickness1 = useBand != 0 ? factorZ : 1.0;
#elif defined(USE_HYPERSTREAMLINES)
    stressX = max(abs(stressX), minimumHyperstreamlineWidth);
    stressZ = max(abs(stressZ), minimumHyperstreamlineWidth);
    thickness0 = useBand != 0 ? stressX : 1.0;
    thickness1 = useBand != 0 ? stressZ : 1.0;
#else
    thickness = useBand != 0 ? MIN_THICKNESS : 1.0;
#endif
#endif

#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    phi = vertexData.phi;
    /*
     * https://www.khronos.org/registry/vulkan/specs/1.3-extensions/html/vkspec.html#drawing-triangle-lists
     * The provoking vertex is the first vertex of the triangle. In order to wrap when interpolating between the last
     * and the first vertex (from pi to 0), interpolateWrap needs to be set to 1 for the final circle index.
     */
    interpolateWrap = circleIdx == NUM_TUBE_SUBDIVISIONS - 1 ? 1 : 0;
#endif
#ifdef USE_BANDS
    linePosition = lineCenterPosition;
    lineNormal = linePointData.lineNormal;
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
    fragmentPrincipalStressIndex = principalStressIndex;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    fragmentLineAppearanceOrder = stressLinePointData.lineLineAppearanceOrder;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentLineHierarchyLevel = stressLinePointData.lineLineHierarchyLevel;
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
    interpolationFactorLine = float(linePointIdx - linePointData.lineStartIndex);
    fragmentVertexIdUint = linePointData.lineStartIndex;//linePointIdx;
#endif

#if defined(USE_ROTATING_HELICITY_BANDS) && defined(USE_CAPPED_TUBES)
    float fragmentRotationAdapted = linePointData.lineRotation * helicityRotationFactor;
    if (bitfieldExtract(vertexData.vertexLinePointIndex, 31, 1) > 0u) {
        float fragmentRotationDelta = 0.0;
        float segmentLength = 1.0;
        vec3 planeNormal = vec3(0.0);
        LinePointData linePointDataOther;
        bool found = false;
        if (linePointIdx != 0) {
            linePointDataOther = linePoints[linePointIdx - 1];
            found = linePointDataOther.lineStartIndex == linePointData.lineStartIndex;
            fragmentRotationDelta =
                    (linePointData.lineRotation - linePointDataOther.lineRotation) * helicityRotationFactor;
            planeNormal = linePointData.linePosition - linePointDataOther.linePosition;
        }
        if (!found) {
            linePointDataOther = linePoints[linePointIdx + 1];
            fragmentRotationDelta =
                    (linePointData.lineRotation - linePointDataOther.lineRotation) * helicityRotationFactor;
            planeNormal = linePointData.linePosition - linePointDataOther.linePosition;
        }
        segmentLength = length(planeNormal);
        planeNormal /= segmentLength;
        //planeNormal = linePointData.lineTangent;
        float planeDist = -dot(planeNormal, linePointData.linePosition);
        float distToPlane = dot(planeNormal, vertexPosition) + planeDist;
        fragmentRotationAdapted += fragmentRotationDelta * distToPlane / segmentLength;
    }
    fragmentRotation = fragmentRotationAdapted;
#elif defined(USE_ROTATING_HELICITY_BANDS)
    fragmentRotation = linePointData.lineRotation * helicityRotationFactor;
#endif

    fragmentAttribute = linePointData.lineAttribute;
    fragmentTangent = linePointData.lineTangent;

    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
    fragmentNormal = vertexNormal;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
    screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
#endif
}
