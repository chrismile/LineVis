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
#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
layout(location = 16) out float phi;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
layout(location = 17) out float fragmentRotation;
#endif

#define M_PI 3.14159265358979323846

void main() {
    uint linePointIdx = gl_VertexIndex / NUM_TUBE_SUBDIVISIONS;
    uint circleIdx = gl_VertexIndex % NUM_TUBE_SUBDIVISIONS;
    LinePointData linePointData = linePoints[linePointIdx];

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
    const mat4 pvMatrix = pMatrix * vMatrix;

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

    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
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
