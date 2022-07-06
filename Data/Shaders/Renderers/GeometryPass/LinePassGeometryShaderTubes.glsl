/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

-- VBO.Vertex

#version 450 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexTangent;
layout(location = 3) in vec3 vertexNormal;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 4) in uint vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 5) in float vertexLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 6) in uint vertexLineAppearanceOrder;
#endif
#ifdef USE_PRINCIPAL_STRESSES
layout(location = 7) in float vertexMajorStress;
layout(location = 8) in float vertexMediumStress;
layout(location = 9) in float vertexMinorStress;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
// Used for flow lines when the helicity should be used for showing rotated bands on the tube surface.
layout(location = 10) in float vertexRotation;
#endif

layout(location = 0) out vec3 linePositionIn;
layout(location = 1) out float lineAttribute;
layout(location = 2) out vec3 lineTangent;
layout(location = 3) out vec3 lineNormalIn;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 4) out uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 5) out float lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 6) out uint lineLineAppearanceOrder;
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
layout(location = 7) out uint lineVertexId;
#endif
#ifdef USE_PRINCIPAL_STRESSES
layout(location = 8) out float lineMajorStress;
layout(location = 9) out float lineMediumStress;
layout(location = 10) out float lineMinorStress;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
layout(location = 11) out float lineRotation;
#endif

#include "LineUniformData.glsl"
#include "TransferFunction.glsl"

void main() {
    linePositionIn = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
    lineNormalIn = vertexNormal;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
    linePrincipalStressIndex = vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    lineLineHierarchyLevel = vertexLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    lineLineAppearanceOrder = vertexLineAppearanceOrder;
#endif
#ifdef USE_PRINCIPAL_STRESSES
    lineMajorStress = vertexMajorStress;
    lineMediumStress = vertexMediumStress;
    lineMinorStress = vertexMinorStress;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
    lineRotation = vertexRotation * helicityRotationFactor;
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
    lineVertexId = uint(gl_VertexIndex);
#endif
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 450 core

layout(lines) in;
layout(triangle_strip, max_vertices = NUM_TUBE_SUBDIVISIONS * 4) out;

#include "LineUniformData.glsl"

layout(location = 0) in vec3 linePositionIn[];
layout(location = 1) in float lineAttribute[];
layout(location = 2) in vec3 lineTangent[];
layout(location = 3) in vec3 lineNormalIn[];
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 4) in uint linePrincipalStressIndex[];
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 5) in float lineLineHierarchyLevel[];
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 6) in uint lineLineAppearanceOrder[];
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
layout(location = 7) in uint lineVertexId[];
#endif
#ifdef USE_PRINCIPAL_STRESSES
layout(location = 8) in float lineMajorStress[];
layout(location = 9) in float lineMediumStress[];
layout(location = 10) in float lineMinorStress[];
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
layout(location = 11) in float lineRotation[];
#endif

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

#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && (defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)) && defined(VISUALIZE_SEEDING_PROCESS)
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 5) flat out uint fragmentPrincipalStressIndexAndLineAppearanceOrder;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 6) flat out float fragmentLineHierarchyLevel;
#endif
#else
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 5) flat out uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 6) flat out float fragmentLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 7) flat out uint fragmentLineAppearanceOrder;
#endif
#endif

#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
layout(location = 8) out float interpolationFactorLine;
layout(location = 9) flat out uint fragmentVertexIdUint;
#endif
#ifdef USE_BANDS
#ifdef COMPRESSED_GEOMETRY_OUTPUT_DATA
int useBand;
#else
layout(location = 10) flat out int useBand;
#endif
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
#endif

#define M_PI 3.14159265358979323846

void main() {
    vec3 linePosition0 = (mMatrix * vec4(linePositionIn[0], 1.0)).xyz;
    vec3 linePosition1 = (mMatrix * vec4(linePositionIn[1], 1.0)).xyz;

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
    uint principalStressIndex = linePrincipalStressIndex[0];
#endif

#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
    useBand = psUseBands[principalStressIndex];
#else
    useBand = 1;
#endif

#if !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
    thickness = useBand != 0 ? MIN_THICKNESS : 1.0;
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

    vec3 circlePointsCurrent[NUM_TUBE_SUBDIVISIONS];
    vec3 circlePointsNext[NUM_TUBE_SUBDIVISIONS];
    vec3 vertexNormalsCurrent[NUM_TUBE_SUBDIVISIONS];
    vec3 vertexNormalsNext[NUM_TUBE_SUBDIVISIONS];

    vec3 normalCurrent = lineNormalIn[0];
    vec3 tangentCurrent = lineTangent[0];
    vec3 binormalCurrent = cross(tangentCurrent, normalCurrent);
    vec3 normalNext = lineNormalIn[1];
    vec3 tangentNext = lineTangent[1];
    vec3 binormalNext = cross(tangentNext, normalNext);

    mat3 tangentFrameMatrixCurrent = mat3(normalCurrent, binormalCurrent, tangentCurrent);
    mat3 tangentFrameMatrixNext = mat3(normalNext, binormalNext, tangentNext);

#ifdef USE_BANDS
    for (int i = 0; i < NUM_TUBE_SUBDIVISIONS; i++) {
        float t = float(i) / float(NUM_TUBE_SUBDIVISIONS) * 2.0 * M_PI;
        float cosAngle = cos(t);
        float sinAngle = sin(t);

#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        float stressXCurrent;
        float stressZCurrent;
        float stressXNext;
        float stressZNext;
        if (principalStressIndex == 0) {
            stressXCurrent = lineMediumStress[0];
            stressZCurrent = lineMinorStress[0];
            stressXNext = lineMediumStress[1];
            stressZNext = lineMinorStress[1];
        } else if (principalStressIndex == 1) {
            stressXCurrent = lineMinorStress[0];
            stressZCurrent = lineMajorStress[0];
            stressXNext = lineMinorStress[1];
            stressZNext = lineMajorStress[1];
        } else {
            stressXCurrent = lineMediumStress[0];
            stressZCurrent = lineMajorStress[0];
            stressXNext = lineMediumStress[1];
            stressZNext = lineMajorStress[1];
        }
#endif

#if defined(USE_NORMAL_STRESS_RATIO_TUBES)
        float factorXCurrent = clamp(abs(stressXCurrent / stressZCurrent), 0.0, 1.0);
        float factorZCurrent = clamp(abs(stressZCurrent / stressXCurrent), 0.0, 1.0);
        float factorXNext = clamp(abs(stressXNext / stressZNext), 0.0, 1.0);
        float factorZNext = clamp(abs(stressZNext / stressXNext), 0.0, 1.0);
        vec3 localPositionCurrent = vec3(cosAngle * factorXCurrent, sinAngle * factorZCurrent, 0.0);
        vec3 localNormalCurrent = vec3(cosAngle * factorZCurrent, sinAngle * factorXCurrent, 0.0);
        vec3 localPositionNext = vec3(cosAngle * factorXNext, sinAngle * factorZNext, 0.0);
        vec3 localNormalNext = vec3(cosAngle * factorZNext, sinAngle * factorXNext, 0.0);
        circlePointsCurrent[i] = lineRadius * (tangentFrameMatrixCurrent * localPositionCurrent) + linePosition0;
        circlePointsNext[i] = lineRadius * (tangentFrameMatrixNext * localPositionNext) + linePosition1;
        vertexNormalsCurrent[i] = normalize(tangentFrameMatrixCurrent * localNormalCurrent);
        vertexNormalsNext[i] = normalize(tangentFrameMatrixNext * localNormalNext);
        thickness0Current[i] = useBand != 0 ? factorXCurrent : 1.0;
        thickness0Next[i] = useBand != 0 ? factorXNext : 1.0;
        thickness1Current[i] = useBand != 0 ? factorZCurrent : 1.0;
        thickness1Next[i] = useBand != 0 ? factorZNext : 1.0;
#elif defined(USE_HYPERSTREAMLINES)
        stressXCurrent = abs(stressXCurrent);
        stressZCurrent = abs(stressZCurrent);
        stressXNext = abs(stressXNext);
        stressZNext = abs(stressZNext);
        vec3 localPositionCurrent = vec3(cosAngle * stressXCurrent, sinAngle * stressZCurrent, 0.0);
        vec3 localNormalCurrent = vec3(cosAngle * stressZCurrent, sinAngle * stressXCurrent, 0.0);
        vec3 localPositionNext = vec3(cosAngle * stressXNext, sinAngle * stressZNext, 0.0);
        vec3 localNormalNext = vec3(cosAngle * stressZNext, sinAngle * stressXNext, 0.0);
        circlePointsCurrent[i] = lineRadius * (tangentFrameMatrixCurrent * localPositionCurrent) + linePosition0;
        circlePointsNext[i] = lineRadius * (tangentFrameMatrixNext * localPositionNext) + linePosition1;
        vertexNormalsCurrent[i] = normalize(tangentFrameMatrixCurrent * localNormalCurrent);
        vertexNormalsNext[i] = normalize(tangentFrameMatrixNext * localNormalNext);
        thickness0Current[i] = useBand != 0 ? stressXCurrent : 1.0;
        thickness0Next[i] = useBand != 0 ? stressXNext : 1.0;
        thickness1Current[i] = useBand != 0 ? stressZCurrent : 1.0;
        thickness1Next[i] = useBand != 0 ? stressZNext : 1.0;
#else
        // Bands with minimum thickness.
        vec3 localPosition = vec3(thickness * cosAngle, sinAngle, 0.0);
        vec3 localNormal = vec3(cosAngle, thickness * sinAngle, 0.0);
        circlePointsCurrent[i] = lineRadius * (tangentFrameMatrixCurrent * localPosition) + linePosition0;
        circlePointsNext[i] = lineRadius * (tangentFrameMatrixNext * localPosition) + linePosition1;
        vertexNormalsCurrent[i] = normalize(tangentFrameMatrixCurrent * localNormal);
        vertexNormalsNext[i] = normalize(tangentFrameMatrixNext * localNormal);
#endif
    }
#else
    // Tubes with circular profile.
    const float theta = 2.0 * M_PI / float(NUM_TUBE_SUBDIVISIONS);
    const float tangetialFactor = tan(theta); // opposite / adjacent
    const float radialFactor = cos(theta); // adjacent / hypotenuse

    vec2 position = vec2(lineRadius, 0.0);
    for (int i = 0; i < NUM_TUBE_SUBDIVISIONS; i++) {
        vec3 point2D0 = tangentFrameMatrixCurrent * vec3(position, 0.0);
        vec3 point2D1 = tangentFrameMatrixNext * vec3(position, 0.0);
        circlePointsCurrent[i] = point2D0.xyz + linePosition0;
        circlePointsNext[i] = point2D1.xyz + linePosition1;
        vertexNormalsCurrent[i] = normalize(circlePointsCurrent[i] - linePosition0);
        vertexNormalsNext[i] = normalize(circlePointsNext[i] - linePosition1);

        // Add the tangent vector and correct the position using the radial factor.
        vec2 circleTangent = vec2(-position.y, position.x);
        position += tangetialFactor * circleTangent;
        position *= radialFactor;
    }
#endif


#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    const float factor = 2.0 * M_PI / float(NUM_TUBE_SUBDIVISIONS);
#endif

    // Emit the tube triangle vertices
    for (int i = 0; i < NUM_TUBE_SUBDIVISIONS; i++) {
        int iNext = (i+1)%NUM_TUBE_SUBDIVISIONS;

#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
        phi = float(i) * factor;
#endif
#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
        useBand = psUseBands[principalStressIndex];
#else
        useBand = 1;
#endif
#endif
#if defined(USE_BANDS) && !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
        thickness = useBand != 0 ? MIN_THICKNESS : 1.0;
#endif
#if defined(USE_BANDS) && (defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES))
        thickness0 = thickness0Current[i];
        thickness1 = thickness1Current[i];
#endif
#ifdef USE_BANDS
        linePosition = linePosition0;
        lineNormal = normalCurrent;
#endif

#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && (defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)) && defined(VISUALIZE_SEEDING_PROCESS)
        fragmentPrincipalStressIndexAndLineAppearanceOrder = principalStressIndex | lineLineAppearanceOrder[0] << 2;
#else
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
        fragmentPrincipalStressIndex = principalStressIndex;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
        fragmentLineAppearanceOrder = lineLineAppearanceOrder[0];
#endif
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
        fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
        interpolationFactorLine = 0.0;
        fragmentVertexIdUint = lineVertexId[0];
        //fragmentVertexId = float(lineVertexId[0]);
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
        fragmentRotation = lineRotation[0] * helicityRotationFactor;
#endif
        fragmentAttribute = lineAttribute[0];
        fragmentTangent = tangentCurrent;

        gl_Position = vpMatrix * vec4(circlePointsCurrent[i], 1.0);
        fragmentNormal = vertexNormalsCurrent[i];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsCurrent[i], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsCurrent[i], 1.0)).xyz;
#endif
        EmitVertex();

#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
        phi = float(i + 1) * factor;
#endif
#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
        useBand = psUseBands[principalStressIndex];
#else
        useBand = 1;
#endif
#endif
#if defined(USE_BANDS) && !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
        thickness = useBand != 0 ? MIN_THICKNESS : 1.0;
#endif
#if defined(USE_BANDS) && (defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES))
        thickness0 = thickness0Current[iNext];
        thickness1 = thickness1Current[iNext];
#endif
#ifdef USE_BANDS
        linePosition = linePosition0;
        lineNormal = normalCurrent;
#endif

#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && (defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)) && defined(VISUALIZE_SEEDING_PROCESS)
        fragmentPrincipalStressIndexAndLineAppearanceOrder = principalStressIndex | lineLineAppearanceOrder[0] << 2;
#else
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
        fragmentPrincipalStressIndex = principalStressIndex;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
        fragmentLineAppearanceOrder = lineLineAppearanceOrder[0];
#endif
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
        fragmentLineHierarchyLevel = lineLineHierarchyLevel[0];
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
        interpolationFactorLine = 0.0;
        fragmentVertexIdUint = lineVertexId[0];
        //fragmentVertexId = float(lineVertexId[0]);
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
        fragmentRotation = lineRotation[0] * helicityRotationFactor;
#endif
        fragmentAttribute = lineAttribute[0];
        fragmentTangent = tangentCurrent;

        gl_Position = vpMatrix * vec4(circlePointsCurrent[iNext], 1.0);
        fragmentNormal = vertexNormalsCurrent[iNext];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsCurrent[iNext], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsCurrent[iNext], 1.0)).xyz;
#endif
        EmitVertex();


#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
        phi = float(i) * factor;
#endif
#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
        useBand = psUseBands[principalStressIndex];
#else
        useBand = 1;
#endif
#endif
#if defined(USE_BANDS) && !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
        thickness = useBand != 0 ? MIN_THICKNESS : 1.0;
#endif
#if defined(USE_BANDS) && (defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES))
        thickness0 = thickness0Next[i];
        thickness1 = thickness1Next[i];
#endif
#ifdef USE_BANDS
        linePosition = linePosition1;
        lineNormal = normalNext;
#endif

#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && (defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)) && defined(VISUALIZE_SEEDING_PROCESS)
        fragmentPrincipalStressIndexAndLineAppearanceOrder = principalStressIndex | lineLineAppearanceOrder[0] << 2;
#else
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
        fragmentPrincipalStressIndex = principalStressIndex;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
        fragmentLineAppearanceOrder = lineLineAppearanceOrder[0];
#endif
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
        fragmentLineHierarchyLevel = lineLineHierarchyLevel[1];
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
        interpolationFactorLine = 1.0;
        fragmentVertexIdUint = lineVertexId[1];
        //fragmentVertexId = float(lineVertexId[1]);
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
        fragmentRotation = lineRotation[1] * helicityRotationFactor;
#endif
        fragmentAttribute = lineAttribute[1];
        fragmentTangent = tangentNext;

        gl_Position = vpMatrix * vec4(circlePointsNext[i], 1.0);
        fragmentNormal = vertexNormalsNext[i];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsNext[i], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsNext[i], 1.0)).xyz;
#endif
        EmitVertex();

#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
        phi = float(i + 1) * factor;
#endif
#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
        useBand = psUseBands[principalStressIndex];
#else
        useBand = 1;
#endif
#endif
#if defined(USE_BANDS) && !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
        thickness = useBand != 0 ? MIN_THICKNESS : 1.0;
#endif
#if defined(USE_BANDS) && (defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES))
        thickness0 = thickness0Next[iNext];
        thickness1 = thickness1Next[iNext];
#endif
#ifdef USE_BANDS
        linePosition = linePosition1;
        lineNormal = normalNext;
#endif

#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && (defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)) && defined(VISUALIZE_SEEDING_PROCESS)
        fragmentPrincipalStressIndexAndLineAppearanceOrder = principalStressIndex | lineLineAppearanceOrder[0] << 2;
#else
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
        fragmentPrincipalStressIndex = principalStressIndex;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
        fragmentLineAppearanceOrder = lineLineAppearanceOrder[0];
#endif
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
        fragmentLineHierarchyLevel = lineLineHierarchyLevel[1];
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
        interpolationFactorLine = 1.0;
        fragmentVertexIdUint = lineVertexId[1];
        //fragmentVertexId = float(lineVertexId[1]);
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
        fragmentRotation = lineRotation[1] * helicityRotationFactor;
#endif
        fragmentAttribute = lineAttribute[1];
        fragmentTangent = tangentNext;

        gl_Position = vpMatrix * vec4(circlePointsNext[iNext], 1.0);
        fragmentNormal = vertexNormalsNext[iNext];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsNext[iNext], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsNext[iNext], 1.0)).xyz;
#endif
        EmitVertex();

        EndPrimitive();
    }
}

-- Fragment

#version 450 core
#extension GL_EXT_scalar_block_layout : require
// for DEBUG_MESHLETS
//#extension GL_NV_fragment_shader_barycentric : require

#include "LineUniformData.glsl"

layout(location = 0) in vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
layout(location = 1) in vec3 screenSpacePosition;
#endif
layout(location = 2) in float fragmentAttribute;
layout(location = 3) in vec3 fragmentNormal;
layout(location = 4) in vec3 fragmentTangent;

/*
 * maxGeometryTotalOutputComponents is 1024 on NVIDIA hardware. When using stress line bands and ambient occlusion,
 * this value is exceeded for NUM_TUBE_SUBDIVISIONS = 8. Thus, we need to merge some attributes in this case.
 */
#if NUM_TUBE_SUBDIVISIONS >= 8 && defined(USE_AMBIENT_OCCLUSION) && defined(USE_BANDS) && defined(USE_GEOMETRY_SHADER)
#define COMPRESSED_GEOMETRY_OUTPUT_DATA
#endif

#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && (defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)) && defined(VISUALIZE_SEEDING_PROCESS)
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 5) flat in uint fragmentPrincipalStressIndexAndLineAppearanceOrder;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 6) flat in float fragmentLineHierarchyLevel;
#endif
#else
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 5) flat in uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 6) flat in float fragmentLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 7) flat in uint fragmentLineAppearanceOrder;
#endif
#endif

#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
layout(location = 8) in float interpolationFactorLine;
layout(location = 9) flat in uint fragmentVertexIdUint;
float fragmentVertexId;
#endif

#ifdef USE_BANDS
#ifdef COMPRESSED_GEOMETRY_OUTPUT_DATA
int useBand;
#else
layout(location = 10) flat in int useBand;
#endif
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
layout(location = 11) in float thickness0;
layout(location = 12) in float thickness1;
#else
layout(location = 13) in float thickness;
#endif
//in mat3 tangentFrameMatrix;
layout(location = 14) in vec3 lineNormal;
layout(location = 15) in vec3 linePosition;
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
layout(location = 16) in float fragmentRotation;
#endif
#if defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
#if defined(USE_GEOMETRY_SHADER)
layout(location = 17) in float phi;
#else
float phi;
layout(location = 17) in float phiNotWrapInterpolated;
layout(location = 18) flat in int interpolateWrap;
#endif
#endif
#ifdef USE_CAPPED_TUBES
layout(location = 19) in float isCap;
#endif

//#define DEBUG_MESHLETS
#ifdef DEBUG_MESHLETS
layout(location = 20) flat in uint fragmentMeshletIdx;
#endif

#if defined(DIRECT_BLIT_GATHER)
layout(location = 0) out vec4 fragColor;
#endif

#if defined(USE_LINE_HIERARCHY_LEVEL) && defined(USE_TRANSPARENCY)
layout(binding = LINE_HIERARCHY_IMPORTANCE_MAP_BINDING) uniform sampler1DArray lineHierarchyImportanceMap;
#endif

#define M_PI 3.14159265358979323846

#include "TransferFunction.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#define GEOMETRY_PASS_TUBE
#include "DepthHelper.glsl"
#include "Lighting.glsl"
#include "Antialiasing.glsl"
#include "MultiVar.glsl"

#ifdef UNIFORM_HELICITY_BAND_WIDTH
#ifdef USE_GEOMETRY_SHADER
layout(scalar, binding = LINE_POINTS_BUFFER_BINDING) readonly buffer LinePositionsBuffer {
    vec3 linePositions[];
};
layout(std430, binding = STRESS_LINE_POINTS_BUFFER_BINDING) readonly buffer LineRotationsBuffer {
    float lineRotations[];
};
#else
#include "LineDataSSBO.glsl"
#endif
#endif

//#define USE_ORTHOGRAPHIC_TUBE_PROJECTION

#ifndef USE_ORTHOGRAPHIC_TUBE_PROJECTION
mat3 shearSymmetricMatrix(vec3 p) {
    return mat3(vec3(0.0, -p.z, p.y), vec3(p.z, 0.0, -p.x), vec3(-p.y, p.x, 0.0));
}
#endif


#if defined(USE_ROTATING_HELICITY_BANDS) || defined(USE_MULTI_VAR_RENDERING)
void drawSeparatorStripe(inout vec4 surfaceColor, in float varFraction, in float globalPos, in float separatorWidth) {
    float aaf = fwidth(globalPos);
    float alphaBorder1 = smoothstep(aaf, 0.0, varFraction);
    float alphaBorder2 = smoothstep(separatorWidth - aaf * 0.5, separatorWidth + aaf * 0.5, varFraction);
    surfaceColor.rgb = surfaceColor.rgb * max(alphaBorder1, alphaBorder2);
}
#endif

#ifdef USE_HELICITY_BANDS_TEXTURE
layout(binding = HELICITY_BANDS_TEXTURE_BINDING) uniform sampler2D helicityBandsTexture;
vec4 sampleHelicityBandsTexture(in float u, in float globalPos) {
    //return texture(helicityBandsTexture, vec2(u, 0.5));
    return textureGrad(helicityBandsTexture, vec2(u, 0.5), vec2(dFdx(globalPos), 0.0), vec2(dFdy(globalPos), 0.0));
}
#endif

void main() {
#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && (defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)) && defined(VISUALIZE_SEEDING_PROCESS)
    uint fragmentPrincipalStressIndex = fragmentPrincipalStressIndexAndLineAppearanceOrder & 0x4u;
    uint fragmentLineAppearanceOrder = fragmentPrincipalStressIndexAndLineAppearanceOrder >> 2u;
#endif
#if defined(COMPRESSED_GEOMETRY_OUTPUT_DATA) && defined(USE_BANDS)
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(IS_PSL_DATA) || defined(USE_LINE_HIERARCHY_LEVEL)
    useBand = psUseBands[fragmentPrincipalStressIndex];
#else
    useBand = 1;
#endif
#endif

#if defined(USE_LINE_HIERARCHY_LEVEL) && !defined(USE_TRANSPARENCY)
    float slider = lineHierarchySlider[fragmentPrincipalStressIndex];
    if (slider > fragmentLineHierarchyLevel) {
        discard;
    }
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    if (int(fragmentLineAppearanceOrder) > currentSeedIdx) {
        discard;
    }
#endif

#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING) || defined(UNIFORM_HELICITY_BAND_WIDTH)
    fragmentVertexId = interpolationFactorLine + float(fragmentVertexIdUint);
#endif

#if (defined(USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)) && !defined(USE_GEOMETRY_SHADER)
    if (interpolateWrap == 0) {
        phi = phiNotWrapInterpolated;
    } else {
        /*
         * https://www.khronos.org/registry/vulkan/specs/1.3-extensions/html/vkspec.html#drawing-triangle-lists
         * The provoking vertex is the first vertex of the triangle. In order to wrap when interpolating between the last
         * and the first vertex (from pi to 0), interpolateWrap needs to be set to 1 for the final circle index.
         */
        float lower = 2.0 * M_PI * float(NUM_TUBE_SUBDIVISIONS - 1) / float(NUM_TUBE_SUBDIVISIONS);
        float upper = 2.0 * M_PI;
        phi = lower + (phiNotWrapInterpolated - lower) / (-lower) * (upper - lower);
    }
#endif

    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 t = normalize(fragmentTangent);
    // Project v into plane perpendicular to t to get newV.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));

#if defined(USE_HALOS) || defined(USE_MULTI_VAR_RENDERING)
    float ribbonPosition;

#ifdef USE_CAPPED_TUBES
    if (isCap > 1e-6) {
        vec3 crossProdVn = cross(v, n);
        ribbonPosition = length(crossProdVn);
    
        // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
        // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
        vec3 crossProdVn2 = cross(newV, n);
        float ribbonPosition2 = length(crossProdVn2);
    
        // Side note: We can also use the code below, as for a, b with length 1:
        // sqrt(1 - dot^2(a,b)) = len(cross(a,b))
        // Due to:
        // - dot(a,bribbonPosition) = ||a|| ||b|| cos(phi)
        // - len(cross(a,b)) = ||a|| ||b|| |sin(phi)|
        // - sin^2(phi) + cos^2(phi) = 1
        //ribbonPosition = dot(newV, n);
        //ribbonPosition = sqrt(1 - ribbonPosition * ribbonPosition);
    
        // Get the winding of newV relative to n, taking into account that t is the normal of the plane both vectors lie in.
        // NOTE: dot(a, cross(b, c)) = det(a, b, c), which is the signed volume of the parallelepiped spanned by a, b, c.
        if (dot(t, crossProdVn) < 0.0) {
            ribbonPosition2 = -ribbonPosition2;
        }
        // Normalize the ribbon position: [-1, 1] -> [0, 1].
        //ribbonPosition = ribbonPosition / 2.0 + 0.5;
        ribbonPosition2 = clamp(ribbonPosition2, -1.0, 1.0);
    
        ribbonPosition = min(ribbonPosition, abs(ribbonPosition2));
    } else {
#endif

#ifdef USE_BANDS
        vec3 lineN = normalize(lineNormal);
        vec3 lineB = cross(t, lineN);
        mat3 tangentFrameMatrix = mat3(lineN, lineB, t);
    
#ifdef USE_ORTHOGRAPHIC_TUBE_PROJECTION
        vec2 localV = normalize((transpose(tangentFrameMatrix) * newV).xy);
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        vec2 p = vec2(thickness0 * cos(phi), thickness1 * sin(phi));
#else
        vec2 p = vec2(thickness * cos(phi), sin(phi));
#endif
        float d = length(p);
        p = normalize(p);
        float alpha = acos(dot(localV, p));
    
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        float phiMax = atan(thickness1 * localV.x, -thickness0 * localV.y);
        vec2 pointMax0 = vec2(thickness0 * cos(phiMax), thickness1 * sin(phiMax));
        vec2 pointMax1 = vec2(thickness0 * cos(phiMax + M_PI), thickness1 * sin(phiMax + M_PI));
#else
        float phiMax = atan(localV.x, -thickness * localV.y);
        vec2 pointMax0 = vec2(thickness * cos(phiMax), sin(phiMax));
        vec2 pointMax1 = vec2(thickness * cos(phiMax + M_PI), sin(phiMax + M_PI));
#endif
    
        vec2 planeDir = pointMax1 - pointMax0;
        float totalDist = length(planeDir);
        planeDir = normalize(planeDir);
    
        float beta = acos(dot(planeDir, localV));
    
        float x = d / sin(beta) * sin(alpha);
        ribbonPosition = x / totalDist * 2.0;
#else
        // Project onto the tangent plane.
        const vec3 cNorm = cameraPosition - linePosition;
        const float dist = dot(cNorm, fragmentTangent);
        const vec3 cHat = transpose(tangentFrameMatrix) * (cNorm - dist * fragmentTangent);
        const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
    
        // Homogeneous, normalized coordinates of the camera position in the tangent plane.
        const vec3 c = vec3(cHat.xy / lineRadius, 1.0);
    
        // Primal conic section matrix.
        //const mat3 A = mat3(
        //        1.0 / (thickness * thickness), 0.0, 0.0,
        //        0.0, 1.0, 0.0,
        //        0.0, 0.0, -1.0
        //);
    
        // Polar of c.
        //const vec3 l = A * c;
    
        // Polar of c.
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        const float a = 1.0 / (thickness0 * thickness0);
        const float b = 1.0 / (thickness1 * thickness1);
        const vec3 l = vec3(a * c.x, b * c.y, -1.0);
#else
        const float a = 1.0 / (thickness * thickness);
        const vec3 l = vec3(a * c.x, c.y, -1.0);
#endif
    
        const mat3 M_l = shearSymmetricMatrix(l);
        //const mat3 B = transpose(M_l) * A * M_l;
    
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        const mat3 B = mat3(
            b*l.z*l.z - l.y*l.y, l.x*l.y, -b*l.x*l.z,
            l.x*l.y, a*l.z*l.z - l.x*l.x, -a*l.y*l.z,
            -b*l.x*l.z, -a*l.y*l.z, a*l.y*l.y + b*l.x*l.x
        );
#else
        const mat3 B = mat3(
            l.z*l.z - l.y*l.y, l.x*l.y, -l.x*l.z,
            l.x*l.y, a*l.z*l.z - l.x*l.x, -a*l.y*l.z,
            -l.x*l.z, -a*l.y*l.z, a*l.y*l.y + l.x*l.x
        );
#endif
    
        const float EPSILON = 1e-4;
        float alpha = 0.0;
        float discr = 0.0;
        if (abs(l.z) > EPSILON) {
            discr = -B[0][0] * B[1][1] + B[0][1] * B[1][0];
            alpha = sqrt(discr) / l.z;
        } else if (abs(l.y) > EPSILON) {
            discr = -B[0][0] * B[2][2] + B[0][2] * B[2][0];
            alpha = sqrt(discr) / l.y;
        } else if (abs(l.x) > EPSILON) {
            discr = -B[1][1] * B[2][2] + B[1][2] * B[2][1];
            alpha = sqrt(discr) / l.x;
        }
    
        mat3 C = B + alpha * M_l;
    
        vec2 pointMax0 = vec2(0.0);
        vec2 pointMax1 = vec2(0.0);
        for (int i = 0; i < 2; ++i) {
            if (abs(C[i][i]) > EPSILON) {
                pointMax0 = C[i].xy / C[i].z; // column vector
                pointMax1 = vec2(C[0][i], C[1][i]) / C[2][i]; // row vector
            }
        }
    
#if defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES)
        vec2 p = vec2(thickness0 * cos(phi), thickness1 * sin(phi));
#else
        vec2 p = vec2(thickness * cos(phi), sin(phi));
#endif
    
        vec3 pLineHomogeneous = cross(l, cross(c, vec3(p, 1.0)));
        vec2 pLine = pLineHomogeneous.xy / pLineHomogeneous.z;
    
        ribbonPosition = length(pLine - pointMax0) / length(pointMax1 - pointMax0) * 2.0 - 1.0;
#endif
    
#else
        // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
        // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
        vec3 crossProdVn = cross(newV, n);
        ribbonPosition = length(crossProdVn);
    
        // Side note: We can also use the code below, as for a, b with length 1:
        // sqrt(1 - dot^2(a,b)) = len(cross(a,b))
        // Due to:
        // - dot(a,b) = ||a|| ||b|| cos(phi)
        // - len(cross(a,b)) = ||a|| ||b|| |sin(phi)|
        // - sin^2(phi) + cos^2(phi) = 1
        //ribbonPosition = dot(newV, n);
        //ribbonPosition = sqrt(1 - ribbonPosition * ribbonPosition);
    
        // Get the winding of newV relative to n, taking into account that t is the normal of the plane both vectors lie in.
        // NOTE: dot(a, cross(b, c)) = det(a, b, c), which is the signed volume of the parallelepiped spanned by a, b, c.
        if (dot(t, crossProdVn) < 0.0) {
            ribbonPosition = -ribbonPosition;
        }
        // Normalize the ribbon position: [-1, 1] -> [0, 1].
        //ribbonPosition = ribbonPosition / 2.0 + 0.5;
        ribbonPosition = clamp(ribbonPosition, -1.0, 1.0);
#endif
    
#ifdef USE_CAPPED_TUBES
    }
#endif

#if !defined(USE_CAPPED_TUBES) && defined(USE_BANDS) && (defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES))
    if (useBand != 0 && dot(n, v) < 0.0) {
        ribbonPosition = 0.0;
    }
#endif

#endif // defined(USE_HALOS) || defined(USE_MULTI_VAR_RENDERING)


#ifdef USE_ROTATING_HELICITY_BANDS
#ifdef USE_MULTI_VAR_RENDERING
    float varFraction = mod(phi + fragmentRotation, 2.0 / float(numSubdivisionsBands) * float(M_PI));
#else
    float varFraction = mod(phi + fragmentRotation, 2.0 / float(numSubdivisionsBands) * float(M_PI));
#endif
#elif defined(USE_MULTI_VAR_RENDERING)
    int numSubdivisionsView = int(numSelectedAttributes);
    float varFraction = mod((ribbonPosition * 0.5 + 0.5) * float(numSubdivisionsView), 1.0);
#endif

#ifdef USE_MULTI_VAR_RENDERING
    vec4 fragmentColor = vec4(vec3(0.5), 1.0);
    if (numSelectedAttributes > 0u) {
#ifdef USE_ROTATING_HELICITY_BANDS
        uint attributeIdx = uint(mod((phi + fragmentRotation) * 0.5  / float(M_PI), 1.0) * float(numSubdivisionsBands)) % numSelectedAttributes;
#else
        uint attributeIdx = uint((ribbonPosition * 0.5 + 0.5) * float(numSubdivisionsView)) % numSelectedAttributes;
#endif
        uint attributeIdxReal = getRealAttributeIndex(attributeIdx);
        float sampledFragmentAttribute = sampleAttributeLinear(fragmentVertexId, attributeIdxReal);
        fragmentColor = transferFunction(sampledFragmentAttribute, attributeIdxReal);
    }
#else
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    vec4 fragmentColor = transferFunction(fragmentAttribute, fragmentPrincipalStressIndex);
#else
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#endif
#endif

#if defined(USE_LINE_HIERARCHY_LEVEL) && defined(USE_TRANSPARENCY)
    fragmentColor.a *= texture(
            lineHierarchyImportanceMap, vec2(fragmentLineHierarchyLevel, float(fragmentPrincipalStressIndex))).r;
#endif

    fragmentColor = blinnPhongShadingTube(fragmentColor, n, t);

#ifdef USE_ROTATING_HELICITY_BANDS
    float separatorWidth = separatorBaseWidth;
#ifdef UNIFORM_HELICITY_BAND_WIDTH
    uint vertexIdx0 = uint(floor(fragmentVertexId));
    uint vertexIdx1 = vertexIdx0 + 1;//uint(ceil(fragmentVertexId));
#ifdef USE_GEOMETRY_SHADER
    float rotDx = length(linePositions[vertexIdx1] - linePositions[vertexIdx0]);
    float rotDy = (lineRotations[vertexIdx1] - lineRotations[vertexIdx0]) * helicityRotationFactor;
#else
    LinePointData linePointData0 = linePoints[vertexIdx0];
    LinePointData linePointData1 = linePoints[vertexIdx1];
    float rotDx = length(linePointData1.linePosition - linePointData0.linePosition);
    float rotDy = (linePointData1.lineRotation - linePointData0.lineRotation) * helicityRotationFactor;
#endif
    // Space conversion world <-> surface: circumference / arc length == M_PI * lineWidth / (2.0 * M_PI) == 0.5 * lineWidth
    float rotationSeparatorScale = cos(atan(rotDy * 0.5 * lineWidth, rotDx));
    separatorWidth = separatorWidth / rotationSeparatorScale;
#endif

#ifdef USE_MULTI_VAR_RENDERING
    drawSeparatorStripe(
            fragmentColor, mod(phi + fragmentRotation + 0.1, 2.0 / float(numSubdivisionsBands) * float(M_PI)),
            phi + fragmentRotation, separatorWidth);
    //drawSeparatorStripe(fragmentColor, varFraction, phi + fragmentRotation, bandWidth);
#else
#ifdef USE_HELICITY_BANDS_TEXTURE
    const float twoPi = 2.0 * float(M_PI);
    fragmentColor *= sampleHelicityBandsTexture(
            mod(phi + fragmentRotation + 0.1, twoPi) / twoPi,
            (phi + fragmentRotation) / twoPi);
#else
    drawSeparatorStripe(
            fragmentColor, mod(phi + fragmentRotation + 0.1, 2.0 / float(numSubdivisionsBands) * float(M_PI)),
            phi + fragmentRotation, separatorWidth);
#endif
#endif
#elif defined(USE_MULTI_VAR_RENDERING)
    float separatorWidth = numSelectedAttributes > 1 ? 0.4 / float(numSelectedAttributes) : 0.2;
    if (numSelectedAttributes > 0) {
        drawSeparatorStripe(
                fragmentColor, mod((ribbonPosition * 0.5 + 0.5) * float(numSubdivisionsView) + 0.5 * separatorWidth, 1.0),
                (ribbonPosition * 0.5 + 0.5) * float(numSubdivisionsView), separatorWidth);
    }
#endif

#if defined(USE_HALOS) || defined(USE_MULTI_VAR_RENDERING)
    float absCoords = abs(ribbonPosition);
#else
    float absCoords = 0.0;
#endif

    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
#if defined(USE_ROTATING_HELICITY_BANDS)
    const float WHITE_THRESHOLD = 0.8;
#elif defined(USE_MULTI_VAR_RENDERING)
    const float WHITE_THRESHOLD = max(1.0 - separatorWidth, 0.8);
#else
    const float WHITE_THRESHOLD = 0.7;
#endif

    float EPSILON_OUTLINE = 0.0;
#if defined(USE_HALOS) || defined(USE_MULTI_VAR_RENDERING)
#ifdef USE_BANDS
    //float EPSILON_OUTLINE = clamp(getAntialiasingFactor(fragmentDistance / (useBand != 0 ? bandWidth : lineWidth) * 2.0), 0.0, 0.49);
    float EPSILON_WHITE = fwidth(ribbonPosition);
#else
    //float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / lineWidth, 0.0, 0.49);
    float EPSILON_WHITE = fwidth(ribbonPosition);
#endif
    float coverage = 1.0 - smoothstep(1.0 - EPSILON_OUTLINE, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(ribbonPosition));
#else
    float EPSILON_WHITE = 0.0;
    float coverage = 1.0;
#endif

#if !defined(USE_CAPPED_TUBES) && defined(USE_BANDS) && (defined(USE_NORMAL_STRESS_RATIO_TUBES) || defined(USE_HYPERSTREAMLINES))
    if (useBand != 0) {
        coverage = 1.0;
    }
#endif
    vec4 colorOut = vec4(
            mix(fragmentColor.rgb, foregroundColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON_WHITE, WHITE_THRESHOLD + EPSILON_WHITE, absCoords)),
            fragmentColor.a * coverage);

#ifdef DEBUG_MESHLETS
    vec3 colorMap[2] = {
        vec3(0.6196078431372549, 0.792156862745098, 0.8823529411764706),
        vec3(0.19215686274509805, 0.5098039215686274, 0.7411764705882353),
    };
    colorOut.rgb = colorMap[fragmentMeshletIdx % 2];
    vec3 baryCoord = gl_BaryCoordNV;
    vec3 derivatives = fwidth(baryCoord);
    vec3 thickness = derivatives * 0.75;
    baryCoord = smoothstep(thickness, thickness + derivatives, baryCoord);
    float minBaryCoord = min(baryCoord.x, min(baryCoord.y, baryCoord.z));
    colorOut.rgb *= minBaryCoord;
    if (fragmentPositionWorld.z < -0.2) {
        discard;
    }
    float depthCueFactor = clamp((-screenSpacePosition.z - minDepth) / (maxDepth - minDepth), 0.0, 1.0);
    depthCueFactor = depthCueFactor * depthCueFactor * depthCueStrength;
    colorOut.rgb = mix(colorOut.rgb, vec3(0.5, 0.5, 0.5), depthCueFactor * 4.0);
#endif

#include "LinePassGather.glsl"
}
