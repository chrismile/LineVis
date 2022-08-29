/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

-- Vertex

#version 450 core

#ifdef DRAW_INDIRECT
// Host side adds extension GL_ARB_shader_draw_parameters.

layout(location = 0) flat out uint primitiveIDOffset;

struct VkDrawIndexedIndirectCommand {
    uint indexCount;
    uint instanceCount;
    uint firstIndex;
    int vertexOffset;
    uint firstInstance;
};
// Uses stride of 8 bytes, or scalar layout otherwise.
layout(std430, binding = 0) readonly buffer DrawIndexedIndirectCommandBuffer {
    VkDrawIndexedIndirectCommand commands[];
};
#endif

//struct TubeTriangleVertexData {
//    vec3 vertexPosition;
//    uint vertexLinePointIndex; ///< Pointer to LinePointData entry.
//    vec3 vertexNormal;
//    float phi; ///< Angle.
//};
//
//layout(std430, binding = TUBE_TRIANGLE_MESH_VERTEX_BUFFER_BINDING) readonly buffer TubeTriangleVertexDataBuffer {
//    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
//};

// Reads data from TubeTriangleVertexDataBuffer with stride sizeof(TubeTriangleVertexData).
layout(location = 0) in vec3 vertexPosition;

void main() {
#ifdef DRAW_INDIRECT
    primitiveIDOffset = uint(commands[gl_DrawIDARB].firstIndex / 3);
#endif

    //TubeTriangleVertexData vertexData = tubeTriangleVertexDataBuffer[gl_VertexIndex];
    //vec3 vertexPosition = vertexData.vertexPosition;

    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Vertex.ProgrammablePull

#version 450 core

#include "LineUniformData.glsl"
#include "LineDataSSBO.glsl"

#define M_PI 3.14159265358979323846

void main() {
    uint linePointIdx = gl_VertexIndex / NUM_TUBE_SUBDIVISIONS;
    uint circleIdx = gl_VertexIndex % NUM_TUBE_SUBDIVISIONS;
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
    int useBand = psUseBands[principalStressIndex];
#else
    int useBand = 1;
#endif

#if !defined(USE_NORMAL_STRESS_RATIO_TUBES) && !defined(USE_HYPERSTREAMLINES)
    float thickness = useBand != 0 ? MIN_THICKNESS : 1.0;
#endif

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
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
    vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
#elif defined(USE_HYPERSTREAMLINES)
    stressX = abs(stressX);
    stressZ = abs(stressZ);
    vec3 localPosition = vec3(cosAngle * stressX, sinAngle * stressZ, 0.0);
    vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
#else
    // Bands with minimum thickness.
    vec3 localPosition = vec3(thickness * cosAngle, sinAngle, 0.0);
    vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
#endif
#else
    vec3 localPosition = vec3(cosAngle, sinAngle, 0.0);
    vec3 vertexPosition = lineRadius * (tangentFrameMatrix * localPosition) + lineCenterPosition;
#endif

    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 450 core

#ifdef DRAW_INDIRECT
layout(location = 0) flat in uint primitiveIDOffset;
#endif

// Primitive index attachment uses clear value 0xFFFFFFFFu.
layout(location = 0) out uint fragmentPrimitiveIndex;

void main() {
#ifdef DRAW_INDIRECT
    fragmentPrimitiveIndex = uint(gl_PrimitiveID) + primitiveIDOffset;
#else
    fragmentPrimitiveIndex = uint(gl_PrimitiveID);
#endif
}
