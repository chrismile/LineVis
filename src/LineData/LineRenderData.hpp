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

#ifndef LINEVIS_LINERENDERDATA_HPP
#define LINEVIS_LINERENDERDATA_HPP

namespace sgl {

class GeometryBuffer;
typedef std::shared_ptr<GeometryBuffer> GeometryBufferPtr;

namespace vk {

class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;

}

}

// --- For quads ---
struct LinePassQuadsRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexAttributeBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;
    sgl::vk::BufferPtr vertexTangentBuffer;
    sgl::vk::BufferPtr vertexOffsetLeftBuffer;
    sgl::vk::BufferPtr vertexOffsetRightBuffer;
    sgl::vk::BufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::vk::BufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
    sgl::vk::BufferPtr vertexLineAppearanceOrderBuffer; ///< Empty for flow lines.
};

struct LinePassQuadsLinePointDataProgrammablePull {
    glm::vec3 vertexPosition;
    float vertexAttribute;
    glm::vec3 vertexTangent;
    uint32_t principalStressIndex; ///< Padding in case of flow lines.
};

struct LinePassQuadsRenderDataProgrammablePull {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr linePointsBuffer;
    sgl::vk::BufferPtr lineHierarchyLevelsBuffer; ///< Empty for flow lines.
};


// --- For tubes rendered from line input data.
/// For geometry shader.
struct LinePassTubeRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexAttributeBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;
    sgl::vk::BufferPtr vertexTangentBuffer;
    sgl::vk::BufferPtr vertexRotationBuffer; ///< Only for flow lines.
    sgl::vk::BufferPtr multiVarAttributeDataBuffer; ///< Only for flow lines with multi-var rendering mode.
    sgl::vk::BufferPtr vertexPrincipalStressIndexBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexLineHierarchyLevelBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexLineAppearanceOrderBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexMajorStressBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexMediumStressBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexMinorStressBuffer; ///< Only for stress lines.
};

/// For mesh shaders and programmable pull.
struct LinePointDataUnified {
    glm::vec3 linePosition;
    float lineAttribute;
    glm::vec3 lineTangent;
    float lineRotation = 0.0f;
    glm::vec3 lineNormal;
    uint32_t lineStartIndex = 0;
};

struct StressLinePointDataUnified {
    uint32_t linePrincipalStressIndex = 0;
    uint32_t lineLineAppearanceOrder = 0;
    float lineLineHierarchyLevel = 0.0f;
    float stressLinePointPadding = 0.0f;
};

struct StressLinePointPrincipalStressDataUnified {
    float lineMajorStress = 1.0f;
    float lineMediumStress = 1.0f;
    float lineMinorStress = 1.0f;
    float principalStressPadding = 0.0f;
};

struct MeshletData {
    uint32_t linePointIndexStart = 0;
    uint32_t numLinePoints = 0;
};

struct LinePassTubeRenderDataMeshShader {
    uint32_t numMeshlets = 0;
    sgl::vk::BufferPtr meshletDataBuffer;
    sgl::vk::BufferPtr linePointDataBuffer;
    sgl::vk::BufferPtr stressLinePointDataBuffer;
    sgl::vk::BufferPtr stressLinePointPrincipalStressDataBuffer;
    sgl::vk::BufferPtr multiVarAttributeDataBuffer; ///< Only for flow lines with multi-var rendering mode.
};

struct LinePassTubeRenderDataProgrammablePull {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr linePointDataBuffer;
    sgl::vk::BufferPtr stressLinePointDataBuffer;
    sgl::vk::BufferPtr stressLinePointPrincipalStressDataBuffer;
    sgl::vk::BufferPtr multiVarAttributeDataBuffer; ///< Only for flow lines with multi-var rendering mode.
};


// --- Reduced tube data for opacity optimization ---
struct TubeRenderDataOpacityOptimization {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexAttributeBuffer;
    sgl::vk::BufferPtr vertexTangentBuffer;
    sgl::vk::BufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::vk::BufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
};


// --- For point data ---
struct PointRenderData {
    sgl::vk::BufferPtr vertexPositionBuffer;
};


// --- For simulation outline triangle mesh ---
struct SimulationMeshOutlineRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;
};


// --- For hardware-accelerated ray tracing ---
struct TubeTriangleVertexData {
    glm::vec3 vertexPosition;
    uint32_t vertexLinePointIndex; ///< Pointer to LinePointDataUnified entry.
    glm::vec3 vertexNormal;
    float phi; ///< Angle.
};

struct LinePointReference {
    LinePointReference() = default;
    LinePointReference(uint32_t trajectoryIndex, uint32_t linePointIndex)
            : trajectoryIndex(trajectoryIndex), linePointIndex(linePointIndex) {}
    uint32_t trajectoryIndex = 0; ///< Index of the trajectory.
    uint32_t linePointIndex = 0; ///< Index of the line point within the trajectory.
};

struct HullTriangleVertexData {
    glm::vec3 vertexPosition;
    float padding0;
    glm::vec3 vertexNormal;
    float padding1;
};

struct TubeTriangleRenderData {
    sgl::vk::BufferPtr indexBuffer; // uvec3 objects.
    sgl::vk::BufferPtr vertexBuffer; // TubeTriangleVertexData objects.
    sgl::vk::BufferPtr linePointDataBuffer; // LinePointDataUnified objects.
    sgl::vk::BufferPtr stressLinePointDataBuffer; // StressLinePointDataUnified objects.
    sgl::vk::BufferPtr stressLinePointPrincipalStressDataBuffer; // StressLinePointPrincipalStressDataUnified objects.
    sgl::vk::BufferPtr multiVarAttributeDataBuffer; ///< Only for flow lines with multi-var rendering mode.
    // If the acceleration structure was split, the buffer below stores the triangle index offset of each instance.
    sgl::vk::BufferPtr instanceTriangleIndexOffsetBuffer; // uint32_t objects.
};
struct TubeAabbRenderData {
    sgl::vk::BufferPtr indexBuffer; // Two consecutive uint32_t indices map one AABB to two LinePointDataUnified objects.
    sgl::vk::BufferPtr aabbBuffer; // VkAabbPositionsKHR objects.
    sgl::vk::BufferPtr linePointDataBuffer; // LinePointDataUnified objects.
    sgl::vk::BufferPtr stressLinePointDataBuffer; // StressLinePointDataUnified objects.
    sgl::vk::BufferPtr stressLinePointPrincipalStressDataBuffer; // StressLinePointPrincipalStressDataUnified objects.
    sgl::vk::BufferPtr multiVarAttributeDataBuffer; ///< Only for flow lines with multi-var rendering mode.
};
struct HullTriangleRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexBuffer; // HullTriangleVertexData objects.
};

struct TubeTriangleSplitData {
    std::vector<uint32_t> numBatchIndices;
};

#endif //LINEVIS_LINERENDERDATA_HPP
