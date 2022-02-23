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

#ifdef USE_VULKAN_INTEROP
namespace vk {

class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;

}
#endif

}

struct TubeRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexAttributeBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;
    sgl::vk::BufferPtr vertexTangentBuffer;
    sgl::vk::BufferPtr vertexRotationBuffer; ///< Only for flow lines.
    sgl::vk::BufferPtr vertexPrincipalStressIndexBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexLineHierarchyLevelBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexLineAppearanceOrderBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexMajorStressBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexMediumStressBuffer; ///< Only for stress lines.
    sgl::vk::BufferPtr vertexMinorStressBuffer; ///< Only for stress lines.
};

struct BandRenderData {
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

/// For internal use of subclasses.
struct LinePointDataProgrammableFetch {
    glm::vec3 vertexPosition;
    float vertexAttribute;
    glm::vec3 vertexTangent;
    uint32_t principalStressIndex; ///< Padding in case of flow lines.
};

struct TubeRenderDataProgrammableFetch {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr linePointsBuffer;
    sgl::vk::BufferPtr lineHierarchyLevelsBuffer; ///< Empty for flow lines.
};

struct TubeRenderDataOpacityOptimization {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexAttributeBuffer;
    sgl::vk::BufferPtr vertexTangentBuffer;
    sgl::vk::BufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::vk::BufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
};

struct PointRenderData {
    sgl::vk::BufferPtr vertexPositionBuffer;
};

struct SimulationMeshOutlineRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;
};

struct TubeTriangleVertexData {
    glm::vec3 vertexPosition;
    uint32_t vertexLinePointIndex; ///< Pointer to TubeLinePointData entry.
    glm::vec3 vertexNormal;
    float phi; ///< Angle.
};

struct TubeLinePointData {
    glm::vec3 linePosition;
    float lineAttribute;
    glm::vec3 lineTangent;
    float lineHierarchyLevel; ///< Zero for flow lines.
    glm::vec3 lineNormal;
    float lineAppearanceOrder; ///< Zero for flow lines.
    glm::uvec3 padding;
    uint32_t principalStressIndex; ///< Zero for flow lines.
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

#ifdef USE_VULKAN_INTEROP
struct VulkanTubeTriangleRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexBuffer; // TubeTriangleVertexData objects.
    sgl::vk::BufferPtr linePointBuffer; // TubeLinePointData objects.
};
struct VulkanTubeAabbRenderData {
    sgl::vk::BufferPtr indexBuffer; // Two consecutive uint32_t indices map one AABB to two TubeLinePointData objects.
    sgl::vk::BufferPtr aabbBuffer; // VkAabbPositionsKHR objects.
    sgl::vk::BufferPtr linePointBuffer; // TubeLinePointData objects.
};
struct VulkanHullTriangleRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexBuffer; // HullTriangleVertexData objects.
};
#endif

#endif //LINEVIS_LINERENDERDATA_HPP
