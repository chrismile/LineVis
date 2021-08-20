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
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineAppearanceOrderBuffer; ///< Empty for flow lines.
};

struct BandRenderData {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexOffsetLeftBuffer;
    sgl::GeometryBufferPtr vertexOffsetRightBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineAppearanceOrderBuffer; ///< Empty for flow lines.
};

/// For internal use of subclasses.
struct LinePointDataProgrammableFetch {
    glm::vec3 vertexPosition;
    float vertexAttribute;
    glm::vec3 vertexTangent;
    uint32_t principalStressIndex; ///< Padding in case of flow lines.
};

struct TubeRenderDataProgrammableFetch {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr linePointsBuffer;
    sgl::GeometryBufferPtr lineHierarchyLevelsBuffer; ///< Empty for flow lines.
};

struct TubeRenderDataOpacityOptimization {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
};

struct PointRenderData {
    sgl::GeometryBufferPtr vertexPositionBuffer;
};

struct SimulationMeshOutlineRenderData {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
};

struct TubeTriangleVertexData {
    glm::vec3 vertexPosition;
    uint32_t vertexLinePointIndex; ///< Pointer to TubeTriangleLinePointData entry.
    glm::vec3 vertexNormal;
    float phi; ///< Angle.
};

struct TubeTriangleLinePointData {
    glm::vec3 lineTangent;
    float lineAttribute;
    glm::vec3 lineNormal;
    float lineHierarchyLevel; ///< Zero for flow lines.
    float lineAppearanceOrder; ///< Zero for flow lines.
    uint32_t principalStressIndex; ///< Zero for flow lines.
    float padding0, padding1;
};

struct LinePointReference {
    LinePointReference() = default;
    LinePointReference(uint32_t trajectoryIndex, uint32_t linePointIndex)
            : trajectoryIndex(trajectoryIndex), linePointIndex(linePointIndex) {}
    uint32_t trajectoryIndex = 0; ///< Index of the trajectory.
    uint32_t linePointIndex = 0; ///< Index of the line point within the trajectory.
};

#ifdef USE_VULKAN_INTEROP
struct VulkanTubeTriangleRenderData {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexBuffer; // TubeTriangleVertexData objects.
    sgl::vk::BufferPtr linePointBuffer; // TubeTriangleLinePointData objects.
};
#endif

#endif //LINEVIS_LINERENDERDATA_HPP
