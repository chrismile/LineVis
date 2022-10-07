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

#ifndef LINEVIS_NODESBVHTREEPAYLOAD_HPP
#define LINEVIS_NODESBVHTREEPAYLOAD_HPP

#include "Renderers/Deferred/DeferredModes.hpp"
#include "../LineRenderData.hpp"

struct BVHTreeNode {
    glm::vec3 worldSpaceAabbMin{};
    uint32_t indexCount = 0;
    glm::vec3 worldSpaceAabbMax{};
    uint32_t firstChildOrPrimitiveIndex = 0;
};

struct BVHTreeNodeTaskMesh {
    glm::vec3 worldSpaceAabbMin{};
    uint32_t indexCount = 0;
    glm::vec3 worldSpaceAabbMax{};
    uint32_t firstChildOrMeshletIndex = 0;
};

struct BVHTreeLeafMeshlet {
    uint32_t meshletFirstPrimitiveIdx = 0; ///< Value for gl_PrimitiveID.
    uint32_t vertexStart = 0; ///< Pointer into dedupVerticesBuffer and dedupVertexIndexToOrigIndexMapBuffer.
    uint32_t primitiveStart = 0; ///< Pointer into dedupTriangleIndicesBuffer.
    ///< Bit 0-15: Vertex count. Bi 16-31: Primitive count.
    uint32_t vertexAndPrimitiveCountCombined = 0;
};

class NodesBVHTreePayload : public TubeTriangleRenderDataPayload {
public:
    explicit NodesBVHTreePayload(
            uint32_t maxNumPrimitivesPerMeshlet, BvhBuildAlgorithm bvhBuildAlgorithm,
            BvhBuildGeometryMode bvhBuildGeometryMode, BvhBuildPrimitiveCenterMode bvhBuildPrimitiveCenterMode)
            : maxNumPrimitivesPerMeshlet(maxNumPrimitivesPerMeshlet), bvhBuildAlgorithm(bvhBuildAlgorithm),
              bvhBuildGeometryMode(bvhBuildGeometryMode), bvhBuildPrimitiveCenterMode(bvhBuildPrimitiveCenterMode) {}
    [[nodiscard]] Type getType() const override { return Type::NODES_HLBVH_TREE; }
    [[nodiscard]] bool settingsEqual(TubeTriangleRenderDataPayload* other) const override;

    void createPayloadPre(
            sgl::vk::Device* device, uint32_t tubeNumSubdivisions, std::vector<uint32_t>& tubeTriangleIndices,
            std::vector<TubeTriangleVertexData>& tubeTriangleVertexDataList,
            const std::vector<LinePointDataUnified>& tubeTriangleLinePointDataList) override;
    void createPayloadPost(sgl::vk::Device* device, TubeTriangleRenderData& tubeTriangleRenderData) override;

    [[nodiscard]] inline uint32_t getNumNodes() const { return nodeCount; }
    [[nodiscard]] inline uint32_t getNumLeafNodes() const { return numLeafNodes; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getNodeDataBuffer() const { return nodeDataBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getQueueStateBuffer() const { return queueStateBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getQueueStateBufferRecheck() const { return queueStateBufferRecheck; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getQueueBuffer() const { return queueBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getQueueBufferRecheck() const { return queueBufferRecheck; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getIndirectDrawBuffer() const { return indirectDrawBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getIndirectDrawCountBuffer() const {
        return indirectDrawCountBuffer;
    }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getQueueInfoBuffer() const { return queueInfoBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getMaxWorkLeftTestBuffer() const { return maxWorkLeftTestBuffer; }

private:
    // Settings.
    uint32_t maxNumPrimitivesPerMeshlet = 128;
    BvhBuildAlgorithm bvhBuildAlgorithm = BvhBuildAlgorithm::SWEEP_SAH_CPU;
    BvhBuildGeometryMode bvhBuildGeometryMode = BvhBuildGeometryMode::TRIANGLES;
    BvhBuildPrimitiveCenterMode bvhBuildPrimitiveCenterMode = BvhBuildPrimitiveCenterMode::PRIMITIVE_CENTROID;

    // Data.
    uint32_t nodeCount = 0;
    uint32_t numLeafNodes = 0;
    sgl::vk::BufferPtr nodeDataBuffer; ///< BVHTreeNodePayload objects.
    ///< QueueStateBuffer object (see QueueStateBuffer in IndirectNodeCulling.glsl; 4x int/uint).
    sgl::vk::BufferPtr queueStateBuffer, queueStateBufferRecheck;
    ///< uint * nodeCount (maximum amount of queue elements).
    sgl::vk::BufferPtr queueBuffer, queueBufferRecheck;
    sgl::vk::BufferPtr indirectDrawBuffer; ///< Padded VkDrawIndexedIndirectCommand objects.
    sgl::vk::BufferPtr indirectDrawCountBuffer; ///< uint32_t object.
    sgl::vk::BufferPtr queueInfoBuffer; ///< uint32_t object.
    sgl::vk::BufferPtr maxWorkLeftTestBuffer; ///< int32_t object, for debugging.
};

#endif //LINEVIS_NODESBVHTREEPAYLOAD_HPP
