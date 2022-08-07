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

#ifndef LINEVIS_MESHLETSTASKMESHSHADERPAYLOAD_HPP
#define LINEVIS_MESHLETSTASKMESHSHADERPAYLOAD_HPP

#include "../LineRenderData.hpp"

struct MeshletTaskMeskShaderPayloadData {
    glm::vec3 worldSpaceAabbMin{};
    float padding = 0.0f;
    glm::vec3 worldSpaceAabbMax{};
    uint32_t meshletFirstPrimitiveIdx = 0; ///< Value for gl_PrimitiveID.
    uint32_t vertexStart = 0; ///< Pointer into dedupVerticesBuffer and dedupVertexIndexToOrigIndexMapBuffer.
    uint32_t vertexCount = 0;
    uint32_t primitiveStart = 0; ///< Pointer into dedupTriangleIndicesBuffer.
    uint32_t primitiveCount = 0;
};

class MeshletsTaskMeshShaderPayload : public TubeTriangleRenderDataPayload {
public:
    explicit MeshletsTaskMeshShaderPayload(uint32_t maxNumPrimitivesPerMeshlet, uint32_t maxNumVerticesPerMeshlet)
            : maxNumPrimitivesPerMeshlet(maxNumPrimitivesPerMeshlet),
              maxNumVerticesPerMeshlet(maxNumVerticesPerMeshlet) {}
    [[nodiscard]] Type getType() const override { return Type::MESHLETS_TASK_MESH_SHADER; }
    [[nodiscard]] bool settingsEqual(TubeTriangleRenderDataPayload* other) const override;

    void createPayloadPre(
            sgl::vk::Device* device, std::vector<uint32_t>& tubeTriangleIndices,
            std::vector<TubeTriangleVertexData>& tubeTriangleVertexDataList,
            const std::vector<LinePointDataUnified>& tubeTriangleLinePointDataList) override;
    void createPayloadPost(sgl::vk::Device* device, TubeTriangleRenderData& tubeTriangleRenderData) override;

    [[nodiscard]] inline uint32_t getNumMeshlets() const { return numMeshlets; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getMeshletDataBuffer() const { return meshletDataBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getMeshletVisibilityArrayBuffer() const {
        return meshletVisibilityArrayBuffer;
    }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getDedupVerticesBuffer() const { return dedupVerticesBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getDedupVertexIndexToOrigIndexMapBuffer() const {
        return dedupVertexIndexToOrigIndexMapBuffer;
    }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getDedupTriangleIndicesBuffer() const {
        return dedupTriangleIndicesBuffer;
    }

private:
    // Settings (choose valid values for entries of VkPhysicalDeviceMeshShaderPropertiesNV).
    uint32_t maxNumPrimitivesPerMeshlet = 126; ///< <= maxMeshOutputPrimitives (512 for NVIDIA).
    uint32_t maxNumVerticesPerMeshlet = 64; ///< <= maxMeshOutputVertices (256 for NVIDIA).

    // Data.
    uint32_t numMeshlets = 0;
    sgl::vk::BufferPtr meshletDataBuffer; ///< MeshletTaskMeskShaderPayloadData objects.
    sgl::vk::BufferPtr meshletVisibilityArrayBuffer; ///< uint32_t objects.
    // Vertices need to be deduplicated, as vertices may be shared between adjacent meshlets.
    sgl::vk::BufferPtr dedupVerticesBuffer; ///< vec3 objects.
    sgl::vk::BufferPtr dedupVertexIndexToOrigIndexMapBuffer; ///< uint32_t objects.
    sgl::vk::BufferPtr dedupTriangleIndicesBuffer; ///< uint8_t objects.
};

#endif //LINEVIS_MESHLETSTASKMESHSHADERPAYLOAD_HPP
