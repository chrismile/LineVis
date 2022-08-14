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

#ifndef LINEVIS_MESHLETSDRAWINDIRECTPAYLOAD_HPP
#define LINEVIS_MESHLETSDRAWINDIRECTPAYLOAD_HPP

#include "../LineRenderData.hpp"

struct MeshletDrawIndirectPayloadData {
    glm::vec3 worldSpaceAabbMin{};
    uint32_t indexCount = 0;
    glm::vec3 worldSpaceAabbMax{};
    uint32_t firstIndex = 0;
};

class MeshletsDrawIndirectPayload : public TubeTriangleRenderDataPayload {
public:
    explicit MeshletsDrawIndirectPayload(uint32_t maxNumPrimitivesPerMeshlet)
            : maxNumPrimitivesPerMeshlet(maxNumPrimitivesPerMeshlet) {}
    [[nodiscard]] Type getType() const override { return Type::MESHLETS_DRAW_INDIRECT; }
    [[nodiscard]] bool settingsEqual(TubeTriangleRenderDataPayload* other) const override;

    void createPayloadPre(
            sgl::vk::Device* device, uint32_t tubeNumSubdivisions, std::vector<uint32_t>& tubeTriangleIndices,
            std::vector<TubeTriangleVertexData>& tubeTriangleVertexDataList,
            const std::vector<LinePointDataUnified>& tubeTriangleLinePointDataList) override;
    void createPayloadPost(sgl::vk::Device* device, TubeTriangleRenderData& tubeTriangleRenderData) override;

    [[nodiscard]] inline uint32_t getNumMeshlets() const { return numMeshlets; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getMeshletDataBuffer() const { return meshletDataBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getMeshletVisibilityArrayBuffer() const {
        return meshletVisibilityArrayBuffer;
    }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getIndirectDrawBuffer() const { return indirectDrawBuffer; }
    [[nodiscard]] inline const sgl::vk::BufferPtr& getIndirectDrawCountBuffer() const {
        return indirectDrawCountBuffer;
    }

private:
    // Settings.
    uint32_t maxNumPrimitivesPerMeshlet = 128;

    // Data.
    uint32_t numMeshlets = 0;
    sgl::vk::BufferPtr meshletDataBuffer; ///< MeshletDrawIndirectPayloadData objects.
    sgl::vk::BufferPtr meshletVisibilityArrayBuffer; ///< uint32_t objects.
    sgl::vk::BufferPtr indirectDrawBuffer; ///< Padded VkDrawIndexedIndirectCommand objects.
    sgl::vk::BufferPtr indirectDrawCountBuffer; ///< uint32_t objects.
};

#endif //LINEVIS_MESHLETSDRAWINDIRECTPAYLOAD_HPP
