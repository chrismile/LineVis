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

#include <Math/Math.hpp>
#include <Math/Geometry/AABB3.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include "MeshletsDrawIndirectPayload.hpp"

bool MeshletsDrawIndirectPayload::settingsEqual(TubeTriangleRenderDataPayload* other) const {
    if (!TubeTriangleRenderDataPayload::settingsEqual(other)) {
        return false;
    }
    auto* otherCast = static_cast<MeshletsDrawIndirectPayload*>(other);
    return this->maxNumPrimitivesPerMeshlet == otherCast->maxNumPrimitivesPerMeshlet;
}

void MeshletsDrawIndirectPayload::createPayloadPre(
        sgl::vk::Device* device, uint32_t tubeNumSubdivisions, std::vector<uint32_t>& tubeTriangleIndices,
        std::vector<TubeTriangleVertexData>& tubeTriangleVertexDataList,
        const std::vector<LinePointDataUnified>& tubeTriangleLinePointDataList) {
    std::vector<MeshletDrawIndirectPayloadData> meshlets;
    meshlets.reserve(sgl::uiceil(uint32_t(tubeTriangleIndices.size()), maxNumPrimitivesPerMeshlet));

    MeshletDrawIndirectPayloadData currentMeshlet{};
    sgl::AABB3 currentMeshletBB;
    uint32_t currentLineIdx = 0;

    for (size_t primitiveIdx = 0; primitiveIdx < tubeTriangleIndices.size(); primitiveIdx += 3) {
        uint32_t idx0 = tubeTriangleIndices.at(primitiveIdx);
        uint32_t idx1 = tubeTriangleIndices.at(primitiveIdx + 1);
        uint32_t idx2 = tubeTriangleIndices.at(primitiveIdx + 2);
        const auto& v0 = tubeTriangleVertexDataList.at(idx0);
        const auto& v1 = tubeTriangleVertexDataList.at(idx1);
        const auto& v2 = tubeTriangleVertexDataList.at(idx2);
        uint32_t l0 = tubeTriangleLinePointDataList.at(v0.vertexLinePointIndex & 0x7FFFFFFFu).lineStartIndex;

        if (currentMeshlet.indexCount >= maxNumPrimitivesPerMeshlet * 3
                || (currentLineIdx != l0 && currentMeshlet.indexCount > 0)) {
            currentMeshlet.worldSpaceAabbMin = currentMeshletBB.min;
            currentMeshlet.worldSpaceAabbMax = currentMeshletBB.max;
            meshlets.push_back(currentMeshlet);
            currentMeshletBB = {};
            currentMeshlet = {};
            currentMeshlet.firstIndex = uint32_t(primitiveIdx);
        }
        currentLineIdx = l0;

        currentMeshletBB.combine(v0.vertexPosition);
        currentMeshletBB.combine(v1.vertexPosition);
        currentMeshletBB.combine(v2.vertexPosition);
        currentMeshlet.indexCount += 3;
    }

    // Add any missing meshlet.
    if (currentMeshlet.indexCount > 0) {
        currentMeshlet.worldSpaceAabbMin = currentMeshletBB.min;
        currentMeshlet.worldSpaceAabbMax = currentMeshletBB.max;
        meshlets.push_back(currentMeshlet);
    }

    numMeshlets = uint32_t(meshlets.size());
    meshletDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, meshlets.size() * sizeof(MeshletDrawIndirectPayloadData), meshlets.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    meshletVisibilityArrayBuffer = std::make_shared<sgl::vk::Buffer>(
            device, meshlets.size() * sizeof(uint32_t),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    indirectDrawBuffer = std::make_shared<sgl::vk::Buffer>(
            device, meshlets.size() * sizeof(VkDrawIndexedIndirectCommand),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    indirectDrawCountBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT
            | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void MeshletsDrawIndirectPayload::createPayloadPost(
        sgl::vk::Device* device, TubeTriangleRenderData& tubeTriangleRenderData) {
}
