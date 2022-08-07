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

#include <unordered_map>
#include <Math/Math.hpp>
#include <Math/Geometry/AABB3.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include "MeshletsTaskMeshShaderPayload.hpp"

bool MeshletsTaskMeshShaderPayload::settingsEqual(TubeTriangleRenderDataPayload* other) const {
    if (!TubeTriangleRenderDataPayload::settingsEqual(other)) {
        return false;
    }
    auto* otherCast = static_cast<MeshletsTaskMeshShaderPayload*>(other);
    return this->maxNumPrimitivesPerMeshlet == otherCast->maxNumPrimitivesPerMeshlet
            && this->maxNumVerticesPerMeshlet == otherCast->maxNumVerticesPerMeshlet;
}

void MeshletsTaskMeshShaderPayload::createPayloadPre(
        sgl::vk::Device* device, std::vector<uint32_t>& tubeTriangleIndices,
        std::vector<TubeTriangleVertexData>& tubeTriangleVertexDataList,
        const std::vector<LinePointDataUnified>& tubeTriangleLinePointDataList) {
    std::vector<MeshletTaskMeskShaderPayloadData> meshlets;
    meshlets.reserve(sgl::uiceil(uint32_t(tubeTriangleIndices.size()), maxNumPrimitivesPerMeshlet));
    std::vector<glm::vec3> dedupVertices;
    dedupVertices.reserve(tubeTriangleVertexDataList.size());
    std::vector<uint32_t> dedupVertexIndexToOrigIndexMap;
    dedupVertexIndexToOrigIndexMap.reserve(tubeTriangleVertexDataList.size());
    std::vector<uint8_t> dedupTriangleIndices;
    dedupTriangleIndices.reserve(tubeTriangleIndices.size());

    MeshletTaskMeskShaderPayloadData currentMeshlet{};
    std::unordered_map<uint32_t, uint8_t> currentMeshletIndices;
    sgl::AABB3 currentMeshletBB;
    uint32_t currentNumPrimitives = 0;
    uint32_t currentNumVertices = 0;
    uint32_t currentLineIdx = 0;

    for (size_t primitiveIdx = 0; primitiveIdx < tubeTriangleIndices.size(); primitiveIdx += 3) {
        currentNumVertices = currentMeshlet.vertexCount;
        currentNumPrimitives = currentMeshlet.primitiveCount;
        uint32_t idx0 = tubeTriangleIndices.at(primitiveIdx);
        uint32_t idx1 = tubeTriangleIndices.at(primitiveIdx + 1);
        uint32_t idx2 = tubeTriangleIndices.at(primitiveIdx + 2);
        const auto& v0 = tubeTriangleVertexDataList.at(idx0);
        const auto& v1 = tubeTriangleVertexDataList.at(idx1);
        const auto& v2 = tubeTriangleVertexDataList.at(idx2);
        uint32_t l0 = tubeTriangleLinePointDataList.at(v0.vertexLinePointIndex & 0x7FFFFFFFu).lineStartIndex;
        auto it0 = currentMeshletIndices.find(idx0);
        auto it1 = currentMeshletIndices.find(idx1);
        auto it2 = currentMeshletIndices.find(idx2);
        if (it0 == currentMeshletIndices.end()) {
            currentNumVertices++;
        }
        if (it1 == currentMeshletIndices.end()) {
            currentNumVertices++;
        }
        if (it2 == currentMeshletIndices.end()) {
            currentNumVertices++;
        }
        currentNumPrimitives++;

        if (currentNumPrimitives > maxNumPrimitivesPerMeshlet
                || currentNumVertices > maxNumVerticesPerMeshlet
                || (currentLineIdx != l0 && currentMeshlet.primitiveCount > 0)) {
            currentMeshlet.worldSpaceAabbMin = currentMeshletBB.min;
            currentMeshlet.worldSpaceAabbMax = currentMeshletBB.max;
            meshlets.push_back(currentMeshlet);
            currentMeshletBB = {};
            currentMeshletIndices.clear();
            currentMeshlet = {};
            currentMeshlet.meshletFirstPrimitiveIdx = primitiveIdx / 3;
            currentMeshlet.vertexStart = dedupVertexIndexToOrigIndexMap.size();
            currentMeshlet.primitiveStart = dedupTriangleIndices.size() / 3u;
        }
        currentLineIdx = l0;

        if (currentMeshlet.primitiveCount == 0 || it0 == currentMeshletIndices.end()) {
            dedupVertices.push_back(v0.vertexPosition);
            dedupVertexIndexToOrigIndexMap.push_back(idx0);
            dedupTriangleIndices.push_back(uint8_t(currentMeshlet.vertexCount));
            currentMeshletIndices.insert(std::make_pair(idx0, uint8_t(currentMeshlet.vertexCount)));
            currentMeshlet.vertexCount++;
        } else {
            dedupTriangleIndices.push_back(it0->second);
        }
        if (currentMeshlet.primitiveCount == 0 || it1 == currentMeshletIndices.end()) {
            dedupVertices.push_back(v1.vertexPosition);
            dedupVertexIndexToOrigIndexMap.push_back(idx1);
            dedupTriangleIndices.push_back(uint8_t(currentMeshlet.vertexCount));
            currentMeshletIndices.insert(std::make_pair(idx1, uint8_t(currentMeshlet.vertexCount)));
            currentMeshlet.vertexCount++;
        } else {
            dedupTriangleIndices.push_back(it1->second);
        }
        if (currentMeshlet.primitiveCount == 0 || it2 == currentMeshletIndices.end()) {
            dedupVertices.push_back(v2.vertexPosition);
            dedupVertexIndexToOrigIndexMap.push_back(idx2);
            currentMeshletIndices.insert(std::make_pair(idx2, uint8_t(currentMeshlet.vertexCount)));
            dedupTriangleIndices.push_back(uint8_t(currentMeshlet.vertexCount));
            currentMeshlet.vertexCount++;
        } else {
            dedupTriangleIndices.push_back(it2->second);
        }
        currentMeshlet.primitiveCount++;

        currentMeshletBB.combine(v0.vertexPosition);
        currentMeshletBB.combine(v1.vertexPosition);
        currentMeshletBB.combine(v2.vertexPosition);
    }

    // Add any missing meshlet.
    if (currentMeshlet.primitiveCount > 0) {
        currentMeshlet.worldSpaceAabbMin = currentMeshletBB.min;
        currentMeshlet.worldSpaceAabbMax = currentMeshletBB.max;
        meshlets.push_back(currentMeshlet);
    }

    numMeshlets = uint32_t(meshlets.size());
    meshletDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, meshlets.size() * sizeof(MeshletTaskMeskShaderPayloadData), meshlets.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    meshletVisibilityArrayBuffer = std::make_shared<sgl::vk::Buffer>(
            device, meshlets.size() * sizeof(uint32_t),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    dedupVerticesBuffer = std::make_shared<sgl::vk::Buffer>(
            device, dedupVertices.size() * sizeof(glm::vec3), dedupVertices.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    dedupVertexIndexToOrigIndexMapBuffer = std::make_shared<sgl::vk::Buffer>(
            device, dedupVertexIndexToOrigIndexMap.size() * sizeof(uint32_t),
            dedupVertexIndexToOrigIndexMap.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    dedupTriangleIndicesBuffer = std::make_shared<sgl::vk::Buffer>(
            device, dedupTriangleIndices.size() * sizeof(uint8_t), dedupTriangleIndices.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
}

void MeshletsTaskMeshShaderPayload::createPayloadPost(
        sgl::vk::Device* device, TubeTriangleRenderData& tubeTriangleRenderData) {
}
