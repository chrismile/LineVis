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

#include <bvh/bvh.hpp>
#include <bvh/vector.hpp>
#include <bvh/triangle.hpp>
#include <bvh/binned_sah_builder.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include <bvh/locally_ordered_clustering_builder.hpp>
#include <bvh/linear_bvh_builder.hpp>
// Spatial split BVH unsupported, as it relies on splitting the underlying primitives.
//#include <bvh/spatial_split_bvh_builder.hpp>

#include <Math/Math.hpp>
#include <Math/Geometry/AABB3.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include "MeshletsDrawIndirectPayload.hpp"
#include "NodesBVHTreePayload.hpp"

using Scalar = float;
using Vector3 = bvh::Vector3<Scalar>;
using BoundingBox = bvh::BoundingBox<Scalar>;
using Triangle = bvh::Triangle<Scalar>;
using Bvh = bvh::Bvh<Scalar>;

bool NodesBVHTreePayload::settingsEqual(TubeTriangleRenderDataPayload* other) const {
    if (!TubeTriangleRenderDataPayload::settingsEqual(other)) {
        return false;
    }
    auto* otherCast = static_cast<NodesBVHTreePayload*>(other);
    return this->maxNumPrimitivesPerMeshlet == otherCast->maxNumPrimitivesPerMeshlet
            && this->bvhBuildAlgorithm == otherCast->bvhBuildAlgorithm
            && this->bvhBuildGeometryMode == otherCast->bvhBuildGeometryMode
            && this->bvhBuildPrimitiveCenterMode == otherCast->bvhBuildPrimitiveCenterMode;
}

/*
 * TODO: Add support for Parallel LBVH construction.
 * - https://developer.nvidia.com/blog/thinking-parallel-part-iii-tree-construction-gpu/
 * - https://fuchsia.googlesource.com/fuchsia/+/refs/heads/main/src/graphics/lib/compute/radix_sort/
 */

void NodesBVHTreePayload::createPayloadPre(
        sgl::vk::Device* device, uint32_t tubeNumSubdivisions, std::vector<uint32_t>& tubeTriangleIndices,
        std::vector<TubeTriangleVertexData>& tubeTriangleVertexDataList,
        const std::vector<LinePointDataUnified>& tubeTriangleLinePointDataList) {
    size_t numTriangles = tubeTriangleIndices.size() / 3;
    size_t numPrimitives = 0;
    std::vector<BoundingBox> bboxes;
    std::vector<Vector3> centers;

    // Optional, if bvhBuildGeometryMode == BvhBuildGeometryMode::MESHLETS.
    std::vector<MeshletDrawIndirectPayloadData> meshlets;

    if (bvhBuildGeometryMode == BvhBuildGeometryMode::TRIANGLES) {
        numPrimitives = tubeTriangleIndices.size() / 3;
        bboxes.reserve(numPrimitives);
        centers.reserve(numPrimitives);
        for (size_t indexOffset = 0; indexOffset < tubeTriangleIndices.size(); indexOffset += 3) {
            uint32_t idx0 = tubeTriangleIndices.at(indexOffset);
            uint32_t idx1 = tubeTriangleIndices.at(indexOffset + 1);
            uint32_t idx2 = tubeTriangleIndices.at(indexOffset + 2);
            const glm::vec3& pt0 = tubeTriangleVertexDataList.at(idx0).vertexPosition;
            const glm::vec3& pt1 = tubeTriangleVertexDataList.at(idx1).vertexPosition;
            const glm::vec3& pt2 = tubeTriangleVertexDataList.at(idx2).vertexPosition;
            BoundingBox bb;
            bb.min = Vector3(
                    std::min(pt0.x, std::min(pt1.x, pt2.x)),
                    std::min(pt0.y, std::min(pt1.y, pt2.y)),
                    std::min(pt0.z, std::min(pt1.z, pt2.z)));
            bb.max = Vector3(
                    std::max(pt0.x, std::max(pt1.x, pt2.x)),
                    std::max(pt0.y, std::max(pt1.y, pt2.y)),
                    std::max(pt0.z, std::max(pt1.z, pt2.z)));
            bboxes.push_back(bb);
            if (bvhBuildPrimitiveCenterMode == BvhBuildPrimitiveCenterMode::PRIMITIVE_CENTROID) {
                glm::vec3 center = (pt0 + pt1 + pt2) / 3.0f;
                centers.emplace_back(center.x, center.y, center.z);
            } else if (bvhBuildPrimitiveCenterMode == BvhBuildPrimitiveCenterMode::BOUNDING_BOX_CENTER) {
                centers.emplace_back(bb.center());
            }
        }
    } else if (bvhBuildGeometryMode == BvhBuildGeometryMode::MESHLETS) {
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

        numPrimitives = meshlets.size();
        bboxes.reserve(numPrimitives);
        centers.reserve(numPrimitives);

        for (size_t meshletIdx = 0; meshletIdx < meshlets.size(); meshletIdx++) {
            const MeshletDrawIndirectPayloadData& meshlet = meshlets.at(meshletIdx);
            BoundingBox bb;
            bb.min = Vector3(
                    meshlet.worldSpaceAabbMin.x, meshlet.worldSpaceAabbMin.y, meshlet.worldSpaceAabbMin.z);
            bb.max = Vector3(
                    meshlet.worldSpaceAabbMax.x, meshlet.worldSpaceAabbMax.y, meshlet.worldSpaceAabbMax.z);
            bboxes.push_back(bb);
            if (bvhBuildPrimitiveCenterMode == BvhBuildPrimitiveCenterMode::PRIMITIVE_CENTROID) {
                uint32_t maxIdx = meshlet.firstIndex + meshlet.indexCount;
                glm::vec3 avgVertexPos(0.0f);
                for (uint32_t idx = meshlet.firstIndex; idx < maxIdx; idx++) {
                    uint32_t vertexIdx = tubeTriangleIndices.at(idx);
                    const auto& vertexPos = tubeTriangleVertexDataList.at(vertexIdx).vertexPosition;
                    avgVertexPos += vertexPos;
                }
                avgVertexPos /= float(meshlet.indexCount);
                centers.emplace_back(avgVertexPos.x, avgVertexPos.y, avgVertexPos.z);
            } else if (bvhBuildPrimitiveCenterMode == BvhBuildPrimitiveCenterMode::BOUNDING_BOX_CENTER) {
                centers.emplace_back(bb.center());
            }
        }
    }

    BoundingBox globalBbox = bvh::compute_bounding_boxes_union(bboxes.data(), numPrimitives);

    // Build the BVH.
    Bvh bvh;
    if (bvhBuildAlgorithm == BvhBuildAlgorithm::BINNED_SAH_CPU) {
        constexpr size_t binCount = 16;
        bvh::BinnedSahBuilder<Bvh, binCount> binnedSahBuilder(bvh);
        if (bvhBuildGeometryMode == BvhBuildGeometryMode::TRIANGLES) {
            binnedSahBuilder.max_leaf_size = maxNumPrimitivesPerMeshlet;
            uint32_t minNumLeaves = sgl::uiceil(uint32_t(numTriangles), maxNumPrimitivesPerMeshlet);
            uint32_t maxHeight = uint32_t(std::ceil(std::log2(double(minNumLeaves)))) + 1;
            binnedSahBuilder.max_depth = maxHeight;
        } else if (bvhBuildGeometryMode == BvhBuildGeometryMode::MESHLETS) {
            binnedSahBuilder.max_leaf_size = 1;
        }
        binnedSahBuilder.build(
                globalBbox, bboxes.data(), centers.data(), numPrimitives);
    } else if (bvhBuildAlgorithm == BvhBuildAlgorithm::SWEEP_SAH_CPU) {
        bvh::SweepSahBuilder<Bvh> sweepSahBuilder(bvh);
        if (bvhBuildGeometryMode == BvhBuildGeometryMode::TRIANGLES) {
            sweepSahBuilder.max_leaf_size = maxNumPrimitivesPerMeshlet;
            uint32_t minNumLeaves = sgl::uiceil(uint32_t(numTriangles), maxNumPrimitivesPerMeshlet);
            uint32_t maxHeight = uint32_t(std::ceil(std::log2(double(minNumLeaves)))) + 1;
            sweepSahBuilder.max_depth = maxHeight;
        } else if (bvhBuildGeometryMode == BvhBuildGeometryMode::MESHLETS) {
            sweepSahBuilder.max_leaf_size = 1;
        }
        sweepSahBuilder.build(
                globalBbox, bboxes.data(), centers.data(), numPrimitives);
    } else if (bvhBuildAlgorithm == BvhBuildAlgorithm::LOCALLY_ORDERED_CLUSTERING_CPU) {
        using Morton = uint32_t;
        bvh::LocallyOrderedClusteringBuilder<Bvh, Morton> locallyOrderedClusteringBuilder(bvh);
        locallyOrderedClusteringBuilder.build(
                globalBbox, bboxes.data(), centers.data(), numPrimitives);
    } else if (bvhBuildAlgorithm == BvhBuildAlgorithm::LINEAR_BVH_CPU) {
        using Morton = uint32_t;
        bvh::LinearBvhBuilder<Bvh, Morton> linearBvhBuilder(bvh);
        linearBvhBuilder.build(
                globalBbox, bboxes.data(), centers.data(), numPrimitives);
    }

    // Get statistics.
    Bvh::Node* nodes = bvh.nodes.get();
    nodeCount = uint32_t(bvh.node_count);
    numLeafNodes = 0;
    uint32_t minNumPrimitivesPerNode = std::numeric_limits<uint32_t>::max(), maxNumPrimitivesPerNode = 0;
    uint32_t minNumTrianglesPerNode = std::numeric_limits<uint32_t>::max(), maxNumTrianglesPerNode = 0;
    for (size_t i = 0; i < bvh.node_count; i++) {
        const Bvh::Node& node = nodes[i];
        if (node.is_leaf()) {
            minNumPrimitivesPerNode = std::min(minNumPrimitivesPerNode, node.primitive_count);
            maxNumPrimitivesPerNode = std::max(maxNumPrimitivesPerNode, node.primitive_count);
            numLeafNodes++;
        }
    }

    // Create tree nodes data for transfer to GPU.
    std::vector<BVHTreeNode> treeNodes;
    treeNodes.resize(bvh.node_count);
    auto* primitiveIndices = bvh.primitive_indices.get();
    auto tubeTriangleIndicesOld = tubeTriangleIndices;
    tubeTriangleIndices.clear();
    if (bvhBuildGeometryMode == BvhBuildGeometryMode::TRIANGLES) {
        for (size_t i = 0; i < bvh.node_count; i++) {
            const Bvh::Node& node = nodes[i];
            BVHTreeNode& treeNode = treeNodes.at(i);
            treeNode.worldSpaceAabbMin = glm::vec3(node.bounds[0], node.bounds[2], node.bounds[4]);
            treeNode.worldSpaceAabbMax = glm::vec3(node.bounds[1], node.bounds[3], node.bounds[5]);
            treeNode.indexCount = node.primitive_count * 3;
            if (node.is_leaf()) {
                treeNode.firstChildOrPrimitiveIndex = uint32_t(tubeTriangleIndices.size());
                for (size_t primitiveIdx = 0; primitiveIdx < node.primitive_count; primitiveIdx++) {
                    size_t triangleIndicesIdx = primitiveIndices[node.first_child_or_primitive + primitiveIdx] * 3;
                    tubeTriangleIndices.push_back(tubeTriangleIndicesOld.at(triangleIndicesIdx));
                    tubeTriangleIndices.push_back(tubeTriangleIndicesOld.at(triangleIndicesIdx + 1));
                    tubeTriangleIndices.push_back(tubeTriangleIndicesOld.at(triangleIndicesIdx + 2));
                }
            } else {
                treeNode.firstChildOrPrimitiveIndex = node.first_child_or_primitive;
            }
        }
        minNumTrianglesPerNode = minNumPrimitivesPerNode;
        maxNumTrianglesPerNode = maxNumPrimitivesPerNode;
    } else {
        for (size_t i = 0; i < bvh.node_count; i++) {
            const Bvh::Node& node = nodes[i];
            BVHTreeNode& treeNode = treeNodes.at(i);
            treeNode.worldSpaceAabbMin = glm::vec3(node.bounds[0], node.bounds[2], node.bounds[4]);
            treeNode.worldSpaceAabbMax = glm::vec3(node.bounds[1], node.bounds[3], node.bounds[5]);
            treeNode.indexCount = 0;
            if (node.is_leaf()) {
                treeNode.firstChildOrPrimitiveIndex = uint32_t(tubeTriangleIndices.size());
                for (size_t primitiveIdx = 0; primitiveIdx < node.primitive_count; primitiveIdx++) {
                    size_t meshletIdx = primitiveIndices[node.first_child_or_primitive + primitiveIdx];
                    auto& meshlet = meshlets.at(meshletIdx);
                    uint32_t endIdx = meshlet.firstIndex + meshlet.indexCount;
                    for (uint32_t triIdx = meshlet.firstIndex; triIdx < endIdx; triIdx++) {
                        tubeTriangleIndices.push_back(tubeTriangleIndicesOld.at(triIdx));
                    }
                    treeNode.indexCount += meshlet.indexCount;
                }
                minNumTrianglesPerNode = std::min(minNumTrianglesPerNode, treeNode.indexCount / 3u);
                maxNumTrianglesPerNode = std::max(maxNumTrianglesPerNode, treeNode.indexCount / 3u);
            } else {
                treeNode.firstChildOrPrimitiveIndex = node.first_child_or_primitive;
            }
        }
    }

    // Compute tree height.
    std::vector<std::pair<const Bvh::Node*, uint32_t>> nodeStack;
    uint32_t treeHeight = 0;
    nodeStack.emplace_back(nodes, 1);
    while (!nodeStack.empty()) {
        auto nodeHeightPair = nodeStack.back();
        const Bvh::Node* node = nodeHeightPair.first;
        uint32_t h = nodeHeightPair.second;
        treeHeight = std::max(treeHeight, h);
        nodeStack.pop_back();
        if (!node->is_leaf()) {
            nodeStack.emplace_back(&nodes[Bvh::sibling(node->first_child_or_primitive)], h + 1);
            nodeStack.emplace_back(&nodes[node->first_child_or_primitive], h + 1);
        }
    }

    sgl::Logfile::get()->writeInfo("Tree height: " + std::to_string(treeHeight));
    sgl::Logfile::get()->writeInfo("Number of nodes: " + std::to_string(getNumNodes()));
    sgl::Logfile::get()->writeInfo("Number of leaves: " + std::to_string(getNumLeafNodes()));
    if (bvhBuildGeometryMode == BvhBuildGeometryMode::MESHLETS) {
        sgl::Logfile::get()->writeInfo("Min primitives/leaf: " + std::to_string(minNumPrimitivesPerNode));
        sgl::Logfile::get()->writeInfo("Max primitives/leaf: " + std::to_string(maxNumPrimitivesPerNode));
        sgl::Logfile::get()->writeInfo("Avg primitives/leaf: " + std::to_string(
                float(numPrimitives) / float(numLeafNodes)));
    }
    sgl::Logfile::get()->writeInfo("Min triangles/leaf: " + std::to_string(minNumTrianglesPerNode));
    sgl::Logfile::get()->writeInfo("Max triangles/leaf: " + std::to_string(maxNumTrianglesPerNode));
    sgl::Logfile::get()->writeInfo("Avg triangles/leaf: " + std::to_string(
            float(numTriangles) / float(numLeafNodes)));

    // Test cases for debugging purposes.
    /*{
        nodeCount = 3;
        numLeafNodes = 2;
        treeNodes.clear();
        BVHTreeNode testNode;
        testNode.worldSpaceAabbMin = glm::vec3(-0.5f);
        testNode.worldSpaceAabbMax = glm::vec3(0.5f);
        testNode.indexCount = 0;
        testNode.firstChildOrPrimitiveIndex = 1;
        treeNodes.push_back(testNode);
        testNode.indexCount = maxNumPrimitivesPerMeshlet * 3;
        testNode.firstChildOrPrimitiveIndex = 0;
        treeNodes.push_back(testNode);
        testNode.indexCount = maxNumPrimitivesPerMeshlet * 3;
        testNode.firstChildOrPrimitiveIndex = maxNumPrimitivesPerMeshlet * 3;
        treeNodes.push_back(testNode);
    }*/
    /*{
        treeNodes.clear();
        BVHTreeNode testNode;
        testNode.worldSpaceAabbMin = glm::vec3(-0.5f);
        testNode.worldSpaceAabbMax = glm::vec3(0.5f);
        testNode.indexCount = 0;
        int treeHeight = 12;
        int nextLevelOffset = 0;
        for (int h = 0; h < treeHeight; h++) {
            int numNodesAtLevel = 1 << h;
            nextLevelOffset += numNodesAtLevel;
            for (int w = 0; w < numNodesAtLevel; w++) {
                if (h == treeHeight - 1) {
                    testNode.firstChildOrPrimitiveIndex = w * maxNumPrimitivesPerMeshlet * 3;
                    testNode.indexCount = maxNumPrimitivesPerMeshlet * 3;
                } else {
                    testNode.firstChildOrPrimitiveIndex = nextLevelOffset + w * 2;
                }
                treeNodes.push_back(testNode);
            }
        }
        nodeCount = uint32_t(treeNodes.size());
        numLeafNodes = 1 << (treeHeight - 1);
    }*/

    if (treeNodes.empty()) {
        return;
    }
    nodeDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, treeNodes.size() * sizeof(BVHTreeNode), treeNodes.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    queueStateBuffer = std::make_shared<sgl::vk::Buffer>(
            device, 4 * sizeof(uint32_t), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    queueStateBufferRecheck = std::make_shared<sgl::vk::Buffer>(
            device, 4 * sizeof(uint32_t), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    queueBuffer = std::make_shared<sgl::vk::Buffer>(
            device, nodeCount * sizeof(uint32_t), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    queueBufferRecheck = std::make_shared<sgl::vk::Buffer>(
            device, nodeCount * sizeof(uint32_t), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    indirectDrawBuffer = std::make_shared<sgl::vk::Buffer>(
            device, numLeafNodes * sizeof(VkDrawIndexedIndirectCommand),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    indirectDrawCountBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT
            | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    int32_t startTestVal = 0;
    maxWorkLeftTestBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(int32_t), &startTestVal,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void NodesBVHTreePayload::createPayloadPost(sgl::vk::Device* device, TubeTriangleRenderData& tubeTriangleRenderData) {
}
