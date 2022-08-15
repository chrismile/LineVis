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

#include <Graphics/Vulkan/Buffers/Buffer.hpp>
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
    size_t numPrimitives = 0;
    if (bvhBuildGeometryMode == BvhBuildGeometryMode::TRIANGLES) {
        numPrimitives = tubeTriangleIndices.size() / 3;
    } else if (bvhBuildGeometryMode == BvhBuildGeometryMode::MESHLETS) {
        // TODO
        numPrimitives = tubeTriangleIndices.size();
    }

    std::vector<BoundingBox> bboxes;
    std::vector<Vector3> centers;
    bboxes.reserve(numPrimitives);
    centers.reserve(numPrimitives);
    if (bvhBuildGeometryMode == BvhBuildGeometryMode::TRIANGLES) {
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
        // TODO
    }

    BoundingBox globalBbox = bvh::compute_bounding_boxes_union(bboxes.data(), numPrimitives);

    // Build the BVH.
    Bvh bvh;
    if (bvhBuildAlgorithm == BvhBuildAlgorithm::BINNED_SAH_CPU) {
        constexpr size_t binCount = 16;
        bvh::BinnedSahBuilder<Bvh, binCount> binnedSahBuilder(bvh);
        binnedSahBuilder.build(
                globalBbox, bboxes.data(), centers.data(), numPrimitives);
        //binnedSahBuilder.max_depth;
        //binnedSahBuilder.max_leaf_size;
    } else if (bvhBuildAlgorithm == BvhBuildAlgorithm::SWEEP_SAH_CPU) {
        bvh::SweepSahBuilder<Bvh> sweepSahBuilder(bvh);
        sweepSahBuilder.build(
                globalBbox, bboxes.data(), centers.data(), numPrimitives);
        //sweepSahBuilder.max_depth;
        //sweepSahBuilder.max_leaf_size;
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

    uint32_t numLeafNodes = 0;
    std::vector<BVHTreeNodePayload> treeNodes;
    treeNodes.reserve(bvh.node_count);
    Bvh::Node* nodes = bvh.nodes.get();
    auto* primitiveIndices = bvh.primitive_indices.get();
    auto tubeTriangleIndicesOld = tubeTriangleIndices;
    tubeTriangleIndices.clear();
    for (size_t i = 0; i < bvh.node_count; i++) {
        const Bvh::Node& node = nodes[i];
        BVHTreeNodePayload treeNode;
        treeNode.worldSpaceAabbMin = glm::vec3(node.bounds[0], node.bounds[2], node.bounds[4]);
        treeNode.worldSpaceAabbMax = glm::vec3(node.bounds[1], node.bounds[3], node.bounds[5]);
        treeNode.indexCount = node.primitive_count * 3;
        if (node.is_leaf()) {
            treeNode.firstChildOrPrimitiveIndex = uint32_t(tubeTriangleIndices.size());
            for (size_t primitiveIdx = 0; primitiveIdx < node.primitive_count; primitiveIdx++) {
                size_t triangleIndicesIdx = primitiveIndices[node.first_child_or_primitive + primitiveIdx] * 3;
                tubeTriangleIndices.push_back(tubeTriangleIndicesOld.at(triangleIndicesIdx));
                tubeTriangleIndices.push_back(tubeTriangleIndicesOld.at(triangleIndicesIdx) + 1);
                tubeTriangleIndices.push_back(tubeTriangleIndicesOld.at(triangleIndicesIdx) + 2);
            }
            numLeafNodes++;
        } else {
            treeNode.firstChildOrPrimitiveIndex = node.first_child_or_primitive;
        }
    }

    nodeDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, treeNodes.size() * sizeof(BVHTreeNodePayload), treeNodes.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    meshletVisibilityArrayBuffer = std::make_shared<sgl::vk::Buffer>(
            device, numLeafNodes * sizeof(uint32_t),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    indirectDrawBuffer = std::make_shared<sgl::vk::Buffer>(
            device, numLeafNodes * sizeof(VkDrawIndexedIndirectCommand),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    indirectDrawCountBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT
            | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void NodesBVHTreePayload::createPayloadPost(sgl::vk::Device* device, TubeTriangleRenderData& tubeTriangleRenderData) {
}
