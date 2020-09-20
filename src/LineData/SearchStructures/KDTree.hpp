/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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

#ifndef KDTREE_H_
#define KDTREE_H_

#include <algorithm>
#include <vector>

#include "SearchStructure.hpp"

/**
 * A node in the k-d-tree. It stores in which axis the space is partitioned (x,y,z)
 * as an index, the position of the node, and its left and right children.
 */
class KDNode
{
public:
    KDNode() : axis(0), point(nullptr), left(nullptr), right(nullptr) {}
    int axis;
    IndexedPoint* point;
    KDNode* left;
    KDNode* right;
};

/**
 * The k-d-tree class. Used for searching point sets in space efficiently.
 * NOTE: The ownership of the memory the SPHPoint objects lies in the responsibility of the user.
 */
class KDTree : public SearchStructure
{
public:
    KDTree();
    ~KDTree() override;

    /**
     * Builds a k-d-tree from the passed point array.
     * @param indexedPoints The point array.
     */
    void build(const std::vector<IndexedPoint*>& indexedPoints) override;

    /**
     * Performs an area search in the k-d-tree and returns all points within a certain bounding box.
     * @param box The bounding box.
     * @return The points stored in the k-d-tree inside of the bounding box.
     */
    std::vector<IndexedPoint*> findPointsInAxisAlignedBox(const AxisAlignedBox& box) override;

    /**
     * Performs an area search in the k-d-tree and returns all points within a certain distance to some center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @return The points stored in the k-d-tree inside of the search radius.
     */
    std::vector<IndexedPoint*> findPointsInSphere(const glm::vec3& center, float radius) override;

    /**
     * Returns the nearest neighbor in the k-d-tree to the passed point position.
     * @param point The point to which to find the closest neighbor to.
     * @return The closest neighbor within the maximum distance.
     */
    IndexedPoint* findNearestNeighbor(const glm::vec3& point);

private:
    // Internal implementations.
    /**
     * Builds a k-d-tree from the passed point array recursively (for internal use only).
     * @param points The point array.
     * @return The parent node of the current sub-tree.
     */
    KDNode* _build(std::vector<IndexedPoint*> points, int depth);

    /**
     * Performs an area search in the k-d-tree and returns all points within a certain bounding box
     * (for internal use only).
     * @param box The bounding box.
     * @param node The current k-d-tree node that is searched.
     * @param points The points of the k-d-tree inside of the bounding box.
     */
    void _findPointsInAxisAlignedBox(const AxisAlignedBox& box, std::vector<IndexedPoint*> &points, KDNode* node);

    /**
     * Returns the nearest neighbor in the k-d-tree to the passed point position.
     * @param point The point to which to find the closest neighbor to.
     * @param nearestNeighborDistance The distance to the nearest neighbor found so far.
     * @param nearestNeighbor The nearest neighbor found so far.
     * @param node The current k-d-tree node that is searched.
     */
    void _findNearestNeighbor(
            const glm::vec3& point, float& nearestNeighborDistance, IndexedPoint*& nearestNeighbor, KDNode* node);

    /// Root of the tree
    KDNode* root;

    // List of all nodes
    std::vector<KDNode> nodes;
    int nodeCounter = 0;
};

#endif //KDTREE_H_
