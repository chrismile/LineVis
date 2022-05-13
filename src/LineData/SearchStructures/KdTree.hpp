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
#include <cmath>

#include "SearchStructure.hpp"

#ifdef __has_cpp_attribute
#if __has_cpp_attribute(no_unique_address) && !(defined(__GNUC__) && (__cplusplus < 201100))
#define ATTRIBUTE_NO_UNIQUE_ADDRESS [[no_unique_address]]
#endif
#endif

#ifndef ATTRIBUTE_NO_UNIQUE_ADDRESS
#define ATTRIBUTE_NO_UNIQUE_ADDRESS
#endif

/**
 * A node in the k-d-tree. It stores in which axis the space is partitioned (x,y,z)
 * as an index, the position of the node, and its left and right children.
 */
template<class T>
class KdNode {
public:
    KdNode() : axis(0), left(nullptr), right(nullptr) {}
    int axis;
    glm::vec3 point{};
    ATTRIBUTE_NO_UNIQUE_ADDRESS T data; // Use [[no_unique_address]] in case T == Empty.
    KdNode* left;
    KdNode* right;
};

/**
 * The k-d-tree class. Used for searching point sets in space efficiently.
 */
template<class T>
class KdTree : public SearchStructure<T>
{
public:
    KdTree() : root(nullptr) {}
    ~KdTree() override = default;

    // Forbid the use of copy operations.
    KdTree& operator=(const KdTree& other) = delete;
    KdTree(const KdTree& other) = delete;

    /**
     * Builds a k-d-tree from the passed point and data array.
     * @param points The point array.
     * @param dataArray The data array.
     */
    void build(const std::vector<std::pair<glm::vec3, T>>& pointsAndData) override {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif

        if (pointsAndData.empty()) {
            return;
        }

        std::vector<std::pair<glm::vec3, T>> pointsAndDataCopy = pointsAndData;
        nodes.resize(pointsAndData.size());
        nodeCounter = 0;
        root = _build(pointsAndDataCopy, 0, 0, pointsAndData.size());
    }
    using SearchStructure<T>::build;


    /**
     * Reserves memory for use with @see add.
     * @param maxNumNodes The maximum number of nodes that can be added using @see addPoint.
     */
    void reserveDynamic(size_t maxNumNodes) override {
        nodes.resize(maxNumNodes);
        nodeCounter = 0;
    }

    /**
     * Adds the passed point and data to the k-d-tree.
     * WARNING: This function may be less efficient than @see build if the points are added in an order suboptimal
     * for the search structure. Furthermore, @see reserveDynamic must be called before calling this function.
     * @param point The point to add.
     * @param data The corresponding data to add.
     */
    void add(const glm::vec3& point, const T& data) override {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif
        //root = _addPoint(root, 0, point, data);

        const int k = 3; // Number of dimensions

        int depth = 0;
        int axis = 0;
        KdNode<T>** node = &root;
        while (*node) {
            axis = depth % k;

            if ((axis == 0 && point.x < (*node)->point.x)
                    || (axis == 1 && point.y < (*node)->point.y)
                    || (axis == 2 && point.z < (*node)->point.z)) {
                node = &(*node)->left;
            } else {
                node = &(*node)->right;
            }

            depth++;
        }

        assert(size_t(nodeCounter) < nodes.size());
        KdNode<T>* newNode = nodes.data() + nodeCounter;
        newNode->axis = axis;
        newNode->point = point;
        newNode->data = data;
        nodeCounter++;
        *node = newNode;
    }

    /**
     * Performs an area search in the k-d-tree and returns all points within a certain bounding box.
     * @param box The bounding box.
     * @param pointsAndData The points and data stored in the k-d-tree inside of the bounding box.
     */
    void findPointsAndDataInAxisAlignedBox(
            const AxisAlignedBox& box, std::vector<std::pair<glm::vec3, T>>& pointsAndData) override {
        _findPointsAndDataInAxisAlignedBox(box, pointsAndData, root);
    }

    /**
     * Performs an area search in the k-d-tree and returns all points within a certain distance to some center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @return The points stored in the k-d-tree inside of the search radius.
     */
    void findPointsAndDataInSphere(
            const glm::vec3& center, float radius, std::vector<std::pair<glm::vec3, T>>& pointsWithDistance) override {
        std::vector<std::pair<glm::vec3, T>> pointsInRect;

        // First, find all points within the bounding box containing the search circle.
        AxisAlignedBox box;
        box.min = center - glm::vec3(radius, radius, radius);
        box.max = center + glm::vec3(radius, radius, radius);
        _findPointsAndDataInAxisAlignedBox(box, pointsInRect, root);

        // Now, filter all points out that are not within the search radius.
        float squaredRadius = radius*radius;
        glm::vec3 differenceVector;
        for (const std::pair<glm::vec3, T>& point : pointsInRect) {
            differenceVector = point.first - center;
            if (differenceVector.x * differenceVector.x + differenceVector.y * differenceVector.y
                    + differenceVector.z * differenceVector.z <= squaredRadius) {
                pointsWithDistance.push_back(point);
            }
        }
    }

    /**
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @return Whether there is at least one point stored in the k-d-tree inside of the search radius.
     */
    bool getHasPointCloserThan(const glm::vec3& center, float radius) override {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif

        auto closestNeighbor = findNearestNeighbor(center);
        glm::vec3 differenceVector = closestNeighbor->first - center;
        if (differenceVector.x * differenceVector.x + differenceVector.y * differenceVector.y
                + differenceVector.z * differenceVector.z <= radius * radius) {
            return true;
        }
        return false;
    }

    /**
     * Performs an area search in the k-d-tree and returns the number of points within a certain distance to some
     * center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @param searchCache A vector that can be used to store the points.
     * @return The number of points stored in the k-d-tree inside of the search radius.
     */
    size_t getNumPointsInSphere(
            const glm::vec3& center, float radius, std::vector<std::pair<glm::vec3, T>>& searchCache) override {
        size_t numPointsInSphere = 0;

        // First, find all points within the bounding box containing the search circle.
        AxisAlignedBox box;
        box.min = center - glm::vec3(radius, radius, radius);
        box.max = center + glm::vec3(radius, radius, radius);
        _findPointsAndDataInAxisAlignedBox(box, searchCache, root);

        // Now, filter all points out that are not within the search radius.
        float squaredRadius = radius*radius;
        glm::vec3 differenceVector;
        for (const std::pair<glm::vec3, T>& point : searchCache) {
            differenceVector = point.first - center;
            if (differenceVector.x * differenceVector.x + differenceVector.y * differenceVector.y
                    + differenceVector.z * differenceVector.z <= squaredRadius) {
                numPointsInSphere++;
            }
        }

        return numPointsInSphere;
    }


    /**
     * Returns the nearest neighbor in the k-d-tree to the passed point position.
     * @param point The point to which to find the closest neighbor to.
     * @return The closest neighbor within the maximum distance.
     */
    std::optional<std::pair<glm::vec3, T>> findNearestNeighbor(const glm::vec3& point) {
        std::pair<glm::vec3, T> pointAndData;
        float nearestNeighborDistance = std::numeric_limits<float>::max();
        _findNearestNeighbor(point, nearestNeighborDistance, pointAndData, root);
        return pointAndData;
    }


private:
    /// Root of the tree
    KdNode<T>* root;

    // List of all nodes
    std::vector<KdNode<T>> nodes;
    int nodeCounter = 0;


    /**
     * Builds a k-d-tree from the passed point array recursively (for internal use only).
     * @param points The point array.
     * @param dataArray The data array.
     * @param depth The current depth in the tree (starting at 0).
     * @return The parent node of the current sub-tree.
     */
    KdNode<T>* _build(std::vector<std::pair<glm::vec3, T>>& pointsAndData, int depth, size_t startIdx, size_t endIdx) {
        const int k = 3; // Number of dimensions

        if (endIdx - startIdx == 0) {
            return nullptr;
        }

        KdNode<T> *node = nodes.data() + nodeCounter;
        nodeCounter++;

        int axis = depth % k;
        std::sort(
                pointsAndData.begin() + startIdx, pointsAndData.begin() + endIdx,
                [axis](const std::pair<glm::vec3, T>& a, const std::pair<glm::vec3, T>& b) {
            return a.first[axis] < b.first[axis];
        });
        size_t medianIndex = startIdx + (endIdx - startIdx) / 2;

        node->axis = axis;
        node->point = pointsAndData.at(medianIndex).first;
        node->data = pointsAndData.at(medianIndex).second;
        node->left = _build(pointsAndData, depth + 1, startIdx, medianIndex);
        node->right = _build(pointsAndData, depth + 1, medianIndex + 1, endIdx);
        return node;
    }

    /**
     * Adds the passed point and data to the k-d-tree.
     * WARNING: This function may be less efficient than @see build if the points are added in an order suboptimal
     * for the acceleration structure.
     * @param point The point to add.
     * @param data The corresponding data to add.
     */
    KdNode<T>* _add(KdNode<T>* node, int depth, const glm::vec3& point, const T& data) {
        const int k = 3; // Number of dimensions
        int axis = depth % k;

        if (node == nullptr) {
            assert(size_t(nodeCounter) < nodes.size());
            KdNode<T>* newNode = nodes.data() + nodeCounter;
            newNode->axis = axis;
            newNode->point = point;
            newNode->data = data;
            nodeCounter++;
            return newNode;
        }

        if ((axis == 0 && point.x < node->point.x)
            || (axis == 1 && point.y < node->point.y)
            || (axis == 2 && point.z < node->point.z)) {
            node->left = _add(node->left, depth + 1, point, data);
        } else {
            node->right = _add(node->right, depth + 1, point, data);
        }
        return node;
    }

    /**
     * Performs an area search in the k-d-tree and returns all points within a certain bounding box
     * (for internal use only).
     * @param box The bounding box.
     * @param node The current k-d-tree node that is searched.
     * @param points The points of the k-d-tree inside of the bounding box.
     */
    void _findPointsAndDataInAxisAlignedBox(
            const AxisAlignedBox& box, std::vector<std::pair<glm::vec3, T>> &points, KdNode<T>* node) {
        if (node == nullptr) {
            return;
        }

        if (box.contains(node->point)) {
            points.push_back(std::make_pair(node->point, node->data));
        }

        if (box.min[node->axis] <= node->point[node->axis]) {
            _findPointsAndDataInAxisAlignedBox(box, points, node->left);
        }
        if (box.max[node->axis] >= node->point[node->axis]) {
            _findPointsAndDataInAxisAlignedBox(box, points, node->right);
        }
    }

    /**
     * Returns the nearest neighbor in the k-d-tree to the passed point position.
     * @param point The point to which to find the closest neighbor to.
     * @param nearestNeighborDistance The distance to the nearest neighbor found so far.
     * @param nearestNeighbor The nearest neighbor found so far.
     * @param node The current k-d-tree node that is searched.
     */
    void _findNearestNeighbor(
            const glm::vec3& point, float& nearestNeighborDistance,
            std::pair<glm::vec3, T>& nearestNeighbor, KdNode<T>* node) {
        if (node == nullptr) {
            return;
        }

        // Descend on side of split planes where the point lies.
        bool isPointOnLeftSide = point[node->axis] <= node->point[node->axis];
        if (isPointOnLeftSide) {
            _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->left);
        } else {
            _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->right);
        }

        // Compute the distance of this node to the point.
        glm::vec3 diff = point - node->point;
        float newDistance = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
        if (newDistance < nearestNeighborDistance) {
            nearestNeighborDistance = newDistance;
            nearestNeighbor = std::make_pair(node->point, node->data);
        }

        // Check whether there could be a closer point on the opposite side.
        if (isPointOnLeftSide && point[node->axis] + nearestNeighborDistance >= node->point[node->axis]) {
            _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->right);
        }
        if (!isPointOnLeftSide && point[node->axis] - nearestNeighborDistance <= node->point[node->axis]) {
            _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->left);
        }
    }
};

#endif //KDTREE_H_
