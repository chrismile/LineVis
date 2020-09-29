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

#include <cmath>
#include "KdTree.hpp"

KdTree::KdTree() {
    root = nullptr;
}

KdTree::~KdTree() {
}

void KdTree::build(const std::vector<IndexedPoint*> &indexedPoints) {
    nodes.resize(indexedPoints.size());
    nodeCounter = 0;
	root = _build(indexedPoints, 0);
}

KdNode *KdTree::_build(std::vector<IndexedPoint*> points, int depth) {
    const int k = 3; // Number of dimensions

    if (points.size() == 0) {
        return nullptr;
    }

    KdNode *node = nodes.data() + nodeCounter;
    nodeCounter++;

    int axis = depth % k;
    std::sort(points.begin(), points.end(), [axis](IndexedPoint *a, IndexedPoint *b) {
        const glm::vec3& posA = a->position;
        const glm::vec3& posB = b->position;
        return posA[axis] < posB[axis];
    });
    size_t medianIndex = points.size() / 2;
    std::vector<IndexedPoint*> leftPoints = std::vector<IndexedPoint*>(points.begin(), points.begin() + medianIndex);
    std::vector<IndexedPoint*> rightPoints = std::vector<IndexedPoint*>(points.begin() + medianIndex + 1, points.end());

    node->axis = axis;
    node->point = points.at(medianIndex);
    node->left = _build(leftPoints, depth + 1);
    node->right = _build(rightPoints, depth + 1);
    return node;
}

std::vector<IndexedPoint*> KdTree::findPointsInAxisAlignedBox(const AxisAlignedBox &box) {
    std::vector<IndexedPoint*> points;
    _findPointsInAxisAlignedBox(box, points, root);
    return points;
}

std::vector<IndexedPoint*> KdTree::findPointsInSphere(const glm::vec3& center, float radius) {
    std::vector<IndexedPoint*> pointsWithDistance;
    std::vector<IndexedPoint*> pointsInRect;

    // First, find all points within the bounding box containing the search circle.
    AxisAlignedBox box;
    box.min = center - glm::vec3(radius, radius, radius);
    box.max = center + glm::vec3(radius, radius, radius);
    _findPointsInAxisAlignedBox(box, pointsInRect, root);

    // Now, filter all points out that are not within the search radius.
    float squaredRadius = radius*radius;
    glm::vec3 differenceVector;
    for (IndexedPoint *point : pointsInRect) {
        differenceVector = point->position - center;
        if (differenceVector.x*differenceVector.x + differenceVector.y*differenceVector.y
                + differenceVector.z*differenceVector.z <= squaredRadius) {
            pointsWithDistance.push_back(point);
        }
    }

    return pointsWithDistance;
}

void KdTree::_findPointsInAxisAlignedBox(const AxisAlignedBox &rect, std::vector<IndexedPoint*> &points, KdNode *node) {
	if (node == nullptr) {
		return;
	}

	if (rect.contains(node->point->position)) {
        points.push_back(node->point);
    }

    const glm::vec3& pointPosition = node->point->position;
    if (rect.min[node->axis] <= pointPosition[node->axis]) {
        _findPointsInAxisAlignedBox(rect, points, node->left);
    }
    if (rect.max[node->axis] >= pointPosition[node->axis]) {
        _findPointsInAxisAlignedBox(rect, points, node->right);
    }
}

IndexedPoint* KdTree::findNearestNeighbor(const glm::vec3& point) {
    IndexedPoint* indexedPoint = nullptr;
    float nearestNeighborDistance = std::numeric_limits<float>::max();
    _findNearestNeighbor(point, nearestNeighborDistance, indexedPoint, root);
    return indexedPoint;
}

void KdTree::_findNearestNeighbor(
        const glm::vec3& point, float& nearestNeighborDistance, IndexedPoint*& nearestNeighbor, KdNode* node) {
    if (node == nullptr) {
        return;
    }

    // Descend on side of split planes where the point lies.
    bool isPointOnLeftSide = point[node->axis] <= node->point->position[node->axis];
    if (isPointOnLeftSide) {
        _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->left);
    } else {
        _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->right);
    }

    // Compute the distance of this node to the point.
    glm::vec3 diff = point - node->point->position;
    float newDistance = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    if (newDistance < nearestNeighborDistance) {
        nearestNeighborDistance = newDistance;
        nearestNeighbor = node->point;
    }

    // Check whether there could be a closer point on the opposite side.
    if (isPointOnLeftSide && point[node->axis] + nearestNeighborDistance >= node->point->position[node->axis]) {
        _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->right);
    }
    if (!isPointOnLeftSide && point[node->axis] - nearestNeighborDistance <= node->point->position[node->axis]) {
        _findNearestNeighbor(point, nearestNeighborDistance, nearestNeighbor, node->left);
    }
}
