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
#include "HashedGrid.hpp"

HashedGrid::HashedGrid(size_t numEntries, float cellSize) : cellSize(cellSize) {
	hashTableEntries.resize(numEntries);
}

void HashedGrid::convertPointToGridPosition(const glm::vec3& pos, ptrdiff_t& xg, ptrdiff_t& yg, ptrdiff_t& zg) {
	xg = static_cast<ptrdiff_t>(std::floor(pos.x / cellSize));
	yg = static_cast<ptrdiff_t>(std::floor(pos.y / cellSize));
	zg = static_cast<ptrdiff_t>(std::floor(pos.z / cellSize));
}

size_t HashedGrid::convertPointPositionToTableIndex(const glm::vec3& pos) {
	ptrdiff_t xg, yg, zg;
	convertPointToGridPosition(pos, xg, yg, zg);
	return hashFunction(xg, yg, zg);
}

// Hash Function: H(i,j,k) = (ip_1 xor jp_2 xor jp_3) mod n
size_t HashedGrid::hashFunction(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k) {
	const ptrdiff_t PRIME_NUMBERS[] = { 50331653, 12582917, 3145739 };
	return static_cast<size_t>(((i * PRIME_NUMBERS[0]) ^ (j * PRIME_NUMBERS[1])) ^ (k * PRIME_NUMBERS[2])) % hashTableEntries.size();
}

void HashedGrid::build(const std::vector<IndexedPoint*>& points) {
	// Clear the table entries.
	for (std::vector<IndexedPoint*>& hashTableEntry : hashTableEntries) {
		hashTableEntry.clear();
	}

	// Build the hash map with the points.
	for (IndexedPoint* indexedPoint : points) {
		size_t tableIndex = convertPointPositionToTableIndex(indexedPoint->position);
		hashTableEntries.at(tableIndex).push_back(indexedPoint);
	}
}

void HashedGrid::reserveDynamic(size_t maxNumNodes) {
    // Clear the table entries.
    for (std::vector<IndexedPoint*>& hashTableEntry : hashTableEntries) {
        hashTableEntry.clear();
    }
}

void HashedGrid::addPoint(IndexedPoint* indexedPoint) {
    size_t tableIndex = convertPointPositionToTableIndex(indexedPoint->position);
    hashTableEntries.at(tableIndex).push_back(indexedPoint);
}

std::vector<IndexedPoint*> HashedGrid::findPointsInAxisAlignedBox(const AxisAlignedBox& box) {
	std::vector<IndexedPoint*> points;
	ptrdiff_t lowerGrid[3];
	ptrdiff_t upperGrid[3];
	
	// Compute lower and upper grid positions.
	for (int i = 0; i < 3; i++) {
		convertPointToGridPosition(box.min, lowerGrid[0], lowerGrid[1], lowerGrid[2]);
		convertPointToGridPosition(box.max, upperGrid[0], upperGrid[1], upperGrid[2]);
	}
	
	// Iterate over all covered cells.
	for (ptrdiff_t z = lowerGrid[2]; z <= upperGrid[2]; z++) {
		for (ptrdiff_t y = lowerGrid[1]; y <= upperGrid[1]; y++) {
			for (ptrdiff_t x = lowerGrid[0]; x <= upperGrid[0]; x++) {
				size_t tableIndex = hashFunction(x, y, z);
				std::vector<IndexedPoint*>& hashTableEntry = hashTableEntries.at(tableIndex);
                points.insert(points.end(), hashTableEntry.begin(), hashTableEntry.end());
			}
		}
	}
	
	return points;
}

std::vector<IndexedPoint*> HashedGrid::findPointsInSphere(const glm::vec3& center, float radius) {
	std::vector<IndexedPoint*> pointsWithDistance;

    // First, find all points within the bounding box containing the search circle.
	glm::vec3 lower = center - glm::vec3(radius, radius, radius);
	glm::vec3 upper = center + glm::vec3(radius, radius, radius);
	std::vector<IndexedPoint*> pointsInRect = findPointsInAxisAlignedBox(AxisAlignedBox(lower, upper));

    // Now, filter all points out that are not within the search radius.
    float squaredRadius = radius * radius;
    glm::vec3 differenceVector;
    for (IndexedPoint* indexedPoint : pointsInRect) {
		differenceVector = indexedPoint->position - center;
		if (differenceVector.x * differenceVector.x + differenceVector.y * differenceVector.y
				+ differenceVector.z * differenceVector.z <= squaredRadius) {
            pointsWithDistance.push_back(indexedPoint);
		}
	}

	return pointsWithDistance;
}
