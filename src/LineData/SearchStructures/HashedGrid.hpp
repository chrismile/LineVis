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

#ifndef HASHED_GRID_H_
#define HASHED_GRID_H_

#include <algorithm>
#include <vector>
#include <cmath>
#include <tracy/Tracy.hpp>
#include "SearchStructure.hpp"

template<class T>
class HashedGrid : public SearchStructure<T>
{
public:
    /**
     * Creates a hashed grid acceleration data structure.
     * @param numEntries The number of entries the hash array should have.
     * @param cellSize The size of a cell in x, y and z direction (uniform).
     */
	explicit HashedGrid(size_t numEntries = 53, float cellSize = 0.1) : cellSize(cellSize) {
        hashTableEntries.resize(numEntries);
    }

    /**
     * Builds a hashed grid from the passed point and data array.
     * @param points The point and data array.
     */
    void build(const std::vector<std::pair<glm::vec3, T>>& pointsAndData) override {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif

        // Clear the table entries.
        for (std::vector<std::pair<glm::vec3, T>>& hashTableEntry : hashTableEntries) {
            hashTableEntry.clear();
        }

        // Build the hash map with the points.
        for (const std::pair<glm::vec3, T>& pointAndData : pointsAndData) {
            size_t tableIndex = convertPointPositionToTableIndex(pointAndData.first);
            hashTableEntries.at(tableIndex).push_back(pointAndData);
        }
    }
    using SearchStructure<T>::build;

    /**
     * Reserves memory for use with @see add.
     * @param maxNumNodes The maximum number of nodes that can be added using @see add.
     */
    void reserveDynamic(size_t maxNumNodes) override {
        // Clear the table entries.
        for (std::vector<std::pair<glm::vec3, T>>& hashTableEntry : hashTableEntries) {
            hashTableEntry.clear();
        }
    }

    /**
     * Clear the table entries.
     */
    void clear() {
        for (std::vector<std::pair<glm::vec3, T>>& hashTableEntry : hashTableEntries) {
            hashTableEntry.clear();
        }
    }

    /**
     * Adds the passed point to the hashed grid.
     * WARNING: This function may be less efficient than @see build if the points are added in an order suboptimal
     * for the search structure.
     * @param pointAndData The point and data to add.
     */
    void add(const std::pair<glm::vec3, T>& pointAndData) override {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif

        size_t tableIndex = convertPointPositionToTableIndex(pointAndData.first);
        hashTableEntries.at(tableIndex).push_back(pointAndData);
    }


    /**
     * Performs an area search in the hashed grid and returns all points within a certain bounding box.
     * @param box The bounding box.
     * @param pointsAndData The points stored in the hashed grid inside of the bounding box.
     */
    void findPointsAndDataInAxisAlignedBox(
            const AxisAlignedBox& box, std::vector<std::pair<glm::vec3, T>>& pointsAndData) override {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif

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
                    std::vector<std::pair<glm::vec3, T>>& hashTableEntry = hashTableEntries.at(tableIndex);
                    pointsAndData.insert(pointsAndData.end(), hashTableEntry.begin(), hashTableEntry.end());
                }
            }
        }
    }

    /**
     * Performs an area search in the hashed grid and returns all points within a certain distance to some center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @param pointsWithDistance The points stored in the hashed grid inside of the search radius.
     */
    void findPointsAndDataInSphere(
            const glm::vec3& center, float radius,
            std::vector<std::pair<glm::vec3, T>>& pointsWithDistance) override {
        // First, find all points within the bounding box containing the search circle.
        glm::vec3 lower = center - glm::vec3(radius, radius, radius);
        glm::vec3 upper = center + glm::vec3(radius, radius, radius);
        ptrdiff_t lowerGrid[3];
        ptrdiff_t upperGrid[3];

        // Compute lower and upper grid positions.
        for (int i = 0; i < 3; i++) {
            convertPointToGridPosition(lower, lowerGrid[0], lowerGrid[1], lowerGrid[2]);
            convertPointToGridPosition(upper, upperGrid[0], upperGrid[1], upperGrid[2]);
        }

        // Iterate over all covered cells and filter all points out that are not within the search radius.
        float squaredRadius = radius * radius;
        glm::vec3 differenceVector;
        for (ptrdiff_t z = lowerGrid[2]; z <= upperGrid[2]; z++) {
            for (ptrdiff_t y = lowerGrid[1]; y <= upperGrid[1]; y++) {
                for (ptrdiff_t x = lowerGrid[0]; x <= upperGrid[0]; x++) {
                    size_t tableIndex = hashFunction(x, y, z);
                    std::vector<std::pair<glm::vec3, T>>& hashTableEntry = hashTableEntries.at(tableIndex);
                    for (const std::pair<glm::vec3, T>& pointAndIndex : hashTableEntry) {
                        differenceVector = pointAndIndex.first - center;
                        if (differenceVector.x * differenceVector.x + differenceVector.y * differenceVector.y
                                + differenceVector.z * differenceVector.z <= squaredRadius) {
                            pointsWithDistance.push_back(pointAndIndex);
                        }
                    }
                }
            }
        }

        /*std::vector<std::pair<glm::vec3, T>> pointsInRect;
        findPointsAndDataInAxisAlignedBox(AxisAlignedBox(lower, upper), pointsInRect);

        // Now, filter all points out that are not within the search radius.
        float squaredRadius = radius * radius;
        glm::vec3 differenceVector;
        for (const std::pair<glm::vec3, T>& pointAndIndex : pointsInRect) {
            differenceVector = pointAndIndex.first - center;
            if (differenceVector.x * differenceVector.x + differenceVector.y * differenceVector.y
                    + differenceVector.z * differenceVector.z <= squaredRadius) {
                pointsWithDistance.push_back(pointAndIndex);
            }
        }*/
    }

    /**
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @return Whether there is at least one point stored in the hashed grid inside of the search radius.
     */
    bool getHasPointCloserThan(const glm::vec3& center, float radius) override {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif

        glm::vec3 lower = center - glm::vec3(radius, radius, radius);
        glm::vec3 upper = center + glm::vec3(radius, radius, radius);

        // Compute lower and upper grid positions.
        ptrdiff_t lowerGrid[3];
        ptrdiff_t upperGrid[3];
        for (int i = 0; i < 3; i++) {
            convertPointToGridPosition(lower, lowerGrid[0], lowerGrid[1], lowerGrid[2]);
            convertPointToGridPosition(upper, upperGrid[0], upperGrid[1], upperGrid[2]);
        }

        // Iterate over all covered cells.
        float squaredRadius = radius * radius;
        glm::vec3 differenceVector;
        for (ptrdiff_t z = lowerGrid[2]; z <= upperGrid[2]; z++) {
            for (ptrdiff_t y = lowerGrid[1]; y <= upperGrid[1]; y++) {
                for (ptrdiff_t x = lowerGrid[0]; x <= upperGrid[0]; x++) {
                    size_t tableIndex = hashFunction(x, y, z);
                    std::vector<std::pair<glm::vec3, T>>& hashTableEntry = hashTableEntries.at(tableIndex);
                    for (const std::pair<glm::vec3, T>& pointAndIndex : hashTableEntry) {
                        differenceVector = pointAndIndex.first - center;
                        if (differenceVector.x * differenceVector.x + differenceVector.y * differenceVector.y
                                + differenceVector.z * differenceVector.z <= squaredRadius) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }


    // ---- Statistical data ----
    std::vector<size_t> getNumberOfElementsPerBucket() {
        std::vector<size_t> numberOfElementsPerBucket;
        numberOfElementsPerBucket.reserve(hashTableEntries.size());
        for (std::vector<std::pair<glm::vec3, T>>& hashTableEntry : hashTableEntries) {
            numberOfElementsPerBucket.push_back(hashTableEntry.size());
        }
        return numberOfElementsPerBucket;
    }


private:
    float cellSize; //< Cell size in x, y and z direction (uniform)
    std::vector<std::vector<std::pair<glm::vec3, T>>> hashTableEntries; //< Hash table entries


    /**
	 * The hash function.
	 * @param x The integer grid cell position in x direction.
	 * @param y The integer grid cell position in y direction.
	 * @param z The integer grid cell position in z direction.
	 */
	size_t hashFunction(ptrdiff_t x, ptrdiff_t y, ptrdiff_t z) {
        // Hash Function: H(i,j,k) = (ip_1 xor jp_2 xor jp_3) mod n
        const ptrdiff_t PRIME_NUMBERS[] = { 50331653, 12582917, 3145739 };
        return static_cast<size_t>(((x * PRIME_NUMBERS[0]) ^ (y * PRIME_NUMBERS[1])) ^ (z * PRIME_NUMBERS[2])) % hashTableEntries.size();
    }

	/**
	 * Converts a floating point point position to an integer grid position.
	 * @param pos The point position.
	 * @param xg The integer grid cell position in x direction. 
	 * @param yg The integer grid cell position in y direction. 
	 * @param zg The integer grid cell position in z direction. 
	 */
	void convertPointToGridPosition(const glm::vec3& pos, ptrdiff_t& xg, ptrdiff_t& yg, ptrdiff_t& zg) {
        xg = static_cast<ptrdiff_t>(std::floor(pos.x / cellSize));
        yg = static_cast<ptrdiff_t>(std::floor(pos.y / cellSize));
        zg = static_cast<ptrdiff_t>(std::floor(pos.z / cellSize));
    }

    /**
	 * Converts a point position to its according hash map entry index.
	 * @param pos The point position.
	 * @return The index in the hash map the point belongs to.
	 */
	size_t convertPointPositionToTableIndex(const glm::vec3& pos) {
        ptrdiff_t xg, yg, zg;
        convertPointToGridPosition(pos, xg, yg, zg);
        return hashFunction(xg, yg, zg);
    }
};

#endif //HASHED_GRID_H_
