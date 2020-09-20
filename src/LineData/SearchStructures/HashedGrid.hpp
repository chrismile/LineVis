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
#include "SearchStructure.hpp"

class HashedGrid : public SearchStructure
{
public:
    /**
     * Creates a hashed grid acceleration data structure.
     * @param numEntries The number of entries the hash array should have.
     * @param cellSize The size of a cell in x, y and z direction (uniform).
     */
	HashedGrid(size_t numEntries = 53, float cellSize = 0.1);

    /**
     * Builds a hashed grid from the passed point array.
     * @param points The point array.
     */
    void build(const std::vector<IndexedPoint*>& points) override;

    /**
     * Performs an area search in the hashed grid and returns all points within a certain bounding box.
     * @param box The bounding box.
     * @return The points stored in the hashed grid inside of the bounding box.
     */
    std::vector<IndexedPoint*> findPointsInAxisAlignedBox(const AxisAlignedBox &box) override;

    /**
     * Performs an area search in the hashed grid and returns all points within a certain distance to some center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @return The points stored in the hashed grid inside of the search radius.
     */
    std::vector<IndexedPoint*> findPointsInSphere(const glm::vec3& center, float radius) override;

private:
	/**
	 * The hash function.
	 * @param x The integer grid cell position in x direction.
	 * @param y The integer grid cell position in y direction.
	 * @param z The integer grid cell position in z direction.
	 */
	size_t hashFunction(ptrdiff_t x, ptrdiff_t y, ptrdiff_t z);

	/**
	 * Converts a floating point point position to an integer grid position.
	 * @param pos The point position.
	 * @param xg The integer grid cell position in x direction. 
	 * @param yg The integer grid cell position in y direction. 
	 * @param zg The integer grid cell position in z direction. 
	 */
	void convertPointToGridPosition(const glm::vec3& pos, ptrdiff_t& xg, ptrdiff_t& yg, ptrdiff_t& zg);

	/**
	 * Converts a point position to its according hash map entry index.
	 * @param pos The point position.
	 * @return The index in the hash map the point belongs to.
	 */
	size_t convertPointPositionToTableIndex(const glm::vec3& pos);

	/**
	 * Returns the points in the hash table entry corresponding to the passed cell position.
	 * @param xg The integer grid cell position in x direction. 
	 * @param yg The integer grid cell position in y direction. 
	 * @param zg The integer grid cell position in z direction. 
	 * @return The points in the cell.
	 */
	std::vector<IndexedPoint*> getInCell(ptrdiff_t x, ptrdiff_t y, ptrdiff_t z);
	
	float cellSize; //< Cell size in x, y and z direction (uniform)
	std::vector<std::vector<IndexedPoint*>> hashTableEntries; //< Hash table entries
};

#endif //HASHED_GRID_H_
