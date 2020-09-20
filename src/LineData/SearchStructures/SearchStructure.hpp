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

#ifndef SEARCH_STRUCTURE_H_
#define SEARCH_STRUCTURE_H_

#include <vector>
#include <glm/vec3.hpp>

struct IndexedPoint {
    glm::vec3 position;
    ptrdiff_t index;
};

/**
 * An axis aligned (bounding) box data structures used for search queries.
 */
class AxisAlignedBox
{
public:
	AxisAlignedBox() {}
	AxisAlignedBox(const glm::vec3& min, const glm::vec3& max) : min(min), max(max) {}

    /// The minimum and maximum coordinate corners of the 3D cuboid.
    glm::vec3 min, max;

    /**
     * Tests whether the axis aligned box contains a point.
     * @param pt The point.
     * @return True if the box contains the point.
     */
    bool contains(const glm::vec3 &pt) const;
};

/**
 * This class is the parent class for point search structures.
 */
class SearchStructure {
public:
    virtual ~SearchStructure() {}

    /// All types of search structures
    enum SearchStructureType {
        SEARCH_STRUCTURE_KD_TREE, SEARCH_STRUCTURE_HASHED_GRID, SEARCH_STRUCTURE_NAIVE
    };

    /**
     * Builds a k-d-tree from the passed point array.
     * @param points The point array.
     */
    virtual void build(const std::vector<IndexedPoint*>& indexedPoints)=0;

    /**
     * Performs an area search and returns all points within a certain bounding box.
     * @param box The bounding box.
     * @return The points stored in the search structure inside of the bounding box.
     */
    virtual std::vector<IndexedPoint*> findPointsInAxisAlignedBox(const AxisAlignedBox& box)=0;

    /**
     * Performs an area search and returns all points within a certain distance to some center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @return The points stored in the search structure inside of the search radius.
     */
    virtual std::vector<IndexedPoint*> findPointsInSphere(const glm::vec3& center, float radius)=0;
};

#endif //SEARCH_STRUCTURE_H_
