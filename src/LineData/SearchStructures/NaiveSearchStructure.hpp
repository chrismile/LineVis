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

#ifndef NAIVE_SEARCH_STRUCTURE_HPP_
#define NAIVE_SEARCH_STRUCTURE_HPP_

#include <vector>
#include "SearchStructure.hpp"

/**
 * A naive O(n^2) search structure that always returns all points.
 */
template<class T>
class NaiveSearchStructure : public SearchStructure<T>
{
public:
    /**
     * Builds a search structure from the passed point and data array.
     * @param points The point array.
     */
    void build(const std::vector<std::pair<glm::vec3, T>>& pointsAndData) override {
        this->pointsAndData = pointsAndData;
    }
    using SearchStructure<T>::build;

    /**
     * Reserves memory for use with @see add.
     * @param maxNumNodes The maximum number of nodes that can be added using @see addPoint.
     */
    void reserveDynamic(size_t maxNumNodes) override {
        pointsAndData.clear();
        pointsAndData.reserve(maxNumNodes);
    }

    /**
     * Adds the passed point to the search structure.
     * WARNING: This function may be less efficient than @see build if the points are added in an order suboptimal
     * for the search structure.
     * @param pointAndData The point and data to add.
     */
    void add(const std::pair<glm::vec3, T>& pointAndData) override {
        pointsAndData.push_back(pointAndData);
    }

    /**
     * Performs an area search in the search structure and returns all points within a certain bounding box.
     * @param box The bounding box.
     * @return The points stored in the search structure inside of the bounding box.
     */
    void findPointsAndDataInAxisAlignedBox(
            const AxisAlignedBox &box,
            std::vector<std::pair<glm::vec3, T>>& pointsAndDataInAxisAlignedBoundingBox) override {
        for (auto& entry : pointsAndData) {
            if (box.contains(entry.first)) {
                pointsAndDataInAxisAlignedBoundingBox.push_back(entry);
            }
        }
    }

    /**
     * Performs an area search in the search structure and returns all points and data within a certain distance to
     * some center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @param pointsInSphere The points and data stored in the search structure inside of the search radius.
     */
    void findPointsAndDataInSphere(
            const glm::vec3& center, float radius, std::vector<std::pair<glm::vec3, T>>& pointsAndDataInSphere) {
        for (auto& entry : pointsAndData) {
            if (glm::distance(center, entry.first) <= radius) {
                pointsAndDataInSphere.push_back(entry);
            }
        }
    }

private:
    std::vector<std::pair<glm::vec3, T>> pointsAndData;
};

#endif //NAIVE_SEARCH_STRUCTURE_HPP_
