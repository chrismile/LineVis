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
#include <optional>
#include <tracy/Tracy.hpp>
#include <glm/glm.hpp>

#if __cplusplus >= 201703L
#include <variant>
#endif

//#define TRACY_PROFILE_TRACING

/**
 * An axis aligned (bounding) box data structures used for search queries.
 */
class AxisAlignedBox
{
public:
	AxisAlignedBox() = default;
	AxisAlignedBox(const glm::vec3& min, const glm::vec3& max) : min(min), max(max) {}

    /// The minimum and maximum coordinate corners of the 3D cuboid.
    glm::vec3 min{}, max{};

    /**
     * Tests whether the axis aligned box contains a point.
     * @param pt The point.
     * @return True if the box contains the point.
     */
    [[nodiscard]] inline bool contains(const glm::vec3 &pt) const {
        if (pt.x >= min.x && pt.y >= min.y && pt.z >= min.z
            && pt.x <= max.x && pt.y <= max.y && pt.z <= max.z)
            return true;
        return false;
    }
};

// Can be used for SearchStructure<Empty> if no additional data should be stored besides the points.
#if __cplusplus >= 201703L
typedef std::monostate Empty;
#else
typedef int Empty;
#endif

/**
 * This class is the parent class for point search structures.
 */
template<class T>
class SearchStructure {
public:
    virtual ~SearchStructure() = default;

    /// All types of search structures
    enum SearchStructureType {
        SEARCH_STRUCTURE_KD_TREE, SEARCH_STRUCTURE_HASHED_GRID, SEARCH_STRUCTURE_NAIVE
    };

    /**
     * Builds the search structure from the passed point and data array.
     * @param points The point array.
     * @param dataArray The data array.
     */
    virtual void build(const std::vector<std::pair<glm::vec3, T>>& pointsAndData)=0;
    virtual void build(const std::vector<glm::vec3>& points, const std::vector<T>& dataArray) {
        assert(points.size() == dataArray.size());
        std::vector<std::pair<glm::vec3, T>> pointsAndData;
        pointsAndData.reserve(points.size());
        for (size_t i = 0; i < points.size(); i++) {
            pointsAndData.push_back(std::make_pair(points.at(i), dataArray.at(i)));
        }
        build(pointsAndData);
    }
    virtual void build(const std::vector<glm::vec3>& points) {
        std::vector<std::pair<glm::vec3, T>> pointsAndData;
        pointsAndData.reserve(points.size());
        for (const glm::vec3& point : points) {
            pointsAndData.push_back(std::make_pair(point, T{}));
        }
        build(pointsAndData);
    }

    /**
     * Reserves memory for use with @see add.
     * @param maxNumEntries The maximum number of entries that can be added using @see addPoint.
     */
    virtual void reserveDynamic(size_t maxNumEntries)=0;

    /**
     * Adds the passed point and data to the search structure.
     * WARNING: This function may be less efficient than @see build if the points are added in an order suboptimal
     * for the search structure.
     * @param point The point to add.
     * @param data The corresponding data.
     */
    virtual void add(const glm::vec3& point, const T& data) {
        add(std::make_pair(point, data));
    }
    virtual void add(const std::pair<glm::vec3, T>& pointAndData) {
        add(pointAndData.first, pointAndData.second);
    }


    /**
     * Performs an area search and returns all points within a certain bounding box.
     * @param box The bounding box.
     * @param pointsInAxisAlignedBox The points and/or data stored in the search structure inside of the bounding box.
     */
    virtual void findPointsInAxisAlignedBox(
            const AxisAlignedBox& box, std::vector<glm::vec3>& pointsInAxisAlignedBox) {
        std::vector<std::pair<glm::vec3, T>> pointsAndDataInAxisAlignedBox;
        findPointsAndDataInAxisAlignedBox(box, pointsAndDataInAxisAlignedBox);
        pointsInAxisAlignedBox.reserve(pointsAndDataInAxisAlignedBox.size());
        for (const std::pair<glm::vec3, T>& pointAndData : pointsAndDataInAxisAlignedBox) {
            pointsInAxisAlignedBox.push_back(pointAndData.first);
        }
    }
    virtual void findDataInAxisAlignedBox(const AxisAlignedBox& box, std::vector<T>& dataInAxisAlignedBox) {
        std::vector<std::pair<glm::vec3, T>> pointsAndDataInAxisAlignedBox;
        findPointsAndDataInAxisAlignedBox(box, pointsAndDataInAxisAlignedBox);
        dataInAxisAlignedBox.reserve(pointsAndDataInAxisAlignedBox.size());
        for (const std::pair<glm::vec3, T>& pointAndData : pointsAndDataInAxisAlignedBox) {
            dataInAxisAlignedBox.push_back(pointAndData.second);
        }
    }
    virtual void findPointsAndDataInAxisAlignedBox(
            const AxisAlignedBox& box, std::vector<std::pair<glm::vec3, T>>& pointsAndData)=0;

    /**
     * Performs an area search and returns all points within a certain distance to some center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @param pointsInSphere The points and/or data stored in the search structure inside of the search radius.
     */
    virtual void findPointsInSphere(
            const glm::vec3& center, float radius, std::vector<glm::vec3>& pointsInSphere) {
        std::vector<std::pair<glm::vec3, T>> pointsAndDataInSphere;
        findPointsAndDataInSphere(center, radius, pointsAndDataInSphere);
        pointsInSphere.reserve(pointsAndDataInSphere.size());
        for (const std::pair<glm::vec3, T>& pointAndData : pointsAndDataInSphere) {
            pointsInSphere.push_back(pointAndData.first);
        }
    }
    virtual void findDataInSphere(const glm::vec3& center, float radius, std::vector<T>& dataInSphere) {
        std::vector<std::pair<glm::vec3, T>> pointsAndDataInSphere;
        findPointsAndDataInSphere(center, radius, pointsAndDataInSphere);
        dataInSphere.reserve(pointsAndDataInSphere.size());
        for (const std::pair<glm::vec3, T>& pointAndData : pointsAndDataInSphere) {
            dataInSphere.push_back(pointAndData.second);
        }
    }
    virtual void findPointsAndDataInSphere(
            const glm::vec3& center, float radius,
            std::vector<std::pair<glm::vec3, T>>& pointsAndDataInSphere)=0;

    /**
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @return Whether there is at least one point stored in the search structure inside of the search radius.
     */
    virtual bool getHasPointCloserThan(const glm::vec3& center, float radius)=0;

    /**
     * Performs an area search in the k-d-tree and returns the number of points within a certain distance to some
     * center point.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @param searchCache A vector that can be used to store the points.
     * @return The number of points stored in the k-d-tree inside of the search radius.
     */
    virtual size_t getNumPointsInSphere(
            const glm::vec3& center, float radius, std::vector<std::pair<glm::vec3, T>>& searchCache) {
        findPointsAndDataInSphere(center, radius, searchCache);
        return searchCache.size();
    }


    /**
     * Performs an area search and returns the closest point within the specified radius.
     * @param centerPoint The center point.
     * @param radius The search radius.
     * @param searchCache A vector that can be used to store the points.
     * @return The points and/or data stored in the search structure inside of the search radius.
     */
    virtual std::optional<glm::vec3> findPointClosest(
            const glm::vec3& center, float radius, std::vector<glm::vec3>& searchCache) {
        findPointsInSphere(center, radius, searchCache);
        float minDistance = std::numeric_limits<float>::max();
        std::optional<glm::vec3> closestPoint{};
        for (const glm::vec3& point : searchCache) {
            float currentDistance = glm::distance(center, point);
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                closestPoint = point;
            }
        }
        return closestPoint;
    }
    virtual std::optional<T> findDataClosest(
            const glm::vec3& center, float radius, std::vector<std::pair<glm::vec3, T>>& searchCache) {
#ifdef TRACY_PROFILE_TRACING
        ZoneScoped;
#endif

        findPointsAndDataInSphere(center, radius, searchCache);
        float minDistance = std::numeric_limits<float>::max();
        std::optional<T> closestData{};
        for (const std::pair<glm::vec3, T>& pointAndData : searchCache) {
            float currentDistance = glm::distance(center, pointAndData.first);
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                closestData = pointAndData.second;
            }
        }
        return closestData;
    }
    virtual std::optional<std::pair<glm::vec3, T>> findPointAndDataClosest(
            const glm::vec3& center, float radius, std::vector<std::pair<glm::vec3, T>>& searchCache) {
        findPointsAndDataInSphere(center, radius, searchCache);
        float minDistance = std::numeric_limits<float>::max();
        std::optional<std::pair<glm::vec3, T>> closestPointAndData{};
        for (const std::pair<glm::vec3, T>& pointAndData : searchCache) {
            float currentDistance = glm::distance(center, pointAndData.first);
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                closestPointAndData = pointAndData;
            }
        }
        return closestPointAndData;
    }
};

#endif //SEARCH_STRUCTURE_H_
