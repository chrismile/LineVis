/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#ifndef LINEVIS_STREAMLINESEEDER_HPP
#define LINEVIS_STREAMLINESEEDER_HPP

#include <vector>
#include <queue>
#include <random>
#include <memory>

#include <glm/vec3.hpp>
#include <Math/Geometry/AABB3.hpp>

#include "Utils/InternalState.hpp"
#include "LineData/SearchStructures/KdTree.hpp"
#include "LineData/SearchStructures/HashedGrid.hpp"
#include "StreamlineTracingDefines.hpp"

struct Trajectory;
struct StreamlineTracingSettings;
class StreamlineTracingGrid;

/**
 * Returns seed points for streamline tracing.
 */
class StreamlineSeeder {
public:
    virtual ~StreamlineSeeder() = default;

    /**
     * @return A copy of the streamline seeder that can be passed to the worker thread.
     * NOTE: The returned pointer needs to be destroyed using "delete[]".
     */
    virtual StreamlineSeeder* copy() = 0;

    /**
     * @return Whether regular tracing is used.
     */
    [[nodiscard]] virtual bool getIsRegular() const = 0;

    /**
     * Sets the simulation grid box in the GUI thread.
     */
    virtual void setNewGridBox(const sgl::AABB3& gridBox) = 0;

    /**
     * Resets the internal state of the streamline tracer.
     * @param tracingSettings The settings used for streamline tracing.
     * @param newGrid The grid used for tracing.
     */
    virtual void reset(StreamlineTracingSettings& tracingSettings, StreamlineTracingGrid* newGrid) = 0;

    /**
     * @return Returns whether a next point can still be queried using @see getNextPoint.
     * NOTE: @see reset needs to be called before calling this function.
     */
    virtual bool hasNextPoint() = 0;

    /**
     * @return The next seed point.
     * NOTE: @see reset needs to be called before calling this function.
     */
    virtual glm::vec3 getNextPoint() = 0;

    /**
     * Renders the GUI for changing the internal settings using ImGui.
     */
    virtual bool renderGui() = 0;

    /**
     * For changing internal settings programmatically and not via the GUI.
     */
    virtual bool setNewSettings(const SettingsMap& settings) = 0;
};

typedef std::shared_ptr<StreamlineSeeder> StreamlineSeederPtr;


/**
 * A seeding approach that returns seed points on a plane.
 */
class StreamlinePlaneSeeder : public StreamlineSeeder {
public:
    ~StreamlinePlaneSeeder() override = default;
    StreamlineSeeder* copy() override;
    [[nodiscard]] bool getIsRegular() const override { return regular; }
    void setNewGridBox(const sgl::AABB3& gridBox) override;
    void reset(StreamlineTracingSettings& tracingSettings, StreamlineTracingGrid* newGrid) override;
    bool hasNextPoint() override;
    glm::vec3 getNextPoint() override;
    bool renderGui() override;
    bool setNewSettings(const SettingsMap& settings) override;

private:
    std::mt19937 generator;
    std::uniform_real_distribution<float> uniformDistribution = std::uniform_real_distribution<float>(0, 1);
    StreamlineTracingGrid* grid = nullptr;
    sgl::AABB3 box;
    float maxDimension = 0.0f;

    float planeSlice = 0.5f;
    glm::vec3 direction = glm::vec3(0.0f, 1.0f, 0.0f);
    float planeOffset = 0.0f;
    glm::vec3 planeNormal = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 axis0, axis1;

    int seed = 2;
    bool regular = false;
    int currentSampleIdx = 0;
    int numSamplesX = 32, numSamplesY = 32;
    int numSamplesRandom = 1024;
    int numSamplesArrayGui[2] = { 32, 32 };
};

/**
 * A seeding approach that returns seed points within the whole grid volume.
 */
class StreamlineVolumeSeeder : public StreamlineSeeder {
public:
    ~StreamlineVolumeSeeder() override = default;
    StreamlineSeeder* copy() override;
    [[nodiscard]] bool getIsRegular() const override { return regular; }
    void setNewGridBox(const sgl::AABB3& gridBox) override;
    void reset(StreamlineTracingSettings& tracingSettings, StreamlineTracingGrid* newGrid) override;
    bool hasNextPoint() override;
    glm::vec3 getNextPoint() override;
    bool renderGui() override;
    bool setNewSettings(const SettingsMap& settings) override;

private:
    std::mt19937 generator;
    std::uniform_real_distribution<float> uniformDistribution = std::uniform_real_distribution<float>(0, 1);
    StreamlineTracingGrid* grid = nullptr;
    sgl::AABB3 box, gridBoxUi;
    float maxDimension = 0.0f;

    int seed = 2;
    bool regular = false;
    int currentSampleIdx = 0;
    int numSamplesX = 1, numSamplesY = 1, numSamplesZ = 1;
    int numSamplesRandom = 1024;
    int numSamplesArrayGui[3] = { 1, 1, 1 };
};

/**
 * A seeding approach based on the following paper:
 *
 * D. Rees, R. S. Laramee, D. Nguyen, L. Zhang, G. Chen, H. Yeh, and E. Zhang.
 * A stream ribbon seeding strategy. In Proceedings of the Eurographics/IEEE VGTC Conference on Visualization:
 * Short Papers, EuroVis '17, page 67-71, Goslar, DEU, 2017. Eurographics Association.
 */
class StreamlineMaxHelicityFirstSeeder: public StreamlineSeeder {
public:
    ~StreamlineMaxHelicityFirstSeeder() override = default;
    StreamlineSeeder* copy() override { return new StreamlineMaxHelicityFirstSeeder; }
    [[nodiscard]] bool getIsRegular() const override { return true; }
    void setNewGridBox(const sgl::AABB3& gridBox) override {}
    void reset(StreamlineTracingSettings& tracingSettings, StreamlineTracingGrid* newGrid) override;
    bool hasNextPoint() override;
    glm::vec3 getNextPoint() override;
    bool isPointTerminated(const glm::vec3& point);
    void addFinishedTrajectory(const Trajectory& trajectory);
    bool renderGui() override { return false; }
    bool setNewSettings(const SettingsMap& settings) override { return false; }

private:
    StreamlineTracingGrid* grid = nullptr;
    sgl::AABB3 box;
    int xs = 0, ys = 0, zs = 0; ///< Size of the grid in data points.
    float dx = 0.0f, dy = 0.0f, dz = 0.0f; ///< Distance between two neighboring points in x/y/z direction.
    float minimumSeparationDistance = 0.0f;
    TerminationCheckType terminationCheckType = TerminationCheckType::KD_TREE_BASED;
    KdTree<Empty> kdTree; ///< For terminationCheckType == TerminationCheckType::KD_TREE_BASED.
    HashedGrid<Empty> hashedGrid; ///< For terminationCheckType == TerminationCheckType::HASHED_GRID_BASED.
    std::vector<glm::vec3> kdTreePointCache;

    struct GridSample {
        float attributeValue;
        glm::vec3 samplePosition;

        GridSample(float attributeValue, glm::vec3 samplePosition)
                : attributeValue(attributeValue), samplePosition(samplePosition) {}

        bool operator<(const GridSample& other) const {
            return attributeValue < other.attributeValue;
        }
    };

    std::priority_queue<GridSample> samplePriorityQueue;
    std::vector<bool> cellOccupancyGrid;
    glm::vec3 nextSamplePoint{};
};

#endif //LINEVIS_STREAMLINESEEDER_HPP
