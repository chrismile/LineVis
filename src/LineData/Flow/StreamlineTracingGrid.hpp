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

#ifndef LINEVIS_STREAMLINETRACINGGRID_HPP
#define LINEVIS_STREAMLINETRACINGGRID_HPP

#include <string>
#include <map>

#include "Loaders/TrajectoryFile.hpp"

template<class T>
class HashedGrid;
struct StreamlineTracingSettings;
class StreamlineSeeder;

/**
 * Stores a Cartesian grid. At each grid point, scalar data and velocity data is stored.
 * The velocity data can be used for streamline and streamribbon tracing.
 */
class StreamlineTracingGrid {
public:
    ~StreamlineTracingGrid();
    void setGridMetadata(int xs, int ys, int zs, float dx, float dy, float dz);
    void addVectorField(float* vectorField, const std::string& vectorName);
    void addScalarField(float* scalarField, const std::string& scalarName);
    std::vector<std::string> getVectorFieldNames();
    std::vector<std::string> getScalarFieldNames();
    [[nodiscard]] inline const sgl::AABB3& getBox() const { return box; }
    [[nodiscard]] inline int getGridSizeX() const { return xs; }
    [[nodiscard]] inline int getGridSizeY() const { return ys; }
    [[nodiscard]] inline int getGridSizeZ() const { return zs; }
    [[nodiscard]] inline float getDx() const { return dx; }
    [[nodiscard]] inline float getDy() const { return dy; }
    [[nodiscard]] inline float getDz() const { return dz; }
    [[nodiscard]] inline float* getVelocityField() const { return velocityField; }
    [[nodiscard]] inline float* getVorticityField() const { return vorticityField; }
    [[nodiscard]] inline float* getHelicityField() const { return helicityField; }

    void traceStreamlines(StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories);
    void traceStreamribbons(
            StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories,
            std::vector<std::vector<glm::vec3>>& filteredRibbonsDirections);

    void computeSimulationBoundaryMesh(
            std::vector<uint32_t>& cachedSimulationMeshOutlineTriangleIndices,
            std::vector<glm::vec3>& cachedSimulationMeshOutlineVertexPositions);

private:
    void _setVectorField(StreamlineTracingSettings& tracingSettings);
    float _getScalarFieldAtIdx(const float* scalarField, const glm::ivec3& gridIdx) const;
    float _getScalarFieldAtPosition(const float* scalarField, const glm::vec3& particlePosition) const;
    [[nodiscard]] glm::vec3 _getVectorAtIdx(const glm::ivec3& gridIdx, bool forwardMode) const;
    [[nodiscard]] glm::vec3 _getVectorAtPosition(const glm::vec3& particlePosition, bool forwardMode) const;
    [[nodiscard]] glm::dvec3 _getVectorAtIdxDouble(const glm::ivec3& gridIdx, bool forwardMode) const;
    [[nodiscard]] glm::dvec3 _getVectorAtPositionDouble(const glm::dvec3& particlePosition, bool forwardMode) const;
    static bool _rayBoxIntersection(
            const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& lower, const glm::vec3& upper,
            float& tNear, float& tFar);
    static bool rayBoxPlaneIntersection(
            float rayOriginX, float rayDirectionX, float lowerX, float upperX, float& tNear, float& tFar);
    void _pushTrajectoryAttributes(Trajectory& trajectory) const;
    void _pushRibbonDirections(
            const StreamlineTracingSettings& tracingSettings,
            const Trajectory& trajectory, std::vector<glm::vec3>& ribbonDirections, bool forwardMode) const;
    static void _reverseTrajectory(Trajectory& trajectory);
    static void _reverseRibbon(Trajectory& trajectory, std::vector<glm::vec3>& ribbonDirections);
    static void _insertBackwardTrajectory(const Trajectory& trajectoryBackward, Trajectory& trajectory);
    static void _insertBackwardRibbon(
            const Trajectory& trajectoryBackward, const std::vector<glm::vec3>& ribbonDirectionsBackward,
            Trajectory& trajectory, std::vector<glm::vec3>& ribbonDirections);

    inline void _traceStreamline(
            const StreamlineTracingSettings& tracingSettings, Trajectory& trajectory, const glm::vec3& seedPoint,
            bool forwardMode) const {
        std::vector<glm::vec3> ribbonDirections;
        return _trace(tracingSettings, trajectory, ribbonDirections, seedPoint, forwardMode);
    }
    inline void _traceStreamribbon(
            const StreamlineTracingSettings& tracingSettings, Trajectory& trajectory,
            std::vector<glm::vec3>& ribbonDirections, const glm::vec3& seedPoint, bool forwardMode) const {
        return _trace(tracingSettings, trajectory, ribbonDirections, seedPoint, forwardMode);
    }
    void _trace(
            const StreamlineTracingSettings& tracingSettings, Trajectory& trajectory,
            std::vector<glm::vec3>& ribbonDirections, const glm::vec3& seedPoint, bool forwardMode) const;

    void _traceStreamlinesDecreasingHelicity(
            StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories);
    void _traceStreamribbonsDecreasingHelicity(
            StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories,
            std::vector<std::vector<glm::vec3>>& filteredRibbonsDirections);
    void _traceStreamlineDecreasingHelicity(
            const StreamlineTracingSettings& tracingSettings,
            Trajectory& currentTrajectory, Trajectories& trajectories,
            std::vector<glm::vec3>& ribbonDirections, const glm::vec3& seedPoint,
            float& dt, bool forwardMode);
    float _computeTrajectoryLength(const Trajectory& trajectory);
    bool _isTerminated(
            const StreamlineTracingSettings& tracingSettings,
            Trajectory& currentTrajectory, const glm::vec3& currentPoint,
            Trajectories& trajectories, float& segmentLength, int& iterationCounter);

    void _integrationStepExplicitEuler(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepImplicitEuler(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepHeun(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepMidpoint(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepRK4(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepRKF45(
            const StreamlineTracingSettings& tracingSettings, glm::vec3& fP0, float& fDt, bool forwardMode) const;
    void _integrationStep(
            const StreamlineTracingSettings& tracingSettings, glm::vec3& p0, float& dt, bool forwardMode) const;

    int xs = 0, ys = 0, zs = 0; ///< Size of the grid in data points.
    float dx = 0.0f, dy = 0.0f, dz = 0.0f; ///< Distance between two neighboring points in x/y/z direction.
    sgl::AABB3 box; ///< Box encompassing all grid points.
    float* velocityField = nullptr;
    float* vorticityField = nullptr;
    float* helicityField = nullptr;
    glm::vec3* V = nullptr;
    float maxVectorMagnitude = 0.0f;
    float maxHelicityMagnitude = 0.0f;
    std::map<std::string, float*> vectorFields;
    std::map<std::string, float> maxVectorFieldMagnitudes;
    std::map<std::string, float*> scalarFields;
    float terminationDistanceStart = 0.0f;
    HashedGrid<glm::vec3>* gridSelf = nullptr;
    std::vector<std::pair<glm::vec3, glm::vec3>> closePoints;
};

#endif //LINEVIS_STREAMLINETRACINGGRID_HPP
