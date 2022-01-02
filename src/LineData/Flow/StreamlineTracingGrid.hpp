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

struct StreamlineTracingSettings;
class StreamlineSeeder;

/**
 * Stores a Cartesian grid. At each grid point, scalar data and velocity data is stored.
 * The velocity data can be used for streamline and streamribbon tracing.
 */
class StreamlineTracingGrid {
public:
    ~StreamlineTracingGrid();
    void setVelocityField(float* data, int xs, int ys, int zs, float dx, float dy, float dz);
    void addScalarField(float* scalarField, const std::string& scalarName);
    std::vector<std::string> getScalarAttributeNames();
    [[nodiscard]] inline const sgl::AABB3& getBox() const { return box; }

    Trajectories traceStreamlines(StreamlineTracingSettings& tracingSettings);

    void computeSimulationBoundaryMesh(
            std::vector<uint32_t>& cachedSimulationMeshOutlineTriangleIndices,
            std::vector<glm::vec3>& cachedSimulationMeshOutlineVertexPositions);

private:
    float _getScalarFieldAtIdx(const float* scalarField, const glm::ivec3& gridIdx) const;
    [[nodiscard]] glm::vec3 _getVelocityAtIdx(const glm::ivec3& gridIdx, bool forwardMode) const;
    [[nodiscard]] glm::vec3 _getVelocityVectorAt(const glm::vec3& particlePosition, bool forwardMode) const;
    [[nodiscard]] glm::dvec3 _getVelocityAtIdxDouble(const glm::ivec3& gridIdx, bool forwardMode) const;
    [[nodiscard]] glm::dvec3 _getVelocityVectorAtDouble(const glm::dvec3& particlePosition, bool forwardMode) const;
    static bool _rayBoxIntersection(
            const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& lower, const glm::vec3& upper,
            float& tNear, float& tFar);
    static bool rayBoxPlaneIntersection(
            float rayOriginX, float rayDirectionX, float lowerX, float upperX, float& tNear, float& tFar);
    void _pushTrajectoryAttributes(Trajectory& trajectory) const;
    static void _reverseTrajectory(Trajectory& trajectory);
    static void _insertBackwardTrajectory(const Trajectory& trajectoryBackward, Trajectory& trajectory);
    void _traceStreamline(
            const StreamlineTracingSettings& tracingSettings, Trajectory& trajectory, const glm::vec3& seedPoint,
            bool forwardMode) const;

    void _integrationStepExplicitEuler(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepImplicitEuler(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepHeun(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepMidpoint(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepRK4(glm::vec3& p0, float& dt, bool forwardMode) const;
    void _integrationStepRKF45(
            const StreamlineTracingSettings& tracingSettings, glm::vec3& fP0, float& fDt, bool forwardMode) const;

    int xs = 0, ys = 0, zs = 0; ///< Size of the grid in data points.
    float dx = 0.0f, dy = 0.0f, dz = 0.0f; ///< Distance between two neighboring points in x/y/z direction.
    sgl::AABB3 box; ///< Box encompassing all grid points.
    float* velocityField = nullptr;
    glm::vec3* V = nullptr;
    float maxVelocityMagnitude = 0.0f;
    std::map<std::string, float*> scalarFields;
};

#endif //LINEVIS_STREAMLINETRACINGGRID_HPP
