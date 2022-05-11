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

#include <algorithm>
#include <queue>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

#include <tracy/Tracy.hpp>

#include <Utils/File/Logfile.hpp>
#include <Math/Geometry/Plane.hpp>
#include <Math/Math.hpp>

#include "StreamlineTracingDefines.hpp"
#include "StreamlineSeeder.hpp"
#include "StreamlineTracingGrid.hpp"

//#define TRACY_PROFILE_TRACING

StreamlineTracingGrid::~StreamlineTracingGrid() {
    for (auto& it : vectorFields) {
        delete[] it.second;
    }
    vectorFields.clear();

    for (auto& it : scalarFields) {
        delete[] it.second;
    }
    scalarFields.clear();
}

void StreamlineTracingGrid::setGridMetadata(int xs, int ys, int zs, float dx, float dy, float dz) {
    this->xs = xs;
    this->ys = ys;
    this->zs = zs;
    this->dx = dx;
    this->dy = dy;
    this->dz = dz;
    box = sgl::AABB3(
            glm::vec3(0.0f),
            glm::vec3(float(xs - 1) * dx, float(ys - 1) * dy, float(zs - 1) * dz));
}

void StreamlineTracingGrid::addVectorField(float* vectorField, const std::string& vectorName) {
    vectorFields.insert(std::make_pair(vectorName, vectorField));

    if (vectorName == "Velocity") {
        velocityField = vectorField;
    } else if (vectorName == "Vorticity") {
        vorticityField = vectorField;
    }

    float maxVectorMagnitude = 0.0f;
#if _OPENMP >= 201107
    //#pragma omp parallel for shared(vectorField) reduction(max: maxVectorMagnitude) default(none)
#endif
    for (int z = 0; z < zs; z++) {
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                float vx = vectorField[IDXV(x, y, z, 0)];
                float vy = vectorField[IDXV(x, y, z, 1)];
                float vz = vectorField[IDXV(x, y, z, 2)];
                float vectorMagnitude = std::sqrt(vx * vx + vy * vy + vz * vz);
                maxVectorMagnitude = std::max(maxVectorMagnitude, vectorMagnitude);
            }
        }
    }
    maxVectorFieldMagnitudes.insert(std::make_pair(vectorName, maxVectorMagnitude));
}

void StreamlineTracingGrid::_setVectorField(StreamlineTracingSettings& tracingSettings) {
    tracingSettings.vectorFieldIndex = std::clamp(
            tracingSettings.vectorFieldIndex, 0, int(vectorFields.size()) - 1);
    float* vectorField = nullptr;
    int i = 0;
    for (auto& it : vectorFields) {
        if (i == tracingSettings.vectorFieldIndex) {
            vectorField = it.second;
            maxVectorMagnitude = maxVectorFieldMagnitudes.find(it.first)->second;
            break;
        }
        i++;
    }
    V = reinterpret_cast<glm::vec3*>(vectorField);
}

void StreamlineTracingGrid::addScalarField(float* scalarField, const std::string& scalarName) {
    scalarFields.insert(std::make_pair(scalarName, scalarField));
    if (scalarName == "Helicity") {
        helicityField = scalarField;

        maxHelicityMagnitude = 0.0f;
#if _OPENMP >= 201107
        #pragma omp parallel for shared(xs, ys, zs) reduction(max: maxHelicityMagnitude) default(none)
#endif
        for (int z = 0; z < zs; z++) {
            for (int y = 0; y < ys; y++) {
                for (int x = 0; x < xs; x++) {
                    float helicityMagnitude = std::abs(helicityField[IDXS(x, y, z)]);
                    maxHelicityMagnitude = std::max(maxHelicityMagnitude, helicityMagnitude);
                }
            }
        }
    }
}

std::vector<std::string> StreamlineTracingGrid::getVectorFieldNames() {
    std::vector<std::string> vectorAttributeNames;
    for (auto& it : vectorFields) {
        vectorAttributeNames.push_back(it.first);
    }
    return vectorAttributeNames;
}

std::vector<std::string> StreamlineTracingGrid::getScalarFieldNames() {
    std::vector<std::string> scalarAttributeNames;
    for (auto& it : scalarFields) {
        scalarAttributeNames.push_back(it.first);
    }
    return scalarAttributeNames;
}

void StreamlineTracingGrid::traceStreamlines(
        StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories) {
    _setVectorField(tracingSettings);
    if (tracingSettings.streamlineSeedingStrategy == StreamlineSeedingStrategy::MAX_HELICITY_FIRST) {
        _traceStreamlinesDecreasingHelicity(tracingSettings, filteredTrajectories);
        return;
    }

#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    auto seeder = tracingSettings.seeder;
    seeder->reset(tracingSettings, this);
    int numTrajectories = tracingSettings.numPrimitives;

    Trajectories trajectories;
    trajectories.resize(numTrajectories);
    if (tracingSettings.streamlineSeedingStrategy == StreamlineSeedingStrategy::VOLUME
            || tracingSettings.streamlineSeedingStrategy == StreamlineSeedingStrategy::PLANE) {
        std::vector<glm::vec3> seedPoints;
        seedPoints.resize(numTrajectories);
        for (glm::vec3& seedPoint : seedPoints) {
            seedPoint = seeder->getNextPoint();
        }

        #pragma omp parallel for shared(numTrajectories, trajectories, seedPoints, tracingSettings) default(none)
        for (int i = 0; i < numTrajectories; i++) {
            Trajectory& trajectory = trajectories.at(i);
            const glm::vec3& seedPoint = seedPoints.at(i);
            if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::FORWARD) {
                _traceStreamline(tracingSettings, trajectory, seedPoint, true);
            } else if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::BACKWARD) {
                _traceStreamline(tracingSettings, trajectory, seedPoint, false);
                _reverseTrajectory(trajectory);
            } else {
                Trajectory trajectoryBackward;
                _traceStreamline(tracingSettings, trajectory, seedPoint, true);
                _traceStreamline(tracingSettings, trajectoryBackward, seedPoint, false);
                _reverseTrajectory(trajectoryBackward);
                _insertBackwardTrajectory(trajectoryBackward, trajectory);
            }
        }
    } else {
        for (int i = 0; i < numTrajectories; i++) {
            Trajectory& trajectory = trajectories.at(i);
            glm::vec3 seedPoint = seeder->getNextPoint();
            if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::FORWARD) {
                _traceStreamline(tracingSettings, trajectory, seedPoint, true);
            } else if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::BACKWARD) {
                _traceStreamline(tracingSettings, trajectory, seedPoint, false);
                _reverseTrajectory(trajectory);
            } else {
                Trajectory trajectoryBackward;
                _traceStreamline(tracingSettings, trajectory, seedPoint, true);
                _traceStreamline(tracingSettings, trajectoryBackward, seedPoint, false);
                _reverseTrajectory(trajectoryBackward);
                _insertBackwardTrajectory(trajectoryBackward, trajectory);
            }
        }
    }

    for (auto& trajectory: trajectories) {
        if (trajectory.positions.empty()) {
            continue;
        }
        float trajectoryLength = 0.0f;
        for (size_t i = 1; i < trajectory.positions.size(); i++) {
            trajectoryLength += glm::length(trajectory.positions.at(i) - trajectory.positions.at(i - 1));
        }
        if (trajectoryLength > tracingSettings.minimumLength) {
            filteredTrajectories.push_back(trajectory);
        }
    }
}

void StreamlineTracingGrid::traceStreamribbons(
        StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories,
        std::vector<std::vector<glm::vec3>>& filteredRibbonsDirections) {
    if (!helicityField) {
        sgl::Logfile::get()->writeError(
                "Error in StreamlineTracingGrid::traceStreamribbons: No helicity field is given!");
        return;
    }
    _setVectorField(tracingSettings);
    if (tracingSettings.streamlineSeedingStrategy == StreamlineSeedingStrategy::MAX_HELICITY_FIRST) {
        _traceStreamribbonsDecreasingHelicity(tracingSettings, filteredTrajectories, filteredRibbonsDirections);
        return;
    }

#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    auto seeder = tracingSettings.seeder;
    seeder->reset(tracingSettings, this);
    int numTrajectories = tracingSettings.numPrimitives;

    Trajectories trajectories;
    std::vector<std::vector<glm::vec3>> ribbonsDirections;
    trajectories.resize(numTrajectories);
    ribbonsDirections.resize(numTrajectories);
    if (tracingSettings.streamlineSeedingStrategy == StreamlineSeedingStrategy::VOLUME
            || tracingSettings.streamlineSeedingStrategy == StreamlineSeedingStrategy::PLANE) {
        std::vector<glm::vec3> seedPoints;
        seedPoints.resize(numTrajectories);
        for (glm::vec3& seedPoint : seedPoints) {
            seedPoint = seeder->getNextPoint();
        }

        #pragma omp parallel for shared(numTrajectories, trajectories, ribbonsDirections, seedPoints, tracingSettings) \
        default(none)
        for (int i = 0; i < numTrajectories; i++) {
            Trajectory& trajectory = trajectories.at(i);
            std::vector<glm::vec3>& ribbonDirections = ribbonsDirections.at(i);
            const glm::vec3& seedPoint = seedPoints.at(i);
            if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::FORWARD) {
                _traceStreamribbon(tracingSettings, trajectory, ribbonDirections, seedPoint, true);
            } else if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::BACKWARD) {
                _traceStreamribbon(tracingSettings, trajectory, ribbonDirections, seedPoint, false);
                _reverseRibbon(trajectory, ribbonDirections);
            } else {
                Trajectory trajectoryBackward;
                std::vector<glm::vec3> ribbonDirectionsBackward;
                _traceStreamribbon(tracingSettings, trajectory, ribbonDirections, seedPoint, true);
                _traceStreamribbon(
                        tracingSettings, trajectoryBackward, ribbonDirectionsBackward, seedPoint,
                        false);
                _reverseRibbon(trajectoryBackward, ribbonDirectionsBackward);
                _insertBackwardRibbon(trajectoryBackward, ribbonDirectionsBackward, trajectory, ribbonDirections);
            }
        }
    } else {
        for (int i = 0; i < numTrajectories; i++) {
            Trajectory& trajectory = trajectories.at(i);
            std::vector<glm::vec3>& ribbonDirections = ribbonsDirections.at(i);
            glm::vec3 seedPoint = seeder->getNextPoint();
            if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::FORWARD) {
                _traceStreamribbon(tracingSettings, trajectory, ribbonDirections, seedPoint, true);
            } else if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::BACKWARD) {
                _traceStreamribbon(tracingSettings, trajectory, ribbonDirections, seedPoint, false);
                _reverseRibbon(trajectory, ribbonDirections);
            } else {
                Trajectory trajectoryBackward;
                std::vector<glm::vec3> ribbonDirectionsBackward;
                _traceStreamribbon(tracingSettings, trajectory, ribbonDirections, seedPoint, true);
                _traceStreamribbon(
                        tracingSettings, trajectoryBackward, ribbonDirectionsBackward, seedPoint,
                        false);
                _reverseRibbon(trajectoryBackward, ribbonDirectionsBackward);
                _insertBackwardRibbon(trajectoryBackward, ribbonDirectionsBackward, trajectory, ribbonDirections);
            }
        }
    }

    for (int trajectoryIdx = 0; trajectoryIdx < numTrajectories; trajectoryIdx++) {
        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        std::vector<glm::vec3>& ribbonDirections = ribbonsDirections.at(trajectoryIdx);
        if (trajectory.positions.empty()) {
            continue;
        }
        float trajectoryLength = 0.0f;
        for (size_t i = 1; i < trajectory.positions.size(); i++) {
            trajectoryLength += glm::length(trajectory.positions.at(i) - trajectory.positions.at(i - 1));
        }
        if (trajectoryLength > tracingSettings.minimumLength) {
            filteredTrajectories.push_back(trajectory);
            filteredRibbonsDirections.push_back(ribbonDirections);
        }
    }
}


float StreamlineTracingGrid::_computeTrajectoryLength(const Trajectory& trajectory) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    int n = int(trajectory.positions.size());
    float trajectoryLength = 0.0f;
    for (int i = 0; i < n - 1; i++) {
        trajectoryLength += glm::length(trajectory.positions.at(i) - trajectory.positions.at(i + 1));
    }
    return trajectoryLength;
}

bool StreamlineTracingGrid::_isTerminated(
        const StreamlineTracingSettings& tracingSettings,
        Trajectory& currentTrajectory, const glm::vec3& currentPoint,
        Trajectories& trajectories, float& segmentLength, int& iterationCounter) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    const int MAX_ITERATIONS = std::min(
            int(std::round(float(tracingSettings.maxNumIterations) / tracingSettings.timeStepScale)),
            tracingSettings.maxNumIterations * 10) * 10;
    float terminationDistance = 1e-6f * tracingSettings.terminationDistance;

    // Have we reached a singular point?
    if (iterationCounter > MAX_ITERATIONS) {
        return true;
    }
    if (!currentTrajectory.positions.empty() && segmentLength < terminationDistance) {
        return true;
    }

    // Check if we have left the domain.
    if (!box.contains(currentPoint)) {
        if (!currentTrajectory.positions.empty()) {
            // Clamp the position to the boundary.
            glm::vec3 rayOrigin = currentTrajectory.positions.back();
            glm::vec3 rayDirection = glm::normalize(currentPoint - rayOrigin);
            float tNear, tFar;
            _rayBoxIntersection(
                    rayOrigin, rayDirection, box.getMinimum(), box.getMaximum(), tNear, tFar);
            glm::vec3 boundaryParticlePosition;
            if (tNear > 0.0f) {
                boundaryParticlePosition = rayOrigin + tNear * rayDirection;
            } else {
                boundaryParticlePosition = rayOrigin + tFar * rayDirection;
            }
            currentTrajectory.positions.emplace_back(boundaryParticlePosition);
            _pushTrajectoryAttributes(currentTrajectory);
        }
        return true;
    }

    if (currentTrajectory.positions.size() > 1) {
        const glm::vec3& pt0 = currentTrajectory.positions.at(0);
        const glm::vec3& pt1 = currentTrajectory.positions.at(1);
        glm::vec3 dir0 = pt1 - pt0;
        float dist0 = glm::length(dir0);
        dir0 /= dist0;
        glm::vec3 dirNow = currentPoint - pt0;
        float distNow = glm::length(dirNow);
        dirNow /= distNow;
        sgl::Plane plane0(dir0, pt0);
        bool approachesPt0 = plane0.getDistance(currentPoint) < 0.0f;
        if (approachesPt0 && distNow < terminationDistanceStart) {
            return true;
        }
    }

    if (tracingSettings.terminationCheckType == TerminationCheckType::NAIVE) {
        for (const Trajectory& trajectory : trajectories) {
            for (const glm::vec3& point : trajectory.positions) {
                if (glm::distance(currentPoint, point) < tracingSettings.minimumSeparationDistance) {
                    return true;
                }
            }
        }
    } else {
        auto* seeder = static_cast<StreamlineMaxHelicityFirstSeeder*>(tracingSettings.seeder.get());
        if (seeder->isPointTerminated(currentPoint)) {
            return true;
        }
    }

    return false;
}

void StreamlineTracingGrid::_traceStreamlineDecreasingHelicity(
        const StreamlineTracingSettings& tracingSettings,
        Trajectory& currentTrajectory, Trajectories& trajectories,
        std::vector<glm::vec3>& ribbonDirections, const glm::vec3& seedPoint,
        float& dt, bool forwardMode) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    glm::vec3 currentPoint = seedPoint;
    glm::vec3 lastPoint = seedPoint;
    float segmentLength = 0.0f;
    int iterationCounter = 0;
    while (true) {
        if (_isTerminated(
                tracingSettings, currentTrajectory, currentPoint, trajectories, segmentLength, iterationCounter)) {
            return;
        } else {
            currentTrajectory.positions.push_back(currentPoint);
            _pushTrajectoryAttributes(currentTrajectory);
        }

        // Integrate to the new position using one of the implemented integrators.
        _integrationStep(tracingSettings, currentPoint, dt, forwardMode);
        iterationCounter++;
        segmentLength += glm::length(currentPoint - lastPoint);
        lastPoint = currentPoint;
    }
}

void StreamlineTracingGrid::_traceStreamlinesDecreasingHelicity(
        StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories) {
    std::vector<std::vector<glm::vec3>> filteredRibbonsDirections;
    _traceStreamribbonsDecreasingHelicity(tracingSettings, filteredTrajectories, filteredRibbonsDirections);
}

/**
 * Uses a seeding approach based on the following paper:
 *
 * D. Rees, R. S. Laramee, D. Nguyen, L. Zhang, G. Chen, H. Yeh, and E. Zhang.
 * A stream ribbon seeding strategy. In Proceedings of the Eurographics/IEEE VGTC Conference on Visualization:
 * Short Papers, EuroVis '17, page 67-71, Goslar, DEU, 2017. Eurographics Association.
 */
void StreamlineTracingGrid::_traceStreamribbonsDecreasingHelicity(
        StreamlineTracingSettings& tracingSettings, Trajectories& filteredTrajectories,
        std::vector<std::vector<glm::vec3>>& filteredRibbonsDirections) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    if (!helicityField) {
        sgl::Logfile::get()->writeError(
                "Error in StreamlineTracingGrid::_traceStreamribbonsDecreasingHelicity: "
                "No helicity field was found.");
        return;
    }

    terminationDistanceStart = glm::length(box.getDimensions()) / 100.0f * tracingSettings.terminationDistanceSelf;

    auto seeder = static_cast<StreamlineMaxHelicityFirstSeeder*>(tracingSettings.seeder.get());
    seeder->reset(tracingSettings, this);

    float dt = 1.0f / maxVectorMagnitude * std::min(dx, std::min(dy, dz)) * tracingSettings.timeStepScale;

    while (seeder->hasNextPoint()) {
        glm::vec3 seedPoint = seeder->getNextPoint();

        Trajectory trajectory;
        std::vector<glm::vec3> ribbonDirections;
        bool isValid = false;
        if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::FORWARD) {
            _traceStreamlineDecreasingHelicity(
                    tracingSettings, trajectory, filteredTrajectories, ribbonDirections, seedPoint,
                    dt, true);
            isValid = _computeTrajectoryLength(trajectory) >= tracingSettings.minimumLength;
            if (isValid) {
                if (tracingSettings.flowPrimitives == FlowPrimitives::STREAMRIBBONS) {
                    _pushRibbonDirections(tracingSettings, trajectory, ribbonDirections, true);
                }
            }
        } else if (tracingSettings.integrationDirection == StreamlineIntegrationDirection::BACKWARD) {
            _traceStreamlineDecreasingHelicity(
                    tracingSettings, trajectory, filteredTrajectories, ribbonDirections, seedPoint,
                    dt, false);
            isValid = _computeTrajectoryLength(trajectory) >= tracingSettings.minimumLength;
            if (isValid) {
                if (tracingSettings.flowPrimitives == FlowPrimitives::STREAMRIBBONS) {
                    _pushRibbonDirections(tracingSettings, trajectory, ribbonDirections, true);
                    _reverseRibbon(trajectory, ribbonDirections);
                } else {
                    _reverseTrajectory(trajectory);
                }
            }
        } else {
            Trajectory trajectoryBackward;
            std::vector<glm::vec3> ribbonDirectionsBackward;
            _traceStreamlineDecreasingHelicity(
                    tracingSettings, trajectory, filteredTrajectories, ribbonDirections, seedPoint,
                    dt, true);
            _traceStreamlineDecreasingHelicity(
                    tracingSettings, trajectoryBackward, filteredTrajectories, ribbonDirectionsBackward,
                    seedPoint, dt, false);
            float totalLength =
                    _computeTrajectoryLength(trajectory) + _computeTrajectoryLength(trajectoryBackward);
            isValid = totalLength >= tracingSettings.minimumLength;

            if (isValid) {
                if (tracingSettings.flowPrimitives == FlowPrimitives::STREAMRIBBONS) {
                    _pushRibbonDirections(
                            tracingSettings, trajectory, ribbonDirections, true);
                    _pushRibbonDirections(
                            tracingSettings, trajectoryBackward, ribbonDirectionsBackward, true);
                    _reverseRibbon(trajectoryBackward, ribbonDirectionsBackward);
                    _insertBackwardRibbon(trajectoryBackward, ribbonDirectionsBackward, trajectory, ribbonDirections);
                } else {
                    _reverseTrajectory(trajectoryBackward);
                    _insertBackwardTrajectory(trajectoryBackward, trajectory);
                }
            }
        }

        if (isValid) {
            filteredTrajectories.push_back(trajectory);
            if (tracingSettings.flowPrimitives == FlowPrimitives::STREAMRIBBONS) {
                filteredRibbonsDirections.push_back(ribbonDirections);
            }
            seeder->addFinishedTrajectory(trajectory);
        }
    }
}


float StreamlineTracingGrid::_getScalarFieldAtIdx(const float* scalarField, const glm::ivec3& gridIdx) const {
    if (gridIdx.x < 0 || gridIdx.y < 0 || gridIdx.z < 0 || gridIdx.x >= xs || gridIdx.y >= ys || gridIdx.z >= zs) {
        return 0.0f;
    }
    return scalarField[gridIdx.x + gridIdx.y * xs + gridIdx.z * xs * ys];
}

float StreamlineTracingGrid::_getScalarFieldAtPosition(
        const float* scalarField, const glm::vec3& particlePosition) const {
    glm::vec3 gridPositionFloat = particlePosition - box.getMinimum();
    gridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
    auto gridPosition = glm::ivec3(gridPositionFloat);
    glm::vec3 frac = glm::fract(gridPositionFloat);
    glm::vec3 invFrac = glm::vec3(1.0) - frac;

    float interpolationValue =
            invFrac.x * invFrac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,0,0))
            + frac.x * invFrac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,0,0))
            + invFrac.x * frac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,1,0))
            + frac.x * frac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,1,0))
            + invFrac.x * invFrac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,0,1))
            + frac.x * invFrac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,0,1))
            + invFrac.x * frac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,1,1))
            + frac.x * frac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,1,1));

    return interpolationValue;
}

glm::vec3 StreamlineTracingGrid::_getVectorAtIdx(const glm::ivec3& gridIdx, bool forwardMode) const {
    if (gridIdx.x < 0 || gridIdx.y < 0 || gridIdx.z < 0 || gridIdx.x >= xs || gridIdx.y >= ys || gridIdx.z >= zs) {
        return glm::vec3(0.0f);
    }
    if (forwardMode) {
        return V[gridIdx.x + gridIdx.y * xs + gridIdx.z * xs * ys];
    } else {
        return -V[gridIdx.x + gridIdx.y * xs + gridIdx.z * xs * ys];
    }
}

glm::vec3 StreamlineTracingGrid::_getVectorAtPosition(
        const glm::vec3& particlePosition, bool forwardMode) const {
    glm::vec3 gridPositionFloat = particlePosition - box.getMinimum();
    gridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
    auto gridPosition = glm::ivec3(gridPositionFloat);
    glm::vec3 frac = glm::fract(gridPositionFloat);
    glm::vec3 invFrac = glm::vec3(1.0) - frac;
    glm::vec3 interpolationValue =
            invFrac.x * invFrac.y * invFrac.z * _getVectorAtIdx(gridPosition + glm::ivec3(0,0,0), forwardMode)
            + frac.x * invFrac.y * invFrac.z * _getVectorAtIdx(gridPosition + glm::ivec3(1,0,0), forwardMode)
            + invFrac.x * frac.y * invFrac.z * _getVectorAtIdx(gridPosition + glm::ivec3(0,1,0), forwardMode)
            + frac.x * frac.y * invFrac.z * _getVectorAtIdx(gridPosition + glm::ivec3(1,1,0), forwardMode)
            + invFrac.x * invFrac.y * frac.z * _getVectorAtIdx(gridPosition + glm::ivec3(0,0,1), forwardMode)
            + frac.x * invFrac.y * frac.z * _getVectorAtIdx(gridPosition + glm::ivec3(1,0,1), forwardMode)
            + invFrac.x * frac.y * frac.z * _getVectorAtIdx(gridPosition + glm::ivec3(0,1,1), forwardMode)
            + frac.x * frac.y * frac.z * _getVectorAtIdx(gridPosition + glm::ivec3(1,1,1), forwardMode);
    return interpolationValue;
}

glm::dvec3 StreamlineTracingGrid::_getVectorAtIdxDouble(
        const glm::ivec3 &gridIdx, bool forwardMode) const {
    if (gridIdx.x < 0 || gridIdx.y < 0 || gridIdx.z < 0 || gridIdx.x >= xs || gridIdx.y >= ys || gridIdx.z >= zs) {
        return glm::dvec3(0.0);
    }
    if (forwardMode) {
        return V[gridIdx.x + gridIdx.y * xs + gridIdx.z * xs * ys];
    } else {
        return -V[gridIdx.x + gridIdx.y * xs + gridIdx.z * xs * ys];
    }
}

glm::dvec3 StreamlineTracingGrid::_getVectorAtPositionDouble(
        const glm::dvec3& particlePosition, bool forwardMode) const {
    glm::dvec3 gridPositionFloat = particlePosition - glm::dvec3(box.getMinimum());
    gridPositionFloat *= glm::dvec3(1.0 / double(dx), 1.0 / double(dy), 1.0 / double(dz));
    auto gridPosition = glm::ivec3(gridPositionFloat);
    glm::dvec3 frac = glm::fract(gridPositionFloat);
    glm::dvec3 invFrac = glm::dvec3(1.0) - frac;
    glm::dvec3 interpolationValue =
            invFrac.x * invFrac.y * invFrac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(0,0,0), forwardMode)
            + frac.x * invFrac.y * invFrac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(1,0,0), forwardMode)
            + invFrac.x * frac.y * invFrac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(0,1,0), forwardMode)
            + frac.x * frac.y * invFrac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(1,1,0), forwardMode)
            + invFrac.x * invFrac.y * frac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(0,0,1), forwardMode)
            + frac.x * invFrac.y * frac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(1,0,1), forwardMode)
            + invFrac.x * frac.y * frac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(0,1,1), forwardMode)
            + frac.x * frac.y * frac.z * _getVectorAtIdxDouble(gridPosition + glm::ivec3(1,1,1), forwardMode);
    return interpolationValue;
}

/**
 * Helper function for StreamlineTracingGrid::rayBoxIntersection (see below).
 */
bool StreamlineTracingGrid::rayBoxPlaneIntersection(
        float rayOriginX, float rayDirectionX, float lowerX, float upperX, float& tNear, float& tFar) {
    if (std::abs(rayDirectionX) < 0.00001f) {
        // Ray is parallel to the x planes
        if (rayOriginX < lowerX || rayOriginX > upperX) {
            return false;
        }
    } else {
        // Not parallel to the x planes. Compute the intersection distance to the planes.
        float t0 = (lowerX - rayOriginX) / rayDirectionX;
        float t1 = (upperX - rayOriginX) / rayDirectionX;
        if (t0 > t1) {
            // Since t0 intersection with near plane
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }

        if (t0 > tNear) {
            // We want the largest tNear
            tNear = t0;
        }
        if (t1 < tFar) {
            // We want the smallest tFar
            tFar = t1;
        }
        if (tNear > tFar) {
            // Box is missed
            return false;
        }
        if (tFar < 0) {
            // Box is behind ray
            return false;
        }
    }
    return true;
}

/**
 * Implementation of ray-box intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter3.htm
 */
bool StreamlineTracingGrid::_rayBoxIntersection(
        const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& lower, const glm::vec3& upper,
        float& tNear, float& tFar) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    tNear = std::numeric_limits<float>::lowest();
    tFar = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; i++) {
        if (!rayBoxPlaneIntersection(
                rayOrigin[i], rayDirection[i], lower[i], upper[i], tNear, tFar)) {
            return false;
        }
    }

    //entrancePoint = rayOrigin + tNear * rayDirection;
    //exitPoint = rayOrigin + tFar * rayDirection;
    return true;
}

void StreamlineTracingGrid::_pushTrajectoryAttributes(Trajectory& trajectory) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    // Necessary to initialize the data first?
    if (trajectory.attributes.empty()) {
        trajectory.attributes.resize(scalarFields.size());
    }

    glm::vec3 particlePosition = trajectory.positions.back();
    glm::vec3 gridPositionFloat = particlePosition - box.getMinimum();
    gridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
    auto gridPosition = glm::ivec3(gridPositionFloat);
    glm::vec3 frac = glm::fract(gridPositionFloat);
    glm::vec3 invFrac = glm::vec3(1.0) - frac;

    int attributeIdx = 0;
    for (auto& it : scalarFields) {
        std::vector<float>& attributes = trajectory.attributes.at(attributeIdx);
        float* scalarField = it.second;

        float interpolationValue =
                invFrac.x * invFrac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,0,0))
                + frac.x * invFrac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,0,0))
                + invFrac.x * frac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,1,0))
                + frac.x * frac.y * invFrac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,1,0))
                + invFrac.x * invFrac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,0,1))
                + frac.x * invFrac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,0,1))
                + invFrac.x * frac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(0,1,1))
                + frac.x * frac.y * frac.z * _getScalarFieldAtIdx(scalarField, gridPosition + glm::ivec3(1,1,1));

        attributes.push_back(interpolationValue);
        attributeIdx++;
    }
}

void StreamlineTracingGrid::_pushRibbonDirections(
        const StreamlineTracingSettings& tracingSettings,
        const Trajectory& trajectory, std::vector<glm::vec3>& ribbonDirections, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    glm::vec3 lastRibbonDirection = glm::normalize(tracingSettings.initialRibbonDirection);
    ribbonDirections.reserve(trajectory.positions.size());

    size_t n = trajectory.positions.size();
    for (size_t i = 0; i < n; i++) {
        glm::vec3 tangent;
        if (i == 0) {
            tangent = trajectory.positions[i + 1] - trajectory.positions[i];
        } else if (i == n - 1) {
            tangent = trajectory.positions[i] - trajectory.positions[i - 1];
        } else {
            tangent = trajectory.positions[i + 1] - trajectory.positions[i - 1];
        }

        float tangentLength = glm::length(tangent);
        if (tangentLength < 1e-7f) {
            //sgl::Logfile::get()->writeWarning(
            //        "Warning in StreamlineTracingGrid::_pushRibbonDirections: "
            //        "The line segment length is smaller than 1e-7.");
        }
        tangent = glm::normalize(tangent);

        glm::vec3 particlePosition = trajectory.positions.at(i);

        glm::vec3 helperAxis = lastRibbonDirection;
        if (glm::length(glm::cross(helperAxis, tangent)) < 1e-2f) {
            // If tangent == lastRibbonDirection
            helperAxis = glm::vec3(0.0f, 0.0f, 1.0f);
            if (glm::length(glm::cross(helperAxis, tangent)) < 1e-2f) {
                // If tangent == helperAxis
                helperAxis = glm::vec3(0.0f, 1.0f, 0.0f);
            }
        }
        // Gram-Schmidt
        glm::vec3 ribbonDirection = glm::normalize(helperAxis - glm::dot(helperAxis, tangent) * tangent);

        if (tracingSettings.useHelicity) {
            float helicity = _getScalarFieldAtPosition(helicityField, particlePosition);
            if (!forwardMode) {
                helicity *= -1.0f;
            }
            float lineSegmentLength = 0.0f;
            if (i < trajectory.positions.size() - 1) {
                lineSegmentLength = glm::length(trajectory.positions.at(i + 1) - trajectory.positions.at(i));
            }
            float helicityAngle =
                    helicity / maxHelicityMagnitude * sgl::PI * tracingSettings.maxHelicityTwist
                    * lineSegmentLength / 0.005f;
            ribbonDirection = glm::rotate(ribbonDirection, helicityAngle, tangent);
        }

        ribbonDirections.push_back(ribbonDirection);
        lastRibbonDirection = ribbonDirection;
    }
}

void StreamlineTracingGrid::_reverseTrajectory(Trajectory& trajectory) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    if (trajectory.positions.size() <= 1) {
        return;
    }

    std::reverse(trajectory.positions.begin(), trajectory.positions.end());
    for (auto& attribute : trajectory.attributes) {
        std::reverse(attribute.begin(), attribute.end());
    }
}

void StreamlineTracingGrid::_reverseRibbon(Trajectory& trajectory, std::vector<glm::vec3>& ribbonDirections) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    if (trajectory.positions.size() <= 1) {
        return;
    }

    std::reverse(trajectory.positions.begin(), trajectory.positions.end());
    std::reverse(ribbonDirections.begin(), ribbonDirections.end());
    for (auto& attribute : trajectory.attributes) {
        std::reverse(attribute.begin(), attribute.end());
    }
}

void StreamlineTracingGrid::_insertBackwardTrajectory(const Trajectory& trajectoryBackward, Trajectory& trajectory) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    if (trajectoryBackward.positions.size() > 1) {
        trajectory.positions.insert(
                trajectory.positions.begin(),
                trajectoryBackward.positions.begin(),
                trajectoryBackward.positions.end() - 1);
        for (size_t attrIdx = 0; attrIdx < trajectoryBackward.attributes.size(); attrIdx++) {
            trajectory.attributes.at(attrIdx).insert(
                    trajectory.attributes.at(attrIdx).begin(),
                    trajectoryBackward.attributes.at(attrIdx).begin(),
                    trajectoryBackward.attributes.at(attrIdx).end() - 1);
        }
    }
}

void StreamlineTracingGrid::_insertBackwardRibbon(
        const Trajectory& trajectoryBackward, const std::vector<glm::vec3>& ribbonDirectionsBackward,
        Trajectory& trajectory, std::vector<glm::vec3>& ribbonDirections) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    if (trajectoryBackward.positions.size() > 1) {
        trajectory.positions.insert(
                trajectory.positions.begin(),
                trajectoryBackward.positions.begin(),
                trajectoryBackward.positions.end() - 1);
        ribbonDirections.insert(
                ribbonDirections.begin(),
                ribbonDirectionsBackward.begin(),
                ribbonDirectionsBackward.end() - 1);
        for (size_t attrIdx = 0; attrIdx < trajectoryBackward.attributes.size(); attrIdx++) {
            trajectory.attributes.at(attrIdx).insert(
                    trajectory.attributes.at(attrIdx).begin(),
                    trajectoryBackward.attributes.at(attrIdx).begin(),
                    trajectoryBackward.attributes.at(attrIdx).end() - 1);
        }
    }
}

void StreamlineTracingGrid::_trace(
        const StreamlineTracingSettings& tracingSettings, Trajectory& trajectory,
        std::vector<glm::vec3>& ribbonDirections, const glm::vec3& seedPoint, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    float dt = 1.0f / maxVectorMagnitude * std::min(dx, std::min(dy, dz)) * tracingSettings.timeStepScale;
    float terminationDistance = 1e-6f * tracingSettings.terminationDistance;

    glm::vec3 particlePosition = seedPoint;
    glm::vec3 oldParticlePosition;

    int iterationCounter = 0;
    const int MAX_ITERATIONS = std::min(
            int(std::round(float(tracingSettings.maxNumIterations) / tracingSettings.timeStepScale)),
            tracingSettings.maxNumIterations * 10);
    float lineLength = 0.0;
    const float MAX_LINE_LENGTH =
            glm::length(box.getDimensions()) * (float(tracingSettings.maxNumIterations) / float(2000));
    while (iterationCounter <= MAX_ITERATIONS && lineLength <= MAX_LINE_LENGTH) {
        oldParticlePosition = particlePosition;

        // Break if the position is outside of the domain.
        if (!box.contains(particlePosition)) {
            if (!trajectory.positions.empty()) {
                // Clamp the position to the boundary.
                glm::vec3 rayOrigin = trajectory.positions.back();
                glm::vec3 rayDirection = glm::normalize(particlePosition - rayOrigin);
                float tNear, tFar;
                _rayBoxIntersection(
                        rayOrigin, rayDirection, box.getMinimum(), box.getMaximum(), tNear, tFar);
                glm::vec3 boundaryParticlePosition;
                if (tNear > 0.0f) {
                    boundaryParticlePosition = rayOrigin + tNear * rayDirection;
                } else {
                    boundaryParticlePosition = rayOrigin + tFar * rayDirection;
                }
                trajectory.positions.emplace_back(boundaryParticlePosition);
                _pushTrajectoryAttributes(trajectory);
            }
            break;
        }

        // Add line segment between last and new position.
        trajectory.positions.push_back(particlePosition);
        _pushTrajectoryAttributes(trajectory);

        // Integrate to the new position using one of the implemented integrators.
        _integrationStep(tracingSettings, particlePosition, dt, forwardMode);

        float segmentLength = glm::length(particlePosition - oldParticlePosition);
        lineLength += segmentLength;

        // Have we reached a singular point?
        if (segmentLength < terminationDistance) {
            break;
        }

        iterationCounter++;
    }

    if (tracingSettings.flowPrimitives == FlowPrimitives::STREAMRIBBONS) {
        _pushRibbonDirections(tracingSettings, trajectory, ribbonDirections, forwardMode);
    }
}


void StreamlineTracingGrid::_integrationStep(
        const StreamlineTracingSettings& tracingSettings, glm::vec3& p0, float& dt, bool forwardMode) const {
    if (tracingSettings.integrationMethod == StreamlineIntegrationMethod::EXPLICIT_EULER) {
        _integrationStepExplicitEuler(p0, dt, forwardMode);
    } else if (tracingSettings.integrationMethod == StreamlineIntegrationMethod::IMPLICIT_EULER) {
        _integrationStepImplicitEuler(p0, dt, forwardMode);
    } else if (tracingSettings.integrationMethod == StreamlineIntegrationMethod::HEUN) {
        _integrationStepHeun(p0, dt, forwardMode);
    } else if (tracingSettings.integrationMethod == StreamlineIntegrationMethod::MIDPOINT) {
        _integrationStepMidpoint(p0, dt, forwardMode);
    } else if (tracingSettings.integrationMethod == StreamlineIntegrationMethod::RK4) {
        _integrationStepRK4(p0, dt, forwardMode);
    } else if (tracingSettings.integrationMethod == StreamlineIntegrationMethod::RKF45) {
        _integrationStepRKF45(tracingSettings, p0, dt, forwardMode);
    }
}

void StreamlineTracingGrid::_integrationStepExplicitEuler(glm::vec3& p0, float& dt, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif
    p0 += dt * _getVectorAtPosition(p0, forwardMode);
}

void StreamlineTracingGrid::_integrationStepImplicitEuler(glm::vec3& p0, float& dt, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    const float EPSILON = 1e-6f;
    const int MAX_NUM_ITERATIONS = 100;
    int iteration = 0;
    glm::vec3 p_last = p0;
    float diff;
    do {
        glm::vec3 p_next = p0 + dt * _getVectorAtPosition(p_last, forwardMode);
        diff = glm::length(p_last - p_next);
        p_last = p_next;
        iteration++;
    } while (diff > EPSILON && iteration < MAX_NUM_ITERATIONS);
    if (iteration >= MAX_NUM_ITERATIONS) {
        sgl::Logfile::get()->writeError(
                "Error in StreamlineTracingGrid::_integrationStepImplicitEuler: The fixed-point iteration has not "
                "converged within a reasonable number of iterations.");
    }
    p0 = p_last;
}

void StreamlineTracingGrid::_integrationStepHeun(glm::vec3& p0, float& dt, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    glm::vec3 v0 = _getVectorAtPosition(p0, forwardMode);
    glm::vec3 p1Euler = p0 + dt * v0;
    glm::vec3 v1Euler = _getVectorAtPosition(p1Euler, forwardMode);
    p0 += dt * float(0.5) * (v0 + v1Euler);
}

void StreamlineTracingGrid::_integrationStepMidpoint(glm::vec3& p0, float& dt, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    glm::vec3 pPrime = p0 + dt * float(0.5) * _getVectorAtPosition(p0, forwardMode);
    p0 += dt * _getVectorAtPosition(pPrime, forwardMode);
}

void StreamlineTracingGrid::_integrationStepRK4(glm::vec3& p0, float& dt, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    glm::vec3 k1 = dt * _getVectorAtPosition(p0, forwardMode);
    glm::vec3 k2 = dt * _getVectorAtPosition(p0 + k1 * float(0.5), forwardMode);
    glm::vec3 k3 = dt * _getVectorAtPosition(p0 + k2 * float(0.5), forwardMode);
    glm::vec3 k4 = dt * _getVectorAtPosition(p0 + k3, forwardMode);
    p0 += k1 / float(6.0) + k2 / float(3.0) + k3 / float(3.0) + k4 / float(6.0);
}

void StreamlineTracingGrid::_integrationStepRKF45(
        const StreamlineTracingSettings& tracingSettings, glm::vec3& fP0, float& fDt, bool forwardMode) const {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    // Integrate to the new position using the Runge-Kutta-Fehlberg method (RKF45).
    // For more details see:
    // - https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method
    // - https://maths.cnam.fr/IMG/pdf/RungeKuttaFehlbergProof.pdf
    const double EPSILON =
            double(2.0 * 1e-5) * double(std::min(dx, std::min(dy, dz))) * double(tracingSettings.timeStepScale);
    const int MAX_NUM_ITERATIONS = 100;
    double dt = fDt;
    int iteration = 0;
    glm::dvec3 p0 = fP0;
    glm::dvec3 approximationRK4, approximationRK5;
    bool timestepNeedsAdaptation;
    do {
        glm::dvec3 k1 = dt * _getVectorAtPositionDouble(p0, forwardMode);
        glm::dvec3 k2 = dt * _getVectorAtPositionDouble(p0 + k1 * double(1.0 / 4.0), forwardMode);
        glm::dvec3 k3 = dt * _getVectorAtPositionDouble(
                p0 + k1 * double(3.0 / 32.0) + k2 * double(9.0 / 32.0), forwardMode);
        glm::dvec3 k4 = dt * _getVectorAtPositionDouble(
                p0 + k1 * double(1932.0 / 2197.0) - k2 * double(7200.0 / 2197.0) + k3 * double(7296.0 / 2197.0),
                forwardMode);
        glm::dvec3 k5 = dt * _getVectorAtPositionDouble(
                p0 + k1 * double(439.0 / 216.0) - k2 * double(8.0) + k3 * double(3680.0 / 513.0)
                - k4 * double(845.0 / 4104.0), forwardMode);
        glm::dvec3 k6 = dt * _getVectorAtPositionDouble(
                p0 - k1 * double(8.0 / 27.0) + k2 * double(2.0) - k3 * double(3544.0 / 2565.0)
                + k4 * double(1859.0 / 4104.0) - k5 * double(11.0 / 40.0), forwardMode);
        approximationRK4 =
                p0 + k1 * double(25.0/216.0) + k3 * double(1408.0/2565.0) + k4 * double(2197.0/4101.0)
                - k5 * double(1.0/5.0);
        approximationRK5 =
                p0 + k1 * double(16.0/135.0) + k3 * double(6656.0/12825.0) + k4 * double(28561.0/56430.0)
                - k5 * double(9.0/50.0) + k6 * double(2.0/55.0);
        double TE = glm::length(
                k1 * double(1.0/360.0) + k3 * double(-128.0/4275.0) + k4 * double(-2197.0/75240.0)
                + k5 * (1.0/50.0) + k6 * double(2.0/55.0));
        timestepNeedsAdaptation = TE > EPSILON;
        if (timestepNeedsAdaptation) {
            dt = 0.9 * dt * std::pow(EPSILON / TE, double(1.0/5.0));
        }
        iteration++;
    } while(timestepNeedsAdaptation && iteration < MAX_NUM_ITERATIONS);
    if (iteration >= MAX_NUM_ITERATIONS) {
        sgl::Logfile::get()->writeError(
                "Error in StreamlineTracingGrid::_integrationStepRKF45: The timestep adaption has not "
                "converged within a reasonable number of iterations.");
    }
    p0 = approximationRK5;
    fP0 = p0;
    fDt = float(dt);
}

void StreamlineTracingGrid::computeSimulationBoundaryMesh(
        std::vector<uint32_t>& cachedSimulationMeshOutlineTriangleIndices,
        std::vector<glm::vec3>& cachedSimulationMeshOutlineVertexPositions) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    glm::vec3 boxMin = box.getMinimum();
    glm::vec3 boxDim = box.getDimensions();

    // Back face (vertices).
    auto backFaceOffset = uint32_t(cachedSimulationMeshOutlineVertexPositions.size());
    for (int y = 0; y <= ys; y++) {
        for (int x = 0; x <= xs; x++) {
            glm::vec3 pt = boxMin + glm::vec3(float(x) / float(xs), float(y) / float(ys), 0.0f) * boxDim;
            cachedSimulationMeshOutlineVertexPositions.push_back(pt);
        }
    }
    // Front face (vertices).
    auto frontFaceOffset = uint32_t(cachedSimulationMeshOutlineVertexPositions.size());
    for (int y = 0; y <= ys; y++) {
        for (int x = 0; x <= xs; x++) {
            glm::vec3 pt = boxMin + glm::vec3(float(x) / float(xs), float(y) / float(ys), 1.0f) * boxDim;
            cachedSimulationMeshOutlineVertexPositions.push_back(pt);
        }
    }

    // Bottom face (vertices).
    auto bottomFaceOffset = uint32_t(cachedSimulationMeshOutlineVertexPositions.size());
    for (int z = 1; z < zs; z++) {
        for (int x = 0; x <= xs; x++) {
            glm::vec3 pt = boxMin + glm::vec3(float(x) / float(xs), 0.0f, float(z) / float(zs)) * boxDim;
            cachedSimulationMeshOutlineVertexPositions.push_back(pt);
        }
    }
    // Top face (vertices).
    auto topFaceOffset = uint32_t(cachedSimulationMeshOutlineVertexPositions.size());
    for (int z = 1; z < zs; z++) {
        for (int x = 0; x <= xs; x++) {
            glm::vec3 pt = boxMin + glm::vec3(float(x) / float(xs), 1.0f, float(z) / float(zs)) * boxDim;
            cachedSimulationMeshOutlineVertexPositions.push_back(pt);
        }
    }

    // Left face (vertices).
    auto leftFaceOffset = uint32_t(cachedSimulationMeshOutlineVertexPositions.size());
    for (int z = 1; z < zs; z++) {
        for (int y = 1; y < ys; y++) {
            glm::vec3 pt = boxMin + glm::vec3(0.0f, float(y) / float(ys), float(z) / float(zs)) * boxDim;
            cachedSimulationMeshOutlineVertexPositions.push_back(pt);
        }
    }
    // Right face (vertices).
    auto rightFaceOffset = uint32_t(cachedSimulationMeshOutlineVertexPositions.size());
    for (int z = 1; z < zs; z++) {
        for (int y = 1; y < ys; y++) {
            glm::vec3 pt = boxMin + glm::vec3(1.0f, float(y) / float(ys), float(z) / float(zs)) * boxDim;
            cachedSimulationMeshOutlineVertexPositions.push_back(pt);
        }
    }

    // Back face (indices).
    for (int y = 0; y < ys; y++) {
        for (int x = 0; x < xs; x++) {
            cachedSimulationMeshOutlineTriangleIndices.push_back(backFaceOffset + (x + 0) + (y + 0) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(backFaceOffset + (x + 1) + (y + 0) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(backFaceOffset + (x + 1) + (y + 1) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(backFaceOffset + (x + 0) + (y + 0) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(backFaceOffset + (x + 1) + (y + 1) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(backFaceOffset + (x + 0) + (y + 1) * (xs + 1));
        }
    }
    // Front face (indices).
    for (int y = 0; y < ys; y++) {
        for (int x = 0; x < xs; x++) {
            cachedSimulationMeshOutlineTriangleIndices.push_back(frontFaceOffset + (x + 1) + (y + 1) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(frontFaceOffset + (x + 1) + (y + 0) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(frontFaceOffset + (x + 0) + (y + 0) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(frontFaceOffset + (x + 0) + (y + 1) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(frontFaceOffset + (x + 1) + (y + 1) * (xs + 1));
            cachedSimulationMeshOutlineTriangleIndices.push_back(frontFaceOffset + (x + 0) + (y + 0) * (xs + 1));
        }
    }

    // Bottom face (indices).
    for (int z = 0; z < zs; z++) {
        for (int x = 0; x < xs; x++) {
            uint32_t mxmz;
            uint32_t pxmz;
            uint32_t mxpz;
            uint32_t pxpz;
            if (z == 0) {
                mxmz = backFaceOffset + (x + 0) + 0 * (xs + 1);
                pxmz = backFaceOffset + (x + 1) + 0 * (xs + 1);
            } else {
                mxmz = bottomFaceOffset + (x + 0) + (z - 1) * (xs + 1);
                pxmz = bottomFaceOffset + (x + 1) + (z - 1) * (xs + 1);
            }
            if (z == zs - 1) {
                mxpz = frontFaceOffset + (x + 0) + 0 * (xs + 1);
                pxpz = frontFaceOffset + (x + 1) + 0 * (xs + 1);
            } else {
                mxpz = bottomFaceOffset + (x + 0) + (z + 0) * (xs + 1);
                pxpz = bottomFaceOffset + (x + 1) + (z + 0) * (xs + 1);
            }

            cachedSimulationMeshOutlineTriangleIndices.push_back(pxpz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pxmz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mxmz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mxpz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pxpz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mxmz);
        }
    }
    // Top face (indices).
    for (int z = 0; z < zs; z++) {
        for (int x = 0; x < xs; x++) {
            uint32_t mxmz;
            uint32_t pxmz;
            uint32_t mxpz;
            uint32_t pxpz;
            if (z == 0) {
                mxmz = backFaceOffset + (x + 0) + ys * (xs + 1);
                pxmz = backFaceOffset + (x + 1) + ys * (xs + 1);
            } else {
                mxmz = topFaceOffset + (x + 0) + (z - 1) * (xs + 1);
                pxmz = topFaceOffset + (x + 1) + (z - 1) * (xs + 1);
            }
            if (z == zs - 1) {
                mxpz = frontFaceOffset + (x + 0) + ys * (xs + 1);
                pxpz = frontFaceOffset + (x + 1) + ys * (xs + 1);
            } else {
                mxpz = topFaceOffset + (x + 0) + (z + 0) * (xs + 1);
                pxpz = topFaceOffset + (x + 1) + (z + 0) * (xs + 1);
            }

            cachedSimulationMeshOutlineTriangleIndices.push_back(mxmz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pxmz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pxpz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mxmz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pxpz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mxpz);
        }
    }

    // Left face (indices).
    for (int z = 0; z < zs; z++) {
        for (int y = 0; y < ys; y++) {
            uint32_t mymz;
            uint32_t pymz;
            uint32_t mypz;
            uint32_t pypz;
            if (z == 0) {
                mymz = backFaceOffset + 0 + (y + 0) * (xs + 1);
                pymz = backFaceOffset + 0 + (y + 1) * (xs + 1);
            }
            if (z == zs - 1) {
                mypz = frontFaceOffset + 0 + (y + 0) * (xs + 1);
                pypz = frontFaceOffset + 0 + (y + 1) * (xs + 1);
            }
            if (z != 0 && y == 0) {
                mymz = bottomFaceOffset + 0 + (z - 1) * (xs + 1);
            }
            if (z != 0 && y == ys - 1) {
                pymz = topFaceOffset + 0 + (z - 1) * (xs + 1);
            }
            if (z != zs - 1 && y == 0) {
                mypz = bottomFaceOffset + 0 + (z + 0) * (xs + 1);
            }
            if (z != zs - 1 && y == ys - 1) {
                pypz = topFaceOffset + 0 + (z + 0) * (xs + 1);
            }
            if (z != 0 && y != 0) {
                mymz = leftFaceOffset + (y - 1) + (z - 1) * (ys - 1);
            }
            if (z != 0 && y != ys - 1) {
                pymz = leftFaceOffset + (y + 0) + (z - 1) * (ys - 1);
            }
            if (z != zs - 1 && y != 0) {
                mypz = leftFaceOffset + (y - 1) + (z + 0) * (ys - 1);
            }
            if (z != zs - 1 && y != ys - 1) {
                pypz = leftFaceOffset + (y + 0) + (z + 0) * (ys - 1);
            }

            cachedSimulationMeshOutlineTriangleIndices.push_back(mymz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pymz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pypz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mymz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pypz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mypz);
        }
    }
    // Right face (indices).
    for (int z = 0; z < zs; z++) {
        for (int y = 0; y < ys; y++) {
            uint32_t mymz;
            uint32_t pymz;
            uint32_t mypz;
            uint32_t pypz;
            if (z == 0) {
                mymz = backFaceOffset + xs + (y + 0) * (xs + 1);
                pymz = backFaceOffset + xs + (y + 1) * (xs + 1);
            }
            if (z == zs - 1) {
                mypz = frontFaceOffset + xs + (y + 0) * (xs + 1);
                pypz = frontFaceOffset + xs + (y + 1) * (xs + 1);
            }
            if (z != 0 && y == 0) {
                mymz = bottomFaceOffset + xs + (z - 1) * (xs + 1);
            }
            if (z != 0 && y == ys - 1) {
                pymz = topFaceOffset + xs + (z - 1) * (xs + 1);
            }
            if (z != zs - 1 && y == 0) {
                mypz = bottomFaceOffset + xs + (z + 0) * (xs + 1);
            }
            if (z != zs - 1 && y == ys - 1) {
                pypz = topFaceOffset + xs + (z + 0) * (xs + 1);
            }
            if (z != 0 && y != 0) {
                mymz = rightFaceOffset + (y - 1) + (z - 1) * (ys - 1);
            }
            if (z != 0 && y != ys - 1) {
                pymz = rightFaceOffset + (y + 0) + (z - 1) * (ys - 1);
            }
            if (z != zs - 1 && y != 0) {
                mypz = rightFaceOffset + (y - 1) + (z + 0) * (ys - 1);
            }
            if (z != zs - 1 && y != ys - 1) {
                pypz = rightFaceOffset + (y + 0) + (z + 0) * (ys - 1);
            }

            cachedSimulationMeshOutlineTriangleIndices.push_back(pypz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pymz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mymz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mypz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(pypz);
            cachedSimulationMeshOutlineTriangleIndices.push_back(mymz);
        }
    }
}
