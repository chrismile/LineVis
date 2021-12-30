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

#include "StreamlineTracingDefines.hpp"
#include "StreamlineSeeder.hpp"
#include "StreamlineTracingGrid.hpp"

StreamlineTracingGrid::~StreamlineTracingGrid() {
    if (velocityField) {
        delete[] velocityField;
        velocityField = nullptr;
    }

    for (auto& it : scalarFields) {
        delete[] it.second;
    }
    scalarFields.clear();
}

void StreamlineTracingGrid::setVelocityField(float *data, int xs, int ys, int zs, float dx, float dy, float dz) {
    velocityField = data;
    V = reinterpret_cast<glm::vec3*>(velocityField);
    maxVelocityMagnitude = 0.0f;
#if _OPENMP >= 201107
    #pragma omp parallel for shared(xs, ys, zs) reduction(max: maxVelocityMagnitude) default(none)
#endif
    for (int z = 0; z < zs; z++) {
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                float vx = velocityField[IDXV(x, y, z, 0)];
                float vy = velocityField[IDXV(x, y, z, 1)];
                float vz = velocityField[IDXV(x, y, z, 2)];
                float velocityMagnitude = std::sqrt(vx * vx + vy * vy + vz * vz);
                maxVelocityMagnitude = std::max(maxVelocityMagnitude, velocityMagnitude);
            }
        }
    }

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

void StreamlineTracingGrid::addScalarField(float* scalarField, const std::string& scalarName) {
    scalarFields.insert(std::make_pair(scalarName, scalarField));
}

std::vector<std::string> StreamlineTracingGrid::getScalarAttributeNames() {
    std::vector<std::string> scalarAttributeNames;
    for (auto& it : scalarFields) {
        scalarAttributeNames.push_back(it.first);
    }
    return scalarAttributeNames;
}

Trajectories StreamlineTracingGrid::traceStreamlines(StreamlineTracingSettings& tracingSettings) {
    auto seeder = tracingSettings.seeder;
    seeder->reset(tracingSettings, this);
    int numTrajectories = tracingSettings.numPrimitives;

    Trajectories trajectories;
    trajectories.resize(numTrajectories);
    for (int i = 0; i < numTrajectories; i++) {
        Trajectory& trajectory = trajectories.at(i);
        glm::vec3 seedPoint = seeder->getNextPoint();
        _traceStreamline(trajectory, seedPoint);
    }

    return trajectories;
}


float StreamlineTracingGrid::_getScalarFieldAtIdx(const float* scalarField, const glm::ivec3& gridIdx) const {
    if (gridIdx.x < 0 || gridIdx.y < 0 || gridIdx.z < 0 || gridIdx.x >= xs || gridIdx.y >= ys || gridIdx.z >= zs) {
        return 0.0f;
    }
    return scalarField[gridIdx.x + gridIdx.y * xs + gridIdx.z * xs * ys];
}

glm::vec3 StreamlineTracingGrid::_getVelocityAtIdx(const glm::ivec3& gridIdx) const {
    if (gridIdx.x < 0 || gridIdx.y < 0 || gridIdx.z < 0 || gridIdx.x >= xs || gridIdx.y >= ys || gridIdx.z >= zs) {
        return glm::vec3(0.0f);
    }
    return V[gridIdx.x + gridIdx.y * xs + gridIdx.z * xs * ys];
}

glm::vec3 StreamlineTracingGrid::_getVelocityVectorAt(const glm::vec3& particlePosition) const {
    glm::vec3 gridPositionFloat = particlePosition - box.getMinimum();
    gridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
    auto gridPosition = glm::ivec3(gridPositionFloat);
    glm::vec3 frac = glm::fract(gridPositionFloat);
    glm::vec3 invFrac = glm::vec3(1.0) - frac;
    glm::vec3 interpolationValue =
            invFrac.x * invFrac.y * invFrac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(0,0,0))
            + frac.x * invFrac.y * invFrac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(1,0,0))
            + invFrac.x * frac.y * invFrac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(0,1,0))
            + frac.x * frac.y * invFrac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(1,1,0))
            + invFrac.x * invFrac.y * frac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(0,0,1))
            + frac.x * invFrac.y * frac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(1,0,1))
            + invFrac.x * frac.y * frac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(0,1,1))
            + frac.x * frac.y * frac.z * _getVelocityAtIdx(gridPosition + glm::ivec3(1,1,1));
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

void StreamlineTracingGrid::_pushTrajectoryAttributes(Trajectory& trajectory) {
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

void StreamlineTracingGrid::_traceStreamline(Trajectory& trajectory, const glm::vec3& seedPoint) {
    const float dt = 1.0f / maxVelocityMagnitude * std::min(dx, std::min(dy, dz));

    glm::vec3 particlePosition = seedPoint;
    glm::vec3 oldParticlePosition;

    int iterationCounter = 0;
    const int MAX_ITERATIONS = 2000;
    float lineLength = 0.0;
    const float MAX_LINE_LENGTH = glm::length(box.getDimensions());
    while (iterationCounter <= MAX_ITERATIONS && lineLength <= MAX_LINE_LENGTH) {
        oldParticlePosition = particlePosition;
        // Break if the position is outside of the domain.

        if (!box.contains(particlePosition)) {
            if (!trajectory.positions.empty()) {
                // Clamp the position to the boundary.
                glm::vec3 rayOrigin = trajectory.positions.back();
                glm::vec3 rayDirection = particlePosition - rayOrigin;
                float tNear, tFar;
                _rayBoxIntersection(
                        rayOrigin, rayDirection, box.getMinimum(), box.getMaximum(), tNear, tFar);

                glm::vec3 boundaryParticlePosition = rayOrigin + tNear * rayDirection;
                trajectory.positions.push_back(boundaryParticlePosition);
                _pushTrajectoryAttributes(trajectory);
            }
            break;
        }

        // Add line segment between last and new position.
        trajectory.positions.push_back(particlePosition);
        _pushTrajectoryAttributes(trajectory);

        // Integrate to the new position using Runge-Kutta of 4th order.

        glm::vec3 k1 = dt * _getVelocityVectorAt(particlePosition);
        glm::vec3 k2 = dt * _getVelocityVectorAt(particlePosition + k1/float(2.0));
        glm::vec3 k3 = dt * _getVelocityVectorAt(particlePosition + k2/float(2.0));
        glm::vec3 k4 = dt * _getVelocityVectorAt(particlePosition + k3);
        particlePosition = particlePosition + k1/float(6.0) + k2/float(3.0) + k3/float(3.0) + k4/float(6.0);

        // Integrate to the new position using the Runge-Kutta-Fehlberg method (RKF45).
        // For more details see: https://maths.cnam.fr/IMG/pdf/RungeKuttaFehlbergProof.pdf
        /*const float tol = float(2.0 * 1e-5);
        glm::vec3 approximationRK4, approximationRK5;
        bool timestepNeedsAdaptation = false;
        do {
            glm::vec3 k1 = dt * _getVelocityVectorAt(particlePosition);
            glm::vec3 k2 = dt * _getVelocityVectorAt(particlePosition + k1*float(1.0/4.0);
            glm::vec3 k3 = dt * _getVelocityVectorAt(particlePosition + k1*float(3.0/32.0) + k2*float(9.0/32.0));
            glm::vec3 k4 = dt * _getVelocityVectorAt(
                    particlePosition + k1*float(1932.0/2197.0) - k2*float(7200.0/2197.0) + k3*float(7296.0/2197.0));
            glm::vec3 k5 = dt * _getVelocityVectorAt(
                    particlePosition + k1*float(439.0/216.0) - k2*float(8.0) + k3*float(3680.0/513.0)
                    - k4*float(845.0/4104.0));
            glm::vec3 k6 = dt * _getVelocityVectorAt(
                    particlePosition - k1*float(8.0/27.0) + k2*float(2.0) - k3*float(3544.0/2565.0)
                    + k4*float(1859.0/4104.0) - k5*float(11.0/40.0));
            approximationRK4 = particlePosition + k1*float(25.0/216.0) + k3*float(1408.0/2565.0)
                    + k4*float(2197.0/4101.0) - k5*float(1.0/5.0);
            approximationRK5 = particlePosition + k1*float(16.0/135.0) + k3*float(6656.0/12825.0)
                    + k4*float(28561.0/56430.0) - k5*float(9.0/50.0) + k6*float(2.0/55.0);
            float s = std::pow(tol*dt / (float(2.0) * glm::length(approximationRK5 - approximationRK4)), float(1.0/4.0));
            if (s < 0.9 || s > 1.1) {
                timestepNeedsAdaptation = true;
            }
            dt = s*dt;
        } while(timestepNeedsAdaptation);
        particlePosition = approximationRK4;*/

        float segmentLength = glm::length(particlePosition - oldParticlePosition);
        lineLength += segmentLength;

        // Have we reached a singular point?
        if (segmentLength < float(0.000001)) {
            break;
        }

        oldParticlePosition = particlePosition;
        iterationCounter++;
    }
}
