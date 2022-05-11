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

#include <iostream>
#include <Math/Geometry/AABB3.hpp>
#include <Math/Geometry/Sphere.hpp>
#include <ImGui/imgui.h>
#include <ImGui/imgui_custom.h>
#include "StreamlineTracingDefines.hpp"
#include "StreamlineTracingGrid.hpp"
#include "StreamlineSeeder.hpp"

StreamlineSeeder* StreamlinePlaneSeeder::copy() {
    auto* seederCopy = new StreamlinePlaneSeeder;
    seederCopy->regular = regular;
    seederCopy->seed = seed;
    seederCopy->numSamplesX = numSamplesX;
    seederCopy->numSamplesY = numSamplesY;
    seederCopy->numSamplesRandom = numSamplesRandom;
    seederCopy->planeSlice = this->planeSlice;
    seederCopy->direction = this->direction;
    seederCopy->planeOffset = this->planeOffset;
    seederCopy->planeNormal = this->planeNormal;
    return seederCopy;
}

void StreamlinePlaneSeeder::setNewGridBox(const sgl::AABB3& gridBox) {
}

void StreamlinePlaneSeeder::reset(StreamlineTracingSettings& tracingSettings, StreamlineTracingGrid* newGrid) {
    grid = newGrid;
    box = grid->getBox();
    generator = std::mt19937(seed);
    numSamplesRandom = tracingSettings.numPrimitives;

    glm::vec3 dimensions = box.getDimensions();
    maxDimension = 0.0f;
    maxDimension = std::max(dimensions.x, std::max(dimensions.y, dimensions.z));

    glm::vec3 minVec = box.getMinimum();
    glm::vec3 maxVec = box.getMaximum();
    glm::vec3 cornerPoints[8] = {
            glm::vec3(minVec.x, minVec.y, minVec.z),
            glm::vec3(maxVec.x, minVec.y, minVec.z),
            glm::vec3(minVec.x, maxVec.y, minVec.z),
            glm::vec3(maxVec.x, maxVec.y, minVec.z),
            glm::vec3(minVec.x, minVec.y, maxVec.z),
            glm::vec3(maxVec.x, minVec.y, maxVec.z),
            glm::vec3(minVec.x, maxVec.y, maxVec.z),
            glm::vec3(maxVec.x, maxVec.y, maxVec.z),
    };

    float minOffset = std::numeric_limits<float>::max();
    float maxOffset = std::numeric_limits<float>::lowest();
    for (const glm::vec3& pt : cornerPoints) {
        float offset = glm::dot(planeNormal, pt - box.getCenter());
        minOffset = std::min(minOffset, offset);
        maxOffset = std::max(maxOffset, offset);
    }
    planeOffset = minOffset + (maxOffset - minOffset) * planeSlice;

    axis0 = glm::vec3(1.0f, 0.0f, 0.0f);
    axis1 = glm::normalize(glm::cross(axis0, planeNormal));
    if (glm::length(axis1) < 1e-3f) {
        axis0 = glm::vec3(0.0f, 1.0f, 0.0f);;
        axis1 = glm::normalize(glm::cross(axis0, planeNormal));
    }
    axis0 = glm::cross(planeNormal, axis1);

    if (regular) {
        currentSampleIdx = 0;
        tracingSettings.numPrimitives = numSamplesX * numSamplesY;
    }
}

bool StreamlinePlaneSeeder::hasNextPoint() {
    if (regular) {
        return currentSampleIdx < numSamplesX * numSamplesY;
    } else {
        return currentSampleIdx < numSamplesRandom;
    }
}

glm::vec3 StreamlinePlaneSeeder::getNextPoint() {
    if (regular) {
        int y = currentSampleIdx / numSamplesX;
        int x = currentSampleIdx % numSamplesX;
        currentSampleIdx++;

        float dx = 1.0f / float(numSamplesX + 1);
        float dy = 1.0f / float(numSamplesY + 1);
        glm::vec3 samplePoint = box.getCenter() + planeOffset * planeNormal;
        samplePoint += axis0 * maxDimension * dx * (float(x) - float(numSamplesX - 1) / 2.0f);
        samplePoint += axis1 * maxDimension * dy * (float(y) - float(numSamplesY - 1) / 2.0f);
        return samplePoint;
    } else {
        const int MAX_NUM_ITERATIONS = 100;
        for (int it = 0; it < MAX_NUM_ITERATIONS; it++) {
            float r0 = uniformDistribution(generator);
            float r1 = uniformDistribution(generator);
            glm::vec3 samplePoint = box.getCenter() + planeOffset * planeNormal;
            float dx = 1.0f / maxDimension;
            float dy = 1.0f / maxDimension;
            samplePoint += axis0 * dx * (r0 - 0.5f);
            samplePoint += axis1 * dy * (r1 - 0.5f);
            if (box.contains(samplePoint)) {
                return samplePoint;
            }
        }
    }
    return box.getCenter(); //< fallback
}

bool StreamlinePlaneSeeder::renderGui() {
    bool changed = false;

    if (ImGui::SliderFloat("Slice", &planeSlice, 0.0f, 1.0f)) {
        changed = true;
    }

    if (ImGui::InputFloat3("Direction", &direction.x)) {
        planeNormal = glm::normalize(direction);
        changed = true;
    }

    if (regular) {
        ImGui::EditMode editMode = ImGui::SliderInt2Edit(
                "#Primitives (X/Z)", numSamplesArrayGui, 1, 128);
        if (editMode == ImGui::EditMode::INPUT_FINISHED) {
            numSamplesX = numSamplesArrayGui[0];
            numSamplesY = numSamplesArrayGui[1];
            changed = true;
        }
    } else {
        if (ImGui::InputInt("Random Seed", &seed)) {
            if (seed < 0) {
                seed = 0;
            }
            changed = true;
        }
    }

    if (ImGui::Checkbox("Regular Seeding", &regular)) {
        changed = true;
    }

    return changed;
}

bool StreamlinePlaneSeeder::setNewSettings(const SettingsMap& settings) {
    bool changed = false;

    changed |= settings.getValueOpt("slice", planeSlice);
    changed |= settings.getValueOpt("direction", direction);
    changed |= settings.getValueOpt("regular", regular);
    if (settings.getValueOpt("num_samples_x", numSamplesX)) {
        numSamplesArrayGui[0] = numSamplesX;
        regular = true;
        changed = true;
    }
    if (settings.getValueOpt("num_samples_y", numSamplesY)) {
        numSamplesArrayGui[1] = numSamplesY;
        regular = true;
        changed = true;
    }
    if (settings.getValueOpt("random_seed", seed)) {
        if (seed < 0) {
            seed = 0;
        }
        changed = true;
    }

    return changed;
}


StreamlineSeeder* StreamlineVolumeSeeder::copy() {
    auto* seederCopy = new StreamlineVolumeSeeder;
    seederCopy->gridBoxUi = gridBoxUi;
    seederCopy->box = gridBoxUi;
    seederCopy->maxDimension = maxDimension;
    seederCopy->regular = regular;
    seederCopy->seed = seed;
    seederCopy->numSamplesX = numSamplesX;
    seederCopy->numSamplesY = numSamplesY;
    seederCopy->numSamplesZ = numSamplesZ;
    seederCopy->numSamplesRandom = numSamplesRandom;
    return seederCopy;
}

void StreamlineVolumeSeeder::setNewGridBox(const sgl::AABB3& gridBox) {
    gridBoxUi = gridBox;
    glm::vec3 dimensions = gridBoxUi.getDimensions();

    maxDimension = 0.0f;
    maxDimension = std::max(dimensions.x, std::max(dimensions.y, dimensions.z));

    float sideLengthBox = std::cbrt(float(1024));
    float boxVolume = dimensions.x * dimensions.y * dimensions.z;
    float factor = std::cbrt(std::pow(maxDimension, 3.0f) / boxVolume) * sideLengthBox;
    numSamplesX = int(std::round(dimensions.x / maxDimension * factor));
    numSamplesY = int(std::round(dimensions.y / maxDimension * factor));
    numSamplesZ = int(std::round(dimensions.z / maxDimension * factor));
    numSamplesArrayGui[0] = numSamplesX;
    numSamplesArrayGui[1] = numSamplesY;
    numSamplesArrayGui[2] = numSamplesZ;
}

void StreamlineVolumeSeeder::reset(StreamlineTracingSettings& tracingSettings, StreamlineTracingGrid* newGrid) {
    grid = newGrid;
    box = grid->getBox();
    generator = std::mt19937(seed);
    numSamplesRandom = tracingSettings.numPrimitives;

    if (gridBoxUi.getMinimum() != box.getMinimum() || gridBoxUi.getMaximum() != box.getMaximum()) {
        setNewGridBox(box);
    }
    if (regular) {
        currentSampleIdx = 0;
        tracingSettings.numPrimitives = numSamplesX * numSamplesY * numSamplesZ;
    }
}

bool StreamlineVolumeSeeder::hasNextPoint() {
    if (regular) {
        return currentSampleIdx < numSamplesX * numSamplesY * numSamplesZ;
    } else {
        return currentSampleIdx < numSamplesRandom;
    }
}

glm::vec3 StreamlineVolumeSeeder::getNextPoint() {
    if (regular) {
        int z = currentSampleIdx / (numSamplesX * numSamplesY);
        int xy = currentSampleIdx % (numSamplesX * numSamplesY);
        int y = xy / numSamplesX;
        int x = xy % numSamplesX;
        currentSampleIdx++;

        glm::vec3 boxMin = box.getMinimum();
        glm::vec3 dimensions = box.getDimensions();
        float dx = 1.0f / float(numSamplesX + 1);
        float dy = 1.0f / float(numSamplesY + 1);
        float dz = 1.0f / float(numSamplesZ + 1);
        glm::vec3 samplePoint(
                boxMin.x + dimensions.x * dx * float(x + 1),
                boxMin.y + dimensions.y * dy * float(y + 1),
                boxMin.z + dimensions.z * dz * float(z + 1));
        return samplePoint;
    } else {
        const int MAX_NUM_ITERATIONS = 100;
        for (int it = 0; it < MAX_NUM_ITERATIONS; it++) {
            float r0 = uniformDistribution(generator) * maxDimension;
            float r1 = uniformDistribution(generator) * maxDimension;
            float r2 = uniformDistribution(generator) * maxDimension;
            glm::vec3 samplePoint(r0, r1, r2);
            samplePoint += box.getMinimum();
            if (box.contains(samplePoint)) {
                return samplePoint;
            }
        }
        currentSampleIdx++;

        // Fallback if no sample point was found in a reasonable amount of iterations.
        glm::vec3 samplePoint(
                uniformDistribution(generator),
                uniformDistribution(generator),
                uniformDistribution(generator));
        samplePoint *= box.getDimensions();
        samplePoint += box.getMinimum();
        return samplePoint;
    }
}

bool StreamlineVolumeSeeder::renderGui() {
    bool changed = false;

    if (regular) {
        ImGui::EditMode editMode = ImGui::SliderInt3Edit(
                "#Primitives (X/Y/Z)", numSamplesArrayGui, 1, 128);
        if (editMode == ImGui::EditMode::INPUT_FINISHED) {
            numSamplesX = numSamplesArrayGui[0];
            numSamplesY = numSamplesArrayGui[1];
            numSamplesZ = numSamplesArrayGui[2];
            changed = true;
        }
    } else {
        if (ImGui::InputInt("Random Seed", &seed)) {
            if (seed < 0) {
                seed = 0;
            }
            changed = true;
        }
    }

    if (ImGui::Checkbox("Regular Seeding", &regular)) {
        changed = true;
    }

    return changed;
}

bool StreamlineVolumeSeeder::setNewSettings(const SettingsMap& settings) {
    bool changed = false;

    changed |= settings.getValueOpt("regular", regular);
    if (settings.getValueOpt("num_samples_x", numSamplesX)) {
        numSamplesArrayGui[0] = numSamplesX;
        regular = true;
        changed = true;
    }
    if (settings.getValueOpt("num_samples_y", numSamplesY)) {
        numSamplesArrayGui[1] = numSamplesY;
        regular = true;
        changed = true;
    }
    if (settings.getValueOpt("num_samples_z", numSamplesZ)) {
        numSamplesArrayGui[2] = numSamplesZ;
        regular = true;
        changed = true;
    }
    if (settings.getValueOpt("random_seed", seed)) {
        if (seed < 0) {
            seed = 0;
        }
        changed = true;
    }

    return changed;
}


StreamlineMaxHelicityFirstSeeder::StreamlineMaxHelicityFirstSeeder() = default;

StreamlineMaxHelicityFirstSeeder::~StreamlineMaxHelicityFirstSeeder() = default;

void StreamlineMaxHelicityFirstSeeder::reset(
        StreamlineTracingSettings& tracingSettings, StreamlineTracingGrid* newGrid) {
    grid = newGrid;
    box = grid->getBox();
    xs = grid->getGridSizeX();
    ys = grid->getGridSizeY();
    zs = grid->getGridSizeZ();
    dx = grid->getDx();
    dy = grid->getDy();
    dz = grid->getDz();
    minimumSeparationDistance = tracingSettings.minimumSeparationDistance;
    terminationCheckType = tracingSettings.terminationCheckType;
    gridSubsamplingFactor = tracingSettings.gridSubsamplingFactor;

    if (terminationCheckType == TerminationCheckType::GRID_BASED) {
        cellOccupancyGrid.resize((xs - 1) * (ys - 1) * (zs - 1), false);
    } else if (terminationCheckType == TerminationCheckType::HASHED_GRID_BASED) {
        hashedGrid = HashedGrid<Empty>(
                std::max(((xs - 1) * (ys - 1) * (zs - 1)) / 4, 1), std::min(dx, std::min(dy, dz)));
    }

    float* helicityField = grid->getHelicityField();
    if (gridSubsamplingFactor == 1) {
        samplePriorityQueue.reserve((xs - 2) * (ys - 2) * (zs - 2));
        for (int z = 1; z < zs - 1; z++) {
            for (int y = 1; y < ys - 1; y++) {
                for (int x = 1; x < xs - 1; x++) {
                    glm::vec3 boxMin = box.getMinimum();
                    glm::vec3 dimensions = box.getDimensions();
                    glm::vec3 samplePoint(
                            boxMin.x + dimensions.x * float(x) / float(xs),
                            boxMin.y + dimensions.y * float(y) / float(ys),
                            boxMin.z + dimensions.z * float(z) / float(zs));
                    samplePriorityQueue.emplace_back(helicityField[IDXS(x, y, z)], samplePoint);
                }
            }
        }
    } else {
        int numCellsX = (xs - 1) / gridSubsamplingFactor;
        int numCellsY = (ys - 1) / gridSubsamplingFactor;
        int numCellsZ = (zs - 1) / gridSubsamplingFactor;
        samplePriorityQueue.reserve(numCellsX * numCellsY * numCellsZ);
        for (int z = 0; z < numCellsZ; z++) {
            for (int y = 0; y < numCellsY; y++) {
                for (int x = 0; x < numCellsX; x++) {
                    glm::vec3 boxMin = box.getMinimum();
                    glm::vec3 dimensions = box.getDimensions();
                    glm::vec3 samplePoint(
                            boxMin.x + dimensions.x * (float(x) + 0.5f) / float(numCellsX),
                            boxMin.y + dimensions.y * (float(y) + 0.5f) / float(numCellsY),
                            boxMin.z + dimensions.z * (float(z) + 0.5f) / float(numCellsZ));
                    int xgrid = std::min(x * gridSubsamplingFactor, xs - 1);
                    int ygrid = std::min(y * gridSubsamplingFactor, ys - 1);
                    int zgrid = std::min(z * gridSubsamplingFactor, zs - 1);
                    samplePriorityQueue.emplace_back(
                            std::abs(helicityField[IDXS(xgrid, ygrid, zgrid)]), samplePoint);
                }
            }
        }
    }
    samplePriorityQueue.shrink_to_fit();
    std::sort(samplePriorityQueue.begin(), samplePriorityQueue.end());
}

bool StreamlineMaxHelicityFirstSeeder::hasNextPoint() {
    while (!samplePriorityQueue.empty()) {
        nextSamplePoint = samplePriorityQueue.back().samplePosition;
        samplePriorityQueue.pop_back();

        if (terminationCheckType == TerminationCheckType::GRID_BASED) {
            glm::vec3 gridPositionFloat = nextSamplePoint - box.getMinimum();
            gridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
            auto gridPosition = glm::ivec3(gridPositionFloat);
            gridPosition.x = glm::clamp(gridPosition.x, 0, xs - 2);
            gridPosition.y = glm::clamp(gridPosition.y, 0, ys - 2);
            gridPosition.z = glm::clamp(gridPosition.z, 0, zs - 2);

            if (!cellOccupancyGrid.at(IDXS_C(gridPosition.x, gridPosition.y, gridPosition.z))) {
                return true;
            }
        } else if (terminationCheckType == TerminationCheckType::KD_TREE_BASED) {
            if (!kdTree.getHasPointCloserThan(nextSamplePoint, minimumSeparationDistance)) {
                return true;
            }
        } else if (terminationCheckType == TerminationCheckType::HASHED_GRID_BASED) {
            if (!hashedGrid.getHasPointCloserThan(nextSamplePoint, minimumSeparationDistance)) {
                return true;
            }
        } else {
            return true;
        }
    }

    return false;
}

glm::vec3 StreamlineMaxHelicityFirstSeeder::getNextPoint() {
    return nextSamplePoint;
}

void StreamlineMaxHelicityFirstSeeder::addFinishedTrajectory(const Trajectory& trajectory) {
    if (terminationCheckType == TerminationCheckType::GRID_BASED) {
        for (const glm::vec3& position : trajectory.positions) {
            sgl::AABB3 boundingBox(
                    position - glm::vec3(minimumSeparationDistance),
                    position + glm::vec3(minimumSeparationDistance));
            sgl::Sphere boundingSphere(position, minimumSeparationDistance);

            glm::vec3 minGridPositionFloat = boundingBox.getMinimum() - box.getMinimum();
            minGridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
            auto minGridPosition = glm::ivec3(minGridPositionFloat);
            minGridPosition.x = glm::clamp(minGridPosition.x, 0, xs - 2);
            minGridPosition.y = glm::clamp(minGridPosition.y, 0, ys - 2);
            minGridPosition.z = glm::clamp(minGridPosition.z, 0, zs - 2);

            glm::vec3 maxGridPositionFloat = boundingBox.getMaximum() - box.getMinimum();
            maxGridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
            auto maxGridPosition = glm::ivec3(maxGridPositionFloat);
            maxGridPosition.x = glm::clamp(maxGridPosition.x, 0, xs - 2);
            maxGridPosition.y = glm::clamp(maxGridPosition.y, 0, ys - 2);
            maxGridPosition.z = glm::clamp(maxGridPosition.z, 0, zs - 2);

            for (int z = minGridPosition.z; z <= maxGridPosition.z; ++z) {
                for (int y = minGridPosition.y; y <= maxGridPosition.y; ++y) {
                    for (int x = minGridPosition.x; x <= maxGridPosition.x; ++x) {
                        sgl::AABB3 cellBoundingBox(
                                glm::vec3(
                                        float(x) * dx, float(y) * dy, float(z) * dz) + box.getMinimum(),
                                glm::vec3(
                                        float(x+1) * dx, float(y+1) * dy, float(z+1) * dz) + box.getMinimum());
                        if (boundingSphere.contains(cellBoundingBox)
                                || boundingSphere.intersects(cellBoundingBox)) {
                            cellOccupancyGrid.at(IDXS_C(x, y, z)) = true;
                        }
                    }
                }
            }
        }
    } else if (terminationCheckType == TerminationCheckType::KD_TREE_BASED) {
        for (const glm::vec3& position : trajectory.positions) {
            kdTreePointCache.push_back(position);
        }
        kdTree.build(kdTreePointCache);
    } else if (terminationCheckType == TerminationCheckType::HASHED_GRID_BASED) {
        for (const glm::vec3& position : trajectory.positions) {
            hashedGrid.add(std::make_pair(position, Empty{}));
        }
    }
}

bool StreamlineMaxHelicityFirstSeeder::isPointTerminated(const glm::vec3& point) {
    if (terminationCheckType == TerminationCheckType::GRID_BASED) {
        glm::vec3 gridPositionFloat = point - box.getMinimum();
        gridPositionFloat *= glm::vec3(1.0f / dx, 1.0f / dy, 1.0f / dz);
        auto gridPosition = glm::ivec3(gridPositionFloat);
        gridPosition.x = glm::clamp(gridPosition.x, 0, xs - 2);
        gridPosition.y = glm::clamp(gridPosition.y, 0, ys - 2);
        gridPosition.z = glm::clamp(gridPosition.z, 0, zs - 2);
        return cellOccupancyGrid.at(IDXS_C(gridPosition.x, gridPosition.y, gridPosition.z));
    } else if (terminationCheckType == TerminationCheckType::KD_TREE_BASED) {
        return kdTree.getHasPointCloserThan(point, minimumSeparationDistance);
    } else if (terminationCheckType == TerminationCheckType::HASHED_GRID_BASED) {
        return hashedGrid.getHasPointCloserThan(point, minimumSeparationDistance);
    }
    return false;
}
