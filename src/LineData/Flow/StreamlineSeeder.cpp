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

    if (gridBoxUi.getMinimum() != box.getMinimum() || gridBoxUi.getMaximum() != box.getMaximum()) {
        setNewGridBox(box);
    }
    if (regular) {
        currentSampleIdx = 0;
        tracingSettings.numPrimitives = numSamplesX * numSamplesY * numSamplesZ;
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
