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

#ifndef LINEVIS_STRESSLINETRACINGREQUESTER_HPP
#define LINEVIS_STRESSLINETRACINGREQUESTER_HPP

#include <Utils/InternalState.hpp>
#include "StressLineTracingRequesterSocket.hpp"

class DataSetInformation;

enum class SeedStrategy {
    VOLUME, SURFACE, LOADING_AREA, APPROX_TOPOLOGY
};
const char* const SEED_STRATEGY_NAMES[] = {
        "Homogeneous Volume Seeding",
        "Surface Seeding",
        "Loading Areas Seeding",
        "Fixed Area Seeding"
};
const char* const SEED_STRATEGY_ABBREVIATIONS[] = {
        "Volume",
        "Surface",
        "LoadingArea",
        "FixedArea"
};

enum class TracingAlgorithm {
    EULER, RK2, RK4
};
const char* const TRACING_ALGORITHM_NAMES[] = {
        "Euler", "Runge-Kutta 2nd Order", "Runge-Kutta 4th Order"
};
const char* const TRACING_ALGORITHM_ABBREVIATIONS[] = {
        "Euler", "RK2", "RK4"
};

class StressLineTracingRequester {
public:
    /**
     * @param context The ZeroMQ context.
     */
    explicit StressLineTracingRequester(void* context);
    void renderGui();
    void setLineTracerSettings(const SettingsMap& settings);
    bool getHasNewData(DataSetInformation& dataSetInformation);

    /**
     * @return Whether a request is currently processed (for UI progress spinner).
     */
    inline bool getIsProcessingRequest() const { return worker.getIsProcessingRequest(); }

private:
    void loadMeshList();
    void requestNewData();

    // ZeroMQ context
    void* context;
    StressLineTracingRequesterSocket worker;
    bool showWindow = true;
    bool showAdvancedSettings = false;

    // UI settings.
    std::vector<std::string> meshNames;
    std::vector<std::string> meshFilenames;
    int selectedMeshIndex = 0;

    // Line tracer settings.
    std::string meshFilename;
    SeedStrategy seedStrategy = SeedStrategy::VOLUME;
    float lineDensCtrl = 16; // 0 means default
    float seedDensCtrl = 4; // 0 means default
    bool useCustomLineDensity = false, useCustomSeedDensity = false, useCustomNumLevels = false;
    int numLevels = 4; // 0 means default
    // Should we trace major, medium and/or minor principal stress lines?
    bool traceMajorPS = true, traceMediumPS = false, traceMinorPS = true;

    // Expert options.
    TracingAlgorithm tracingAlgorithm = TracingAlgorithm::RK2;
    int maxAngleDeviation = 6;
    bool mergingOpt = true;
    bool snappingOpt = false;
    glm::vec3 multiMergingThresholds = glm::vec3(1, 1, 1);
};

#endif //LINEVIS_STRESSLINETRACINGREQUESTER_HPP
