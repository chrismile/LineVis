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

#include "StressLineTracingRequesterSocket.hpp"

class DataSetInformation;

enum class SeedStrategy {
    VOLUME, SURFACE, LOADING_AREA, APPROX_TOPOLOGY
};
const char* const SEED_STRATEGY_NAMES[] = {
        "Homogeneous Volume Seeding",
        "Surface Seeding",
        "Loading Areas Seeding",
        "Approximate Topology Seeding"
};
const char* const SEED_STRATEGY_ABBREVIATIONS[] = {
        "Volume",
        "Surface",
        "LoadingArea",
        "ApproxTopology"
};
const int NUM_SEED_STRATEGIES = ((int)(sizeof(SEED_STRATEGY_NAMES) / sizeof(*SEED_STRATEGY_NAMES)));

class StressLineTracingRequester {
public:
    StressLineTracingRequester();
    void renderGui();
    bool getHasNewData(DataSetInformation& dataSetInformation);

private:
    void loadMeshList();
    void requestNewData();
    StressLineTracingRequesterSocket worker;
    bool showWindow = true;

    // UI settings.
    std::vector<std::string> meshNames;
    std::vector<std::string> meshFilenames;
    int selectedMeshIndex = 0;

    // Line tracer settings.
    std::string meshFilename;
    SeedStrategy seedStrategy = SeedStrategy::SURFACE;
    int minimumEpsilon = 10;
    int numLevels = 5;

    // Expert options.
    int maxAngleDeviation = 10;
    bool snappingOpt = true;
    int minPslLength = 10;
    int volumeSeedingOpt = 5;
};

#endif //LINEVIS_STRESSLINETRACINGREQUESTER_HPP
