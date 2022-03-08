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

#include <algorithm>

#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/FileUtils.hpp>

#include "AutomaticPerformanceMeasurer.hpp"

AutomaticPerformanceMeasurer::AutomaticPerformanceMeasurer(std::vector<InternalState> _states,
                                   const std::string& _csvFilename, const std::string& _depthComplexityFilename,
                                   std::function<void(const InternalState&)> _newStateCallback)
        : states(_states), currentStateIndex(0), newStateCallback(_newStateCallback), file(_csvFilename),
          depthComplexityFile(_depthComplexityFilename), perfFile("performance_list.csv"),
          ppllFile("PPLLUnified.csv") {
    sgl::FileUtils::get()->ensureDirectoryExists("images/");

    // Write header
    //file.writeRow({"Name", "Average Time (ms)", "Memory (GiB)", "Buffer Size (GiB)", "Time Stamp (s), Frame Time (ns)"});
    file.writeRow({
        "Name", "Average Time (ms)", "Memory (GiB)", "Buffer Size (GiB)", "Average FPS",
        "5% Percentile FPS", "95% Percentile FPS", "PPLL Entries", "PPLL Size (GiB)"});
    depthComplexityFile.writeRow(
            {"Current State", "Frame Number", "Min Depth Complexity", "Max Depth Complexity",
             "Avg Depth Complexity Used", "Avg Depth Complexity All", "Total Number of Fragments",
             "Fragment Buffer Memory (GiB)"});
    perfFile.writeRow({"Name", "Time per frame (ms)"});
    ppllFile.writeRow({"Mode Name", "Time PPLL Clear (ms)", "Time F+C Gather (ms)", "Time PPLL Resolve (ms)"});

    // Set initial state
    setNextState(true);
}

AutomaticPerformanceMeasurer::~AutomaticPerformanceMeasurer() {
    timerVk->finishGPU();
    writeCurrentModeData();
    file.close();
    depthComplexityFile.close();
    perfFile.close();
    ppllFile.close();
}


bool AutomaticPerformanceMeasurer::update(float currentTime) {
    nextModeCounter = currentTime;
    if (nextModeCounter >= TIME_PERFORMANCE_MEASUREMENT + 0.5f) {
        nextModeCounter = 0.0f;
        if (currentStateIndex == states.size()-1) {
            return false; // Terminate program
        }
        setNextState();
    }
    return true;
}


void AutomaticPerformanceMeasurer::writeCurrentModeData() {
    double timeMS = 0.0;
    std::vector<uint64_t> frameTimesNS;

    if (!ppllTimer) {
        timerVk->finishGPU();
        timeMS = timerVk->getTimeMS(currentState.name);
        /*auto& performanceProfile = timerVk->getCurrentFrameTimeList();
        for (auto& perfPair : performanceProfile) {
            frameTimesNS.push_back(perfPair.second);
        }*/
    } else {
        ppllTimer->finishGPU();
        double timePPLLClear = ppllTimer->getTimeMS("PPLLClear");
        double timeFCGather = ppllTimer->getTimeMS("FCGather");
        double timePPLLResolve = ppllTimer->getTimeMS("PPLLResolve");
        ppllFile.writeCell(currentState.name);
        ppllFile.writeCell(std::to_string(timePPLLClear));
        ppllFile.writeCell(std::to_string(timeFCGather));
        ppllFile.writeCell(std::to_string(timePPLLResolve));
        ppllFile.newRow();

        // Push data for performance measurer.
        /*std::map<float, uint64_t> frameTimeMap;
        auto& performanceProfile = ppllTimer->getCurrentFrameTimeList();
        for (auto &perfPair : performanceProfile) {
            frameTimeMap[perfPair.first] += perfPair.second;
        }
        for (auto &perfPair : frameTimeMap) {
            frameTimesNS.push_back(perfPair.second);
        }*/
        timeMS = timePPLLClear + timeFCGather + timePPLLResolve;
    }

    // Write row with performance metrics of this mode
    file.writeCell(currentState.name);
    perfFile.writeCell(currentState.name);
    file.writeCell(sgl::toString(timeMS));

    // Write current memory consumption in gigabytes
    file.writeCell(sgl::toString(getUsedVideoMemorySizeGiB()));
    file.writeCell(sgl::toString(double(currentAlgorithmsBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));


    std::vector<float> frameTimes;
    float averageFrametime = 0.0f;
    for (uint64_t frameTimeNS : frameTimesNS) {
        auto frameTimeMS = float(double(frameTimeNS) / double(1e6));
        auto frameTimeS = float(double(frameTimeNS) / double(1e9));
        frameTimes.push_back(frameTimeS);
        averageFrametime += frameTimeS;
        perfFile.writeCell(sgl::toString(frameTimeMS));
    }
    std::sort(frameTimes.begin(), frameTimes.end());
    averageFrametime /= float(frameTimes.size());

    float averageFps = 1.0f / averageFrametime;
    int percentile5Index = int(double(frameTimes.size()) * 0.5);
    int percentile95Index = int(double(frameTimes.size()) * 0.95);
    float percentile5Fps = 1.0f / frameTimes.at(percentile95Index);
    float percentile95Fps = 1.0f / frameTimes.at(percentile5Index);
    file.writeCell(sgl::toString(averageFps));
    file.writeCell(sgl::toString(percentile5Fps));
    file.writeCell(sgl::toString(percentile95Fps));
    file.writeCell(sgl::toString(maxPPLLNumFragments));
    file.writeCell(sgl::toString(double(maxPPLLNumFragments * 12ull) / 1024.0 / 1024.0 / 1024.0));

    file.newRow();
    perfFile.newRow();
}

void AutomaticPerformanceMeasurer::setNextState(bool first) {
    if (!first) {
        writeCurrentModeData();
        currentStateIndex++;
    }

    depthComplexityFrameNumber = 0;
    currentAlgorithmsBufferSizeBytes = 0;
    currentState = states.at(currentStateIndex);
    sgl::Logfile::get()->writeInfo(std::string() + "New state: " + currentState.name);
    if (currentState.renderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        newDepthComplexityMode = true;
    }
    newStateCallback(currentState);
}

void AutomaticPerformanceMeasurer::startMeasure(float timeStamp) {
    //if (currentState.oitAlgorithm == RENDER_MODE_RAYTRACING) {
        // CPU rendering algorithm, thus use a CPU timer and not a GPU timer.
        //timerVk->startCPU(currentState.name, timeStamp);
    //} else {
    timerVk->startGPU(currentState.name);
    //}
}

void AutomaticPerformanceMeasurer::endMeasure() {
    timerVk->endGPU(currentState.name);
}

void AutomaticPerformanceMeasurer::pushDepthComplexityFrame(
        uint64_t minComplexity, uint64_t maxComplexity, float avgUsed, float avgAll, uint64_t totalNumFragments) {
    if (newDepthComplexityMode) {
        newDepthComplexityMode = false;
        maxPPLLNumFragments = 0ull;
    }
    maxPPLLNumFragments = std::max(maxPPLLNumFragments, totalNumFragments);

    depthComplexityFile.writeCell(currentState.name);
    depthComplexityFile.writeCell(sgl::toString((int)depthComplexityFrameNumber));
    depthComplexityFile.writeCell(sgl::toString((int)minComplexity));
    depthComplexityFile.writeCell(sgl::toString((int)maxComplexity));
    depthComplexityFile.writeCell(sgl::toString(avgUsed));
    depthComplexityFile.writeCell(sgl::toString(avgAll));
    depthComplexityFile.writeCell(sgl::toString((int)totalNumFragments));
    depthComplexityFile.writeCell(sgl::toString(double(totalNumFragments * 12ull) / 1024.0 / 1024.0 / 1024.0));
    depthComplexityFile.newRow();
    depthComplexityFrameNumber++;
}

void AutomaticPerformanceMeasurer::setCurrentAlgorithmBufferSizeBytes(size_t numBytes) {
    currentAlgorithmsBufferSizeBytes = numBytes;
}

float AutomaticPerformanceMeasurer::getUsedVideoMemorySizeGiB() {
    // Fallback
    return 0.0f;
}
