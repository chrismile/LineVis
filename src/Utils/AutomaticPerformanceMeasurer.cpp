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
#include <utility>

#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>

#include "AutomaticPerformanceMeasurer.hpp"

// See: https://stackoverflow.com/questions/2513505/how-to-get-available-memory-c-g
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
size_t getUsedSystemMemoryBytes() {
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    GlobalMemoryStatusEx(&status);
    return status.ullTotalPhys - status.ullAvailPhys;
}
#elif defined(__linux__)
#include <unistd.h>
size_t getUsedSystemMemoryBytes() {
    size_t totalNumPages = sysconf(_SC_PHYS_PAGES);
    size_t availablePages = sysconf(_SC_AVPHYS_PAGES);
    size_t pageSizeBytes = sysconf(_SC_PAGE_SIZE);
    return (totalNumPages - availablePages) * pageSizeBytes;
}
#else
size_t getUsedSystemMemoryBytes() {
    return 0;
}
#endif

AutomaticPerformanceMeasurer::AutomaticPerformanceMeasurer(
        sgl::vk::Renderer* renderer, std::vector<InternalState> _states,
        const std::string& _csvFilename, const std::string& _depthComplexityFilename,
        std::function<void(const InternalState&)> _newStateCallback)
        : renderer(renderer), states(std::move(_states)), currentStateIndex(0),
          newStateCallback(std::move(_newStateCallback)),
          file(_csvFilename), perfFile("performance_list.csv"),
          depthComplexityFilename(_depthComplexityFilename){
    sgl::FileUtils::get()->ensureDirectoryExists("images/");

    // Write header
    //file.writeRow({"Name", "Average Time (ms)", "Memory (GiB)", "Buffer Size (GiB)", "Time Stamp (s), Frame Time (ns)"});
    file.writeRow({
        "State Name", "Data Set Name", "Device Name", "Resolution",
        "Average Time (ms)", "Base Data Size (GiB)", "Buffer Size (GiB)", "OIT Buffer Size (GiB)",
        "Average FPS", "5% Percentile FPS", "95% Percentile FPS", "StdDev FPS", "PPLL Entries", "PPLL Size (GiB)"});
    perfFile.writeRow({"Name", "Time per frame (ms)"});
}

void AutomaticPerformanceMeasurer::openPpllFileIfNecessary() {
    if (!ppllFile.getIsOpen()) {
        ppllFile.open("PPLLUnified.csv");
        ppllFile.writeRow({"Mode Name", "Time PPLL Clear (ms)", "Time F+C Gather (ms)", "Time PPLL Resolve (ms)"});
    }
}

void AutomaticPerformanceMeasurer::openDepthComplexityFileIfNecessary() {
    if (!depthComplexityFile.getIsOpen()) {
        depthComplexityFile.open("PPLLUnified.csv");
        depthComplexityFile.writeRow(
                {"State Name", "Data Set Name", "Device Name", "Resolution",
                 "Frame Number", "Min Depth Complexity", "Max Depth Complexity",
                 "Avg Depth Complexity Used", "Avg Depth Complexity All", "Total Number of Fragments",
                 "Fragment Buffer Memory (GiB)"});
    }
}

void AutomaticPerformanceMeasurer::openDeferredRenderingFileIfNecessary() {
    if (!deferredRenderingFile.getIsOpen()) {
        deferredRenderingFile.open("DeferredRendering.csv");
        deferredRenderingFile.writeRow(
                {"Mode Name", "Time Sum (ms)", "DeferredRaster0 (ms)", "DeferredRaster1 (ms)",
                 "DeferredVisibility0 (ms)", "DeferredVisibility1 (ms)", "DeferredHZB0 (ms)", "DeferredHZB1 (ms)"});
    }
}

/*
 * Use cleanup function, as doing the cleanup in the destructor triggers the following warning when using MSVC.
 * "Warning: src\Utils\AutomaticPerformanceMeasurer.cpp(68): warning C4722:
 * 'AutomaticPerformanceMeasurer::~AutomaticPerformanceMeasurer': destructor never returns, potential memory leak"
 */
void AutomaticPerformanceMeasurer::cleanup() {
    isCleanup = true;

    VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands();
    timerVk->finishGPU(commandBuffer);
    if (ppllTimer) {
        ppllTimer->finishGPU(commandBuffer);
    }
    if (deferredRenderingTimer) {
        deferredRenderingTimer->finishGPU(commandBuffer);
    }
    renderer->getDevice()->endSingleTimeCommands(commandBuffer);

    writeCurrentModeData();
    file.close();
    perfFile.close();
    if (depthComplexityFile.getIsOpen()) {
        depthComplexityFile.close();
    }
    if (ppllFile.getIsOpen()) {
        ppllFile.close();
    }
    if (deferredRenderingFile.getIsOpen()) {
        deferredRenderingFile.close();
    }
}

AutomaticPerformanceMeasurer::~AutomaticPerformanceMeasurer() = default;

bool AutomaticPerformanceMeasurer::update(float currentTime) {
    nextModeCounter = currentTime;
    if (nextModeCounter >= TIME_PERFORMANCE_MEASUREMENT + 0.5f) {
        nextModeCounter = 0.0f;
        if (currentStateIndex == states.size() - 1) {
            return false; // Terminate program
        }
        shallSetNextState = true;
    }
    return true;
}


void AutomaticPerformanceMeasurer::writeCurrentModeData() {
    double timeMS = 0.0;
    std::vector<uint64_t> frameTimesNS;

    if (!ppllTimer) {
        if (!isCleanup) {
            renderer->getDevice()->waitIdle();
            VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands();
            timerVk->finishGPU(commandBuffer);
            renderer->getDevice()->endSingleTimeCommands(commandBuffer);
            //timerVk->finishGPU();
        }
        timeMS = timerVk->getTimeMS(currentState.name);
        frameTimesNS = timerVk->getFrameTimeList(currentState.name);
    } else {
        openPpllFileIfNecessary();
        if (!isCleanup) {
            renderer->getDevice()->waitIdle();
            VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands();
            ppllTimer->finishGPU(commandBuffer);
            renderer->getDevice()->endSingleTimeCommands(commandBuffer);
            ppllTimer->finishGPU();
        }
        double timePPLLClear = ppllTimer->getTimeMS("PPLLClear");
        double timeFCGather = ppllTimer->getTimeMS("FCGather");
        double timePPLLResolve = ppllTimer->getTimeMS("PPLLResolve");
        ppllFile.writeCell(currentState.name);
        ppllFile.writeCell(std::to_string(timePPLLClear));
        ppllFile.writeCell(std::to_string(timeFCGather));
        ppllFile.writeCell(std::to_string(timePPLLResolve));
        ppllFile.newRow();

        timeMS = timePPLLClear + timeFCGather + timePPLLResolve;

        if (!isCleanup) {
            VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands();
            timerVk->finishGPU(commandBuffer);
            renderer->getDevice()->endSingleTimeCommands(commandBuffer);
        }
        frameTimesNS = timerVk->getFrameTimeList(currentState.name);
    }

    if (deferredRenderingTimer) {
        openDeferredRenderingFileIfNecessary();
        if (!isCleanup) {
            VkCommandBuffer commandBuffer = renderer->getDevice()->beginSingleTimeCommands();
            deferredRenderingTimer->finishGPU(commandBuffer);
            renderer->getDevice()->endSingleTimeCommands(commandBuffer);
            //deferredRenderingTimer->finishGPU();
        }
        double deferredRaster0 = deferredRenderingTimer->getOptionalTimeMS("DeferredRaster0");
        double deferredRaster1 = deferredRenderingTimer->getOptionalTimeMS("DeferredRaster1");
        double deferredVisibility0 = deferredRenderingTimer->getOptionalTimeMS("DeferredVisibility0");
        double deferredVisibility1 = deferredRenderingTimer->getOptionalTimeMS("DeferredVisibility1");
        double deferredHZB0 = deferredRenderingTimer->getOptionalTimeMS("DeferredHZB0");
        double deferredHZB1 = deferredRenderingTimer->getOptionalTimeMS("DeferredHZB1");
        double timeSum =
                deferredRaster0 + deferredRaster1 + deferredVisibility0 + deferredVisibility1
                + deferredHZB0 + deferredHZB1;
        deferredRenderingFile.writeCell(currentState.name);
        deferredRenderingFile.writeCell(std::to_string(timeSum));
        deferredRenderingFile.writeCell(std::to_string(deferredRaster0));
        deferredRenderingFile.writeCell(std::to_string(deferredRaster1));
        deferredRenderingFile.writeCell(std::to_string(deferredVisibility0));
        deferredRenderingFile.writeCell(std::to_string(deferredVisibility1));
        deferredRenderingFile.writeCell(std::to_string(deferredHZB0));
        deferredRenderingFile.writeCell(std::to_string(deferredHZB1));
        deferredRenderingFile.newRow();
    }

    // Write row with performance metrics of this mode.
    //         "State Name", "Data Set Name", "Device Name", "Resolution",
    file.writeCell(currentState.nameRaw);
    file.writeCell(currentState.dataSetDescriptor.name);
    file.writeCell(sgl::AppSettings::get()->getPrimaryDevice()->getDeviceName());
    file.writeCell(
            sgl::toString(currentState.windowResolution.x) + "x"
            + sgl::toString(currentState.windowResolution.y));
    perfFile.writeCell(currentState.nameRaw);
    perfFile.writeCell(currentState.dataSetDescriptor.name);
    perfFile.writeCell(sgl::AppSettings::get()->getPrimaryDevice()->getDeviceName());
    perfFile.writeCell(
            sgl::toString(currentState.windowResolution.x) + "x"
            + sgl::toString(currentState.windowResolution.y));
    file.writeCell(sgl::toString(timeMS));

    // Write current memory consumption in gigabytes
    double scaleFactorGiB = 1.0 / (1024.0 * 1024.0 * 1024.0);
    file.writeCell(sgl::toString(double(currentDataSetBaseSizeBytes) * scaleFactorGiB));
    file.writeCell(sgl::toString(double(currentDataSetBufferSizeBytes) * scaleFactorGiB));
    file.writeCell(sgl::toString(double(currentAlgorithmsBufferSizeBytes) * scaleFactorGiB));


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

    float fpsVariance = 0.0f;
    for (uint64_t frameTimeNS : frameTimesNS) {
        auto fps = float(double(1e9) / double(frameTimeNS));
        float diff = fps - averageFps;
        fpsVariance += diff * diff;
    }
    fpsVariance /= float(int(frameTimes.size()) - 1); //< Unbiased estimator uses N - 1.

    int percentile5Index = int(double(frameTimes.size()) * 0.5);
    int percentile95Index = int(double(frameTimes.size()) * 0.95);
    float percentile5Fps = frameTimes.empty() ? 0.0f : 1.0f / frameTimes.at(percentile95Index);
    float percentile95Fps = frameTimes.empty() ? 0.0f : 1.0f / frameTimes.at(percentile5Index);
    file.writeCell(sgl::toString(averageFps));
    file.writeCell(sgl::toString(percentile5Fps));
    file.writeCell(sgl::toString(percentile95Fps));
    file.writeCell(sgl::toString(std::sqrt(fpsVariance)));
    file.writeCell(sgl::toString(maxPPLLNumFragments));
    file.writeCell(sgl::toString(double(maxPPLLNumFragments * 12ull) / 1024.0 / 1024.0 / 1024.0));

    file.newRow();
    perfFile.newRow();
    file.flush();
    perfFile.flush();
}

void AutomaticPerformanceMeasurer::setNextState(bool first) {
    if (!first) {
        writeCurrentModeData();
        currentStateIndex++;
    }

    currentAlgorithmsBufferSizeBytes = 0;
    currentDataSetBufferSizeBytes = 0;
    currentDataSetBaseSizeBytes = 0;

    timerVk = std::make_shared<sgl::vk::Timer>(renderer);
    timerVk->setStoreFrameTimeList(true);

    depthComplexityFrameNumber = 0;
    currentAlgorithmsBufferSizeBytes = 0;
    currentState = states.at(currentStateIndex);
    sgl::Logfile::get()->writeInfo(std::string() + "New state: " + currentState.name);
    if (currentState.renderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        newDepthComplexityMode = true;
    }
    newStateCallback(currentState);
}

void AutomaticPerformanceMeasurer::beginRenderFunction() {
    if (!isInitialized) {
        // Set initial state
        setNextState(true);
        isInitialized = true;
        shallSetNextState = false;
    } else if (shallSetNextState) {
        setNextState();
        shallSetNextState = false;
    }
}

void AutomaticPerformanceMeasurer::startMeasure(float timeStamp) {
    if (currentState.renderingMode == RENDERING_MODE_OSPRAY_RAY_TRACER) {
        // CPU rendering algorithm, thus use a CPU timer and not a GPU timer.
        timerVk->startCPU(currentState.name);
    } else {
        timerVk->startGPU(currentState.name);
    }
}

void AutomaticPerformanceMeasurer::endMeasure() {
    if (currentState.renderingMode == RENDERING_MODE_OSPRAY_RAY_TRACER) {
        // CPU rendering algorithm, thus use a CPU timer and not a GPU timer.
        timerVk->endCPU(currentState.name);
    } else {
        timerVk->endGPU(currentState.name);
    }
}

void AutomaticPerformanceMeasurer::pushDepthComplexityFrame(
        uint64_t minComplexity, uint64_t maxComplexity, float avgUsed, float avgAll, uint64_t totalNumFragments) {
    if (newDepthComplexityMode) {
        newDepthComplexityMode = false;
        maxPPLLNumFragments = 0ull;
    }
    maxPPLLNumFragments = std::max(maxPPLLNumFragments, size_t(totalNumFragments));

    openDepthComplexityFileIfNecessary();
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

void AutomaticPerformanceMeasurer::setCurrentAlgorithmBufferSizeBytes(size_t sizeInBytes) {
    currentAlgorithmsBufferSizeBytes = sizeInBytes;
}

void AutomaticPerformanceMeasurer::setCurrentDataSetBufferSizeBytes(size_t sizeInBytes) {
    currentDataSetBufferSizeBytes = sizeInBytes;
}

void AutomaticPerformanceMeasurer::setCurrentDataSetBaseSizeBytes(size_t sizeInBytes) {
    currentDataSetBaseSizeBytes = sizeInBytes;
}

float AutomaticPerformanceMeasurer::getUsedVideoMemorySizeGiB() {
    // Fallback
    return 0.0f;
}
