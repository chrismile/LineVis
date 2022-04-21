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

#ifndef HEXVOLUMERENDERER_AUTOMATICPERFORMANCEMEASURER_HPP
#define HEXVOLUMERENDERER_AUTOMATICPERFORMANCEMEASURER_HPP

#include <string>
#include <functional>
#include <Utils/File/CsvWriter.hpp>
#include <Graphics/Vulkan/Utils/Timer.hpp>

#include "InternalState.hpp"

const float TIME_PERFORMANCE_MEASUREMENT = 256.0f;

class AutomaticPerformanceMeasurer {
public:
    AutomaticPerformanceMeasurer(
            sgl::vk::Renderer* renderer, std::vector<InternalState> _states,
            const std::string& _csvFilename, const std::string& _depthComplexityFilename,
            std::function<void(const InternalState&)> _newStateCallback);
    void cleanup();
    ~AutomaticPerformanceMeasurer();

    // To be called by the application
    void beginRenderFunction();
    void startMeasure(float timeStamp);
    void endMeasure();

    /// Returns false if all modes were tested and the app should terminate.
    bool update(float currentTime);

    // Called by OIT_DepthComplexity
    void pushDepthComplexityFrame(
            uint64_t minComplexity, uint64_t maxComplexity, float avgUsed, float avgAll, uint64_t totalNumFragments);

    // Called by OIT algorithms.
    void setCurrentAlgorithmBufferSizeBytes(size_t sizeInBytes);
    inline void setPpllTimer(const sgl::vk::TimerPtr& ppllTimer) { this->ppllTimer = ppllTimer; }

    // Called by the renderers to announce the size of the data set visual representation.
    void setCurrentDataSetBufferSizeBytes(size_t sizeInBytes);

    // Called by the renderers to announce the base size of the data set (not the visual representation).
    void setCurrentDataSetBaseSizeBytes(size_t sizeInBytes);

private:
    /// Write out the performance data of "currentState" to "file".
    void writeCurrentModeData();
    /// Switch to the next state in "states".
    void setNextState(bool first = false);

    /// Returns amount of used video memory size in GiB.
    float getUsedVideoMemorySizeGiB();

    sgl::vk::Renderer* renderer;
    bool isInitialized = false;
    bool shallSetNextState = false;
    bool isCleanup = false;

    std::vector<InternalState> states;
    size_t currentStateIndex;
    InternalState currentState;
    std::function<void(const InternalState&)> newStateCallback; // Application callback

    float nextModeCounter = 0.0f;

    sgl::vk::TimerPtr timerVk;
    int initialFreeMemKilobytes{};
    sgl::CsvWriter file;
    sgl::CsvWriter depthComplexityFile;
    sgl::CsvWriter perfFile;
    size_t depthComplexityFrameNumber = 0;
    size_t currentAlgorithmsBufferSizeBytes = 0;
    size_t currentDataSetBufferSizeBytes = 0;
    size_t currentDataSetBaseSizeBytes = 0;

    // For depth complexity renderer.
    bool newDepthComplexityMode = true;
    size_t maxPPLLNumFragments = 0;

    // For per-pixel linked list renderers.
    sgl::CsvWriter ppllFile;
    sgl::vk::TimerPtr ppllTimer = nullptr;
};

size_t getUsedSystemMemoryBytes();

#endif //HEXVOLUMERENDERER_AUTOMATICPERFORMANCEMEASURER_HPP
