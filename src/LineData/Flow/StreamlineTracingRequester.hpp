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

#ifndef LINEVIS_STREAMLINETRACINGREQUESTER_HPP
#define LINEVIS_STREAMLINETRACINGREQUESTER_HPP

#include <thread>
#include <condition_variable>
#include <json/json.h>

#include "Loaders/DataSetList.hpp"
#include "../LineDataFlow.hpp"
#include "StreamlineTracingDefines.hpp"

class StreamlineSeeder;
typedef std::shared_ptr<StreamlineSeeder> StreamlineSeederPtr;
class StreamlineTracingGrid;

class StreamlineTracingRequester {
public:
    explicit StreamlineTracingRequester(sgl::TransferFunctionWindow& transferFunctionWindow);
    ~StreamlineTracingRequester();

    void renderGui();
    void setLineTracerSettings(const SettingsMap& settings);
    void setDatasetFilename(const std::string& newDatasetFilename);
    bool getHasNewData(DataSetInformation& dataSetInformation, LineDataPtr& lineData);

    /**
     * @return Whether a request is currently processed (for UI progress spinner).
     */
    [[nodiscard]] inline bool getIsProcessingRequest() const { return isProcessingRequest; }

private:
    void loadGridDataSetList();
    void requestNewData();

    /// The main loop of the requester thread.
    void mainLoop();
    /// Waits for the requester thread to terminate.
    void join();

    /**
     * Queues the request for tracing .
     * @param request The message to queue.
     */
    void queueRequestStruct(const StreamlineTracingSettings& request);
    /**
     * Checks if a reply was received to a request.
     * @return Whether a reply was received.
     */
    bool getReply(LineDataPtr& lineData);

    /**
     * @param request Information for the requested tracing of lines scattered in the grid.
     * @param lineData An object for storing the traced line data.
     */
    void traceLines(StreamlineTracingSettings& request, std::shared_ptr<LineDataFlow>& lineData);

    sgl::TransferFunctionWindow& transferFunctionWindow;

    std::thread requesterThread;
    std::condition_variable hasRequestConditionVariable;
    std::condition_variable hasReplyConditionVariable;
    std::mutex requestMutex;
    std::mutex replyMutex;

    std::mutex gridInfoMutex;
    bool newGridLoaded = false;
    sgl::AABB3 gridBox;

    bool programIsFinished = false;
    bool hasRequest = false;
    bool hasReply = false;
    bool isProcessingRequest = false;

    StreamlineTracingSettings workerTracingSettings;
    LineDataPtr requestLineData;
    LineDataPtr replyLineData;

    // Line tracing settings.
    StreamlineTracingSettings guiTracingSettings;
    std::map<StreamlineSeedingStrategy, StreamlineSeederPtr> streamlineSeeders;

    // Cache for storing the currently selected streamline tracing grid.
    std::string cachedGridFilename;
    StreamlineTracingGrid* cachedGrid = nullptr;
    std::vector<uint32_t> cachedSimulationMeshOutlineTriangleIndices;
    std::vector<glm::vec3> cachedSimulationMeshOutlineVertexPositions;
    std::vector<glm::vec3> cachedSimulationMeshOutlineVertexNormals;

    // GUI data.
    bool showWindow = true;
    std::string lineDataSetsDirectory;
    std::string gridDataSetFilename;
    std::vector<std::string> gridDataSetNames;
    std::vector<std::string> gridDataSetFilenames;
    int selectedGridDataSetIndex = 0;
};

#endif //LINEVIS_STREAMLINETRACINGREQUESTER_HPP
