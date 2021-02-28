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

#ifndef LINEVIS_LINEDATAREQUESTER_HPP
#define LINEVIS_LINEDATAREQUESTER_HPP

#include <thread>
#include <condition_variable>

#include <ImGui/Widgets/TransferFunctionWindow.hpp>

#include <Loaders/DataSetList.hpp>
#include "LineData.hpp"

/**
 * A multi-threaded data loader for line data.
 * Similar to a mailbox queue of size 1 in the Vulkan API (cmp. VK_PRESENT_MODE_MAILBOX_KHR), it stores the most recent
 * request and reply. Older requests and replies are discarded if they are not handled fast enough.
 */
class LineDataRequester {
public:
    LineDataRequester(sgl::TransferFunctionWindow& transferFunctionWindow);
    ~LineDataRequester();

    /**
     * Stops the requester thread.
     */
    void join();

    /**
     * Queues the loading request for the line data. A pointer to an already initialized LineData object is passed, as
     * some constructors need to create OpenGL textures, and this is not a thread-save operation.
     * @param lineData The pre-initialized @see LineData object.
     * @param fileNames The file names of the line data files.
     * @param dataSetInformation Information on the line data.
     * @param transformationMatrixPtr A pointer to a transform that should be applied to the line data (or nullptr).
     */
    void queueRequest(
            LineDataPtr lineData, const std::vector<std::string>& fileNames,
            const DataSetInformation& dataSetInformation, glm::mat4* transformationMatrixPtr);

    /**
     * @return Whether a request is currently processed (for UI progress spinner).
     */
    inline bool getIsProcessingRequest() const { return isProcessingRequest; }

    /**
     * Checks if a request was finished and returns the loaded data.
     * @param loadedDataSetInformation Information about the loaded data (only if return value is not empty).
     * @return The loaded data or an empty pointer if nothing was loaded.
     */
    LineDataPtr getLoadedData(DataSetInformation& loadedDataSetInformation);

private:
    /// The main loop of the requester thread.
    void mainLoop();

    sgl::TransferFunctionWindow& transferFunctionWindow;
    std::thread requesterThread;

    std::mutex requestMutex;
    std::condition_variable hasRequestConditionVariable;
    bool programIsFinished = false;
    bool hasRequest = false;
    bool isProcessingRequest = false;
    LineDataPtr requestedLineData;
    std::vector<std::string> requestedFileNames;
    DataSetInformation requestedDataSetInformation;
    bool hasRequestedTransformationMatrix = false;
    glm::mat4 requestedTransformationMatrix;

    std::mutex replyMutex;
    std::condition_variable hasReplyConditionVariable;
    LineDataPtr lineData = nullptr;
    DataSetInformation loadedDataSetInformation;
};


#endif //LINEVIS_LINEDATAREQUESTER_HPP
