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

#include <Utils/File/Logfile.hpp>

#include "LineDataFlow.hpp"
#include "LineDataStress.hpp"
#include "LineDataMultiVar.hpp"
#include "LineDataRequester.hpp"

LineDataRequester::LineDataRequester(sgl::TransferFunctionWindow& transferFunctionWindow)
        : transferFunctionWindow(transferFunctionWindow) {
    requesterThread = std::thread(&LineDataRequester::mainLoop, this);
}

LineDataRequester::~LineDataRequester() {
    join();
}

void LineDataRequester::join() {
    if (!programIsFinished) {
        {
            std::lock_guard<std::mutex> lock(requestMutex);
            programIsFinished = true;
            hasRequest = true;

            {
                std::lock_guard<std::mutex> lock(replyMutex);
                this->lineData = LineDataPtr();
            }
            hasReplyConditionVariable.notify_all();
        }
        hasRequestConditionVariable.notify_all();
        if (requesterThread.joinable()) {
            requesterThread.join();
        }
    }
}

void LineDataRequester::queueRequest(
        LineDataPtr lineData, const std::vector<std::string>& fileNames,
        const DataSetInformation& dataSetInformation, glm::mat4* transformationMatrixPtr) {
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        this->requestedLineData = lineData;
        this->requestedFileNames = fileNames;
        this->requestedDataSetInformation = dataSetInformation;
        if (transformationMatrixPtr) {
            hasRequestedTransformationMatrix = true;
            requestedTransformationMatrix = *transformationMatrixPtr;
        } else {
            hasRequestedTransformationMatrix = true;
            requestedTransformationMatrix = sgl::matrixIdentity();
        }
        hasRequest = true;
    }
    hasRequestConditionVariable.notify_all();
}

LineDataPtr LineDataRequester::getLoadedData(DataSetInformation& loadedDataSetInformation) {
    LineDataPtr lineData;
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        lineData = this->lineData;
        loadedDataSetInformation = this->loadedDataSetInformation;

        // Now, new requests can be worked on.
        this->lineData = LineDataPtr();
        this->loadedDataSetInformation = DataSetInformation();
    }
    hasReplyConditionVariable.notify_all();
    return lineData;
}

void LineDataRequester::mainLoop() {
    while (true) {
        std::unique_lock<std::mutex> requestLock(requestMutex);
        hasRequestConditionVariable.wait(requestLock, [this] { return hasRequest; });

        if (programIsFinished) {
            break;
        }

        if (hasRequest) {
            LineDataPtr lineData = this->requestedLineData;
            std::vector<std::string> fileNames = this->requestedFileNames;
            DataSetInformation dataSetInformation = this->requestedDataSetInformation;
            glm::mat4 transformationMatrix = sgl::matrixIdentity();
            glm::mat4* transformationMatrixPtr = nullptr;
            if (this->hasRequestedTransformationMatrix) {
                transformationMatrix = this->requestedTransformationMatrix;
                transformationMatrixPtr = &transformationMatrix;
            }

            hasRequest = false;
            this->requestedLineData = LineDataPtr();
            isProcessingRequest = true;
            requestLock.unlock();

            bool dataLoaded = lineData->loadFromFile(fileNames, dataSetInformation, transformationMatrixPtr);
            if (!dataLoaded) {
                continue;
            }

            std::lock_guard<std::mutex> replyLock(requestMutex);
            this->lineData = lineData;
            this->loadedDataSetInformation = dataSetInformation;
            isProcessingRequest = false;
        }
    }
}
