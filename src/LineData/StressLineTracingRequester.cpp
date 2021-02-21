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

#ifdef USE_ZEROMQ
#include <zmq.hpp>
#endif

#include <Utils/Regex/TransformString.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Loaders/DataSetList.hpp"
#include "StressLineTracingRequester.hpp"

void StressLineTracingRequester::renderGui() {
    if (ImGui::Begin("Stress Line Tracing", &showWindow)) {
        //ImGui::Checkbox("", )
    }
    ImGui::End();
}

void StressLineTracingRequester::requestNewData() {
    worker.queueRequestString("Test");
}

bool StressLineTracingRequester::getHasNewData(DataSetInformation& dataSetInformation) {
    Json::Value reply;
    if (worker.getReplyJson(reply)) {
        dataSetInformation.type = DATA_SET_TYPE_STRESS_LINES;
        dataSetInformation.transformMatrix = parseTransformString("rotate(270°, 1, 0, 0)");
        dataSetInformation.containsBandData = true;

        Json::Value filenames = reply["filenames"];
        if (filenames.isArray()) {
            for (Json::Value::const_iterator filenameIt = filenames.begin();
                 filenameIt != filenames.end(); ++filenameIt) {
                dataSetInformation.filenames.push_back(lineDataSetsDirectory + filenameIt->asString());
            }
        } else {
            dataSetInformation.filenames.push_back(lineDataSetsDirectory + filenames.asString());
        }

        // Optional data: Attribute (importance criteria) display names.
        if (reply.isMember("attributes")) {
            Json::Value attributes = reply["attributes"];
            if (attributes.isArray()) {
                for (Json::Value::const_iterator attributesIt = attributes.begin();
                     attributesIt != attributes.end(); ++attributesIt) {
                    dataSetInformation.attributeNames.push_back(attributesIt->asString());
                }
            } else {
                dataSetInformation.attributeNames.push_back(attributes.asString());
            }
        }

        // Optional stress line data: Mesh file.
        if (reply.isMember("mesh")) {
            dataSetInformation.meshFilename = lineDataSetsDirectory + reply["mesh"].asString();
        }

        // Optional stress line data: Degenerate points file.
        if (reply.isMember("degenerate_points")) {
            dataSetInformation.degeneratePointsFilename =
                    lineDataSetsDirectory + reply["degenerate_points"].asString();
        }

        return true;
    }
    return false;
}

StressLineTracingRequesterSocket::StressLineTracingRequesterSocket(const std::string& address, int port)
        : address(address), port(port) {
    jsonCharReader = readerBuilder.newCharReader();
    requesterThread = std::thread(&StressLineTracingRequesterSocket::mainLoop, this);
}

StressLineTracingRequesterSocket::~StressLineTracingRequesterSocket() {
    join();
    delete jsonCharReader;
    jsonCharReader = nullptr;
}

void StressLineTracingRequesterSocket::join() {
    if (!programIsFinished) {
        {
            std::lock_guard<std::mutex> lock(requestMutex);
            programIsFinished = true;
            hasRequest = true;

            {
                std::lock_guard<std::mutex> lock(replyMutex);
                this->hasReply = false;
                this->replyMessage.clear();
            }
            hasReplyConditionVariable.notify_all();
        }
        hasRequestConditionVariable.notify_all();
        requesterThread.join();
    }
}

void StressLineTracingRequesterSocket::queueRequestString(const std::string& requestMessage) {
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        this->requestMessage = requestMessage;
        hasRequest = true;
    }
    hasRequestConditionVariable.notify_all();
}

void StressLineTracingRequesterSocket::queueRequestJson(const Json::Value& request) {
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        requestMessage = Json::writeString(builder, request);
        hasRequest = true;
    }
    hasRequestConditionVariable.notify_all();
}

bool StressLineTracingRequesterSocket::getReplyString(std::string& replyMessage) {
    bool hasReply;
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        hasReply = this->hasReply;
        if (hasReply) {
            replyMessage = this->replyMessage;
        }

        // Now, new requests can be worked on.
        this->hasReply = false;
        this->replyMessage.clear();
    }
    hasReplyConditionVariable.notify_all();
    return hasReply;
}

bool StressLineTracingRequesterSocket::getReplyJson(Json::Value& reply) {
    bool hasReply;
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        hasReply = this->hasReply;
        if (hasReply) {
            std::string jsonErrorString;
            if (!jsonCharReader->parse(
                    replyMessage.c_str(), replyMessage.c_str() + replyMessage.size(),
                    &reply, &jsonErrorString)) {
                std::cerr << "Error in StressLineTracingRequesterSocket::getReplyJson: Couldn't parse JSON string."
                          << std::endl << jsonErrorString << std::endl;
            }
        }

        // Now, new requests can be worked on.
        this->hasReply = false;
        this->replyMessage.clear();
    }
    hasReplyConditionVariable.notify_all();
    return hasReply;
}

void StressLineTracingRequesterSocket::mainLoop() {
#ifdef USE_ZEROMQ
    zmq::context_t context{1};
    zmq::socket_t socket(context, zmq::socket_type::req);
    std::string endpoint = std::string() + "tcp://" + address + ":" + std::to_string(port);
    socket.connect(endpoint.c_str());
#endif

    while (true) {
        std::unique_lock<std::mutex> requestLock(requestMutex);
        hasRequestConditionVariable.wait(requestLock, [this] { return hasRequest; });

        if (programIsFinished) {
            break;
        }

        if (hasRequest) {
#ifdef USE_ZEROMQ
            socket.send(zmq::buffer(requestMessage), zmq::send_flags::none);
#endif

            hasRequest = false;
            requestLock.unlock();

#ifdef USE_ZEROMQ
            zmq::message_t reply{};
            zmq::recv_result_t result = socket.recv(reply, zmq::recv_flags::none);
            if (result.value() == 0) {
                throw std::runtime_error(
                        "Error in StressLineTracingRequesterSocket::mainLoop: Received empty response.");
            }
            std::string replyString = reply.to_string();

            std::lock_guard<std::mutex> replyLock(requestMutex);
            hasReply = true;
            replyMessage = replyString;
#endif
        }
    }
}
