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

#include "StressLineTracingRequesterSocket.hpp"

StressLineTracingRequesterSocket::StressLineTracingRequesterSocket(void* context, const std::string& address, int port)
        : context(context), address(address), port(port) {
    jsonCharReader = readerBuilder.newCharReader();

    controllerSocketPub = zmq_socket(context, ZMQ_PUB);
    if (controllerSocketPub == nullptr) {
        throw std::runtime_error(
                "Error in StressLineTracingRequesterSocket::StressLineTracingRequesterSocket: "
                "controllerSocketPub == nullptr");
    }
    int lingerIntervalMs = 0;
    zmq_setsockopt(controllerSocketPub, ZMQ_LINGER, &lingerIntervalMs, sizeof(lingerIntervalMs));
    zmq_bind(controllerSocketPub, controllerAddress.c_str());

    requesterThread = std::thread(&StressLineTracingRequesterSocket::mainLoop, this);
}

StressLineTracingRequesterSocket::~StressLineTracingRequesterSocket() {
    join();

    if (controllerSocketPub != nullptr) {
        zmq_close(controllerSocketPub);
        controllerSocketPub = nullptr;
    }

    delete jsonCharReader;
    jsonCharReader = nullptr;
}

void StressLineTracingRequesterSocket::join() {
    if (!programIsFinished) {
        {
            zmq_send(controllerSocketPub, "KILL", 4, ZMQ_DONTWAIT);
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
        if (requesterThread.joinable()) {
            requesterThread.join();
        }
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
    std::string endpoint = std::string() + "tcp://" + address + ":" + std::to_string(port);
    void* socket = zmq_socket(context, ZMQ_REQ);
    if (socket == nullptr) {
        throw std::runtime_error("Error in StressLineTracingRequesterSocket::mainLoop: socket == nullptr");
    }
    int lingerIntervalMs = 0;
    zmq_setsockopt(socket, ZMQ_LINGER, &lingerIntervalMs, sizeof(lingerIntervalMs));
    zmq_connect(socket, endpoint.c_str());

    void* controllerSocketSub = zmq_socket(context, ZMQ_SUB);
    if (controllerSocketSub == nullptr) {
        throw std::runtime_error("Error in StressLineTracingRequesterSocket::mainLoop: controllerSocket == nullptr");
    }
    zmq_setsockopt(controllerSocketSub, ZMQ_LINGER, &lingerIntervalMs, sizeof(lingerIntervalMs));
    zmq_setsockopt(controllerSocketSub, ZMQ_SUBSCRIBE, "", 0);
    zmq_connect(controllerSocketSub, controllerAddress.c_str());

    zmq_pollitem_t items [] = {
            { socket, 0, ZMQ_POLLIN, 0 },
            { controllerSocketSub, 0, ZMQ_POLLIN, 0 }
    };
#endif

    while (true) {
        std::unique_lock<std::mutex> requestLock(requestMutex);
        hasRequestConditionVariable.wait(requestLock, [this] { return hasRequest; });

        if (programIsFinished) {
            break;
        }

        if (hasRequest) {
#ifdef USE_ZEROMQ
            int sentBytes = zmq_send(socket, requestMessage.data(), requestMessage.size(), 0);
            if (sentBytes < 0) {
                if (zmq_errno() == ETERM) {
                    break;
                }
                continue;
            }
#endif

            hasRequest = false;
            isProcessingRequest = true;
            requestLock.unlock();

#ifdef USE_ZEROMQ
            zmq_poll(items, 2, -1);

            if (items[0].revents & ZMQ_POLLIN) {
                zmq_msg_t reply;
                int rc = zmq_msg_init(&reply);
                assert(rc == 0);
                const int receivedBytes = zmq_msg_recv(&reply, socket, 0);
                if (receivedBytes < 0) {
                    zmq_msg_close(&reply);
                    if (zmq_errno() == ETERM) {
                        break;
                    }
                    continue;
                }
                std::string replyString = std::string(
                        static_cast<const char*>(zmq_msg_data(&reply)), zmq_msg_size(&reply));
                zmq_msg_close(&reply);

                std::lock_guard<std::mutex> replyLock(replyMutex);
                hasReply = true;
                replyMessage = replyString;
                isProcessingRequest = false;
            }

            if (items[1].revents & ZMQ_POLLIN) {
                break;
            }
#endif
        }
    }

    zmq_close(socket);
    zmq_close(controllerSocketSub);
}
