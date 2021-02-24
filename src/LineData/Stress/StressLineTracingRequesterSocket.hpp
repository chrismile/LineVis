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

#ifndef LINEVIS_STRESSLINETRACINGREQUESTERSOCKET_HPP
#define LINEVIS_STRESSLINETRACINGREQUESTERSOCKET_HPP

#include <condition_variable>
#include <thread>

#ifdef __MINGW32__
#include <json/json.h>
#else
#include <jsoncpp/json/json.h>
#endif

/**
 * A multi-threaded requester socket for stress line tracing. It listens on port 17384.
 * Similar to a mailbox queue of size 1 in the Vulkan API (cmp. VK_PRESENT_MODE_MAILBOX_KHR), it stores the most recent
 * request and reply. Older requests and reply are discarded if they are not handled fast enough.
 */
class StressLineTracingRequesterSocket {
public:
    StressLineTracingRequesterSocket(const std::string& address = "localhost", int port = 17384);
    ~StressLineTracingRequesterSocket();

    /**
     * Stops the requester thread.
     */
    void join();

    /**
     * Queues the request for sending to the request worker over TCP.
     * @param requestMessage The message to queue.
     */
    void queueRequestString(const std::string& requestMessage);
    /**
     * Queues the request for sending to the request worker over TCP.
     * @param request The message to queue.
     */
    void queueRequestJson(const Json::Value& request);
    /**
     * Checks if a reply was received to a request. If a reply was received, it is stored in replyMessage.
     * @param replyMessage Where to store the reply (if one was received).
     * @return Whether a reply was received.
     */
    bool getReplyString(std::string& replyMessage);
    /**
     * Checks if a reply was received to a request. If a reply was received, it is stored in reply.
     * @param reply Where to store the reply (if one was received).
     * @return Whether a reply was received.
     */
    bool getReplyJson(Json::Value& reply);

private:
    std::thread requesterThread;
    std::condition_variable hasRequestConditionVariable;
    std::condition_variable hasReplyConditionVariable;
    std::mutex requestMutex;
    std::mutex replyMutex;

    std::string address;
    int port;
    bool programIsFinished = false;
    bool hasRequest = false;
    bool hasReply = false;
    std::string requestMessage;
    std::string replyMessage;

    Json::CharReaderBuilder readerBuilder;
    Json::CharReader* jsonCharReader = nullptr;
    Json::StreamWriterBuilder builder;

    /// The main loop of the requester thread.
    void mainLoop();
};

#endif //LINEVIS_STRESSLINETRACINGREQUESTERSOCKET_HPP
