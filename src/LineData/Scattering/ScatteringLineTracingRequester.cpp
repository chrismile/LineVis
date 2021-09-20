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

#include <boost/filesystem.hpp>
#include <json/json.h>

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Events/Stream/Stream.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include "LineDataScattering.hpp"
#include "ScatteringLineTracingRequester.hpp"

ScatteringLineTracingRequester::ScatteringLineTracingRequester(
        sgl::TransferFunctionWindow& transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
        , sgl::vk::Renderer* rendererVk
#endif
) : transferFunctionWindow(transferFunctionWindow)
#ifdef USE_VULKAN_INTEROP
        , rendererVk(rendererVk)
#endif
{
    loadGridDataSetList();
    requesterThread = std::thread(&ScatteringLineTracingRequester::mainLoop, this);
}

ScatteringLineTracingRequester::~ScatteringLineTracingRequester() {
    join();

    if (cachedGridData) {
        delete[] cachedGridData;
        cachedGridData = nullptr;
    }
}

void ScatteringLineTracingRequester::loadGridDataSetList() {
    gridDataSetNames.clear();
    gridDataSetFilenames.clear();
    gridDataSetNames.emplace_back("Local file...");

    const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
    std::string filename = lineDataSetsDirectory + "grids.json";
    if (sgl::FileUtils::get()->exists(filename)) {
        // Parse the passed JSON file.
        std::ifstream jsonFileStream(filename.c_str());
        Json::CharReaderBuilder builder;
        JSONCPP_STRING errorString;
        Json::Value root;
        if (!parseFromStream(builder, jsonFileStream, &root, &errorString)) {
            sgl::Logfile::get()->writeError(errorString);
            return;
        }
        jsonFileStream.close();

        Json::Value sources = root["grids"];
        for (Json::Value::const_iterator sourceIt = sources.begin(); sourceIt != sources.end(); ++sourceIt) {
            DataSetInformation dataSetInformation;
            Json::Value source = *sourceIt;

            gridDataSetNames.push_back(source["name"].asString());
            gridDataSetFilenames.push_back(source["filename"].asString());
        }
    }
}

void ScatteringLineTracingRequester::renderGui() {
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(3072, 1146, 760, 628);
    if (ImGui::Begin("Scattering Line Tracer", &showWindow)) {
        bool changed = false;
        if (ImGui::Combo(
                "Data Set", &selectedGridDataSetIndex, gridDataSetNames.data(),
                int(gridDataSetNames.size()))) {
            if (selectedGridDataSetIndex >= 1) {
                gridDataSetFilename = gridDataSetFilenames.at(selectedGridDataSetIndex - 1);
                changed = true;
            }
        }
        if (selectedGridDataSetIndex == 0) {
            ImGui::InputText("##griddatasetfilenamelabel", &gridDataSetFilename);
            ImGui::SameLine();
            if (ImGui::Button("Load File")) {
                changed = true;
            }
        }

        changed |= ImGui::SliderInt("#Lines to Trace", &numLinesToTrace, 1, 10000);
        changed |= ImGui::SliderFloat("Extinction", &extinctionCoefficient, 0.0f, 50.0f);

        if (changed) {
            requestNewData();
        }
    }
    ImGui::End();
}

void ScatteringLineTracingRequester::requestNewData() {
    if (gridDataSetFilename.empty()) {
        return;
    }

    Json::Value request;
    const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
    request["gridDataSetFilename"] = boost::filesystem::absolute(
            lineDataSetsDirectory + gridDataSetFilename).generic_string();
    request["extinctionCoefficient"] = extinctionCoefficient;

    queueRequestJson(request);
    isProcessingRequest = true;
}

bool ScatteringLineTracingRequester::getHasNewData(DataSetInformation& dataSetInformation, LineDataPtr& lineData) {
    Json::Value reply;
    if (getReplyJson(reply, lineData)) {
        isProcessingRequest = false;
        return true;
    }
    return false;
}

void ScatteringLineTracingRequester::queueRequestJson(const Json::Value& request) {
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        requestMessage = request;
        requestLineData = LineDataPtr(new LineDataScattering(
                transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
                , rendererVk
#endif
        ));
        hasRequest = true;
    }
    hasRequestConditionVariable.notify_all();
}

bool ScatteringLineTracingRequester::getReplyJson(Json::Value& reply, LineDataPtr& lineData) {
    bool hasReply;
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        hasReply = this->hasReply;
        if (hasReply) {
            reply = replyMessage;
            lineData = replyLineData;
        }

        // Now, new requests can be worked on.
        this->hasReply = false;
        this->replyMessage.clear();
        this->replyLineData = LineDataPtr();
    }
    hasReplyConditionVariable.notify_all();
    return hasReply;
}

void ScatteringLineTracingRequester::join() {
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
        if (requesterThread.joinable()) {
            requesterThread.join();
        }
    }
}

void ScatteringLineTracingRequester::mainLoop() {
    while (true) {
        std::unique_lock<std::mutex> requestLock(requestMutex);
        hasRequestConditionVariable.wait(requestLock, [this] { return hasRequest; });

        if (programIsFinished) {
            break;
        }

        if (hasRequest) {
            Json::Value request = requestMessage;
            std::shared_ptr<LineDataScattering> lineData = std::static_pointer_cast<LineDataScattering>(requestLineData);
            hasRequest = false;
            isProcessingRequest = true;
            requestLock.unlock();

            Json::Value reply = traceLines(request, lineData);

            std::lock_guard<std::mutex> replyLock(requestMutex);
            hasReply = true;
            replyMessage = reply;
            replyLineData = lineData;
            isProcessingRequest = false;
        }
    }
}

Json::Value ScatteringLineTracingRequester::traceLines(
        const Json::Value& request, std::shared_ptr<LineDataScattering>& lineData) {
    std::string gridDataSetFilename = request["gridDataSetFilename"].asString();
    float extinctionCoefficient = request["extinctionCoefficient"].asFloat();

    if (gridDataSetFilename != cachedGridDataSetFilename) {
        if (cachedGridData) {
            delete[] cachedGridData;
            cachedGridData = nullptr;
        }

        cachedGridDataSetFilename = gridDataSetFilename;

        uint8_t* fileBuffer = nullptr;
        size_t bufferSize = 0;
        bool loaded = sgl::loadFileFromSource(gridDataSetFilename, fileBuffer, bufferSize, true);
        if (!loaded) {
            sgl::Logfile::get()->writeError(
                    "Error in ScatteringLineTracingRequester::traceLines: Couldn't load data from grid data set file \""
                    + gridDataSetFilename + "\".");
            return {};
        }
        sgl::BinaryReadStream binaryReadStream(fileBuffer, bufferSize);

        uint32_t gridSizeX = 0, gridSizeY = 0, gridSizeZ = 0;
        double voxelSizeX = 0.0, voxelSizeY = 0.0, voxelSizeZ = 0.0;
        binaryReadStream.read(gridSizeX);
        binaryReadStream.read(gridSizeY);
        binaryReadStream.read(gridSizeZ);
        binaryReadStream.read(voxelSizeX);
        binaryReadStream.read(voxelSizeY);
        binaryReadStream.read(voxelSizeZ);

        cachedGridSizeX = gridSizeX;
        cachedGridSizeY = gridSizeY;
        cachedGridSizeZ = gridSizeZ;
        cachedVoxelSizeX = float(voxelSizeX);
        cachedVoxelSizeY = float(voxelSizeY);
        cachedVoxelSizeZ = float(voxelSizeZ);

        cachedGridData = new float[gridSizeX * gridSizeY * gridSizeZ];
        binaryReadStream.read(cachedGridData, gridSizeX * gridSizeY * gridSizeZ * sizeof(float));
    }

    Trajectories trajectories;
    Trajectory trajectory0;
    trajectory0.positions.emplace_back(-1.0f, 0.0f, 1.0f);
    trajectory0.positions.emplace_back(-1.0f, 0.5f, 1.0f);
    trajectory0.positions.emplace_back(-1.0f, 1.0f, 1.0f);
    trajectory0.attributes.push_back({ 0.0f, 0.5f, 1.0f });
    trajectories.push_back(trajectory0);
    Trajectory trajectory1;
    trajectory1.positions.emplace_back(1.0f, 0.0f, 1.0f);
    trajectory1.positions.emplace_back(1.0f, 0.5f, 1.0f);
    trajectory1.positions.emplace_back(1.0f, 1.0f, 1.0f);
    trajectory1.attributes.push_back({ 0.0f, 0.5f, 1.0f });
    trajectories.push_back(trajectory1);
    Trajectory trajectory2;
    trajectory2.positions.emplace_back(0.0f, 0.0f, -1.0f);
    trajectory2.positions.emplace_back(0.0f, 0.5f, -1.0f);
    trajectory2.positions.emplace_back(0.0f, 1.0f, -1.0f);
    trajectory2.attributes.push_back({ 0.0f, 0.5f, 1.0f });
    trajectories.push_back(trajectory2);

    // TODO: This function normalizes the vertex positions of the trajectories; should we also normalize the grid size?
    normalizeTrajectoriesVertexPositions(trajectories, nullptr);

    auto dataArray = new float[cachedGridSizeX * cachedGridSizeY * cachedGridSizeZ];
    memcpy(dataArray, cachedGridData, cachedGridSizeX * cachedGridSizeY * cachedGridSizeZ * sizeof(float));

    lineData->setDataSetInformation(gridDataSetFilename, { "Attribute #1" });
    lineData->setGridData(
            dataArray, cachedGridSizeX, cachedGridSizeY, cachedGridSizeZ,
            cachedVoxelSizeX, cachedVoxelSizeY, cachedVoxelSizeZ);
    lineData->setTrajectoryData(trajectories);
    return {};
}
