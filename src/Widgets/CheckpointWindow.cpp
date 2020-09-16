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

#include <fstream>

#include <Utils/Events/Stream/Stream.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_stdlib.h>

#include "CheckpointWindow.hpp"

CheckpointWindow::CheckpointWindow(SceneData& sceneData) : sceneData(sceneData) {
    sgl::FileUtils::get()->ensureDirectoryExists(saveDirectoryCheckpoints);
    if (sgl::FileUtils::get()->exists(checkpointsFilename)) {
        readFromFile(checkpointsFilename);
    }
}

CheckpointWindow::~CheckpointWindow() {
    if (!loadedDataSetName.empty() && !loadedDataSetCheckpoints.empty()) {
        std::map<std::string, Checkpoint> loadedDataSetCheckpointsMap;
        for (auto& dataSetCheckpoint : loadedDataSetCheckpoints) {
            loadedDataSetCheckpointsMap.insert(dataSetCheckpoint);
        }
        dataSetCheckpointMap[loadedDataSetName] = loadedDataSetCheckpointsMap;
    }
    loadedDataSetCheckpoints.clear();

    writeToFile(checkpointsFilename);
}

void CheckpointWindow::onLoadDataSet(const std::string& dataSetName) {
    // Save previous data set checkpoints in data set map.
    if (!loadedDataSetName.empty() && !loadedDataSetCheckpoints.empty()) {
        std::map<std::string, Checkpoint> loadedDataSetCheckpointsMap;
        for (auto& dataSetCheckpoint : loadedDataSetCheckpoints) {
            loadedDataSetCheckpointsMap.insert(dataSetCheckpoint);
        }
        dataSetCheckpointMap[loadedDataSetName] = loadedDataSetCheckpointsMap;
    }
    loadedDataSetCheckpoints.clear();

    // Load checkpoints if they exist already for this data set.
    loadedDataSetName = dataSetName;
    auto it = dataSetCheckpointMap.find(dataSetName);
    if (it != dataSetCheckpointMap.end()) {
        for (auto& dataSetCheckpoint : it->second) {
            loadedDataSetCheckpoints.push_back(dataSetCheckpoint);
        }
    }
}

bool CheckpointWindow::readFromFile(const std::string& filename) {
    std::ifstream file(filename.c_str(), std::ifstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in CheckpointWindow::readFromFile: File \"" + filename + "\" not found.");
        return false;
    }

    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0);
    char *buffer = new char[size];
    file.read(buffer, size);
    file.close();

    sgl::BinaryReadStream stream(buffer, size);
    uint32_t version;
    stream.read(version);
    if (version != CHECKPOINT_FORMAT_VERSION) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in CheckpointWindow::readFromFile: "
                + "Invalid version in file \"" + filename + "\".");
        return false;
    }

    dataSetCheckpointMap.clear();
    uint32_t numDataSetes = 0;
    stream.read(numDataSetes);
    for (uint32_t dataSetIdx = 0; dataSetIdx < numDataSetes; dataSetIdx++) {
        std::string dataSetName;
        uint32_t numDataSetCheckpoints = 0;
        stream.read(dataSetName);
        stream.read(numDataSetCheckpoints);

        std::map<std::string, Checkpoint> dataSetCheckpoints;
        for (uint32_t dataSetCheckpointIdx = 0; dataSetCheckpointIdx < numDataSetCheckpoints; dataSetCheckpointIdx++) {
            std::string checkpointName;
            Checkpoint checkpoint;
            stream.read(checkpointName);
            stream.read(checkpoint);
            dataSetCheckpoints.insert(std::make_pair(checkpointName, checkpoint));
        }
        dataSetCheckpointMap.insert(std::make_pair(dataSetName, dataSetCheckpoints));
    }

    return true;
}

bool CheckpointWindow::writeToFile(const std::string& filename) {
    std::ofstream file(filename.c_str(), std::ofstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in CheckpointWindow::writeToFile: File \"" + filename + "\" not found.");
        return false;
    }

    sgl::BinaryWriteStream stream;
    stream.write((uint32_t)CHECKPOINT_FORMAT_VERSION);

    uint32_t numDataSetes = (uint32_t)dataSetCheckpointMap.size();
    stream.write(numDataSetes);
    for (auto& dataSetCheckpoints : dataSetCheckpointMap) {
        std::string dataSetName;
        uint32_t numDataSetCheckpoints = dataSetCheckpoints.second.size();
        stream.write(dataSetCheckpoints.first);
        stream.write(numDataSetCheckpoints);

        for (auto& dataSetCheckpoint : dataSetCheckpoints.second) {
            stream.write(dataSetCheckpoint.first);
            stream.write(dataSetCheckpoint.second);
        }
    }

    file.write((const char*)stream.getBuffer(), stream.getSize());
    file.close();

    return true;
}

bool CheckpointWindow::renderGui() {
    bool reRender = false;
    bool shallDeleteElement = false;
    size_t deleteElementId = 0;

    if (ImGui::Begin("Camera Checkpoints", &showWindow)) {
        ImGui::Columns(4, "CheckpointsColumns");
        ImGui::Separator();
        ImGui::Text("Name"); ImGui::NextColumn();
        //ImGui::Text("Position"); ImGui::NextColumn();
        //ImGui::Text("Yaw, Pitch"); ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::NextColumn();
        ImGui::Separator();
        for (size_t i = 0; i < loadedDataSetCheckpoints.size(); i++) {
            auto& dataSetCheckpoint = loadedDataSetCheckpoints.at(i);
            std::string inputTextLabel = "##input-" + std::to_string(i);
            std::string loadLabel = "Load##input-" + std::to_string(i);
            std::string updateLabel = "Update##input-" + std::to_string(i);
            std::string deleteLabel = "Delete##input-" + std::to_string(i);
            ImGui::InputText(inputTextLabel.c_str(), &dataSetCheckpoint.first); ImGui::NextColumn();
            //ImGui::Text(
            //        "%f, %f, %f", dataSetCheckpoint.second.position.x, dataSetCheckpoint.second.position.y,
            //        dataSetCheckpoint.second.position.z); ImGui::NextColumn();
            //ImGui::Text("%f, %f", dataSetCheckpoint.second.yaw, dataSetCheckpoint.second.pitch); ImGui::NextColumn();
            if (ImGui::Button(loadLabel.c_str())) {
                sceneData.camera->setPosition(dataSetCheckpoint.second.position);
                sceneData.camera->setYaw(dataSetCheckpoint.second.yaw);
                sceneData.camera->setPitch(dataSetCheckpoint.second.pitch);
                reRender = true;
            } ImGui::NextColumn();
            if (ImGui::Button(updateLabel.c_str())) {
                dataSetCheckpoint.second.position = sceneData.camera->getPosition();
                dataSetCheckpoint.second.yaw = sceneData.camera->getYaw();
                dataSetCheckpoint.second.pitch = sceneData.camera->getPitch();
            } ImGui::NextColumn();
            if (ImGui::Button(deleteLabel.c_str())) {
                deleteElementId = i;
                shallDeleteElement = true;
            } ImGui::NextColumn();
        }
        ImGui::Columns(1);
        ImGui::Separator();

        if (ImGui::Button("Create Checkpoint")) {
            Checkpoint checkpoint(
                    sceneData.camera->getPosition(),
                    sceneData.camera->getYaw(),
                    sceneData.camera->getPitch());
            loadedDataSetCheckpoints.push_back(std::make_pair("New Checkpoint", checkpoint));
        }
    }
    ImGui::End();

    if (shallDeleteElement) {
        std::vector<std::pair<std::string, Checkpoint>> loadedDataSetCheckpointsTmp = loadedDataSetCheckpoints;
        loadedDataSetCheckpoints.clear();
        for (size_t i = 0; i < loadedDataSetCheckpointsTmp.size(); i++) {
            if (i != deleteElementId) {
                loadedDataSetCheckpoints.push_back(loadedDataSetCheckpointsTmp.at(i));
            }
        }
    }

    return reRender;
}
