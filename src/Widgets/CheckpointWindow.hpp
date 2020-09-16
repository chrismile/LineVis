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

#ifndef HEXVOLUMERENDERER_CHECKPOINTWINDOW_HPP
#define HEXVOLUMERENDERER_CHECKPOINTWINDOW_HPP

#include <string>
#include <map>
#include <glm/glm.hpp>

#include "Renderers/SceneData.hpp"

class Checkpoint
{
public:
    Checkpoint() {}
    Checkpoint(const glm::vec3& position, float yaw, float pitch) : position(position), yaw(yaw), pitch(pitch) {}
    Checkpoint(float tx, float ty, float tz, float yaw, float pitch) : position(tx, ty, tz), yaw(yaw), pitch(pitch) {}

    glm::vec3 position;
    float yaw;
    float pitch;
};

class CheckpointWindow {
public:
    CheckpointWindow(SceneData& sceneData);
    ~CheckpointWindow();
    void onLoadDataSet(const std::string& dataSetName);

    /// @return true if re-rendering the scene is necessary.
    bool renderGui();

private:
    bool readFromFile(const std::string& filename);
    bool writeToFile(const std::string& filename);

    SceneData& sceneData;
    const uint32_t CHECKPOINT_FORMAT_VERSION = 1u;

    const std::string saveDirectoryCheckpoints = "Data/Checkpoints/";
    const std::string checkpointsFilename = saveDirectoryCheckpoints + "checkpoints.bin";
    std::map<std::string, std::map<std::string, Checkpoint>> dataSetCheckpointMap;
    std::string loadedDataSetName;
    std::vector<std::pair<std::string, Checkpoint>> loadedDataSetCheckpoints;

    bool showWindow = true;
};


#endif //HEXVOLUMERENDERER_CHECKPOINTWINDOW_HPP
