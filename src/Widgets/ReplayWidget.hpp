/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020-2021, Christoph Neuhauser
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

#ifndef HEXVOLUMERENDERER_REPLAYWIDGET_HPP
#define HEXVOLUMERENDERER_REPLAYWIDGET_HPP

#include <vector>
#include <string>
#include <functional>

#include <Utils/File/PathWatch.hpp>
#include <Utils/SciVis/CameraPath.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "Utils/VecStringConversion.hpp"
#include "Renderers/SceneData.hpp"
#include "Utils/InternalState.hpp"

namespace sgl {
    class TransferFunctionWindow;
    class CheckpointWindow;
}

enum ReplayDataType {
    REPLAY_DATA_TYPE_STRING,
    REPLAY_DATA_TYPE_BOOLEAN,
    REPLAY_DATA_TYPE_INTEGER,
    REPLAY_DATA_TYPE_REAL,
    REPLAY_DATA_TYPE_VEC2,
    REPLAY_DATA_TYPE_VEC3,
    REPLAY_DATA_TYPE_VEC4,
    REPLAY_DATA_TYPE_QUATERNION
};

class ReplaySettingsMap {
public:
    void insert(const std::string& key, const std::string& value, ReplayDataType dataType) {
        dataMap.insert(std::make_pair(key, std::make_pair(dataType, value)));
    }
    void insert(const std::string& key, const std::string& value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_STRING, value)));
    }
    void insert(const std::string& key, bool value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_BOOLEAN, value ? "true" : "false")));
    }
    void insert(const std::string& key, int value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_INTEGER, std::to_string(value))));
    }
    void insert(const std::string& key, long value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_INTEGER, std::to_string(value))));
    }
    void insert(const std::string& key, float value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_REAL, std::to_string(value))));
    }
    void insert(const std::string& key, double value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_REAL, std::to_string(value))));
    }
    void insert(const std::string& key, const glm::vec2& value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_VEC2, vec2ToString(value))));
    }
    void insert(const std::string& key, const glm::vec3& value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_VEC3, vec3ToString(value))));
    }
    void insert(const std::string& key, const glm::vec4& value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_VEC4, vec4ToString(value))));
    }
    void insert(const std::string& key, const glm::quat& value) {
        dataMap.insert(std::make_pair(key, std::make_pair(REPLAY_DATA_TYPE_QUATERNION, quatToString(value))));
    }

    inline bool empty() const { return dataMap.empty(); }

    void updateReplaySettings(ReplaySettingsMap& mNew) {
        for (auto it : mNew.dataMap) {
            this->dataMap[it.first] = it.second;
        }
    }

    // Settings that can't be interpolated.
    void setStaticSettings(SettingsMap& settingsMap) {
        for (auto it : dataMap) {
            if (it.second.first == REPLAY_DATA_TYPE_STRING
                || it.second.first == REPLAY_DATA_TYPE_BOOLEAN) {
                settingsMap.addKeyValue(it.first, it.second.second);
            }
        }
    }

    // Settings that can be interpolated.
    void setDynamicSettings(SettingsMap& settingsMap) {
        for (auto it : dataMap) {
            if (it.second.first != REPLAY_DATA_TYPE_STRING
                    && it.second.first != REPLAY_DATA_TYPE_BOOLEAN) {
                settingsMap.addKeyValue(it.first, it.second.second);
            }
        }
    }

    // Settings that can be interpolated.
    void setInterpolatedDynamicSettings(ReplaySettingsMap& m1, SettingsMap& settingsMap, float t) {
        for (auto it : m1.dataMap) {
            if (it.second.first == REPLAY_DATA_TYPE_STRING
                    || it.second.first == REPLAY_DATA_TYPE_BOOLEAN) {
                continue;
            }

            auto it2 = this->dataMap.find(it.first);
            std::string& strValOld = it2->second.second;
            std::string& strValNew = it.second.second;

            std::string strValUpdate;
            if (it.second.first == REPLAY_DATA_TYPE_REAL || it.second.first == REPLAY_DATA_TYPE_INTEGER) {
                double valOld = sgl::fromString<double>(strValOld);
                double valNew = sgl::fromString<double>(strValNew);
                strValUpdate = std::to_string(glm::mix(valOld, valNew, t));
            } else if (it.second.first == REPLAY_DATA_TYPE_VEC2) {
                glm::vec2 valOld = stringToVec2(strValOld);
                glm::vec2 valNew = stringToVec2(strValNew);
                strValUpdate = vec2ToString(glm::mix(valOld, valNew, t));
            } else if (it.second.first == REPLAY_DATA_TYPE_VEC3) {
                glm::vec3 valOld = stringToVec3(strValOld);
                glm::vec3 valNew = stringToVec3(strValNew);
                strValUpdate = vec3ToString(glm::mix(valOld, valNew, t));
            } else if (it.second.first == REPLAY_DATA_TYPE_VEC4) {
                glm::vec4 valOld = stringToVec4(strValOld);
                glm::vec4 valNew = stringToVec4(strValNew);
                strValUpdate = vec4ToString(glm::mix(valOld, valNew, t));
            } else if (it.second.first == REPLAY_DATA_TYPE_QUATERNION) {
                glm::quat valOld = stringToQuat(strValOld);
                glm::quat valNew = stringToQuat(strValNew);
                strValUpdate = quatToString(glm::slerp(valOld, valNew, t));
            }

            settingsMap.addKeyValue(it.first, strValUpdate);
        }
    }

private:
    std::map<std::string, std::pair<ReplayDataType, std::string>> dataMap;
};


struct ReplayState {
    // REQUIRED!
    float duration = 0.0f;
    float startTime = 0.0f;
    float stopTime = 0.0f;

    // Mesh name (optional).
    std::string datasetName;
    std::string rendererName;
    int viewIndex = 0;

    // Transfer function data.
    std::string transferFunctionName;
    bool transferFunctionRangeSet = false;
    glm::vec2 transferFunctionRange;

    // Transfer function data (stress or multi-var data).
    std::vector<std::string> multiVarTransferFunctionNames;
    bool multiVarTransferFunctionRangesSet = false;
    std::vector<glm::vec2> multiVarTransferFunctionRanges;

    // The camera state to load (optional).
    bool cameraPositionSet = false;
    bool cameraOrientationSet = false;
    bool cameraFovySet = false;
    std::string cameraCheckpointName;
    glm::vec3 cameraPosition;
    glm::quat cameraOrientation;
    float cameraFovy;

    // For saving screenshots while running the replay states.
    std::string screenshotName;

    // Renderer and data set settings (optional).
    ReplaySettingsMap rendererSettings;
    ReplaySettingsMap datasetSettings;

    // For stress line tracer and scattering line tracer.
    ReplaySettingsMap tracerSettings;
};


class ReplayWidget {
public:
    ReplayWidget(
            SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow,
            sgl::CheckpointWindow& checkpointWindow);
    ~ReplayWidget();
    bool update(float currentTime, bool& stopRecording, bool& stopCameraFlight);

    [[nodiscard]] inline bool getShowWindow() const { return showWindow; }
    [[nodiscard]] inline bool& getShowWindow() { return showWindow; }
    inline void setShowWindow(bool show) { showWindow = show; }

    enum ReplayWidgetUpdateType {
        REPLAY_WIDGET_UPDATE_NONE,
        REPLAY_WIDGET_UPDATE_LOAD,
        REPLAY_WIDGET_UPDATE_STOP,
        REPLAY_WIDGET_UPDATE_START_RECORDING,
        REPLAY_WIDGET_UPDATE_STOP_RECORDING
    };
    ReplayWidgetUpdateType renderGui();

    inline const glm::mat4x4& getViewMatrix() { return currentCameraMatrix; }
    inline float getCameraFovy() { return currentFovy; }
    inline SettingsMap getCurrentRendererSettings() { return currentRendererSettings; }
    inline SettingsMap getCurrentDatasetSettings() { return currentDatasetSettings; }
    inline bool getUseCameraFlight() { return useCameraFlight; }

    /// Callback functions when, e.g., a new renderer is requested.
    void setLoadLineDataCallback(std::function<void(const std::string& datasetName)> loadLineDataCallback);
    void setLoadRendererCallback(
            std::function<void(const std::string& rendererName, int viewIdx)> loadRendererCallback);
    void setLoadTransferFunctionCallback(
            std::function<void(const std::string& tfName)> loadTransferFunctionCallback);
    void setTransferFunctionRangeCallback(
            std::function<void(const glm::vec2& tfRange)> transferFunctionRangeCallback);
    void setLoadMultiVarTransferFunctionsCallback(
            std::function<void(const std::vector<std::string>& tfNames)> loadMultiVarTransferFunctionsCallback);
    void setMultiVarTransferFunctionsRangesCallback(
            std::function<void(const std::vector<glm::vec2>& tfRanges)> multiVarTransferFunctionsRangesCallback);
    void setLineTracerSettingsCallback(
            std::function<void(const SettingsMap& settings)> lineTracerSettingsCallback);
    void setSaveScreenshotCallback(std::function<void(const std::string& screenshotName)> saveScreenshotCallback);

private:
    // Global data.
    SceneData* sceneData;
    sgl::TransferFunctionWindow& transferFunctionWindow;
    sgl::CheckpointWindow& checkpointWindow;

    // Script directory data.
    sgl::PathWatch directoryContentWatch;
    std::string scriptDirectory;
    std::string scriptFileName;
    void updateAvailableReplayScripts();
    std::function<void(const std::string& datasetName)> loadLineDataCallback;
    std::function<void(const std::string& rendererName, int viewIdx)> loadRendererCallback;
    std::function<void(const std::string& tfName)> loadTransferFunctionCallback;
    std::function<void(const glm::vec2& tfRange)> transferFunctionRangeCallback;
    std::function<void(const std::vector<std::string>& tfNames)> loadMultiVarTransferFunctionsCallback;
    std::function<void(const std::vector<glm::vec2>& tfRanges)> multiVarTransferFunctionsRangesCallback;
    std::function<void(const SettingsMap& settings)> lineTracerSettingsCallback;
    std::function<void(const std::string& screenshotName)> saveScreenshotCallback;

    // Script data.
    bool runScript(const std::string& filename);
    std::vector<ReplayState> replayStates;
    bool firstTimeState = true;
    int currentStateIndex = 0;

    ReplaySettingsMap replaySettingsRendererLast, replaySettingsDatasetLast;
    SettingsMap currentRendererSettings, currentDatasetSettings;
    glm::vec3 cameraPositionLast;
    glm::quat cameraOrientationLast;
    float cameraFovyLast;
    glm::mat4x4 currentCameraMatrix;
    float currentFovy;
    bool useCameraFlight = false;

    // Gui functions & data.
    ReplayWidgetUpdateType renderFileDialog();
    bool showWindow = true;
    std::vector<std::string> availableScriptFiles;
    int selectedFileIndex = -1;
    bool recording = false;
};

#endif //HEXVOLUMERENDERER_REPLAYWIDGET_HPP
