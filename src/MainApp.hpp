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

#ifndef MAINAPP_HPP
#define MAINAPP_HPP

#include <string>
#include <vector>
#include <map>

#include <Utils/AppLogic.hpp>
#include <Graphics/Shader/Shader.hpp>
#include <Graphics/Video/VideoWriter.hpp>

#include "Widgets/TransferFunctionWindow.hpp"
#include "Loaders/DataSetList.hpp"
#include "Utils/CameraPath.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"

#include "Widgets/TransferFunctionWindow.hpp"
#include "Widgets/CheckpointWindow.hpp"

class LineRenderer;
class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class MainApp : public sgl::AppLogic {
public:
    MainApp();
    ~MainApp();

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState);

    void render();
    void update(float dt);
    void resolutionChanged(sgl::EventPtr event);
    void processSDLEvent(const SDL_Event &event);

private:
    /// Renders the GUI of the scene settings and all filters and renderers.
    void renderGui();
    /// Renders the GUI for selecting an input file.
    void renderFileSelectionSettingsGui();
    /// Render the scene settings GUI, e.g. for setting the background clear color.
    void renderSceneSettingsGui();
    /// Update the color space (linear RGB vs. sRGB).
    void updateColorSpaceMode();
    /// Override screenshot function to exclude GUI (if wanted by the user)
    void saveScreenshot(const std::string &filename);
    void makeScreenshot() {}
    // Called when the camera moved.
    void hasMoved();

    /// Scene data (e.g., camera, main framebuffer, ...).
    sgl::CameraPtr camera;
    SceneData sceneData;

    // Off-screen rendering
    sgl::FramebufferObjectPtr sceneFramebuffer;
    sgl::TexturePtr sceneTexture;
    sgl::RenderbufferObjectPtr sceneDepthRBO;
    sgl::ShaderProgramPtr gammaCorrectionShader;

    /// Scene data used in user interface.
    bool showSettingsWindow = true;
    sgl::Color clearColor;
    ImVec4 clearColorSelection = ImColor(255, 255, 255, 255);
    RenderingMode renderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;
    bool useLinearRGB = true;
    std::vector<float> fpsArray;
    size_t fpsArrayOffset = 0;
    bool uiOnScreenshot = false;
    bool printNow = false;
    std::string saveDirectoryScreenshots = "Data/Screenshots/";
    std::string saveFilenameScreenshots = "Screenshot";
    int screenshotNumber = 0;
    std::string saveDirectoryVideos = "Data/Videos/";
    std::string saveFilenameVideos = "Video";
    int videoNumber = 0;
    float MOVE_SPEED = 0.2f;
    float MOUSE_ROT_SPEED = 0.05f;
    glm::vec3 modelRotationAxis = glm::vec3(1.0f, 0.0f, 0.0f);
    int rotateModelBy90DegreeTurns = 0;
    glm::ivec2 windowResolution;

    // Data set GUI information.
    void loadAvailableDataSetInformation();
    std::vector<std::string> getSelectedMeshFilenames();
    std::vector<DataSetInformation> dataSetInformation;
    std::vector<std::string> dataSetNames; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int selectedDataSetIndex = 0; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int currentlyLoadedDataSetIndex = -1;
    std::string customDataSetFileName;
    DataSetType dataSetType = DATA_SET_TYPE_FLOW_LINES;
    // Should we show major, medium and/or minor principal stress lines?
    bool useMajorPS = true, useMediumPS = true, useMinorPS = true;

    // Continuous rendering: Re-render each frame or only when scene changes?
    bool continuousRendering = false;
    bool reRender = true;

    // Coloring & filtering dependent on importance criteria.
    int selectedAttributeIndex = 0;
    std::vector<std::string> attributeNames;
    TransferFunctionWindow transferFunctionWindow;

    // For loading and saving camera checkpoints.
    CheckpointWindow checkpointWindow;

    // For recording videos.
    bool recording = false;
    glm::ivec2 recordingResolution = glm::ivec2(2560, 1440); // 1920, 1080
    sgl::VideoWriter* videoWriter = nullptr;
    const int FRAME_RATE_VIDEOS = 30;
    float recordingTime = 0.0f;
    float recordingTimeLast = 0.0f;
    uint64_t recordingTimeStampStart;

    // Camera paths for recording videos without human interaction.
    CameraPath cameraPath;
    bool useCameraFlight = false;
    bool startedCameraFlightPerUI = false;
    bool realTimeCameraFlight = true; // Move camera in real elapsed time or camera frame rate?
    const std::string saveDirectoryCameraPaths = "Data/CameraPaths/";
    float FRAME_TIME_CAMERA_PATH = 1.0f / FRAME_RATE_VIDEOS; ///< Simulate constant frame rate.
    const float CAMERA_PATH_TIME_RECORDING = 30.0f;
    const float CAMERA_PATH_TIME_PERFORMANCE_MEASUREMENT = TIME_PERFORMANCE_MEASUREMENT;

    // For making performance measurements.
    bool usePerformanceMeasurementMode = false;
    AutomaticPerformanceMeasurer *performanceMeasurer = nullptr;
    InternalState lastState;
    bool firstState = true;
    bool usesNewState = true;


    /// --- Visualization pipeline ---

    /// Loads a hexahedral mesh from a file.
    void loadLineDataSet(const std::vector<std::string>& fileName);
    /// Prepares the visualization pipeline for rendering.
    void prepareVisualizationPipeline();
    /// Change the importance criterion used for coloring.
    void changeQualityMeasureType();
    /// Sets the used renderers.
    void setRenderer();

    LineRenderer* lineRenderer = nullptr;
    LineDataPtr lineData;
    bool newMeshLoaded = true;
    sgl::AABB3 modelBoundingBox;
};

#endif // MAINAPP_HPP