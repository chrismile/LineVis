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

#define GLM_ENABLE_EXPERIMENTAL
#include <climits>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <thread>

#include <glm/gtx/color_space.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <GL/glew.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include <Utils/AppSettings.hpp>
#include <Utils/Timer.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/Events/EventManager.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Texture/Bitmap.hpp>
#include <Graphics/OpenGL/SystemGL.hpp>

#include "Loaders/TrajectoryFile.hpp"
#include "LineData/LineDataFlow.hpp"
#include "LineData/LineDataStress.hpp"
#include "Renderers/OpaqueLineRenderer.hpp"
#include "Renderers/PerPixelLinkedListLineRenderer.hpp"
#include "Renderers/OpacityOptimizationRenderer.hpp"
#include "Renderers/DepthComplexityRenderer.hpp"
#include "MainApp.hpp"

void openglErrorCallback() {
    std::cerr << "Application callback" << std::endl;
}

MainApp::MainApp()
        : camera(new sgl::Camera()),
          sceneData(
                  sceneFramebuffer, sceneTexture, sceneDepthRBO, camera, clearColor, performanceMeasurer,
                  recording, useCameraFlight),
          checkpointWindow(sceneData), videoWriter(NULL) {
    // https://www.khronos.org/registry/OpenGL/extensions/NVX/NVX_gpu_memory_info.txt
    GLint freeMemKilobytes = 0;
    if (usePerformanceMeasurementMode
        && sgl::SystemGL::get()->isGLExtensionAvailable("GL_NVX_gpu_memory_info")) {
        glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &freeMemKilobytes);
    }
    glEnable(GL_CULL_FACE);

    sgl::FileUtils::get()->ensureDirectoryExists(saveDirectoryScreenshots);
    sgl::FileUtils::get()->ensureDirectoryExists(saveDirectoryVideos);
    sgl::FileUtils::get()->ensureDirectoryExists(saveDirectoryCameraPaths);
    setPrintFPS(false);

    gammaCorrectionShader = sgl::ShaderManager->getShaderProgram(
            {"GammaCorrection.Vertex", "GammaCorrection.Fragment"});

    sgl::EventManager::get()->addListener(sgl::RESOLUTION_CHANGED_EVENT,
                                          [this](sgl::EventPtr event) { this->resolutionChanged(event); });

    camera->setNearClipDistance(0.001f);
    camera->setFarClipDistance(100.0f);
    camera->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    float fovy = atanf(1.0f / 2.0f) * 2.0f;
    camera->setFOVy(fovy);
    camera->setPosition(glm::vec3(0.0f, 0.0f, 0.8f));

    clearColor = sgl::Color(255, 255, 255, 255);
    clearColorSelection = ImColor(clearColor.getColorRGBA());
    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);

    int desktopWidth = 0;
    int desktopHeight = 0;
    int refreshRate = 60;
    sgl::AppSettings::get()->getDesktopDisplayMode(desktopWidth, desktopHeight, refreshRate);
    sgl::Logfile::get()->writeInfo("Desktop refresh rate: " + std::to_string(refreshRate) + " FPS");

    bool useVsync = sgl::AppSettings::get()->getSettings().getBoolValue("window-vSync");
    if (useVsync) {
        sgl::Timer->setFPSLimit(true, refreshRate);
    } else {
        sgl::Timer->setFPSLimit(false, refreshRate);
    }

    fpsArray.resize(16, refreshRate);
    framerateSmoother = FramerateSmoother(1);

    sgl::Renderer->setErrorCallback(&openglErrorCallback);
    sgl::Renderer->setDebugVerbosity(sgl::DEBUG_OUTPUT_CRITICAL_ONLY);
    resolutionChanged(sgl::EventPtr());

    selectedAttributeIndex = 0;
    changeQualityMeasureType();

    if (usePerformanceMeasurementMode) {
        useCameraFlight = true;
    }
    if (useCameraFlight && recording) {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        window->setWindowSize(recordingResolution.x, recordingResolution.y);
        realTimeCameraFlight = false;
        transferFunctionWindow.loadFunctionFromFile("Data/TransferFunctions/Standard_PerVertex.xml");
        loadLineDataSet({ "Data/LineDataSets/rings.obj" });
        renderingMode = RENDERING_MODE_OPACITY_OPTIMIZATION;
    }

    if (!recording && !usePerformanceMeasurementMode) {
        // Just for convenience...
        if (desktopWidth == 3840 && desktopHeight == 2160) {
            sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
            window->setWindowSize(2186, 1358);
        }
    }

    setRenderer();

    customDataSetFileName = sgl::FileUtils::get()->getUserDirectory();
    loadAvailableDataSetInformation();

    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
    usesNewState = true;
    if (usePerformanceMeasurementMode) {
        sgl::FileUtils::get()->ensureDirectoryExists("images");
        performanceMeasurer = new AutomaticPerformanceMeasurer(
                getTestModesPaper(), "performance.csv", "depth_complexity.csv",
                [this](const InternalState &newState) { this->setNewState(newState); });
        performanceMeasurer->setInitialFreeMemKilobytes(freeMemKilobytes);
    }
}

MainApp::~MainApp() {
    if (usePerformanceMeasurementMode) {
        delete performanceMeasurer;
        performanceMeasurer = nullptr;
    }

    delete lineRenderer;

    if (videoWriter != NULL) {
        delete videoWriter;
    }
}

void MainApp::setNewState(const InternalState &newState) {
    if (performanceMeasurer) {
        performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(0);
    }

    // 1. Change the window resolution?
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int currentWindowWidth = window->getWidth();
    int currentWindowHeight = window->getHeight();
    glm::ivec2 newResolution = newState.windowResolution;
    if (newResolution.x > 0 && newResolution.x > 0 && currentWindowWidth != newResolution.x
        && currentWindowHeight != newResolution.y) {
        window->setWindowSize(newResolution.x, newResolution.y);
    }

    // 1.1. Handle the new tiling mode for SSBO accesses (TODO).
    //setNewTilingMode(newState.tilingWidth, newState.tilingHeight, newState.useMortonCodeForTiling);

    // 1.2. Load the new transfer function if necessary.
    if (!newState.transferFunctionName.empty() && newState.transferFunctionName != lastState.transferFunctionName) {
        transferFunctionWindow.loadFunctionFromFile("Data/TransferFunctions/" + newState.transferFunctionName);
    }

    // 2.1. Do we need to load new renderers?
    if (firstState || newState.renderingMode != lastState.renderingMode
        || newState.rendererSettings != lastState.rendererSettings) {
        delete lineRenderer;
        lineRenderer = nullptr;
        renderingMode = newState.renderingMode;
        setRenderer();
    }

    // 2.2. Pass state change to renderers to handle internally necessary state changes.
    lineRenderer->setNewState(newState);
    lineRenderer->setNewSettings(newState.rendererSettings);

    // 3. Load the correct mesh file.
    if (newState.dataSetDescriptor != lastState.dataSetDescriptor) {
        selectedDataSetIndex = 0;
        std::string nameLower = boost::algorithm::to_lower_copy(newState.dataSetDescriptor.name);
        for (size_t i = 0; i < dataSetInformation.size(); i++) {
            if (boost::algorithm::to_lower_copy(dataSetInformation.at(i).name) == nameLower) {
                selectedDataSetIndex = i + 1;
                break;
            }
        }
        if (selectedDataSetIndex == 0) {
            if (dataSetInformation.at(selectedDataSetIndex - 1).type == DATA_SET_TYPE_STRESS_LINES
                    && newState.dataSetDescriptor.enabledFileIndices.size() == 3) {
                this->useMajorPS = newState.dataSetDescriptor.enabledFileIndices.at(0);
                this->useMediumPS = newState.dataSetDescriptor.enabledFileIndices.at(1);
                this->useMinorPS = newState.dataSetDescriptor.enabledFileIndices.at(2);
            }
            loadLineDataSet(newState.dataSetDescriptor.filenames);
        } else {
            if (newState.dataSetDescriptor.type == DATA_SET_TYPE_STRESS_LINES
                    && newState.dataSetDescriptor.enabledFileIndices.size() == 3) {
                this->useMajorPS = newState.dataSetDescriptor.enabledFileIndices.at(0);
                this->useMediumPS = newState.dataSetDescriptor.enabledFileIndices.at(1);
                this->useMinorPS = newState.dataSetDescriptor.enabledFileIndices.at(2);
            }
            loadLineDataSet(getSelectedMeshFilenames());
        }
    }

    recordingTime = 0.0f;
    recordingTimeLast = 0.0f;
    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
    lastState = newState;
    firstState = false;
    usesNewState = true;
}

void MainApp::setRenderer() {
    if (lineRenderer) {
        delete lineRenderer;
        lineRenderer = nullptr;
    }

    if (renderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
        lineRenderer = new OpaqueLineRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_PER_PIXEL_LINKED_LIST) {
        lineRenderer = new PerPixelLinkedListLineRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_OPACITY_OPTIMIZATION) {
        lineRenderer = new OpacityOptimizationRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        lineRenderer = new DepthComplexityRenderer(sceneData, transferFunctionWindow);
    }
}

void MainApp::resolutionChanged(sgl::EventPtr event) {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    glViewport(0, 0, width, height);
    windowResolution = glm::ivec2(width, height);

    // Buffers for off-screen rendering
    sceneFramebuffer = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA16;
    } else {
        textureSettings.internalFormat = GL_RGBA8; // GL_RGBA8 For i965 driver to accept image load/store (legacy)
    }
    textureSettings.pixelType = GL_UNSIGNED_BYTE;
    textureSettings.pixelFormat = GL_RGB;
    sceneTexture = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
    sceneFramebuffer->bindTexture(sceneTexture);
    sceneDepthRBO = sgl::Renderer->createRBO(width, height, sgl::RBO_DEPTH24_STENCIL8);
    sceneFramebuffer->bindRenderbuffer(sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);

    camera->onResolutionChanged(event);
    if (lineRenderer != nullptr) {
        lineRenderer->onResolutionChanged();
    }

    reRender = true;
}

void MainApp::saveScreenshot(const std::string &filename) {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    sgl::BitmapPtr bitmap(new sgl::Bitmap(width, height, 32));
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, bitmap->getPixels());
    bitmap->savePNG(filename.c_str(), true);
    screenshot = false;
}

void MainApp::updateColorSpaceMode() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();

    // Buffers for off-screen rendering
    sceneFramebuffer = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    if (useLinearRGB) {
        textureSettings.internalFormat = GL_RGBA16;
    } else {
        textureSettings.internalFormat = GL_RGBA8; // GL_RGBA8 For i965 driver to accept image load/store (legacy)
    }
    textureSettings.pixelType = GL_UNSIGNED_BYTE;
    textureSettings.pixelFormat = GL_RGB;
    sceneTexture = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
    sceneFramebuffer->bindTexture(sceneTexture);
    sceneDepthRBO = sgl::Renderer->createRBO(width, height, sgl::RBO_DEPTH24_STENCIL8);
    sceneFramebuffer->bindRenderbuffer(sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);

    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
}

void MainApp::processSDLEvent(const SDL_Event &event) {
    sgl::ImGuiWrapper::get()->processSDLEvent(event);
}

void MainApp::render() {
    if (videoWriter == NULL && recording) {
        videoWriter = new sgl::VideoWriter(saveFilenameVideos + ".mp4", FRAME_RATE_VIDEOS);
    }

    prepareVisualizationPipeline();

    if (lineRenderer != nullptr) {
        reRender = reRender || lineRenderer->needsReRender();
    }

    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int width = window->getWidth();
    int height = window->getHeight();
    glViewport(0, 0, width, height);

    if (reRender || continuousRendering) {
        if (renderingMode != RENDERING_MODE_PER_PIXEL_LINKED_LIST && usePerformanceMeasurementMode) {
            performanceMeasurer->startMeasure(recordingTimeLast);
        }

        sgl::Renderer->bindFBO(sceneFramebuffer);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, clearColor);

        sgl::Renderer->setProjectionMatrix(camera->getProjectionMatrix());
        sgl::Renderer->setViewMatrix(camera->getViewMatrix());
        sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

        glEnable(GL_DEPTH_TEST);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
        glBlendEquation(GL_FUNC_ADD);

        if (lineData.get() != nullptr && lineRenderer != nullptr) {
            lineRenderer->render();
        }

        if (renderingMode != RENDERING_MODE_ALL_LINES_OPAQUE && usePerformanceMeasurementMode) {
            performanceMeasurer->endMeasure();
        }

        reRender = false;
    }

    // Render to screen
    sgl::Renderer->unbindFBO();
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    if (useLinearRGB) {
        sgl::Renderer->blitTexture(
                sceneTexture, sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)),
                gammaCorrectionShader);
    } else {
        sgl::Renderer->blitTexture(
                sceneTexture, sgl::AABB2(glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, 1.0f)));
    }

    if (!uiOnScreenshot && screenshot) {
        printNow = true;
        saveScreenshot(
                saveDirectoryScreenshots + saveFilenameScreenshots
                + "_" + sgl::toString(screenshotNumber++) + ".png");
        printNow = false;
    }

    // Video recording enabled?
    if (recording) {
        videoWriter->pushWindowFrame();
    }

    renderGui();
}

void MainApp::renderGui() {
    sgl::ImGuiWrapper::get()->renderStart();

    if (showSettingsWindow) {
        if (ImGui::Begin("Settings", &showSettingsWindow)) {
            // Draw an FPS counter
            static float displayFPS = 60.0f;
            static uint64_t fpsCounter = 0;
            if (sgl::Timer->getTicksMicroseconds() - fpsCounter > 1e6) {
                displayFPS = ImGui::GetIO().Framerate;
                fpsCounter = sgl::Timer->getTicksMicroseconds();
            }
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / fps, fps);
            ImGui::Separator();

            // Selection of displayed model
            renderFileSelectionSettingsGui();

            ImGui::Separator();

            if (ImGui::CollapsingHeader("Scene Settings", NULL, ImGuiTreeNodeFlags_DefaultOpen)) {
                renderSceneSettingsGui();
            }
        }
        ImGui::End();
    }

    if (transferFunctionWindow.renderGUI()) {
        reRender = true;
        if (transferFunctionWindow.getTransferFunctionMapRebuilt()) {
            if (lineData) {
                lineData->onTransferFunctionMapRebuilt();
            }
            if (lineRenderer) {
                lineRenderer->onTransferFunctionMapRebuilt();
            }
        }
    }

    if (checkpointWindow.renderGui()) {
        reRender = true;
        hasMoved();
    }

    lineRenderer->renderGui();

    sgl::ImGuiWrapper::get()->renderEnd();
}

void MainApp::loadAvailableDataSetInformation() {
    dataSetNames.clear();
    dataSetNames.push_back("Local file...");
    selectedDataSetIndex = 0;

    if (sgl::FileUtils::get()->exists(lineDataSetsDirectory + "datasets.json")) {
        dataSetInformation = loadDataSetList(lineDataSetsDirectory + "datasets.json");
        for (DataSetInformation& dataSetInfo  : dataSetInformation) {
            dataSetNames.push_back(dataSetInfo.name);
        }
    }
}

std::vector<std::string> MainApp::getSelectedMeshFilenames() {
    std::vector<std::string> filenames;
    if (selectedDataSetIndex == 0) {
        dataSetType = DATA_SET_TYPE_FLOW_LINES;
        filenames.push_back(customDataSetFileName);
        return filenames;
    }
    dataSetType = dataSetInformation.at(selectedDataSetIndex - 1).type;
    for (const std::string& filename : dataSetInformation.at(selectedDataSetIndex - 1).filenames) {
        filenames.push_back(lineDataSetsDirectory + filename);
    }
    return filenames;
}

void MainApp::renderFileSelectionSettingsGui() {
    if (ImGui::Combo(
            "Data Set", &selectedDataSetIndex, dataSetNames.data(),
            dataSetNames.size())) {
        loadLineDataSet(getSelectedMeshFilenames());
    }

    if (selectedDataSetIndex == 0) {
        ImGui::InputText("##meshfilenamelabel", &customDataSetFileName);
        ImGui::SameLine();
        if (ImGui::Button("Load File")) {
            loadLineDataSet(getSelectedMeshFilenames());
        }
    } else if (dataSetType == DATA_SET_TYPE_STRESS_LINES) {
        bool usedPsChanged = false;
        usedPsChanged |= ImGui::Checkbox("Major", &useMajorPS); ImGui::SameLine();
        usedPsChanged |= ImGui::Checkbox("Medium", &useMediumPS); ImGui::SameLine();
        usedPsChanged |= ImGui::Checkbox("Minor", &useMinorPS);
        if (usedPsChanged && lineData.get() != nullptr) {
            static_cast<LineDataStress*>(lineData.get())->setUsedPsDirections(
                    {useMajorPS, useMediumPS, useMinorPS});
        }
    }
}

void MainApp::renderSceneSettingsGui() {
    if (ImGui::ColorEdit3("Clear Color", (float*)&clearColorSelection, 0)) {
        clearColor = sgl::colorFromFloat(
                clearColorSelection.x, clearColorSelection.y, clearColorSelection.z, clearColorSelection.w);
        transferFunctionWindow.setClearColor(clearColor);
        reRender = true;
    }

    // Select light direction
    // Spherical coordinates: (r, θ, φ), i.e. with radial distance r, azimuthal angle θ (theta), and polar angle φ (phi)
    /*static float theta = sgl::PI/2;
    static float phi = 0.0f;
    bool angleChanged = false;
    angleChanged = ImGui::SliderAngle("Light Azimuth", &theta, 0.0f) || angleChanged;
    angleChanged = ImGui::SliderAngle("Light Polar Angle", &phi, 0.0f) || angleChanged;
    if (angleChanged) {
        // https://en.wikipedia.org/wiki/List_of_common_coordinate_transformations#To_cartesian_coordinates
        lightDirection = glm::vec3(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta));
        reRender = true;
    }*/

    if (ImGui::Button("Reset Camera")) {
        camera->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
        camera->setYaw(-sgl::PI/2.0f); //< around y axis
        camera->setPitch(0.0f); //< around x axis
        camera->setPosition(glm::vec3(0.0f, 0.0f, 0.8f));
        reRender = true;
        hasMoved();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Continuous Rendering", &continuousRendering);
    ImGui::Checkbox("UI on Screenshot", &uiOnScreenshot);
    ImGui::SameLine();
    if (ImGui::Checkbox("Use Linear RGB", &useLinearRGB)) {
        updateColorSpaceMode();
        reRender = true;
    }
    ImGui::Checkbox("Show Transfer Function Window", &transferFunctionWindow.getShowTransferFunctionWindow());

    if (ImGui::Combo(
            "Rendering Mode", (int*)&renderingMode, RENDERING_MODE_NAMES,
            IM_ARRAYSIZE(RENDERING_MODE_NAMES))) {
        setRenderer();
        reRender = true;
    }

    // Switch importance criterion.
    if (lineData) {
        if (ImGui::Combo(
                "Importance Crit.", (int*)&selectedAttributeIndex,
                attributeNames.data(), attributeNames.size())) {
            changeQualityMeasureType();
            reRender = true;
        }
    }

    ImGui::SliderFloat("Move Speed", &MOVE_SPEED, 0.02f, 0.5f);
    ImGui::SliderFloat("Mouse Speed", &MOUSE_ROT_SPEED, 0.01f, 0.10f);
    if (ImGui::SliderFloat3("Rotation Axis", &modelRotationAxis.x, 0.0f, 1.0f)) {
        if (rotateModelBy90DegreeTurns != 0) {
            loadLineDataSet(getSelectedMeshFilenames());
        }
    }
    if (ImGui::SliderInt("Rotation 90°", &rotateModelBy90DegreeTurns, 0, 3)) {
        loadLineDataSet(getSelectedMeshFilenames());
    }

    if (ImGui::Checkbox("Use Camera Flight", &useCameraFlight)) {
        startedCameraFlightPerUI = true;
        reRender = true;
    }

    ImGui::Separator();

    ImGui::InputText("##savescreenshotlabel", &saveFilenameScreenshots);
    if (ImGui::Button("Save Screenshot")) {
        saveScreenshot(
                saveDirectoryScreenshots + saveFilenameScreenshots
                + "_" + sgl::toString(screenshotNumber++) + ".png");
    }

    ImGui::Separator();

    ImGui::InputText("##savevideolabel", &saveFilenameVideos);
    if (!recording) {
        bool startRecording = false;
        if (ImGui::Button("Start Recording Video")) {
            startRecording = true;
        } ImGui::SameLine();
        if (ImGui::Button("Start Recording Video Camera Path")) {
            startRecording = true;
            useCameraFlight = true;
            startedCameraFlightPerUI = true;
            recordingTime = 0.0f;
            realTimeCameraFlight = false;
            cameraPath.resetTime();
            reRender = true;
        }

        if (startRecording) {
            sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
            if (window->getWindowResolution() != recordingResolution) {
                window->setWindowSize(recordingResolution.x, recordingResolution.y);
            }

            recording = true;
            videoWriter = new sgl::VideoWriter(
                    saveDirectoryVideos + saveFilenameVideos
                    + "_" + sgl::toString(videoNumber++) + ".mp4", FRAME_RATE_VIDEOS);
        }
    } else {
        if (ImGui::Button("Stop Recording Video")) {
            recording = false;
            delete videoWriter;
            videoWriter = nullptr;
        }
    }

    ImGui::Separator();

    ImGui::SliderInt2("Window Resolution", &windowResolution.x, 480, 3840);
    if (ImGui::Button("Set Resolution")) {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        window->setWindowSize(windowResolution.x, windowResolution.y);
    }
}

void MainApp::update(float dt) {
    sgl::AppLogic::update(dt);

    fpsArrayOffset = (fpsArrayOffset + 1) % fpsArray.size();
    fpsArray[fpsArrayOffset] = 1.0f/dt;
    recordingTimeLast = recordingTime;

    if (usePerformanceMeasurementMode && !performanceMeasurer->update(recordingTime)) {
        // All modes were tested -> quit.
        quit();
    }

    if (useCameraFlight && lineData.get() != nullptr) {
        cameraPath.update(recordingTime);
        camera->overwriteViewMatrix(cameraPath.getViewMatrix());
        reRender = true;
        hasMoved();
    }

    // Already recorded full cycle?
    if (useCameraFlight && recording && recordingTime > cameraPath.getEndTime()) {
        if (!startedCameraFlightPerUI) {
            quit();
        } else {
            if (recording) {
                recording = false;
                delete videoWriter;
                videoWriter = nullptr;
                realTimeCameraFlight = true;
            }
            useCameraFlight = false;
        }
        recordingTime = 0.0f;
    }

    if (useCameraFlight) {
        // Update camera position.
        if (usePerformanceMeasurementMode) {
            recordingTime += 1.0f;
        } else{
            if (realTimeCameraFlight) {
                uint64_t currentTimeStamp = sgl::Timer->getTicksMicroseconds();
                uint64_t timeElapsedMicroSec = currentTimeStamp - recordingTimeStampStart;
                recordingTime = timeElapsedMicroSec * 1e-6;
                if (usesNewState) {
                    // A new state was just set. Don't recompute, as this would result in time of ca. 1-2ns
                    usesNewState = false;
                    recordingTime = 0.0f;
                }
            } else {
                recordingTime += FRAME_TIME_CAMERA_PATH;
            }
        }
    }

    transferFunctionWindow.update(dt);

    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureKeyboard && !recording) {
        // Ignore inputs below
        return;
    }

    // Rotate scene around camera origin
    /*if (sgl::Keyboard->isKeyDown(SDLK_x)) {
        glm::quat rot = glm::quat(glm::vec3(dt*ROT_SPEED, 0.0f, 0.0f));
        camera->rotate(rot);
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_y)) {
        glm::quat rot = glm::quat(glm::vec3(0.0f, dt*ROT_SPEED, 0.0f));
        camera->rotate(rot);
        reRender = true;
    }
    if (sgl::Keyboard->isKeyDown(SDLK_z)) {
        glm::quat rot = glm::quat(glm::vec3(0.0f, 0.0f, dt*ROT_SPEED));
        camera->rotate(rot);
        reRender = true;
    }*/
    if (sgl::Keyboard->isKeyDown(SDLK_q)) {
        camera->rotateYaw(-1.9f*dt*MOVE_SPEED);
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_e)) {
        camera->rotateYaw(1.9f*dt*MOVE_SPEED);
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_r)) {
        camera->rotatePitch(1.9f*dt*MOVE_SPEED);
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_f)) {
        camera->rotatePitch(-1.9f*dt*MOVE_SPEED);
        reRender = true;
        hasMoved();
    }

    if (sgl::Keyboard->isKeyDown(SDLK_u)) {
        showSettingsWindow = !showSettingsWindow;
        transferFunctionWindow.setShow(showSettingsWindow);
    }


    glm::mat4 rotationMatrix = camera->getRotationMatrix();//glm::mat4(camera->getOrientation());
    glm::mat4 invRotationMatrix = glm::inverse(rotationMatrix);
    if (sgl::Keyboard->isKeyDown(SDLK_PAGEDOWN)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, -dt*MOVE_SPEED, 0.0f)));
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_PAGEUP)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, dt*MOVE_SPEED, 0.0f)));
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_DOWN) || sgl::Keyboard->isKeyDown(SDLK_s)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, 0.0f, dt*MOVE_SPEED)));
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_UP) || sgl::Keyboard->isKeyDown(SDLK_w)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, 0.0f, -dt*MOVE_SPEED)));
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_LEFT) || sgl::Keyboard->isKeyDown(SDLK_a)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(-dt*MOVE_SPEED, 0.0f, 0.0f)));
        reRender = true;
        hasMoved();
    }
    if (sgl::Keyboard->isKeyDown(SDLK_RIGHT) || sgl::Keyboard->isKeyDown(SDLK_d)) {
        camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(dt*MOVE_SPEED, 0.0f, 0.0f)));
        reRender = true;
        hasMoved();
    }

    if (io.WantCaptureMouse) {
        // Ignore inputs below
        return;
    }

    lineRenderer->update(dt);

    if (!(sgl::Keyboard->getModifier() & (KMOD_CTRL | KMOD_SHIFT))) {
        // Zoom in/out
        if (sgl::Mouse->getScrollWheel() > 0.1 || sgl::Mouse->getScrollWheel() < -0.1) {
            float moveAmount = sgl::Mouse->getScrollWheel()*dt*2.0;
            camera->translate(sgl::transformPoint(invRotationMatrix, glm::vec3(0.0f, 0.0f, -moveAmount*MOVE_SPEED)));
            reRender = true;
            hasMoved();
        }

        // Mouse rotation
        if (sgl::Mouse->isButtonDown(1) && sgl::Mouse->mouseMoved()) {
            sgl::Point2 pixelMovement = sgl::Mouse->mouseMovement();
            float yaw = dt*MOUSE_ROT_SPEED*pixelMovement.x;
            float pitch = -dt*MOUSE_ROT_SPEED*pixelMovement.y;

            glm::quat rotYaw = glm::quat(glm::vec3(0.0f, yaw, 0.0f));
            glm::quat rotPitch = glm::quat(
                    pitch*glm::vec3(rotationMatrix[0][0], rotationMatrix[1][0], rotationMatrix[2][0]));
            camera->rotateYaw(yaw);
            camera->rotatePitch(pitch);
            reRender = true;
            hasMoved();
        }
    }
}

void MainApp::hasMoved() {
    if (lineRenderer != nullptr) {
        lineRenderer->onHasMoved();
    }
}


// --- Visualization pipeline ---

void MainApp::loadLineDataSet(const std::vector<std::string>& fileNames) {
    if (fileNames.size() == 0 || fileNames.front().size() == 0) {
        lineData = LineDataPtr();
        return;
    }
    currentlyLoadedDataSetIndex = selectedDataSetIndex;
    selectedAttributeIndex = 0;
    LineRenderer::setLineWidth(STANDARD_LINE_WIDTH);

    DataSetInformation selectedDataSetInformation;
    if (selectedDataSetIndex > 0 && dataSetInformation.size() > 0) {
        selectedDataSetInformation = dataSetInformation.at(selectedDataSetIndex - 1);
        if (selectedDataSetInformation.hasCustomLineWidth) {
            LineRenderer::setLineWidth(selectedDataSetInformation.lineWidth);
        }
        attributeNames = selectedDataSetInformation.attributeNames;
    } else {
        selectedDataSetInformation.type = dataSetType;
        selectedDataSetInformation.filenames = fileNames;
        attributeNames.clear();
    }

    glm::mat4 transformationMatrix = sgl::matrixIdentity();
    glm::mat4* transformationMatrixPtr = nullptr;
    if (selectedDataSetInformation.hasCustomTransform) {
        transformationMatrix *= selectedDataSetInformation.transformMatrix;
        transformationMatrixPtr = &transformationMatrix;
    }
    if (rotateModelBy90DegreeTurns != 0) {
        transformationMatrix *= glm::rotate(rotateModelBy90DegreeTurns * sgl::HALF_PI, modelRotationAxis);
        transformationMatrixPtr = &transformationMatrix;
    }

    bool dataLoaded = false;
    Trajectories trajectories;
    std::vector<Trajectories> trajectoriesPs;
    std::vector<StressTrajectoriesData> stressTrajectoriesDataPs;
    if (dataSetType == DATA_SET_TYPE_FLOW_LINES) {
        trajectories = loadFlowTrajectoriesFromFile(
                fileNames.front(), true, false, transformationMatrixPtr);
        dataLoaded = !trajectories.empty();
    } else if (dataSetType == DATA_SET_TYPE_STRESS_LINES) {
        loadStressTrajectoriesFromFile(
                fileNames, trajectoriesPs, stressTrajectoriesDataPs,
                true, true, transformationMatrixPtr);
        dataLoaded = !trajectoriesPs.empty();
    }

    if (dataLoaded) {
        newMeshLoaded = true;
        std::string meshDescriptorName = fileNames.front();
        if (fileNames.size() > 1) {
            meshDescriptorName += std::string() + "_" + std::to_string(fileNames.size());
        }
        checkpointWindow.onLoadDataSet(meshDescriptorName);

        if (dataSetType == DATA_SET_TYPE_FLOW_LINES) {
            LineDataFlow* lineDataFlow = new LineDataFlow(transferFunctionWindow);
            lineData = LineDataPtr(lineDataFlow);
            lineDataFlow->setTrajectoryData(trajectories);
            modelBoundingBox = computeTrajectoriesAABB3(trajectories);
        } else if (dataSetType == DATA_SET_TYPE_STRESS_LINES) {
            LineDataStress* lineDataStress = new LineDataStress(transferFunctionWindow);
            lineData = LineDataPtr(lineDataStress);
            lineDataStress->setStressTrajectoryData(trajectoriesPs, stressTrajectoriesDataPs);
            lineDataStress->setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
            modelBoundingBox = computeTrajectoriesPsAABB3(trajectoriesPs);
        }
        lineData->setQualityMeasureIndex(selectedAttributeIndex);

        for (size_t attrIdx = attributeNames.size(); attrIdx < lineData->getNumAttributes(); attrIdx++) {
            attributeNames.push_back(std::string() + "Attribute #" + std::to_string(attrIdx + 1));
        }


        if (true) { // useCameraFlight
            std::string cameraPathFilename =
                    saveDirectoryCameraPaths + sgl::FileUtils::get()->getPathAsList(meshDescriptorName).back()
                    + ".binpath";
            if (sgl::FileUtils::get()->exists(cameraPathFilename)) {
                cameraPath.fromBinaryFile(cameraPathFilename);
            } else {
                cameraPath.fromCirclePath(
                        modelBoundingBox, meshDescriptorName,
                        usePerformanceMeasurementMode
                        ? CAMERA_PATH_TIME_PERFORMANCE_MEASUREMENT : CAMERA_PATH_TIME_RECORDING,
                        usePerformanceMeasurementMode);
                //cameraPath.saveToBinaryFile(cameraPathFilename);
            }
        }
    }
}

void MainApp::prepareVisualizationPipeline() {
    if (lineData && lineRenderer && (lineRenderer->isDirty() || lineData->isDirty())) {
        lineRenderer->setLineData(lineData, newMeshLoaded);
    }
    newMeshLoaded = false;
}

void MainApp::changeQualityMeasureType() {
    if (lineData) {
        lineData->setQualityMeasureIndex(selectedAttributeIndex);
    }
}
