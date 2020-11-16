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
#include <algorithm>

#include <glm/gtx/color_space.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <GL/glew.h>
#include <boost/algorithm/string.hpp>

#include <Utils/Timer.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Input/Keyboard.hpp>
#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include "Loaders/TrajectoryFile.hpp"
#include "LineData/LineDataFlow.hpp"
#include "LineData/LineDataStress.hpp"
#include "LineData/LineDataMultiVar.hpp"
#include "Renderers/OIT/TilingMode.hpp"
#include "Renderers/OpaqueLineRenderer.hpp"
#include "Renderers/OIT/PerPixelLinkedListLineRenderer.hpp"
#include "Renderers/OIT/MLABRenderer.hpp"
#include "Renderers/OIT/OpacityOptimizationRenderer.hpp"
#include "Renderers/OIT/DepthComplexityRenderer.hpp"
#include "MainApp.hpp"

void openglErrorCallback() {
    std::cerr << "Application callback" << std::endl;
}

MainApp::MainApp()
        : sceneData(
                sceneFramebuffer, sceneTexture, sceneDepthRBO, camera,
                clearColor, screenshotTransparentBackground,
                performanceMeasurer, recording, useCameraFlight)  {
    CAMERA_PATH_TIME_PERFORMANCE_MEASUREMENT = TIME_PERFORMANCE_MEASUREMENT;
    usePerformanceMeasurementMode = false;

    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);

    setNewTilingMode(2, 8);

    sgl::Renderer->setErrorCallback(&openglErrorCallback);
    sgl::Renderer->setDebugVerbosity(sgl::DEBUG_OUTPUT_CRITICAL_ONLY);
    resolutionChanged(sgl::EventPtr());

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
        int desktopWidth = 0;
        int desktopHeight = 0;
        int refreshRate = 60;
        sgl::AppSettings::get()->getDesktopDisplayMode(desktopWidth, desktopHeight, refreshRate);
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
        performanceMeasurer->setInitialFreeMemKilobytes(gpuInitialFreeMemKilobytes);
    }
}

MainApp::~MainApp() {
    if (usePerformanceMeasurementMode) {
        delete performanceMeasurer;
        performanceMeasurer = nullptr;
    }

    delete lineRenderer;
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
                LineDataStress::setUseMajorPS(newState.dataSetDescriptor.enabledFileIndices.at(0));
                LineDataStress::setUseMediumPS(newState.dataSetDescriptor.enabledFileIndices.at(1));
                LineDataStress::setUseMinorPS(newState.dataSetDescriptor.enabledFileIndices.at(2));
            }
            loadLineDataSet(newState.dataSetDescriptor.filenames);
        } else {
            if (newState.dataSetDescriptor.type == DATA_SET_TYPE_STRESS_LINES
                    && newState.dataSetDescriptor.enabledFileIndices.size() == 3) {
                LineDataStress::setUseMajorPS(newState.dataSetDescriptor.enabledFileIndices.at(0));
                LineDataStress::setUseMediumPS(newState.dataSetDescriptor.enabledFileIndices.at(1));
                LineDataStress::setUseMinorPS(newState.dataSetDescriptor.enabledFileIndices.at(2));
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
    } else if (renderingMode == RENDERING_MODE_MLAB) {
        lineRenderer = new MLABRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_OPACITY_OPTIMIZATION) {
        lineRenderer = new OpacityOptimizationRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        lineRenderer = new DepthComplexityRenderer(sceneData, transferFunctionWindow);
    }
}

void MainApp::resolutionChanged(sgl::EventPtr event) {
    SciVisApp::resolutionChanged(event);
    if (lineRenderer != nullptr) {
        lineRenderer->onResolutionChanged();
    }
}

void MainApp::updateColorSpaceMode() {
    SciVisApp::updateColorSpaceMode();
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
}

void MainApp::render() {
    SciVisApp::preRender();
    prepareVisualizationPipeline();

    if (lineRenderer != nullptr) {
        reRender = reRender || lineRenderer->needsReRender();
    }

    if (reRender || continuousRendering) {
        if (renderingMode != RENDERING_MODE_PER_PIXEL_LINKED_LIST && usePerformanceMeasurementMode) {
            performanceMeasurer->startMeasure(recordingTimeLast);
        }

        SciVisApp::prepareReRender();

        if (lineData.get() != nullptr && lineRenderer != nullptr) {
            lineRenderer->render();
        }

        if (renderingMode != RENDERING_MODE_PER_PIXEL_LINKED_LIST && usePerformanceMeasurementMode) {
            performanceMeasurer->endMeasure();
        }

        reRender = false;
    }

    SciVisApp::postRender();
}

void MainApp::renderGui() {
    sgl::ImGuiWrapper::get()->renderStart();

    if (showSettingsWindow) {
        if (ImGui::Begin("Settings", &showSettingsWindow)) {
            SciVisApp::renderGuiFpsCounter();

            // Selection of displayed model
            renderFileSelectionSettingsGui();

            if (ImGui::Combo(
                    "Rendering Mode", (int*)&renderingMode, RENDERING_MODE_NAMES,
                    IM_ARRAYSIZE(RENDERING_MODE_NAMES))) {
                setRenderer();
                reRender = true;
            }

            ImGui::Separator();

            if (ImGui::CollapsingHeader("Scene Settings", NULL, ImGuiTreeNodeFlags_DefaultOpen)) {
                renderSceneSettingsGui();
            }
        }
        ImGui::End();
    }

    if (transferFunctionWindow.renderGui()) {
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

    lineRenderer->renderGuiWindow();

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
        filenames.push_back(filename);
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
    }
}

void MainApp::renderSceneSettingsGui() {
    if (ImGui::ColorEdit3("Clear Color", (float*)&clearColorSelection, 0)) {
        clearColor = sgl::colorFromFloat(
                clearColorSelection.x, clearColorSelection.y, clearColorSelection.z, clearColorSelection.w);
        transferFunctionWindow.setClearColor(clearColor);
        reRender = true;
    }

    SciVisApp::renderSceneSettingsGuiPre();
    ImGui::Checkbox("Show Transfer Function Window", &transferFunctionWindow.getShowTransferFunctionWindow());

    SciVisApp::renderSceneSettingsGuiPost();
}

void MainApp::update(float dt) {
    sgl::SciVisApp::update(dt);

    if (usePerformanceMeasurementMode && !performanceMeasurer->update(recordingTime)) {
        // All modes were tested -> quit.
        quit();
    }

    updateCameraFlight(lineData.get() != nullptr, usesNewState);

    transferFunctionWindow.update(dt);

    ImGuiIO &io = ImGui::GetIO();
    if (io.WantCaptureKeyboard && !recording) {
        // Ignore inputs below
        return;
    }

    moveCameraKeyboard(dt);
    if (sgl::Keyboard->isKeyDown(SDLK_u)) {
        transferFunctionWindow.setShow(showSettingsWindow);
    }

    if (io.WantCaptureMouse) {
        // Ignore inputs below
        return;
    }

    moveCameraMouse(dt);

    lineRenderer->update(dt);
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
    LineRenderer::setLineWidth(STANDARD_LINE_WIDTH);

    DataSetInformation selectedDataSetInformation;
    if (selectedDataSetIndex > 0 && dataSetInformation.size() > 0) {
        selectedDataSetInformation = dataSetInformation.at(selectedDataSetIndex - 1);
        if (selectedDataSetInformation.hasCustomLineWidth) {
            LineRenderer::setLineWidth(selectedDataSetInformation.lineWidth);
        }
    } else {
        selectedDataSetInformation.type = dataSetType;
        selectedDataSetInformation.filenames = fileNames;
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

    if (dataSetType == DATA_SET_TYPE_FLOW_LINES) {
        LineDataFlow* lineDataFlow = new LineDataFlow(transferFunctionWindow);
        lineData = LineDataPtr(lineDataFlow);
    } else if (dataSetType == DATA_SET_TYPE_STRESS_LINES) {
        LineDataStress* lineDataStress = new LineDataStress(transferFunctionWindow);
        lineData = LineDataPtr(lineDataStress);
    } else if (dataSetType == DATA_SET_TYPE_FLOW_LINES_MULTIVAR) {
        LineDataMultiVar* lineDataMultiVar = new LineDataMultiVar(transferFunctionWindow);
        lineData = LineDataPtr(lineDataMultiVar);
    }
    bool dataLoaded = lineData->loadFromFile(fileNames, selectedDataSetInformation, transformationMatrixPtr);
    if (!dataLoaded) {
        lineData = LineDataPtr();
    }

    if (dataLoaded) {
        newMeshLoaded = true;
        modelBoundingBox = lineData->getModelBoundingBox();

        std::string meshDescriptorName = fileNames.front();
        if (fileNames.size() > 1) {
            meshDescriptorName += std::string() + "_" + std::to_string(fileNames.size());
        }
        checkpointWindow.onLoadDataSet(meshDescriptorName);

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

void MainApp::reloadDataSet() {
    loadLineDataSet(getSelectedMeshFilenames());
}

void MainApp::prepareVisualizationPipeline() {
    if (lineData && lineRenderer && (lineRenderer->isDirty() || lineData->isDirty())) {
        lineRenderer->setLineData(lineData, newMeshLoaded);
    }
    newMeshLoaded = false;
}
