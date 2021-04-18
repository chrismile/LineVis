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

#ifdef USE_ZEROMQ
#include <zmq.h>
#endif

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

#include "LineData/LineDataFlow.hpp"
#include "LineData/LineDataStress.hpp"
#include "LineData/LineDataMultiVar.hpp"
#include "LineData/Filters/LineLengthFilter.hpp"
#include "LineData/Filters/MaxLineAttributeFilter.hpp"
#include "Renderers/OIT/TilingMode.hpp"
#include "Renderers/OpaqueLineRenderer.hpp"
#include "Renderers/OIT/PerPixelLinkedListLineRenderer.hpp"
#include "Renderers/OIT/MLABRenderer.hpp"
#include "Renderers/OIT/OpacityOptimizationRenderer.hpp"
#include "Renderers/OIT/DepthComplexityRenderer.hpp"
#include "Renderers/OIT/MBOITRenderer.hpp"
#include "Renderers/OIT/MLABBucketRenderer.hpp"
#include "Renderers/OIT/WBOITRenderer.hpp"
#include "Renderers/OIT/DepthPeelingRenderer.hpp"
#include "MainApp.hpp"

void openglErrorCallback() {
    std::cerr << "Application callback" << std::endl;
}

MainApp::MainApp()
        : sceneData(
                sceneFramebuffer, sceneTexture, sceneDepthRBO, camera,
                clearColor, screenshotTransparentBackground,
                performanceMeasurer, recording, useCameraFlight),
#ifdef USE_PYTHON
          replayWidget(sceneData, transferFunctionWindow, checkpointWindow),
#endif
          lineDataRequester(transferFunctionWindow),
#ifdef USE_ZEROMQ
          zeromqContext(zmq_ctx_new()),
#else
          zeromqContext(nullptr),
#endif
          stressLineTracingRequester(new StressLineTracingRequester(zeromqContext)) {
#ifdef USE_PYTHON
    sgl::ColorLegendWidget::setFontScaleStandard(1.0f);

    replayWidget.setLoadMeshCallback([this](const std::string& datasetName) {
        int i;
        int oldSelectedDataSetIndex = selectedDataSetIndex;
        for (i = 0; i < int(dataSetNames.size()); i++) {
            if (dataSetNames.at(i) == datasetName) {
                selectedDataSetIndex = i;
                break;
            }
        }
        if (i != int(dataSetNames.size())) {
            if (selectedDataSetIndex >= 2 && oldSelectedDataSetIndex != selectedDataSetIndex) {
                loadLineDataSet(getSelectedMeshFilenames(), true);
            }
        } else {
            sgl::Logfile::get()->writeError(
                    "Replay widget: loadMeshCallback: Invalid data set name \"" + datasetName + "\".");
        }
    });
    replayWidget.setLoadRendererCallback([this](const std::string& rendererName) {
        RenderingMode renderingModeNew = renderingMode;
        int i;
        for (i = 0; i < IM_ARRAYSIZE(RENDERING_MODE_NAMES); i++) {
            if (RENDERING_MODE_NAMES[i] == rendererName) {
                renderingModeNew = RenderingMode(i);
                break;
            }
        }
        if (i == IM_ARRAYSIZE(RENDERING_MODE_NAMES)) {
            sgl::Logfile::get()->writeError(
                    std::string() + "ERROR in replay widget load renderer callback: Unknown renderer name \""
                    + rendererName + "\".");
        }
        if (renderingModeNew != renderingMode) {
            renderingMode = renderingModeNew;
            setRenderer();
        }
    });
    replayWidget.setLoadTransferFunctionCallback([this](const std::string& tfName) {
        if (lineData) {
            transferFunctionWindow.loadFunctionFromFile(
                    transferFunctionWindow.getSaveDirectory() + tfName);
            lineData->onTransferFunctionMapRebuilt();
            if (lineRenderer) {
                lineRenderer->onTransferFunctionMapRebuilt();
            }
        }
    });
    replayWidget.setTransferFunctionRangeCallback([this](const glm::vec2& tfRange) {
        if (lineData) {
            transferFunctionWindow.setSelectedRange(tfRange);
            updateTransferFunctionRange = true;
            transferFunctionRange = tfRange;
            lineData->onTransferFunctionMapRebuilt();
            if (lineRenderer) {
                lineRenderer->onTransferFunctionMapRebuilt();
            }
        }
    });
    replayWidget.setLoadMultiVarTransferFunctionsCallback([this](
            const std::vector<std::string>& tfNames) {
        if (lineData) {
            MultiVarTransferFunctionWindow* multiVarTransferFunctionWindow;
            if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
                LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataStress->getMultiVarTransferFunctionWindow();
            } else if (lineData->getType() == DATA_SET_TYPE_FLOW_LINES_MULTIVAR) {
                LineDataMultiVar* lineDataMultiVar = static_cast<LineDataMultiVar*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataMultiVar->getMultiVarTransferFunctionWindow();
            } else {
                sgl::Logfile::get()->writeError(
                        "ERROR in replay widget load multi-var transfer functions callback: Invalid data type .");
                return;
            }
            multiVarTransferFunctionWindow->loadFromTfNameList(tfNames);
            lineData->onTransferFunctionMapRebuilt();
            if (lineRenderer) {
                lineRenderer->onTransferFunctionMapRebuilt();
            }
        }
    });
    replayWidget.setMultiVarTransferFunctionsRangesCallback([this](
            const std::vector<glm::vec2>& tfRanges) {
        if (lineData) {
            MultiVarTransferFunctionWindow* multiVarTransferFunctionWindow;
            if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
                LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataStress->getMultiVarTransferFunctionWindow();
            } else if (lineData->getType() == DATA_SET_TYPE_FLOW_LINES_MULTIVAR) {
                LineDataMultiVar* lineDataMultiVar = static_cast<LineDataMultiVar*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataMultiVar->getMultiVarTransferFunctionWindow();
            } else {
                sgl::Logfile::get()->writeError(
                        "ERROR in replay widget multi-var transfer functions ranges callback: Invalid data type .");
                return;
            }

            for (int varIdx = 0; varIdx < int(tfRanges.size()); varIdx++) {
                multiVarTransferFunctionWindow->setSelectedRange(varIdx, tfRanges.at(varIdx));
            }
            lineData->onTransferFunctionMapRebuilt();
            if (lineRenderer) {
                lineRenderer->onTransferFunctionMapRebuilt();
            }
        }
    });
#endif

    checkpointWindow.setStandardWindowSize(1254, 390);
    checkpointWindow.setStandardWindowPosition(841, 53);

    camera->setNearClipDistance(0.01f);
    camera->setFarClipDistance(100.0f);

    CAMERA_PATH_TIME_PERFORMANCE_MEASUREMENT = TIME_PERFORMANCE_MEASUREMENT;
    usePerformanceMeasurementMode = false;
    cameraPath.setApplicationCallback([this](
            const std::string& modelFilename, glm::vec3& centerOffset, float& startAngle, float& pulseFactor,
            float& standardZoom) {
        if (boost::starts_with(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-Vis2021/psl/Vis2021_femur3D")) {
            pulseFactor = 0.0f;
            standardZoom = 2.9f;
            centerOffset.y = 0.001f;
        } else if (boost::starts_with(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-Vis2021")) {
            pulseFactor = 0.0f;
            standardZoom = 1.9f;
        }
    });

    useLinearRGB = false;
    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);

    setNewTilingMode(2, 8);

    sgl::Renderer->setErrorCallback(&openglErrorCallback);
    sgl::Renderer->setDebugVerbosity(sgl::DEBUG_OUTPUT_CRITICAL_ONLY);
    resolutionChanged(sgl::EventPtr());

    dataFilters.push_back(new LineLengthFilter);
    dataFilters.push_back(new MaxLineAttributeFilter);

    if (usePerformanceMeasurementMode) {
        useCameraFlight = true;
    }
    if (useCameraFlight && recording) {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        window->setWindowSize(recordingResolution.x, recordingResolution.y);
        realTimeCameraFlight = false;
        loadLineDataSet({ sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/rings.obj" });
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
    delete stressLineTracingRequester;
    stressLineTracingRequester = nullptr;
#ifdef USE_ZEROMQ
    zmq_ctx_destroy(zeromqContext);
    zeromqContext = nullptr;
#endif

    if (usePerformanceMeasurementMode) {
        delete performanceMeasurer;
        performanceMeasurer = nullptr;
    }

    for (LineFilter* dataFilter : dataFilters) {
        delete dataFilter;
    }
    dataFilters.clear();

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

    // 1.1. Handle the new tiling mode for SSBO accesses.
    setNewTilingMode(newState.tilingWidth, newState.tilingHeight, newState.useMortonCodeForTiling);

    // 1.2. Load the new transfer function if necessary.
    if (!newState.transferFunctionName.empty() && newState.transferFunctionName != lastState.transferFunctionName) {
        transferFunctionWindow.loadFunctionFromFile(
                sgl::AppSettings::get()->getDataDirectory()
                + "TransferFunctions/" + newState.transferFunctionName);
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
    bool reloadGatherShader = false;
    lineRenderer->setNewState(newState);
    reloadGatherShader |= lineRenderer->setNewSettings(newState.rendererSettings);
    if (reloadGatherShader) {
        lineRenderer->reloadGatherShaderExternal();
    }

    // 3. Pass state change to filters to handle internally necessary state changes.
    for (LineFilter* filter : dataFilters) {
        filter->setNewState(newState);
    }
    for (size_t i = 0; i < newState.filterSettings.size(); i++) {
        dataFilters.at(i)->setNewSettings(newState.filterSettings.at(i));
    }

    // 4. Load the correct mesh file.
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
            if (dataSetInformation.at(selectedDataSetIndex - 2).type == DATA_SET_TYPE_STRESS_LINES
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

    if (oldRenderingMode != renderingMode) {
        // User depth buffer with higher accuracy when rendering lines opaquely.
        if (renderingMode == RENDERING_MODE_ALL_LINES_OPAQUE || oldRenderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
            if (renderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
                sceneDepthRBOType = sgl::RBO_DEPTH32F_STENCIL8;
            } else {
                sceneDepthRBOType = sgl::RBO_DEPTH24_STENCIL8;
            }
            createSceneFramebuffer();
        }
        oldRenderingMode = renderingMode;
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
    } else if (renderingMode == RENDERING_MODE_MBOIT) {
        lineRenderer = new MBOITRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_MLAB_BUCKETS) {
        lineRenderer = new MLABBucketRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_WBOIT) {
        lineRenderer = new WBOITRenderer(sceneData, transferFunctionWindow);
    } else if (renderingMode == RENDERING_MODE_DEPTH_PEELING) {
        lineRenderer = new DepthPeelingRenderer(sceneData, transferFunctionWindow);
    }
    lineRenderer->initialize();

    if (lineData) {
        lineData->setRenderingMode(renderingMode);
        lineData->setLineRenderer(lineRenderer);
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
    lineData->setUseLinearRGB(useLinearRGB);
}

void MainApp::render() {
    SciVisApp::preRender();
    prepareVisualizationPipeline();

    if (lineRenderer != nullptr) {
        reRender = reRender || lineRenderer->needsReRender();
    }
    if (lineData != nullptr) {
        reRender = reRender || lineData->needsReRender();
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
        sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(3090, 56, 735, 1085);
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

    if (selectedDataSetIndex == 1) {
        stressLineTracingRequester->renderGui();
    }

    if ((!lineData || lineData->shallRenderTransferFunctionWindow()) && transferFunctionWindow.renderGui()) {
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
        fovDegree = camera->getFOVy() / sgl::PI * 180.0f;
        reRender = true;
        hasMoved();
    }

#ifdef USE_PYTHON
    ReplayWidget::ReplayWidgetUpdateType replayWidgetUpdateType = replayWidget.renderGui();
    if (replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_LOAD) {
        recordingTime = 0.0f;
        //realTimeReplayUpdates = true;
        realTimeReplayUpdates = false;
        sgl::ColorLegendWidget::setFontScale(1.0f);
    }
    if (replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_START_RECORDING) {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        if (useRecordingResolution && window->getWindowResolution() != recordingResolution) {
            window->setWindowSize(recordingResolution.x, recordingResolution.y);
        }

        if (videoWriter) {
            delete videoWriter;
            videoWriter = nullptr;
        }

        recordingTime = 0.0f;
        realTimeReplayUpdates = false;
        recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();

        recording = true;
        sgl::ColorLegendWidget::setFontScale(1.0f);
        videoWriter = new sgl::VideoWriter(
                saveDirectoryVideos + saveFilenameVideos
                + "_" + sgl::toString(videoNumber++) + ".mp4", FRAME_RATE_VIDEOS);
    }
    if (replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_STOP_RECORDING) {
        recording = false;
        sgl::ColorLegendWidget::resetStandardSize();
        if (videoWriter) {
            delete videoWriter;
            videoWriter = nullptr;
        }
    }
    if (replayWidget.getUseCameraFlight()
            && replayWidgetUpdateType != ReplayWidget::REPLAY_WIDGET_UPDATE_STOP_RECORDING) {
        useCameraFlight = true;
        startedCameraFlightPerUI = true;
        realTimeCameraFlight = false;
        cameraPath.resetTime();
        sgl::ColorLegendWidget::setFontScale(1.0f);
    }
    if (replayWidget.getUseCameraFlight()
            && replayWidgetUpdateType == ReplayWidget::REPLAY_WIDGET_UPDATE_STOP_RECORDING) {
        useCameraFlight = false;
        cameraPath.resetTime();
        sgl::ColorLegendWidget::resetStandardSize();
    }
    if (replayWidgetUpdateType != ReplayWidget::REPLAY_WIDGET_UPDATE_NONE) {
        reRender = true;
    }
#endif

    for (LineFilter* dataFilter : dataFilters) {
        dataFilter->renderGui();
    }
    lineRenderer->renderGuiWindow();

    sgl::ImGuiWrapper::get()->renderEnd();
}

void MainApp::loadAvailableDataSetInformation() {
    dataSetNames.clear();
    dataSetNames.push_back("Local file...");
    dataSetNames.push_back("Stress Line Tracer");
    selectedDataSetIndex = 0;

    const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
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
    } else if (selectedDataSetIndex == 1) {
        dataSetType = DATA_SET_TYPE_STRESS_LINES;
        filenames.push_back(customDataSetFileName);
        return filenames;
    }
    dataSetType = dataSetInformation.at(selectedDataSetIndex - 2).type;
    for (const std::string& filename : dataSetInformation.at(selectedDataSetIndex - 2).filenames) {
        filenames.push_back(filename);
    }
    return filenames;
}

void MainApp::renderFileSelectionSettingsGui() {
    if (ImGui::Combo(
            "Data Set", &selectedDataSetIndex, dataSetNames.data(),
            dataSetNames.size())) {
        if (selectedDataSetIndex >= 2) {
            loadLineDataSet(getSelectedMeshFilenames());
        }
    }

    if (lineDataRequester.getIsProcessingRequest() || stressLineTracingRequester->getIsProcessingRequest()) {
        ImGui::SameLine();
        ImGui::ProgressSpinner(
                "##progress-spinner", -1.0f, -1.0f, 4.0f,
                ImVec4(0.1, 0.5, 1.0, 1.0));
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
        lineData->setClearColor(clearColor);
        reRender = true;
    }

    SciVisApp::renderSceneSettingsGuiPre();
    ImGui::Checkbox("Show Transfer Function Window", &transferFunctionWindow.getShowTransferFunctionWindow());
    if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES && renderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
        if (ImGui::Checkbox("Visualize Seeding Process", &visualizeSeedingProcess)) {
            LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
            lineDataStress->setShallRenderSeedingProcess(visualizeSeedingProcess);
            OpaqueLineRenderer* opaqueLineRenderer = static_cast<OpaqueLineRenderer*>(lineRenderer);
            opaqueLineRenderer->setVisualizeSeedingProcess(true);
            recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
            //useCameraFlight = visualizeSeedingProcess;
            //startedCameraFlightPerUI = true;
            recordingTime = 0.0f;
            //realTimeCameraFlight = false;
            //cameraPath.resetTime();
            customEndTime =
                    visualizeSeedingProcess ? TIME_PER_SEED_POINT * (lineDataStress->getNumSeedPoints() + 1) : 0.0f;
            reRender = true;
        }
    }

    SciVisApp::renderSceneSettingsGuiPost();
}

void MainApp::update(float dt) {
    sgl::SciVisApp::update(dt);

    if (visualizeSeedingProcess) {
        const int seedPointIdx = recordingTime / TIME_PER_SEED_POINT - 1;
        LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
        OpaqueLineRenderer* opaqueLineRenderer = static_cast<OpaqueLineRenderer*>(lineRenderer);
        if (seedPointIdx < lineDataStress->getNumSeedPoints()) {
            lineDataStress->setCurrentSeedIdx(seedPointIdx);
        } else {
            visualizeSeedingProcess = false;
            recording = false;
            sgl::ColorLegendWidget::resetStandardSize();
        }
    }

    if (usePerformanceMeasurementMode && !performanceMeasurer->update(recordingTime)) {
        // All modes were tested -> quit.
        quit();
    }

    updateCameraFlight(lineData.get() != nullptr, usesNewState);

#ifdef USE_PYTHON
    bool stopRecording = false;
    bool stopCameraFlight = false;
    if (replayWidget.update(recordingTime, stopRecording, stopCameraFlight)) {
        if (!useCameraFlight) {
            camera->overwriteViewMatrix(replayWidget.getViewMatrix());
            if (camera->getFOVy() != replayWidget.getCameraFovy()) {
                camera->setFOVy(replayWidget.getCameraFovy());
            }
        }
        SettingsMap currentRendererSettings = replayWidget.getCurrentRendererSettings();
        SettingsMap currentDatasetSettings = replayWidget.getCurrentDatasetSettings();
        bool reloadGatherShader = false;
        if (lineRenderer != nullptr) {
            reloadGatherShader |= lineRenderer->setNewSettings(currentRendererSettings);
        }
        if (lineData != nullptr) {
            reloadGatherShader |= lineData->setNewSettings(currentDatasetSettings);
        }
        if (reloadGatherShader) {
            lineRenderer->reloadGatherShaderExternal();
        }

        if (updateTransferFunctionRange) {
            transferFunctionWindow.setSelectedRange(transferFunctionRange);
            updateTransferFunctionRange = false;
            lineData->onTransferFunctionMapRebuilt();
            if (lineRenderer) {
                lineRenderer->onTransferFunctionMapRebuilt();
            }

        }

        replayWidgetRunning = true;
        reRender = true;

        if (!useCameraFlight) {
            if (realTimeReplayUpdates) {
                uint64_t currentTimeStamp = sgl::Timer->getTicksMicroseconds();
                uint64_t timeElapsedMicroSec = currentTimeStamp - recordingTimeStampStart;
                recordingTime = timeElapsedMicroSec * 1e-6;
            } else {
                recordingTime += FRAME_TIME_CAMERA_PATH;
            }
        }
    } else {
        if (replayWidgetRunning) {
            sgl::ColorLegendWidget::resetStandardSize();
            replayWidgetRunning = false;
        }
    }
    if (stopRecording) {
        recording = false;
        sgl::ColorLegendWidget::resetStandardSize();
        if (videoWriter) {
            delete videoWriter;
            videoWriter = nullptr;
        }
        if (useCameraFlight) {
            useCameraFlight = false;
            cameraPath.resetTime();
        }
    }
    if (stopCameraFlight) {
        sgl::ColorLegendWidget::resetStandardSize();
        useCameraFlight = false;
        cameraPath.resetTime();
    }
#endif

    if (stressLineTracingRequester->getHasNewData(stressLineTracerDataSetInformation)) {
        dataSetType = stressLineTracerDataSetInformation.type;
        loadLineDataSet(stressLineTracerDataSetInformation.filenames);
    }
    checkLoadingRequestFinished();

    transferFunctionWindow.update(dt);
    if (lineData) {
        lineData->update(dt);
    }

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

    if (lineRenderer != nullptr) {
        lineRenderer->update(dt);
    }
}

void MainApp::hasMoved() {
    if (lineRenderer != nullptr) {
        lineRenderer->onHasMoved();
    }
}


// --- Visualization pipeline ---

void MainApp::loadLineDataSet(const std::vector<std::string>& fileNames, bool blockingDataLoading) {
    if (fileNames.size() == 0 || fileNames.front().size() == 0) {
        lineData = LineDataPtr();
        return;
    }
    currentlyLoadedDataSetIndex = selectedDataSetIndex;

    DataSetInformation selectedDataSetInformation;
    if (selectedDataSetIndex >= 2 && dataSetInformation.size() > 0) {
        selectedDataSetInformation = dataSetInformation.at(selectedDataSetIndex - 2);
    } else if (selectedDataSetIndex == 1) {
        selectedDataSetInformation = stressLineTracerDataSetInformation;
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

    LineDataPtr lineData;

    if (dataSetType == DATA_SET_TYPE_FLOW_LINES) {
        LineDataFlow* lineDataFlow = new LineDataFlow(transferFunctionWindow);
        lineData = LineDataPtr(lineDataFlow);
    } else if (dataSetType == DATA_SET_TYPE_STRESS_LINES) {
        LineDataStress* lineDataStress = new LineDataStress(transferFunctionWindow);
        lineData = LineDataPtr(lineDataStress);
    } else if (dataSetType == DATA_SET_TYPE_FLOW_LINES_MULTIVAR) {
        LineDataMultiVar* lineDataMultiVar = new LineDataMultiVar(transferFunctionWindow);
        lineData = LineDataPtr(lineDataMultiVar);
    } else {
        sgl::Logfile::get()->writeError("Error in MainApp::loadLineDataSet: Invalid data set type.");
        return;
    }

    if (blockingDataLoading) {
        bool dataLoaded = lineData->loadFromFile(fileNames, selectedDataSetInformation, transformationMatrixPtr);
        sgl::ColorLegendWidget::resetStandardSize();

        if (dataLoaded) {
            if (selectedDataSetInformation.hasCustomLineWidth) {
                LineRenderer::setLineWidth(selectedDataSetInformation.lineWidth);
            }

            this->lineData = lineData;
            lineData->recomputeHistogram();
            lineData->setClearColor(clearColor);
            lineData->setUseLinearRGB(useLinearRGB);
            lineData->setRenderingMode(renderingMode);
            lineData->setLineRenderer(lineRenderer);
            newMeshLoaded = true;
            modelBoundingBox = lineData->getModelBoundingBox();

            std::string meshDescriptorName = fileNames.front();
            if (fileNames.size() > 1) {
                meshDescriptorName += std::string() + "_" + std::to_string(fileNames.size());
            }
            checkpointWindow.onLoadDataSet(meshDescriptorName);

            for (LineFilter* dataFilter : dataFilters) {
                dataFilter->onDataLoaded(lineData);
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
    } else {
        lineDataRequester.queueRequest(lineData, fileNames, selectedDataSetInformation, transformationMatrixPtr);
    }
}

void MainApp::checkLoadingRequestFinished() {
    DataSetInformation loadedDataSetInformation;
    LineDataPtr lineData = lineDataRequester.getLoadedData(loadedDataSetInformation);

    if (lineData) {
        if (loadedDataSetInformation.hasCustomLineWidth) {
            LineRenderer::setLineWidth(loadedDataSetInformation.lineWidth);
        }
        sgl::ColorLegendWidget::resetStandardSize();

        this->lineData = lineData;
        lineData->recomputeHistogram();
        lineData->setClearColor(clearColor);
        lineData->setUseLinearRGB(useLinearRGB);
        lineData->setRenderingMode(renderingMode);
        lineData->setLineRenderer(lineRenderer);
        newMeshLoaded = true;
        modelBoundingBox = lineData->getModelBoundingBox();

        std::string meshDescriptorName = lineData->getFileNames().front();
        if (lineData->getFileNames().size() > 1) {
            meshDescriptorName += std::string() + "_" + std::to_string(lineData->getFileNames().size());
        }
        checkpointWindow.onLoadDataSet(meshDescriptorName);

        for (LineFilter* dataFilter : dataFilters) {
            dataFilter->onDataLoaded(lineData);
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

void MainApp::reloadDataSet() {
    loadLineDataSet(getSelectedMeshFilenames());
}

void MainApp::prepareVisualizationPipeline() {
    if (lineData && lineRenderer) {
        bool isPreviousNodeDirty = lineData->isDirty();
        filterData(isPreviousNodeDirty);
        if (lineRenderer->isDirty() || isPreviousNodeDirty) {
            lineRenderer->setLineData(lineData, newMeshLoaded);
        }
    }
    newMeshLoaded = false;
}

void MainApp::filterData(bool& isDirty) {
    // Test if we need to re-run the filters.
    for (LineFilter* dataFilter : dataFilters) {
        if (!dataFilter->isEnabled()) {
            continue;
        }
        if (dataFilter->isDirty()) {
            isDirty = true;
            reRender = true;
        }
    }

    if (isDirty) {
        lineData->resetTrajectoryFilter();
        // Pass the output of each filter to the next filter.
        for (LineFilter* dataFilter : dataFilters) {
            if (!dataFilter->isEnabled()) {
                continue;
            }
            dataFilter->filterData(lineData);
        }
    }
}
