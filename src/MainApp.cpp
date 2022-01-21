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
#include <csignal>

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
#include <Utils/Regex/TransformString.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/OpenGL/Texture.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/ImGuiFileDialog/ImGuiFileDialog.h>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include "LineData/LineDataFlow.hpp"
#include "LineData/LineDataStress.hpp"
#include "LineData/LineDataMultiVar.hpp"
#include "LineData/Filters/LineLengthFilter.hpp"
#include "LineData/Filters/MaxLineAttributeFilter.hpp"
#include "LineData/Flow/StreamlineTracingRequester.hpp"
#include "LineData/Stress/StressLineTracingRequester.hpp"
#include "LineData/Scattering/ScatteringLineTracingRequester.hpp"
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
#include "Renderers/VRC/VoxelRayCastingRenderer.hpp"

#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Utils/Instance.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include "Renderers/Vulkan/VulkanRayTracer.hpp"
#include "Renderers/Vulkan/VulkanTestRenderer.hpp"
#include "Renderers/Vulkan/Scattering/LineDensityMapRenderer.hpp"
#include "Renderers/Vulkan/Scattering/SphericalHeatMapRenderer.hpp"
#include "Renderers/Vulkan/Scattering/PathTracer/VolumetricPathTracingRenderer.hpp"
#ifdef SUPPORT_OPTIX
#include "Renderers/Vulkan/Scattering/Denoiser/OptixVptDenoiser.hpp"
#endif
#endif

#ifdef USE_OSPRAY
#include "Renderers/Ospray/OsprayRenderer.hpp"
#endif

#include "Widgets/DataView.hpp"
#include "MainApp.hpp"

void openglErrorCallback() {
    SDL_CaptureMouse(SDL_FALSE);
    std::cerr << "Application callback" << std::endl;
}

#ifdef USE_VULKAN_INTEROP
void vulkanErrorCallback() {
    SDL_CaptureMouse(SDL_FALSE);
    std::cerr << "Application callback" << std::endl;
}
#endif

#ifdef __linux__
void signalHandler(int signum) {
    SDL_CaptureMouse(SDL_FALSE);
    std::cerr << "Interrupt signal (" << signum << ") received." << std::endl;
    exit(signum);
}
#endif

MainApp::MainApp()
        : sceneData(
                &sceneFramebuffer, &sceneTexture, &sceneDepthRBO,
                &viewportWidth, &viewportHeight, &camera,
                &clearColor, &screenshotTransparentBackground,
                &performanceMeasurer, &continuousRendering, &recording,
                &useCameraFlight, &MOVE_SPEED, &MOUSE_ROT_SPEED),
#ifdef USE_PYTHON
          replayWidget(&sceneData, transferFunctionWindow, checkpointWindow),
#endif
          lineDataRequester(transferFunctionWindow),
#ifdef USE_ZEROMQ
          zeromqContext(zmq_ctx_new()),
#else
          zeromqContext(nullptr),
#endif
          streamlineTracingRequester(new StreamlineTracingRequester(transferFunctionWindow)),
          stressLineTracingRequester(new StressLineTracingRequester(zeromqContext)),
          scatteringLineTracingRequester(new ScatteringLineTracingRequester(
                  transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
                  , rendererVk
#endif
          )) {
#ifdef USE_VULKAN_INTEROP
    if (sgl::AppSettings::get()->getVulkanInteropCapabilities() != sgl::VulkanInteropCapabilities::NOT_LOADED) {
        sgl::AppSettings::get()->getVulkanInstance()->setDebugCallback(&vulkanErrorCallback);
    }
#ifdef SUPPORT_OPTIX
    optixInitialized = OptixVptDenoiser::initGlobal();
#endif
#endif

#ifdef USE_PYTHON
    sgl::ColorLegendWidget::setFontScaleStandard(1.0f);

    replayWidget.setLoadLineDataCallback([this](const std::string& datasetName) {
        int i;
        int oldSelectedDataSetIndex = selectedDataSetIndex;
        for (i = 0; i < int(dataSetNames.size()); i++) {
            if (dataSetNames.at(i) == datasetName) {
                selectedDataSetIndex = i;
                break;
            }
        }
        if (i != int(dataSetNames.size())) {
            if (selectedDataSetIndex >= NUM_MANUAL_LOADERS && oldSelectedDataSetIndex != selectedDataSetIndex) {
                loadLineDataSet(getSelectedLineDataSetFilenames(), true);
            }
        } else {
            sgl::Logfile::get()->writeError(
                    "Replay widget: loadMeshCallback: Invalid data set name \"" + datasetName + "\".");
        }
    });
    replayWidget.setLineTracerSettingsCallback([this](const SettingsMap& settings) {
        if (selectedDataSetIndex == 0 || selectedDataSetIndex >= NUM_MANUAL_LOADERS) {
            sgl::Logfile::get()->writeError(
                    "Replay widget: lineTracerSettingsCallback: No line tracer is used.");
            return;
        }

        if (selectedDataSetIndex == 1) {
            streamlineTracingRequester->setLineTracerSettings(settings);
        }
        if (selectedDataSetIndex == 2) {
            stressLineTracingRequester->setLineTracerSettings(settings);
        }
        if (selectedDataSetIndex == 3) {
            scatteringLineTracingRequester->setLineTracerSettings(settings);
        }
    });
    replayWidget.setLoadRendererCallback([this](const std::string& rendererName, int viewIdx) {
        SceneData* sceneDataPtr;
        RenderingMode* renderingModeNew;
        RenderingMode* renderingModeOld;
        LineRenderer** lineRendererPtr;
        if (useDockSpaceMode) {
            if (viewIdx >= int(dataViews.size())) {
                addNewDataView();
            }
            sceneDataPtr = &dataViews.at(viewIdx)->sceneData;
            renderingModeNew = &dataViews.at(viewIdx)->renderingMode;
            renderingModeOld = &dataViews.at(viewIdx)->oldRenderingMode;
            lineRendererPtr = &dataViews.at(viewIdx)->lineRenderer;
        } else {
            sceneDataPtr = &sceneData;
            renderingModeNew = &renderingMode;
            renderingModeOld = &oldRenderingMode;
            lineRendererPtr = &lineRenderer;
        }
        int i;
        for (i = 0; i < IM_ARRAYSIZE(RENDERING_MODE_NAMES); i++) {
            if (RENDERING_MODE_NAMES[i] == rendererName) {
                *renderingModeNew = RenderingMode(i);
                break;
            }
        }
        if (i == IM_ARRAYSIZE(RENDERING_MODE_NAMES)) {
            sgl::Logfile::get()->writeError(
                    std::string() + "ERROR in replay widget load renderer callback: Unknown renderer name \""
                    + rendererName + "\".");
        }
        if (*renderingModeNew != *renderingModeOld) {
            setRenderer(*sceneDataPtr, *renderingModeOld, *renderingModeNew, *lineRendererPtr, viewIdx);
        }
        if (useDockSpaceMode) {
            dataViews.at(viewIdx)->updateCameraMode();
        }
    });
    replayWidget.setLoadTransferFunctionCallback([this](const std::string& tfName) {
        if (lineData) {
            transferFunctionWindow.loadFunctionFromFile(
                    transferFunctionWindow.getSaveDirectory() + tfName);
            lineData->onTransferFunctionMapRebuilt();
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
        }
    });
    replayWidget.setTransferFunctionRangeCallback([this](const glm::vec2& tfRange) {
        if (lineData) {
            transferFunctionWindow.setSelectedRange(tfRange);
            updateTransferFunctionRange = true;
            transferFunctionRange = tfRange;
            lineData->onTransferFunctionMapRebuilt();
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
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
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
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
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
        }
    });
    replayWidget.setSaveScreenshotCallback([this](const std::string& screenshotName) {
        if (!screenshotName.empty()) {
            saveFilenameScreenshots = screenshotName;
        }
        screenshot = true;
    });
#endif

    checkpointWindow.setStandardWindowSize(1254, 390);
    checkpointWindow.setStandardWindowPosition(841, 53);

    propertyEditor.setInitWidthValues(sgl::ImGuiWrapper::get()->getScaleDependentSize(280.0f));

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
            this->camera->setFOVy(36.0f / 180.0f * sgl::PI);
        } else if (boost::starts_with(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-Vis2021")) {
            pulseFactor = 0.0f;
            standardZoom = 2.0f;
        } else if (boost::starts_with(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-TVCG01/psl/arched_bridge3D_PSLs")) {
            pulseFactor = 0.0f;
            standardZoom = 1.9f;
        } else if (boost::starts_with(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-TVCG01/psl/arched_bridge3D_PSLs")) {
            pulseFactor = 0.0f;
            standardZoom = 1.9f;
        } else if (boost::starts_with(
                modelFilename, sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/clouds")) {
            pulseFactor = 0.0f;
            standardZoom = 2.9f;
            centerOffset = glm::vec3(0.0f, -0.1f, 0.0f);
        }
    });

    useDockSpaceMode = true;
    sgl::AppSettings::get()->getSettings().getValueOpt("useDockSpaceMode", useDockSpaceMode);
    sgl::AppSettings::get()->getSettings().getValueOpt("useFixedSizeViewport", useFixedSizeViewport);
    showPropertyEditor = useDockSpaceMode;
    sgl::ImGuiWrapper::get()->setUseDockSpaceMode(useDockSpaceMode);

    showFpsOverlay = true;
    sgl::AppSettings::get()->getSettings().getValueOpt("showFpsOverlay", showFpsOverlay);
    sgl::AppSettings::get()->getSettings().getValueOpt("showCoordinateAxesOverlay", showCoordinateAxesOverlay);

    useLinearRGB = false;
    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
    coordinateAxesOverlayWidget.setClearColor(clearColor);

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

    if (!useDockSpaceMode) {
        setRenderer(sceneData, oldRenderingMode, renderingMode, lineRenderer, 0);
    }

    fileDialogInstance = IGFD_Create();
    customDataSetFileName = sgl::FileUtils::get()->getUserDirectory();
    loadAvailableDataSetInformation();

    if (!sgl::AppSettings::get()->getSettings().hasKey("cameraNavigationMode")) {
        cameraNavigationMode = sgl::CameraNavigationMode::TURNTABLE;
        updateCameraNavigationMode();
    }

    addNewDataView();
    if (dataViews.size() == 1) {
        initializeFirstDataView();
    }

    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
    usesNewState = true;
    if (usePerformanceMeasurementMode) {
        sgl::FileUtils::get()->ensureDirectoryExists("images");
        performanceMeasurer = new AutomaticPerformanceMeasurer(
                getTestModesPaper(), "performance.csv", "depth_complexity.csv",
                [this](const InternalState &newState) { this->setNewState(newState); });
        performanceMeasurer->setInitialFreeMemKilobytes(gpuInitialFreeMemKilobytes);
    }

#ifdef __linux__
    signal(SIGSEGV, signalHandler);
#endif
}

MainApp::~MainApp() {
    if (usePerformanceMeasurementMode) {
        delete performanceMeasurer;
        performanceMeasurer = nullptr;
    }

    for (LineFilter* dataFilter : dataFilters) {
        delete dataFilter;
    }
    dataFilters.clear();

    if (lineRenderer) {
        delete lineRenderer;
        lineRenderer = nullptr;
    }
    lineData = {};
    dataViews.clear();

    delete streamlineTracingRequester;
    streamlineTracingRequester = nullptr;
    delete stressLineTracingRequester;
    stressLineTracingRequester = nullptr;
    delete scatteringLineTracingRequester;
    scatteringLineTracingRequester = nullptr;
#ifdef USE_ZEROMQ
    zmq_ctx_destroy(zeromqContext);
    zeromqContext = nullptr;
#endif

    IGFD_Destroy(fileDialogInstance);

#if defined(USE_VULKAN_INTEROP) && defined(SUPPORT_OPTIX)
    if (optixInitialized) {
        OptixVptDenoiser::freeGlobal();
    }
#endif

    sgl::AppSettings::get()->getSettings().addKeyValue("useDockSpaceMode", useDockSpaceMode);
    sgl::AppSettings::get()->getSettings().addKeyValue("useFixedSizeViewport", useFixedSizeViewport);
    sgl::AppSettings::get()->getSettings().addKeyValue("showFpsOverlay", showFpsOverlay);
    sgl::AppSettings::get()->getSettings().addKeyValue("showCoordinateAxesOverlay", showCoordinateAxesOverlay);
}

void MainApp::setNewState(const InternalState &newState) {
    ZoneScoped;

    if (performanceMeasurer) {
        performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(0);
    }

    // 1. Change the window resolution?
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    int currentWindowWidth = window->getWidth();
    int currentWindowHeight = window->getHeight();
    glm::ivec2 newResolution = newState.windowResolution;
    if (newResolution.x > 0 && newResolution.y > 0 && currentWindowWidth != newResolution.x
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
        dataViews.clear();
        renderingMode = newState.renderingMode;
        if (useDockSpaceMode) {
            if (dataViews.empty()) {
                addNewDataView();
            }
            setRenderer(
                    dataViews[0]->sceneData, dataViews[0]->oldRenderingMode, dataViews[0]->renderingMode,
                    dataViews[0]->lineRenderer, 0);
            dataViews[0]->updateCameraMode();
        } else {
            setRenderer(sceneData, oldRenderingMode, renderingMode, lineRenderer, 0);
        }
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
        for (size_t i = 0; i < dataSetInformationList.size(); i++) {
            if (boost::algorithm::to_lower_copy(dataSetInformationList.at(i)->name) == nameLower) {
                selectedDataSetIndex = int(i) + NUM_MANUAL_LOADERS;
                break;
            }
        }
        if (selectedDataSetIndex == 0) {
            if (dataSetInformationList.at(selectedDataSetIndex - NUM_MANUAL_LOADERS)->type
                    == DATA_SET_TYPE_STRESS_LINES && newState.dataSetDescriptor.enabledFileIndices.size() == 3) {
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
            loadLineDataSet(getSelectedLineDataSetFilenames());
        }
    }

    recordingTime = 0.0f;
    recordingTimeLast = 0.0f;
    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
    lastState = newState;
    firstState = false;
    usesNewState = true;
}

void MainApp::scheduleRecreateSceneFramebuffer() {
    scheduledRecreateSceneFramebuffer = true;
}

void MainApp::setRenderer(
        SceneData& sceneDataRef, RenderingMode& oldRenderingMode, RenderingMode& newRenderingMode,
        LineRenderer*& newLineRenderer, int dataViewIndex) {
    if (newLineRenderer) {
        delete newLineRenderer;
        newLineRenderer = nullptr;
    }

    if (oldRenderingMode != newRenderingMode) {
        // User depth buffer with higher accuracy when rendering lines opaquely.
        if (newRenderingMode == RENDERING_MODE_ALL_LINES_OPAQUE
                || oldRenderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
            sgl::RenderbufferType rboType;
            if (newRenderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
                rboType = sgl::RBO_DEPTH32F_STENCIL8;
            } else {
                rboType = sgl::RBO_DEPTH24_STENCIL8;
            }
            if (useDockSpaceMode) {
                dataViews.at(dataViewIndex)->sceneDepthRBOType = rboType;
                sceneDepthRBOType = rboType;
            } else {
                sceneDepthRBOType = rboType;
            }
            scheduleRecreateSceneFramebuffer();
        }
        oldRenderingMode = newRenderingMode;
    }

    if (newRenderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
        newLineRenderer = new OpaqueLineRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_PER_PIXEL_LINKED_LIST) {
        newLineRenderer = new PerPixelLinkedListLineRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_MLAB) {
        newLineRenderer = new MLABRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_OPACITY_OPTIMIZATION) {
        newLineRenderer = new OpacityOptimizationRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        newLineRenderer = new DepthComplexityRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_MBOIT) {
        newLineRenderer = new MBOITRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_MLAB_BUCKETS) {
        newLineRenderer = new MLABBucketRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_WBOIT) {
        newLineRenderer = new WBOITRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_DEPTH_PEELING) {
        newLineRenderer = new DepthPeelingRenderer(&sceneDataRef, transferFunctionWindow);
    }
#ifdef USE_VULKAN_INTEROP
    else if (newRenderingMode == RENDERING_MODE_VULKAN_RAY_TRACER
             && sgl::AppSettings::get()->getVulkanInteropCapabilities()
                == sgl::VulkanInteropCapabilities::EXTERNAL_MEMORY) {
        if (sgl::AppSettings::get()->getPrimaryDevice()->getRayTracingPipelineSupported()) {
            auto* renderer = new sgl::vk::Renderer(sgl::AppSettings::get()->getPrimaryDevice());
            newLineRenderer = new VulkanRayTracer(&sceneDataRef, transferFunctionWindow, renderer);
        } else {
            newRenderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;
            newLineRenderer = new OpaqueLineRenderer(&sceneDataRef, transferFunctionWindow);
            sgl::Logfile::get()->writeError(
                    "Error in MainApp::setRenderer: Vulkan ray pipelines are not supported on this hardware.");
        }
    }
#endif
    else if (newRenderingMode == RENDERING_MODE_VOXEL_RAY_CASTING) {
        newLineRenderer = new VoxelRayCastingRenderer(&sceneDataRef, transferFunctionWindow);
    }
#ifdef USE_VULKAN_INTEROP
    else if (newRenderingMode == RENDERING_MODE_VULKAN_TEST
             && sgl::AppSettings::get()->getVulkanInteropCapabilities()
                == sgl::VulkanInteropCapabilities::EXTERNAL_MEMORY) {
        auto* renderer = new sgl::vk::Renderer(sgl::AppSettings::get()->getPrimaryDevice());
        newLineRenderer = new VulkanTestRenderer(&sceneDataRef, transferFunctionWindow, renderer);
    }
#endif
#ifdef USE_OSPRAY
    else if (newRenderingMode == RENDERING_MODE_OSPRAY_RAY_TRACER) {
        newLineRenderer = new OsprayRenderer(&sceneDataRef, transferFunctionWindow);
    }
#endif
#ifdef USE_VULKAN_INTEROP
    else if (newRenderingMode == RENDERING_MODE_LINE_DENSITY_MAP_RENDERER
             && sgl::AppSettings::get()->getVulkanInteropCapabilities()
                == sgl::VulkanInteropCapabilities::EXTERNAL_MEMORY) {
        auto* renderer = new sgl::vk::Renderer(sgl::AppSettings::get()->getPrimaryDevice());
        newLineRenderer = new LineDensityMapRenderer(&sceneDataRef, transferFunctionWindow, renderer);
    } else if (newRenderingMode == RENDERING_MODE_VOLUMETRIC_PATH_TRACER
               && sgl::AppSettings::get()->getVulkanInteropCapabilities()
                  == sgl::VulkanInteropCapabilities::EXTERNAL_MEMORY) {
        auto* renderer = new sgl::vk::Renderer(sgl::AppSettings::get()->getPrimaryDevice());
        newLineRenderer = new VolumetricPathTracingRenderer(&sceneDataRef, transferFunctionWindow, renderer);
    } else if (newRenderingMode == RENDERING_MODE_SPHERICAL_HEAT_MAP_RENDERER) {
        newLineRenderer = new SphericalHeatMapRenderer(&sceneDataRef, transferFunctionWindow);
    }
#endif
    else {
        newRenderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;
        newLineRenderer = new OpaqueLineRenderer(&sceneDataRef, transferFunctionWindow);
        sgl::Logfile::get()->writeError(
                "Error in MainApp::setRenderer: A renderer unsupported in this build configuration or "
                "incompatible with this system was selected.");
    }
    newLineRenderer->initialize();

    if (lineData) {
        if (useDockSpaceMode) {
            std::vector<LineRenderer*> lineRenderers;
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer) {
                    lineRenderers.push_back(dataView->lineRenderer);
                }
            }
            lineData->setLineRenderers(lineRenderers);
        } else {
            lineData->setLineRenderers({ lineRenderer });
        }
        lineData->setRenderingModes({ newRenderingMode });
    }
}

void MainApp::resolutionChanged(sgl::EventPtr event) {
    ZoneScoped;

    SciVisApp::resolutionChanged(event);
    if (!useDockSpaceMode) {
        if (lineRenderer != nullptr) {
            lineRenderer->onResolutionChanged();
        }
    }
}

void MainApp::updateColorSpaceMode() {
    SciVisApp::updateColorSpaceMode();
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
    lineData->setUseLinearRGB(useLinearRGB);
    if (useDockSpaceMode) {
        for (DataViewPtr& dataView : dataViews) {
            dataView->useLinearRGB = useLinearRGB;
        }
    }
}

void MainApp::render() {
    ZoneScoped;

    if (scheduledRecreateSceneFramebuffer) {
        createSceneFramebuffer();
        scheduledRecreateSceneFramebuffer = false;
    }

    if (visualizeSeedingProcess) {
        if (useDockSpaceMode) {
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_ALL_LINES_OPAQUE
                        || dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER) {
                    dataView->lineRenderer->notifyReRenderTriggeredExternally();
                    dataView->reRender = true;
                }
            }
        }
    }

    SciVisApp::preRender();
    prepareVisualizationPipeline();

    componentOtherThanRendererNeedsReRender = reRender;
    if (lineData != nullptr) {
        bool lineDataNeedsReRender = lineData->needsReRender();
        reRender = reRender || lineDataNeedsReRender;
        componentOtherThanRendererNeedsReRender = componentOtherThanRendererNeedsReRender || lineDataNeedsReRender;
    }

    if (!useDockSpaceMode) {
        if (lineRenderer != nullptr) {
            reRender = reRender || lineRenderer->needsReRender();
            componentOtherThanRendererNeedsReRender |= lineRenderer->needsInternalReRender();
        }
        if (lineRenderer && componentOtherThanRendererNeedsReRender) {
            // If the re-rendering was triggered from an outside source, frame accumulation cannot be used!
            lineRenderer->notifyReRenderTriggeredExternally();
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
    }

    SciVisApp::postRender();
}

void MainApp::renderGui() {
    ZoneScoped;

    focusedWindowIndex = -1;
    mouseHoverWindowIndex = -1;

    if (sgl::Keyboard->keyPressed(SDLK_o) && (sgl::Keyboard->getModifier() & (KMOD_LCTRL | KMOD_RCTRL)) != 0) {
        openFileDialog();
    }

    if (IGFD_DisplayDialog(
            fileDialogInstance,
            "ChooseDataSetFile", ImGuiWindowFlags_NoCollapse,
            sgl::ImGuiWrapper::get()->getScaleDependentSize(1000, 580),
            ImVec2(FLT_MAX, FLT_MAX))) {
        if (IGFD_IsOk(fileDialogInstance)) {
            std::string filePathName = IGFD_GetFilePathName(fileDialogInstance);
            std::string filePath = IGFD_GetCurrentPath(fileDialogInstance);
            std::string filter = IGFD_GetCurrentFilter(fileDialogInstance);
            std::string userDatas;
            if (IGFD_GetUserDatas(fileDialogInstance)) {
                userDatas = std::string((const char*)IGFD_GetUserDatas(fileDialogInstance));
            }
            auto selection = IGFD_GetSelection(fileDialogInstance);

            // Is this line data set or a volume data file for the scattering line tracer?
            const char* currentPath = IGFD_GetCurrentPath(fileDialogInstance);
            std::string filename = currentPath;
            if (!filename.empty() && filename.back() != '/' && filename.back() != '\\') {
                filename += "/";
            }
            filename += selection.table[0].fileName;
            IGFD_Selection_DestroyContent(&selection);
            if (currentPath) {
                free((void*)currentPath);
                currentPath = nullptr;
            }

            std::string filenameLower = boost::to_lower_copy(filename);
            if (boost::ends_with(filenameLower, ".vtk") || boost::ends_with(filenameLower, ".bin")) {
                selectedDataSetIndex = 1;
                streamlineTracingRequester->setDatasetFilename(filename);
            } else if (boost::ends_with(filenameLower, ".stress")
                    || boost::ends_with(filenameLower, ".carti")) {
                selectedDataSetIndex = 2;
                stressLineTracingRequester->setDatasetFilename(filename);
            } else if (boost::ends_with(filenameLower, ".xyz")) {
                selectedDataSetIndex = 3;
                scatteringLineTracingRequester->setDatasetFilename(filename);
            } else {
                selectedDataSetIndex = 0;
                if (boost::ends_with(filenameLower, ".obj")
                        || boost::ends_with(filenameLower, ".nc")
                        || boost::ends_with(filenameLower, ".binlines")) {
                    dataSetType = DATA_SET_TYPE_FLOW_LINES;
                } else if (boost::ends_with(filenameLower, ".dat")) {
                    dataSetType = DATA_SET_TYPE_STRESS_LINES;
                } else {
                    sgl::Logfile::get()->writeError("The selected file name has an unknown extension.");
                }
                customDataSetFileName = filename;
                loadLineDataSet(getSelectedLineDataSetFilenames());
            }
        }
        IGFD_CloseDialog(fileDialogInstance);
    }

    if (useDockSpaceMode) {
        ImGuiID dockSpaceId = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
        ImGuiDockNode* centralNode = ImGui::DockBuilderGetNode(dockSpaceId);
        static bool isProgramStartup = true;
        if (isProgramStartup && centralNode->IsEmpty()) {
            ImGuiID dockLeftId, dockMainId;
            ImGui::DockBuilderSplitNode(
                    dockSpaceId, ImGuiDir_Left, 0.21f, &dockLeftId, &dockMainId);
            ImGui::DockBuilderDockWindow("Opaque Line Renderer (1)###data_view_0", dockMainId);

            ImGuiID dockLeftUpId, dockLeftDownId;
            ImGui::DockBuilderSplitNode(
                    dockLeftId, ImGuiDir_Up, 0.45f, &dockLeftUpId, &dockLeftDownId);
            ImGui::DockBuilderDockWindow("Property Editor", dockLeftUpId);

            ImGuiID dockLeftDownUpId, dockLeftDownDownId;
            ImGui::DockBuilderSplitNode(
                    dockLeftDownId, ImGuiDir_Up, 0.28f,
                    &dockLeftDownUpId, &dockLeftDownDownId);
            ImGui::DockBuilderDockWindow("Transfer Function", dockLeftDownDownId);
            ImGui::DockBuilderDockWindow("Multi-Var Transfer Function", dockLeftDownDownId);
            ImGui::DockBuilderDockWindow("Camera Checkpoints", dockLeftDownUpId);
            ImGui::DockBuilderDockWindow("Replay Widget", dockLeftDownUpId);

            ImGui::DockBuilderFinish(dockLeftId);
            ImGui::DockBuilderFinish(dockSpaceId);
        }
        isProgramStartup = false;

        renderGuiMenuBar();

        for (int i = 0; i < int(dataViews.size()); i++) {
            DataViewPtr& dataView = dataViews.at(i);
            if (dataView->showWindow) {
                std::string windowName = dataView->getWindowName(i);
                bool isViewOpen = true;
                sgl::ImGuiWrapper::get()->setNextWindowStandardSize(800, 600);
                ImGui::SetNextTabbarMenu([this] {
                    if (ImGui::BeginPopup("#NewTab")) {
                        for (int i = 0; i < IM_ARRAYSIZE(RENDERING_MODE_NAMES); i++) {
                            if (ImGui::Selectable(RENDERING_MODE_NAMES[i])) {
                                addNewDataView();
                                DataViewPtr dataView = dataViews.back();
                                dataView->renderingMode = RenderingMode(i);
                                dataView->resize(1, 1); // Use arbitrary size for initialization.
                                setRenderer(
                                        dataView->sceneData, dataView->oldRenderingMode,
                                        dataView->renderingMode, dataView->lineRenderer,
                                        int(dataViews.size()) - 1);
                                dataView->updateCameraMode();
                                prepareVisualizationPipeline();
                            }
                        }
                        ImGui::EndPopup();
                    }

                    return "#NewTab";
                });
                if (ImGui::Begin(windowName.c_str(), &isViewOpen)) {
                    if (ImGui::IsWindowFocused()) {
                        focusedWindowIndex = i;
                    }
                    sgl::ImGuiWrapper::get()->setWindowViewport(i, ImGui::GetWindowViewport());
                    sgl::ImGuiWrapper::get()->setWindowViewport(i, ImGui::GetWindowViewport());
                    sgl::ImGuiWrapper::get()->setWindowPosAndSize(i, ImGui::GetWindowPos(), ImGui::GetWindowSize());

                    ImVec2 sizeContent = ImGui::GetContentRegionAvail();
                    if (useFixedSizeViewport) {
                        sizeContent = ImVec2(float(fixedViewportSize.x), float(fixedViewportSize.y));
                    }
                    if (int(sizeContent.x) != int(dataView->viewportWidth)
                            || int(sizeContent.y) != int(dataView->viewportHeight)) {
                        dataView->resize(int(sizeContent.x), int(sizeContent.y));
                        if (dataView->lineRenderer && dataView->viewportWidth > 0 && dataView->viewportHeight > 0) {
                            dataView->lineRenderer->onResolutionChanged();
                        }
                        dataView->reRender = true;
                    }

                    bool reRenderLocal = reRender || dataView->reRender;
                    dataView->reRender = false;
                    bool componentOtherThanRendererNeedsReRenderLocal = componentOtherThanRendererNeedsReRender;
                    if (dataView->lineRenderer != nullptr) {
                        reRenderLocal |= dataView->lineRenderer->needsReRender();
                        componentOtherThanRendererNeedsReRenderLocal |= dataView->lineRenderer->needsInternalReRender();
                    }
                    if (dataView->lineRenderer && componentOtherThanRendererNeedsReRenderLocal) {
                        // If the re-rendering was triggered from an outside source, frame accumulation cannot be used!
                        dataView->lineRenderer->notifyReRenderTriggeredExternally();
                    }

                    if (dataView->viewportWidth > 0 && dataView->viewportHeight > 0
                            && (reRenderLocal || continuousRendering)) {
                        dataView->beginRender();

                        if (renderingMode != RENDERING_MODE_PER_PIXEL_LINKED_LIST && usePerformanceMeasurementMode) {
                            performanceMeasurer->startMeasure(recordingTimeLast);
                        }

                        if (lineData.get() != nullptr && dataView->lineRenderer != nullptr) {
                            dataView->lineRenderer->render();
                        }

                        if (renderingMode != RENDERING_MODE_PER_PIXEL_LINKED_LIST && usePerformanceMeasurementMode) {
                            performanceMeasurer->endMeasure();
                        }

                        reRenderLocal = false;

                        dataView->endRender();
                    }

                    if (dataView->viewportWidth > 0 && dataView->viewportHeight > 0) {
                        if (!uiOnScreenshot && screenshot) {
                            printNow = true;
                            sgl::Renderer->bindFBO(dataView->getSceneFramebuffer());
                            customScreenshotWidth = int(dataView->viewportWidth);
                            customScreenshotHeight = int(dataView->viewportHeight);
                            std::string screenshotFilename =
                                    saveDirectoryScreenshots + saveFilenameScreenshots
                                    + "_" + sgl::toString(screenshotNumber);
                            if (dataViews.size() > 1) {
                                screenshotFilename += "_" + std::to_string(i);
                            }
                            screenshotFilename += ".png";
                            saveScreenshot(screenshotFilename);
                            customScreenshotWidth = -1;
                            customScreenshotHeight = -1;
                            sgl::Renderer->unbindFBO();
                            printNow = false;
                            screenshot = true;
                        }
                        if (!uiOnScreenshot && recording && !isFirstRecordingFrame && i == 0) {
                            videoWriter->pushFramebuffer(dataView->getSceneFramebuffer());
                        }

                        if (isViewOpen) {
                            ImGui::Image(
                                    (void*)(intptr_t)static_cast<sgl::TextureGL*>(
                                            dataView->sceneTexture.get())->getTexture(),
                                    sizeContent, ImVec2(0, 1), ImVec2(1, 0));
                            if (ImGui::IsItemHovered()) {
                                mouseHoverWindowIndex = i;
                            }
                        }

                        if (i == 0 && showFpsOverlay) {
                            renderGuiFpsOverlay();
                        }
                        if (i == 0 && showCoordinateAxesOverlay) {
                            renderGuiCoordinateAxesOverlay(dataView->camera);
                        }

                        if (i == 0 && dataView->lineRenderer) {
                            dataView->lineRenderer->renderGuiOverlay();
                        }
                    }
                }
                ImGui::End();

                if (dataView->lineRenderer) {
                    dataView->lineRenderer->renderGuiWindowSecondary();
                }

                if (!isViewOpen) {
                    dataViews.erase(dataViews.begin() + i);
                    i--;
                }
            }
        }
        if (!uiOnScreenshot && screenshot) {
            screenshot = false;
            screenshotNumber++;
        }
        if (!dataViews.empty()) {
            sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
            int width = window->getWidth();
            int height = window->getHeight();
            glViewport(0, 0, width, height);
        }
        reRender = false;
    } else {
        if (lineRenderer) {
            lineRenderer->renderGuiOverlay();
        }
    }

    if (selectedDataSetIndex == 1) {
        streamlineTracingRequester->renderGui();
    }
    if (selectedDataSetIndex == 2) {
        stressLineTracingRequester->renderGui();
    }
    if (selectedDataSetIndex == 3) {
        scatteringLineTracingRequester->renderGui();
    }

    if ((!lineData || lineData->shallRenderTransferFunctionWindow()) && transferFunctionWindow.renderGui()) {
        reRender = true;
        if (transferFunctionWindow.getTransferFunctionMapRebuilt()) {
            if (lineData) {
                lineData->onTransferFunctionMapRebuilt();
            }
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
        }
    }

    if (checkpointWindow.renderGui()) {
        fovDegree = camera->getFOVy() / sgl::PI * 180.0f;
        reRender = true;
        hasMoved();
        onCameraReset();
    }

    if (showPropertyEditor) {
        renderGuiPropertyEditorWindow();
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
        if (useRecordingResolution && window->getWindowResolution() != recordingResolution
                && !window->isFullscreen()) {
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
        isFirstRecordingFrame = true;
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

    if (lineData) {
        bool shallReloadGatherShader = lineData->renderGuiWindowSecondary();
        if (shallReloadGatherShader) {
            if (useDockSpaceMode) {
                for (DataViewPtr& dataView : dataViews) {
                    dataView->lineRenderer->reloadGatherShaderExternal();
                }
            } else {
                lineRenderer->reloadGatherShaderExternal();
            }
        }
    }
}

void MainApp::loadAvailableDataSetInformation() {
    dataSetNames.clear();
    dataSetNames.emplace_back("Local file...");
    dataSetNames.emplace_back("Streamline Tracer");
    dataSetNames.emplace_back("Stress Line Tracer");
    dataSetNames.emplace_back("Scattering Line Tracer");
    selectedDataSetIndex = 0;

    const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
    if (sgl::FileUtils::get()->exists(lineDataSetsDirectory + "datasets.json")) {
        dataSetInformationRoot = loadDataSetList(lineDataSetsDirectory + "datasets.json");

        std::stack<std::pair<DataSetInformationPtr, size_t>> dataSetInformationStack;
        dataSetInformationStack.push(std::make_pair(dataSetInformationRoot, 0));
        while (!dataSetInformationStack.empty()) {
            std::pair<DataSetInformationPtr, size_t> dataSetIdxPair = dataSetInformationStack.top();
            DataSetInformationPtr dataSetInformationParent = dataSetIdxPair.first;
            size_t idx = dataSetIdxPair.second;
            dataSetInformationStack.pop();
            while (idx < dataSetInformationParent->children.size()) {
                DataSetInformationPtr dataSetInformationChild =
                        dataSetInformationParent->children.at(idx);
                idx++;
                if (dataSetInformationChild->type == DATA_SET_TYPE_NODE) {
                    dataSetInformationStack.push(std::make_pair(dataSetInformationRoot, idx));
                    dataSetInformationStack.push(std::make_pair(dataSetInformationChild, 0));
                    break;
                } else {
                    dataSetInformationChild->sequentialIndex = int(dataSetNames.size());
                    dataSetInformationList.push_back(dataSetInformationChild);
                    dataSetNames.push_back(dataSetInformationChild->name);
                }
            }
        }
    }
}

std::vector<std::string> MainApp::getSelectedLineDataSetFilenames() {
    std::vector<std::string> filenames;
    if (selectedDataSetIndex == 1) {
        dataSetType = DATA_SET_TYPE_FLOW_LINES;
        filenames.push_back(customDataSetFileName);
        return filenames;
    } else if (selectedDataSetIndex == 2) {
        dataSetType = DATA_SET_TYPE_STRESS_LINES;
        filenames.push_back(customDataSetFileName);
        return filenames;
    } else if (selectedDataSetIndex == 3) {
        dataSetType = DATA_SET_TYPE_SCATTERING_LINES;
        filenames.push_back(customDataSetFileName);
        return filenames;
    }
    if (selectedDataSetIndex == 0) {
        filenames.push_back(customDataSetFileName);
    } else {
        dataSetType = dataSetInformationList.at(selectedDataSetIndex - NUM_MANUAL_LOADERS)->type;
        for (const std::string& filename : dataSetInformationList.at(
                selectedDataSetIndex - NUM_MANUAL_LOADERS)->filenames) {
            filenames.push_back(filename);
        }
    }
    return filenames;
}

void MainApp::renderGuiGeneralSettingsPropertyEditor() {
    if (propertyEditor.addColorEdit3("Clear Color", (float*)&clearColorSelection, 0)) {
        clearColor = sgl::colorFromFloat(
                clearColorSelection.x, clearColorSelection.y, clearColorSelection.z, clearColorSelection.w);
        transferFunctionWindow.setClearColor(clearColor);
        coordinateAxesOverlayWidget.setClearColor(clearColor);
        if (lineData) {
            lineData->setClearColor(clearColor);
        }
        for (DataViewPtr& dataView : dataViews) {
            dataView->setClearColor(clearColor);
        }
        reRender = true;
    }

    bool newDockSpaceMode = useDockSpaceMode;
    if (propertyEditor.addCheckbox("Use Docking Mode", &newDockSpaceMode)) {
        scheduledDockSpaceModeChange = true;
    }

    if (propertyEditor.addCheckbox("Fixed Size Viewport", &useFixedSizeViewport)) {
        reRender = true;
    }
    if (useFixedSizeViewport) {
        if (propertyEditor.addSliderInt2Edit("Viewport Size", &fixedViewportSizeEdit.x, 1, 8192)
                == ImGui::EditMode::INPUT_FINISHED) {
            fixedViewportSize = fixedViewportSizeEdit;
            reRender = true;
        }
    }

    if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES && renderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
        if (useDockSpaceMode) {
            bool rendererSupportsVisualizingSeedingProcess = false;
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_ALL_LINES_OPAQUE
                        || dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER) {
                    rendererSupportsVisualizingSeedingProcess = true;
                }
            }

            if (rendererSupportsVisualizingSeedingProcess) {
                if (propertyEditor.addCheckbox("Visualize Seeding Process", &visualizeSeedingProcess)) {
                    LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
                    lineDataStress->setShallRenderSeedingProcess(visualizeSeedingProcess);
                    for (DataViewPtr& dataView : dataViews) {
                        if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_ALL_LINES_OPAQUE) {
                            OpaqueLineRenderer* opaqueLineRenderer = static_cast<OpaqueLineRenderer*>(
                                    dataView->lineRenderer);
                            opaqueLineRenderer->setVisualizeSeedingProcess(true);
                        }
#ifdef USE_VULKAN_INTEROP
                        else if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER) {
                            VulkanRayTracer* vulkanRayTracer = static_cast<VulkanRayTracer*>(dataView->lineRenderer);
                            vulkanRayTracer->setVisualizeSeedingProcess(true);
                        }
#endif
                    }
                    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
                    recordingTime = 0.0f;
                    customEndTime =
                            visualizeSeedingProcess
                            ? TIME_PER_SEED_POINT * float(lineDataStress->getNumSeedPoints() + 1)
                            : 0.0f;
                }
            }
        } else {
            if (renderingMode == RENDERING_MODE_ALL_LINES_OPAQUE) {
                if (propertyEditor.addCheckbox("Visualize Seeding Process", &visualizeSeedingProcess)) {
                    LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
                    lineDataStress->setShallRenderSeedingProcess(visualizeSeedingProcess);
                    OpaqueLineRenderer* opaqueLineRenderer = static_cast<OpaqueLineRenderer*>(lineRenderer);
                    opaqueLineRenderer->setVisualizeSeedingProcess(true);
                    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
                    recordingTime = 0.0f;
                    customEndTime =
                            visualizeSeedingProcess
                            ? TIME_PER_SEED_POINT * float(lineDataStress->getNumSeedPoints() + 1)
                            : 0.0f;
                    reRender = true;
                }
            }
        }
    }
}

void MainApp::addNewDataView() {
    DataViewPtr dataView = std::make_shared<DataView>(&sceneData, gammaCorrectionShader);
    dataView->useLinearRGB = useLinearRGB;
    dataView->clearColor = clearColor;
    dataViews.push_back(dataView);
}

void MainApp::initializeFirstDataView() {
    DataViewPtr dataView = dataViews.back();
    dataView->renderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;
    dataView->resize(1, 1); // Use arbitrary size for initialization.
    setRenderer(
            dataViews[0]->sceneData, dataViews[0]->oldRenderingMode,
            dataViews[0]->renderingMode, dataViews[0]->lineRenderer, 0);
    dataView->updateCameraMode();
    prepareVisualizationPipeline();
}

void MainApp::openFileDialog() {
    selectedDataSetIndex = 0;
    IGFD_OpenModal(
            fileDialogInstance,
            "ChooseDataSetFile", "Choose a File",
            ".*,.obj,.dat,.binlines,.nc,.vtk,.bin,.stress,.carti,.xyz",
            (sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/").c_str(),
            "", 1, nullptr,
            ImGuiFileDialogFlags_ConfirmOverwrite);
}

void MainApp::renderGuiMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open Dataset...", "CTRL+O")) {
                openFileDialog();
            }

            if (ImGui::BeginMenu("Datasets")) {
                for (int i = 1; i < NUM_MANUAL_LOADERS; i++) {
                    if (ImGui::MenuItem(dataSetNames.at(i).c_str())) {
                        selectedDataSetIndex = i;
                    }
                }

                std::stack<std::pair<DataSetInformationPtr, size_t>> dataSetInformationStack;
                dataSetInformationStack.push(std::make_pair(dataSetInformationRoot, 0));
                while (!dataSetInformationStack.empty()) {
                    std::pair<DataSetInformationPtr, size_t> dataSetIdxPair = dataSetInformationStack.top();
                    DataSetInformationPtr dataSetInformationParent = dataSetIdxPair.first;
                    size_t idx = dataSetIdxPair.second;
                    dataSetInformationStack.pop();
                    while (idx < dataSetInformationParent->children.size()) {
                        DataSetInformationPtr dataSetInformationChild =
                                dataSetInformationParent->children.at(idx);
                        if (dataSetInformationChild->type == DATA_SET_TYPE_NODE) {
                            if (ImGui::BeginMenu(dataSetInformationChild->name.c_str())) {
                                dataSetInformationStack.push(std::make_pair(dataSetInformationRoot, idx + 1));
                                dataSetInformationStack.push(std::make_pair(dataSetInformationChild, 0));
                                break;
                            }
                        } else {
                            if (ImGui::MenuItem(dataSetInformationChild->name.c_str())) {
                                selectedDataSetIndex = int(dataSetInformationChild->sequentialIndex);
                                loadLineDataSet(getSelectedLineDataSetFilenames());
                            }
                        }
                        idx++;
                    }

                    if (idx == dataSetInformationParent->children.size() && !dataSetInformationStack.empty()) {
                        ImGui::EndMenu();
                    }
                }

                ImGui::EndMenu();
            }

            if (ImGui::MenuItem("Quit", "CTRL+Q")) {
                quit();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Window")) {
            if (ImGui::MenuItem("New View...")) {
                addNewDataView();
                if (dataViews.size() == 1) {
                    initializeFirstDataView();
                }
            }
            ImGui::Separator();
            for (int i = 0; i < int(dataViews.size()); i++) {
                DataViewPtr& dataView = dataViews.at(i);
                std::string windowName = dataView->getWindowName(i);
                if (ImGui::MenuItem(windowName.c_str(), nullptr, dataView->showWindow)) {
                    dataView->showWindow = !dataView->showWindow;
                }
            }
            if (ImGui::MenuItem("FPS Overlay", nullptr, showFpsOverlay)) {
                showFpsOverlay = !showFpsOverlay;
            }
            if (ImGui::MenuItem("Coordinate Axes Overlay", nullptr, showCoordinateAxesOverlay)) {
                showCoordinateAxesOverlay = !showCoordinateAxesOverlay;
            }
            if (ImGui::MenuItem("Property Editor", nullptr, showPropertyEditor)) {
                showPropertyEditor = !showPropertyEditor;
            }
            if (ImGui::MenuItem(
                    "Transfer Function Window", nullptr, transferFunctionWindow.getShowWindow())) {
                transferFunctionWindow.setShowWindow(!transferFunctionWindow.getShowWindow());
            }
            if (ImGui::MenuItem("Checkpoint Window", nullptr, checkpointWindow.getShowWindow())) {
                checkpointWindow.setShowWindow(!checkpointWindow.getShowWindow());
            }
            if (ImGui::MenuItem("Replay Widget", nullptr, replayWidget.getShowWindow())) {
                replayWidget.setShowWindow(!replayWidget.getShowWindow());
            }
            ImGui::EndMenu();

        }

        bool isRendererComputationRunning = false;
        for (DataViewPtr& dataView : dataViews) {
            isRendererComputationRunning =
                    isRendererComputationRunning || dataView->lineRenderer->getIsComputationRunning();
            if (isRendererComputationRunning) {
                break;
            }
        }

        if (lineDataRequester.getIsProcessingRequest()
                || streamlineTracingRequester->getIsProcessingRequest()
                || stressLineTracingRequester->getIsProcessingRequest()
                || scatteringLineTracingRequester->getIsProcessingRequest()
                || isRendererComputationRunning) {
            ImGui::SetCursorPosX(ImGui::GetWindowContentRegionWidth() - ImGui::GetTextLineHeight());
            ImGui::ProgressSpinner(
                    "##progress-spinner", -1.0f, -1.0f, 4.0f,
                    ImVec4(0.1f, 0.5f, 1.0f, 1.0f));
        }

        ImGui::EndMainMenuBar();
    }
}

void MainApp::renderGuiPropertyEditorBegin() {
    if (!useDockSpaceMode) {
        renderGuiFpsCounter();

        if (ImGui::Combo(
                "Data Set", &selectedDataSetIndex, dataSetNames.data(),
                int(dataSetNames.size()))) {
            if (selectedDataSetIndex >= NUM_MANUAL_LOADERS) {
                loadLineDataSet(getSelectedLineDataSetFilenames());
            }
        }

        if (lineDataRequester.getIsProcessingRequest()
                || streamlineTracingRequester->getIsProcessingRequest()
                || stressLineTracingRequester->getIsProcessingRequest()
                || scatteringLineTracingRequester->getIsProcessingRequest()
                || lineRenderer->getIsComputationRunning()) {
            ImGui::SameLine();
            ImGui::ProgressSpinner(
                    "##progress-spinner", -1.0f, -1.0f, 4.0f,
                    ImVec4(0.1f, 0.5f, 1.0f, 1.0f));
        }

        if (selectedDataSetIndex == 0) {
            ImGui::InputText("##meshfilenamelabel", &customDataSetFileName);
            ImGui::SameLine();
            if (ImGui::Button("Load File")) {
                loadLineDataSet(getSelectedLineDataSetFilenames());
            }
        }

        if (ImGui::Combo(
                "Rendering Mode", (int*)&renderingMode, RENDERING_MODE_NAMES,
                IM_ARRAYSIZE(RENDERING_MODE_NAMES))) {
            setRenderer(sceneData, oldRenderingMode, renderingMode, lineRenderer, 0);
            reRender = true;
        }

        ImGui::Separator();
    }
}

void MainApp::renderGuiPropertyEditorCustomNodes() {
    if (lineData) {
        if (propertyEditor.beginNode(lineData->getLineDataWindowName())) {
            bool reloadGatherShader = lineData->renderGuiPropertyEditorNodes(propertyEditor);
            if (reloadGatherShader) {
                if (useDockSpaceMode) {
                    for (DataViewPtr& dataView : dataViews) {
                        dataView->lineRenderer->reloadGatherShaderExternal();
                    }
                } else {
                    lineRenderer->reloadGatherShaderExternal();
                }
            }
            propertyEditor.endNode();
        }
    }

    if (lineData) {
        if (propertyEditor.beginNode("Line Filters")) {
            for (LineFilter* dataFilter : dataFilters) {
                dataFilter->renderGuiPropertyEditorNodes(propertyEditor);
            }
            propertyEditor.endNode();
        }
    }

    if (useDockSpaceMode) {
        for (int i = 0; i < int(dataViews.size()); i++) {
            DataViewPtr& dataView = dataViews.at(i);
            bool removeView = false;
            bool beginNode = propertyEditor.beginNode(dataView->getWindowName(i));
            ImGui::SameLine();
            float indentWidth = ImGui::GetContentRegionAvail().x;
            ImGui::Indent(indentWidth);
            if (ImGui::Button("X")) {
                removeView = true;
            }
            ImGui::Unindent(indentWidth);
            if (beginNode) {
                std::string previewValue =
                        dataView->renderingMode == RENDERING_MODE_NONE
                        ? "None" : RENDERING_MODE_NAMES[int(dataView->renderingMode)];
                if (propertyEditor.addBeginCombo("Rendering Mode", previewValue)) {
                    for (int j = 0; j < IM_ARRAYSIZE(RENDERING_MODE_NAMES); j++) {
                        if (ImGui::Selectable(
                                RENDERING_MODE_NAMES[int(j)], false,
                                ImGuiSelectableFlags_::ImGuiSelectableFlags_DontClosePopups)) {
                            ImGui::CloseCurrentPopup();
                            dataView->renderingMode = RenderingMode(j);
                            setRenderer(
                                    dataView->sceneData, dataView->oldRenderingMode, dataView->renderingMode,
                                    dataView->lineRenderer, i);
                            dataView->updateCameraMode();
                            reRender = true;
                        }
                    }
                    propertyEditor.addEndCombo();
                }

                if (propertyEditor.addCheckbox("Sync with Global Camera", &dataView->syncWithParentCamera)) {
                    dataView->reRender = true;
                    if (dataView->lineRenderer) {
                        dataView->lineRenderer->notifyReRenderTriggeredExternally();
                    }
                }

                if (dataView->lineRenderer) {
                    dataView->lineRenderer->renderGuiPropertyEditorNodes(propertyEditor);
                }
                propertyEditor.endNode();
            }
            if (removeView) {
                dataViews.erase(dataViews.begin() + i);
                i--;
            }
        }
    } else {
        if (lineRenderer) {
            if (propertyEditor.beginNode(lineRenderer->getWindowName())) {
                lineRenderer->renderGuiPropertyEditorNodes(propertyEditor);
                propertyEditor.endNode();
            }
        }
    }
}

void MainApp::update(float dt) {
    ZoneScoped;

    sgl::SciVisApp::update(dt);

    if (scheduledDockSpaceModeChange) {
        if (useDockSpaceMode) {
            if (!dataViews.empty() && dataViews.front()->lineRenderer) {
                lineRenderer = dataViews.front()->lineRenderer;
                dataViews.front()->lineRenderer = nullptr;
            }
            dataViews.clear();
        } else {
            if (lineRenderer) {
                addNewDataView();
                DataViewPtr dataView = dataViews.back();
                dataView->lineRenderer = lineRenderer;
                lineRenderer = nullptr;
            }
        }

        useDockSpaceMode = newDockSpaceMode;
        scheduledDockSpaceModeChange = false;
    }

    if (visualizeSeedingProcess) {
        const int seedPointIdx = int(recordingTime / TIME_PER_SEED_POINT - 1);
        LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
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
        SettingsMap currentDatasetSettings = replayWidget.getCurrentDatasetSettings();
        bool reloadGatherShader = false;
        if (lineData) {
            reloadGatherShader |= lineData->setNewSettings(currentDatasetSettings);
        }
        if (useDockSpaceMode) {
            for (DataViewPtr& dataView : dataViews) {
                bool reloadGatherShaderLocal = reloadGatherShader;
                if (dataView->lineRenderer) {
                    SettingsMap currentRendererSettings = replayWidget.getCurrentRendererSettings();
                    reloadGatherShaderLocal |= dataView->lineRenderer->setNewSettings(currentRendererSettings);
                }
                if (dataView->lineRenderer && reloadGatherShaderLocal) {
                    dataView->lineRenderer->reloadGatherShaderExternal();
                }
            }
        } else {
            if (lineRenderer) {
                SettingsMap currentRendererSettings = replayWidget.getCurrentRendererSettings();
                reloadGatherShader |= lineRenderer->setNewSettings(currentRendererSettings);
            }
            if (lineRenderer && reloadGatherShader) {
                lineRenderer->reloadGatherShaderExternal();
            }
        }

        if (updateTransferFunctionRange) {
            transferFunctionWindow.setSelectedRange(transferFunctionRange);
            updateTransferFunctionRange = false;
            lineData->onTransferFunctionMapRebuilt();
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
        }

        replayWidgetRunning = true;
        reRender = true;

        if (!useCameraFlight) {
            if (realTimeReplayUpdates) {
                uint64_t currentTimeStamp = sgl::Timer->getTicksMicroseconds();
                uint64_t timeElapsedMicroSec = currentTimeStamp - recordingTimeStampStart;
                recordingTime = timeElapsedMicroSec * 1e-6f;
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

    checkLoadingRequestFinished();

    transferFunctionWindow.update(dt);
    if (lineData) {
        lineData->update(dt);
    }

    ImGuiIO &io = ImGui::GetIO();
    if (!io.WantCaptureKeyboard || recording || focusedWindowIndex != -1) {
        if (useDockSpaceMode) {
            for (int i = 0; i < int(dataViews.size()); i++) {
                DataViewPtr& dataView = dataViews.at(i);
                if (i != focusedWindowIndex) {
                    continue;
                }
                // 3D camera movement disabled for certain renderers.
                if (dataView->lineRenderer
                        && dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_SPHERICAL_HEAT_MAP_RENDERER) {
                    dataView->moveCamera2dKeyboard(dt);
                    continue;
                }

                sgl::CameraPtr parentCamera = this->camera;
                bool reRenderOld = reRender;
                if (!dataView->syncWithParentCamera) {
                    this->camera = dataView->camera;
                    hasMovedIndex = i;
                }
                this->reRender = false;
                moveCameraKeyboard(dt);
                if (this->reRender && dataView->syncWithParentCamera) {
                    for (DataViewPtr& dataViewLocal : dataViews) {
                        if (dataViewLocal->syncWithParentCamera) {
                            dataViewLocal->reRender = dataView->reRender || this->reRender;
                        }
                    }
                }
                if (!dataView->syncWithParentCamera) {
                    dataView->reRender = dataView->reRender || this->reRender;
                    this->camera = parentCamera;
                    hasMovedIndex = -1;
                }
                this->reRender = reRenderOld;
            }
        } else {
            moveCameraKeyboard(dt);
        }

        if (sgl::Keyboard->isKeyDown(SDLK_u)) {
            transferFunctionWindow.setShowWindow(showSettingsWindow);
        }
    }

    if (!io.WantCaptureMouse || mouseHoverWindowIndex != -1) {
        if (useDockSpaceMode) {
            for (int i = 0; i < int(dataViews.size()); i++) {
                DataViewPtr& dataView = dataViews.at(i);
                if (i != mouseHoverWindowIndex) {
                    continue;
                }
                // 3D camera movement disabled for certain renderers.
                if (dataView->lineRenderer
                        && dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_SPHERICAL_HEAT_MAP_RENDERER) {
                    dataView->moveCamera2dMouse(dt);
                    continue;
                }

                sgl::CameraPtr parentCamera = this->camera;
                bool reRenderOld = reRender;
                if (!dataView->syncWithParentCamera) {
                    this->camera = dataView->camera;
                    hasMovedIndex = i;
                }
                this->reRender = false;
                moveCameraMouse(dt);
                if (this->reRender && dataView->syncWithParentCamera) {
                    for (DataViewPtr& dataViewLocal : dataViews) {
                        if (dataViewLocal->syncWithParentCamera) {
                            dataViewLocal->reRender = dataView->reRender || this->reRender;
                        }
                    }
                }
                if (!dataView->syncWithParentCamera) {
                    dataView->reRender = dataView->reRender || this->reRender;
                    this->camera = parentCamera;
                    hasMovedIndex = -1;
                }
                this->reRender = reRenderOld;
            }
        } else {
            moveCameraMouse(dt);
        }

        if (useDockSpaceMode) {
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer != nullptr) {
                    dataView->lineRenderer->update(dt);
                }
            }
        } else {
            if (lineRenderer != nullptr) {
                lineRenderer->update(dt);
            }
        }
    }
}

void MainApp::hasMoved() {
    if (useDockSpaceMode) {
        if (hasMovedIndex < 0) {
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer != nullptr && dataView->syncWithParentCamera) {
                    dataView->lineRenderer->onHasMoved();
                }
            }
        } else {
            DataViewPtr& dataView = dataViews.at(hasMovedIndex);
            if (dataView->lineRenderer != nullptr) {
                dataView->lineRenderer->onHasMoved();
            }
        }
    } else {
        if (lineRenderer != nullptr) {
            lineRenderer->onHasMoved();
        }
    }
}

void MainApp::onCameraReset() {
    if (useDockSpaceMode) {
        for (DataViewPtr& dataView : dataViews) {
            dataView->camera->setNearClipDistance(camera->getNearClipDistance());
            dataView->camera->setFarClipDistance(camera->getFarClipDistance());
            dataView->camera->setYaw(camera->getYaw());
            dataView->camera->setPitch(camera->getPitch());
            dataView->camera->setFOVy(camera->getFOVy());
            dataView->camera->setPosition(camera->getPosition());
            dataView->camera->resetLookAtLocation();
        }
    }
}


// --- Visualization pipeline ---

void MainApp::loadLineDataSet(const std::vector<std::string>& fileNames, bool blockingDataLoading) {
    if (fileNames.empty() || fileNames.front().empty()) {
        lineData = LineDataPtr();
        return;
    }
    currentlyLoadedDataSetIndex = selectedDataSetIndex;

    DataSetInformation selectedDataSetInformation;
    if (selectedDataSetIndex >= NUM_MANUAL_LOADERS && !dataSetInformationList.empty()) {
        selectedDataSetInformation = *dataSetInformationList.at(selectedDataSetIndex - NUM_MANUAL_LOADERS);
    } else if (selectedDataSetIndex == 2) {
        selectedDataSetInformation = stressLineTracerDataSetInformation;
    } else {
        selectedDataSetInformation.type = dataSetType;
        selectedDataSetInformation.filenames = fileNames;

        // Use standard settings for stress line data sets (assume data format version 3).
        if (dataSetType == DATA_SET_TYPE_STRESS_LINES) {
            selectedDataSetInformation.hasCustomTransform = true;
            selectedDataSetInformation.transformMatrix = parseTransformString("rotate(270°, 1, 0, 0)");
            selectedDataSetInformation.version = 3;
            selectedDataSetInformation.attributeNames.emplace_back("Principal Stress");
            selectedDataSetInformation.attributeNames.emplace_back("Principal Stress Magnitude");
            selectedDataSetInformation.attributeNames.emplace_back("von Mises Stress");
            selectedDataSetInformation.attributeNames.emplace_back("Normal Stress (xx)");
            selectedDataSetInformation.attributeNames.emplace_back("Normal Stress (yy)");
            selectedDataSetInformation.attributeNames.emplace_back("Normal Stress (zz)");
            selectedDataSetInformation.attributeNames.emplace_back("Shear Stress (yz)");
            selectedDataSetInformation.attributeNames.emplace_back("Shear Stress (zx)");
            selectedDataSetInformation.attributeNames.emplace_back("Shear Stress (xy)");
        }
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
    if (selectedDataSetInformation.heightScale != 1.0f) {
        transformationMatrix *= glm::scale(glm::vec3(1.0f, selectedDataSetInformation.heightScale, 1.0f));
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
            if (useDockSpaceMode) {
                std::vector<LineRenderer*> lineRenderers;
                for (DataViewPtr& dataView : dataViews) {
                    if (dataView->lineRenderer) {
                        lineRenderers.push_back(dataView->lineRenderer);
                    }
                }
                lineData->setLineRenderers(lineRenderers);
            } else {
                lineData->setLineRenderers({ lineRenderer });
            }
            lineData->setRenderingModes({ renderingMode });
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
    LineDataPtr lineData;
    DataSetInformation loadedDataSetInformation;

    streamlineTracingRequester->getHasNewData(loadedDataSetInformation, lineData);
    if (stressLineTracingRequester->getHasNewData(stressLineTracerDataSetInformation)) {
        dataSetType = stressLineTracerDataSetInformation.type;
        loadLineDataSet(stressLineTracerDataSetInformation.filenames);
    }
    scatteringLineTracingRequester->getHasNewData(loadedDataSetInformation, lineData);

    if (!lineData) {
        lineData = lineDataRequester.getLoadedData(loadedDataSetInformation);
    }

    if (lineData) {
        if (loadedDataSetInformation.hasCustomLineWidth) {
            LineRenderer::setLineWidth(loadedDataSetInformation.lineWidth);
        }
        sgl::ColorLegendWidget::resetStandardSize();

        this->lineData = lineData;
        lineData->recomputeHistogram();
        lineData->setClearColor(clearColor);
        lineData->setUseLinearRGB(useLinearRGB);
        if (useDockSpaceMode) {
            std::vector<LineRenderer*> lineRenderers;
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer) {
                    lineRenderers.push_back(dataView->lineRenderer);
                }
            }
            lineData->setLineRenderers(lineRenderers);
        } else {
            lineData->setLineRenderers({ lineRenderer });
        }
        lineData->setRenderingModes({ renderingMode });
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
                        lineData->getFocusBoundingBox(), meshDescriptorName,
                        usePerformanceMeasurementMode
                        ? CAMERA_PATH_TIME_PERFORMANCE_MEASUREMENT : CAMERA_PATH_TIME_RECORDING,
                        usePerformanceMeasurementMode);
                //cameraPath.saveToBinaryFile(cameraPathFilename);
            }
        }
    }
}

void MainApp::reloadDataSet() {
    loadLineDataSet(getSelectedLineDataSetFilenames());
}

void MainApp::prepareVisualizationPipeline() {
    ZoneScoped;

    if (useDockSpaceMode) {
        if (lineData && !dataViews.empty()) {
            bool isPreviousNodeDirty = lineData->isDirty();
            filterData(isPreviousNodeDirty);
            if (isPreviousNodeDirty) {
                lineData->setTriangleRepresentationDirty();
            }
            bool isTriangleRepresentationDirty = lineData->isTriangleRepresentationDirty();
            for (DataViewPtr& dataView : dataViews) {
                if (!dataView->lineRenderer) {
                    continue;
                }
                bool isTriangleRepresentationDirtyLocal =
                        isTriangleRepresentationDirty && dataView->lineRenderer->getIsTriangleRepresentationUsed();
                if (dataView->lineRenderer->isDirty() || isPreviousNodeDirty || isTriangleRepresentationDirtyLocal) {
                    dataView->lineRenderer->setLineData(lineData, newMeshLoaded);
                }
            }
        }
    } else {
        if (lineData && lineRenderer) {
            bool isPreviousNodeDirty = lineData->isDirty();
            bool isTriangleRepresentationDirty =
                    lineData->isTriangleRepresentationDirty() && lineRenderer->getIsTriangleRepresentationUsed();
            filterData(isPreviousNodeDirty);
            if (isPreviousNodeDirty) {
                lineData->setTriangleRepresentationDirty();
            }
            if (lineRenderer->isDirty() || isPreviousNodeDirty || isTriangleRepresentationDirty) {
                lineRenderer->setLineData(lineData, newMeshLoaded);
            }
        }
    }
    if (lineData && lineRenderer) {
        bool isPreviousNodeDirty = lineData->isDirty();
        bool isTriangleRepresentationDirty =
                lineData->isTriangleRepresentationDirty() && lineRenderer->getIsTriangleRepresentationUsed();
        filterData(isPreviousNodeDirty);
        if (isPreviousNodeDirty) {
            lineData->setTriangleRepresentationDirty();
        }
        if (lineRenderer->isDirty() || isPreviousNodeDirty || isTriangleRepresentationDirty) {
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
