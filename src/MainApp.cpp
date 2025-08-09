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

#ifdef USE_ZEROMQ
#include <zmq.h>
#endif

#include <Utils/StringUtils.hpp>
#include <Utils/Timer.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/Dialog.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/Regex/TransformString.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Input/Keyboard.hpp>
#include <Input/Mouse.hpp>
#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Vulkan/Utils/Instance.hpp>
#include <Graphics/Vulkan/Utils/DeviceSelectionVulkan.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#ifdef SUPPORT_OPENCL_INTEROP
#include <Graphics/Vulkan/Utils/InteropOpenCL.hpp>
#endif

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/ImGuiFileDialog/ImGuiFileDialog.h>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include "LineData/LineDataFlow.hpp"
#include "LineData/LineDataStress.hpp"
#include "LineData/TriangleMesh/TriangleMeshData.hpp"
#include "LineData/Filters/LineLengthFilter.hpp"
#include "LineData/Filters/MaxLineAttributeFilter.hpp"
#include "LineData/Flow/StreamlineTracingRequester.hpp"
#include "LineData/Stress/StressLineTracingRequester.hpp"
#include "LineData/Scattering/ScatteringLineTracingRequester.hpp"
#include "Loaders/NetCdfLineLoader.hpp"
#include "Renderers/OpaqueLineRenderer.hpp"
#include "Renderers/Deferred/DeferredRenderer.hpp"
#include "Renderers/OIT/PerPixelLinkedListLineRenderer.hpp"
#include "Renderers/OIT/MLABRenderer.hpp"
#include "Renderers/OIT/OpacityOptimizationRenderer.hpp"
#include "Renderers/OIT/DepthComplexityRenderer.hpp"
#include "Renderers/OIT/MBOITRenderer.hpp"
#include "Renderers/OIT/MLABBucketRenderer.hpp"
#include "Renderers/OIT/WBOITRenderer.hpp"
#include "Renderers/OIT/DepthPeelingRenderer.hpp"
#include "Renderers/OIT/AtomicLoop64Renderer.hpp"
#include "Renderers/VRC/VoxelRayCastingRenderer.hpp"

#include "Renderers/RayTracing/VulkanRayTracer.hpp"
#include "Renderers/Scattering/LineDensityMapRenderer.hpp"
#include "Renderers/Scattering/SphericalHeatMapRenderer.hpp"
#include "Renderers/Scattering/PathTracer/VolumetricPathTracingRenderer.hpp"
#ifdef SUPPORT_OPTIX
#include "Renderers/Scattering/Denoiser/OptixVptDenoiser.hpp"
#endif
#if defined(SUPPORT_CUDA_INTEROP) && defined(SUPPORT_OPEN_IMAGE_DENOISE)
#include "Renderers/Scattering/Denoiser/OpenImageDenoiseDenoiser.hpp"
#endif

#ifdef USE_OSPRAY
#include "Renderers/Ospray/OsprayRenderer.hpp"
#endif

#include "Widgets/DataView.hpp"
#include "MainApp.hpp"

void vulkanErrorCallback() {
#ifdef SGL_INPUT_API_V2
    sgl::AppSettings::get()->captureMouse(false);
#else
    SDL_CaptureMouse(SDL_FALSE);
#endif
    std::cerr << "Application callback" << std::endl;
}
bool vulkanFilterDebugMessageCallback(const VkDebugUtilsMessengerCallbackDataEXT* callbackData) {
    if (callbackData->messageIdNumber == 622825338) {
        /*
         * This corresponds to pMessageIdName == "VUID-vkQueuePresentKHR-pWaitSemaphores-03268".
         * This validation error probably comes from the fact that the semaphore is signalled on the CUDA side
         * and the Vulkan validation layer is not able to track this signal call.
         * This is most likely a bug in the validation layer, as it should not try to track this type of information
         * for exported semaphores that may be used in other APIs.
         */
        return true;
    }
    return false;
}

#ifdef __linux__
void signalHandler(int signum) {
#ifdef SGL_INPUT_API_V2
    sgl::AppSettings::get()->captureMouse(false);
#else
    SDL_CaptureMouse(SDL_FALSE);
#endif
    std::cerr << "Interrupt signal (" << signum << ") received." << std::endl;
    exit(signum);
}
#endif

MainApp::MainApp()
        : sceneData(
                &rendererVk, &sceneTextureVk, &sceneDepthTextureVk,
                &viewportWidth, &viewportHeight, camera,
                &clearColor, &screenshotTransparentBackground,
                &performanceMeasurer, &continuousRendering, &recording,
                &useCameraFlight, &MOVE_SPEED, &MOUSE_ROT_SPEED,
                &nonBlockingMsgBoxHandles),
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
                  transferFunctionWindow, rendererVk)) {
    sgl::AppSettings::get()->getVulkanInstance()->setDebugCallback(&vulkanErrorCallback);
    sgl::AppSettings::get()->getVulkanInstance()->setFilterDebugMessageCallback(&vulkanFilterDebugMessageCallback);

#ifdef SUPPORT_CUDA_INTEROP
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    if (device->getDeviceDriverId() == VK_DRIVER_ID_NVIDIA_PROPRIETARY) {
        cudaInteropInitialized = true;
        if (!sgl::initializeCudaDeviceApiFunctionTable()) {
            cudaInteropInitialized = false;
            sgl::Logfile::get()->writeError(
                    "Error in MainApp::MainApp: sgl::initializeCudaDeviceApiFunctionTable() returned false.",
                    false);
        }

        if (cudaInteropInitialized) {
            CUresult cuResult = sgl::g_cudaDeviceApiFunctionTable.cuInit(0);
            if (cuResult == CUDA_ERROR_NO_DEVICE) {
                sgl::Logfile::get()->writeInfo("No CUDA-capable device was found. Disabling CUDA interop support.");
                cudaInteropInitialized = false;
            } else {
                sgl::checkCUresult(cuResult, "Error in cuInit: ");
            }
        }

        if (cudaInteropInitialized) {
            CUresult cuResult = sgl::g_cudaDeviceApiFunctionTable.cuCtxCreate(
                    &cuContext, CU_CTX_SCHED_SPIN, cuDevice);
            sgl::checkCUresult(cuResult, "Error in cuCtxCreate: ");
        }
    }
#endif
#ifdef SUPPORT_OPTIX
    if (cudaInteropInitialized) {
        optixInitialized = OptixVptDenoiser::initGlobal(cuContext, cuDevice);
    }
#endif
#if defined(SUPPORT_CUDA_INTEROP) && defined(SUPPORT_OPEN_IMAGE_DENOISE)
    if (cudaInteropInitialized) {
        OpenImageDenoiseDenoiser::initGlobalCuda(cuContext, cuDevice);
    }
#endif
#ifdef SUPPORT_OPENCL_INTEROP
    openclInteropInitialized = true;
    if (!sgl::initializeOpenCLFunctionTable()) {
        openclInteropInitialized = false;
    }
#endif

    if (LineData::getLinePrimitiveModeUsesGeometryShader(LineData::getLinePrimitiveMode())
            && !device->getPhysicalDeviceFeatures().geometryShader) {
        LineData::setLinePrimitiveMode(LineData::LINE_PRIMITIVES_QUADS_PROGRAMMABLE_PULL);
    }

    sgl::ColorLegendWidget::setFontScaleStandard(1.0f);

#ifdef USE_PYTHON
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
                    std::string() + "Error in replay widget load renderer callback: Unknown renderer name \""
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
            sgl::MultiVarTransferFunctionWindow* multiVarTransferFunctionWindow;
            if (lineData->getType() == DATA_SET_TYPE_FLOW_LINES) {
                LineDataFlow* lineDataFlow = static_cast<LineDataFlow*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataFlow->getMultiVarTransferFunctionWindow();
            } else if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
                LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataStress->getMultiVarTransferFunctionWindow();
            } else if (lineData->getType() == DATA_SET_TYPE_TRIANGLE_MESH) {
                TriangleMeshData* triangleMeshData = static_cast<TriangleMeshData*>(lineData.get());
                multiVarTransferFunctionWindow = &triangleMeshData->getMultiVarTransferFunctionWindow();
            } else {
                sgl::Logfile::get()->writeError(
                        "Error in replay widget load multi-var transfer functions callback: Invalid data type.");
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
            sgl::MultiVarTransferFunctionWindow* multiVarTransferFunctionWindow;
            if (lineData->getType() == DATA_SET_TYPE_FLOW_LINES) {
                LineDataFlow* lineDataFlow = static_cast<LineDataFlow*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataFlow->getMultiVarTransferFunctionWindow();
            } else if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
                LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
                multiVarTransferFunctionWindow = &lineDataStress->getMultiVarTransferFunctionWindow();
            } else if (lineData->getType() == DATA_SET_TYPE_TRIANGLE_MESH) {
                TriangleMeshData* triangleMeshData = static_cast<TriangleMeshData*>(lineData.get());
                multiVarTransferFunctionWindow = &triangleMeshData->getMultiVarTransferFunctionWindow();
            } else {
                sgl::Logfile::get()->writeError(
                        "Error in replay widget multi-var transfer functions ranges callback: Invalid data type.");
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
    if (sgl::FileUtils::get()->get_argc() > 1) {
        if (strcmp(sgl::FileUtils::get()->get_argv()[1], "--perf") == 0) {
            usePerformanceMeasurementMode = true;
        }
    }
    cameraPath.setApplicationCallback([this](
            const std::string& modelFilename, glm::vec3& centerOffset, float& startAngle, float& pulseFactor,
            float& standardZoom) {
        if (sgl::startsWith(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-Vis2021/psl/Vis2021_femur3D")) {
            pulseFactor = 0.0f;
            standardZoom = 2.9f;
            centerOffset.y = 0.001f;
            this->camera->setFOVy(36.0f / 180.0f * sgl::PI);
        } else if (sgl::startsWith(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-Vis2021")) {
            pulseFactor = 0.0f;
            standardZoom = 2.0f;
        } else if (sgl::startsWith(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-TVCG01/psl/arched_bridge3D_PSLs")) {
            pulseFactor = 0.0f;
            standardZoom = 1.9f;
        } else if (sgl::startsWith(
                modelFilename,
                sgl::AppSettings::get()->getDataDirectory()
                + "LineDataSets/stress/PSLs-TVCG01/psl/arched_bridge3D_PSLs")) {
            pulseFactor = 0.0f;
            standardZoom = 1.9f;
        } else if (sgl::startsWith(
                modelFilename, sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/clouds")) {
            pulseFactor = 0.0f;
            standardZoom = 2.9f;
            centerOffset = glm::vec3(0.0f, -0.1f, 0.0f);
        }
    });

    useDockSpaceMode = true;
    sgl::AppSettings::get()->getSettings().getValueOpt("useDockSpaceMode", useDockSpaceMode);
    sgl::AppSettings::get()->getSettings().getValueOpt("useFixedSizeViewport", useFixedSizeViewport);
    sgl::AppSettings::get()->getSettings().getValueOpt("fixedViewportSizeX", fixedViewportSize.x);
    sgl::AppSettings::get()->getSettings().getValueOpt("fixedViewportSizeY", fixedViewportSize.y);
    fixedViewportSizeEdit = fixedViewportSize;
    showPropertyEditor = true;
    sgl::ImGuiWrapper::get()->setUseDockSpaceMode(useDockSpaceMode);
    //useDockSpaceMode = false;

#ifdef NDEBUG
    showFpsOverlay = false;
#else
    showFpsOverlay = true;
#endif
    sgl::AppSettings::get()->getSettings().getValueOpt("showFpsOverlay", showFpsOverlay);
    sgl::AppSettings::get()->getSettings().getValueOpt("showCoordinateAxesOverlay", showCoordinateAxesOverlay);

    deviceSelector = device->getDeviceSelector();

    useLinearRGB = false;
    transferFunctionWindow.setClearColor(clearColor);
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
    coordinateAxesOverlayWidget.setClearColor(clearColor);

    LineRenderer::setNewTilingMode(2, 8);
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

    fileDialogInstance = IGFD_Create();
    customDataSetFileName = sgl::FileUtils::get()->getUserDirectory();
    loadAvailableDataSetInformation();

    if (!recording && !usePerformanceMeasurementMode) {
        // Just for convenience...
        int desktopWidth = 0;
        int desktopHeight = 0;
        int refreshRate = 60;
        sgl::AppSettings::get()->getDesktopDisplayMode(desktopWidth, desktopHeight, refreshRate);
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        if (desktopWidth == 3840 && desktopHeight == 2160 && !window->getWindowSettings().isMaximized) {
            window->setWindowSize(2186, 1358);
        }
    }

    if (!useDockSpaceMode) {
        setRenderer(sceneData, oldRenderingMode, renderingMode, lineRenderer, 0);
    }

    if (!sgl::AppSettings::get()->getSettings().hasKey("cameraNavigationMode")) {
        cameraNavigationMode = sgl::CameraNavigationMode::TURNTABLE;
        updateCameraNavigationMode();
    }

    addNewDataView();

    recordingTimeStampStart = sgl::Timer->getTicksMicroseconds();
    usesNewState = true;
    if (usePerformanceMeasurementMode) {
        sgl::FileUtils::get()->ensureDirectoryExists("images");
        performanceMeasurer = new AutomaticPerformanceMeasurer(
                rendererVk, getTestModes(),
                "performance.csv", "depth_complexity.csv",
                [this](const InternalState &newState) { this->setNewState(newState); });
    }

#ifdef __linux__
    signal(SIGSEGV, signalHandler);
#endif
}

MainApp::~MainApp() {
    device->waitIdle();

    if (usePerformanceMeasurementMode) {
        performanceMeasurer->cleanup();
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

#ifdef SUPPORT_OPTIX
    if (optixInitialized) {
        OptixVptDenoiser::freeGlobal();
    }
#endif
#ifdef SUPPORT_CUDA_INTEROP
    if (sgl::getIsCudaDeviceApiFunctionTableInitialized()) {
        if (cuContext) {
            CUresult cuResult = sgl::g_cudaDeviceApiFunctionTable.cuCtxDestroy(cuContext);
            sgl::checkCUresult(cuResult, "Error in cuCtxDestroy: ");
            cuContext = {};
        }
        sgl::freeCudaDeviceApiFunctionTable();
    }
#endif
#ifdef SUPPORT_OPENCL_INTEROP
    if (sgl::getIsOpenCLFunctionTableInitialized()) {
        sgl::freeOpenCLFunctionTable();
    }
#endif

    for (int i = 0; i < int(nonBlockingMsgBoxHandles.size()); i++) {
        auto& handle = nonBlockingMsgBoxHandles.at(i);
        if (handle->ready(0)) {
            nonBlockingMsgBoxHandles.erase(nonBlockingMsgBoxHandles.begin() + i);
            i--;
        } else {
            handle->kill();
        }
    }
    nonBlockingMsgBoxHandles.clear();

    sgl::AppSettings::get()->getSettings().addKeyValue("useDockSpaceMode", useDockSpaceMode);
    if (!usePerformanceMeasurementMode) {
        sgl::AppSettings::get()->getSettings().addKeyValue("useFixedSizeViewport", useFixedSizeViewport);
        sgl::AppSettings::get()->getSettings().addKeyValue("fixedViewportSizeX", fixedViewportSize.x);
        sgl::AppSettings::get()->getSettings().addKeyValue("fixedViewportSizeY", fixedViewportSize.y);
    }
    sgl::AppSettings::get()->getSettings().addKeyValue("showFpsOverlay", showFpsOverlay);
    sgl::AppSettings::get()->getSettings().addKeyValue("showCoordinateAxesOverlay", showCoordinateAxesOverlay);
}

void MainApp::setNewState(const InternalState &newState) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    rendererVk->getDevice()->waitIdle();

    if (performanceMeasurer) {
        performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(0);
    }

    // 1. Change the window resolution?
    glm::ivec2 newResolution = newState.windowResolution;
    if (useDockSpaceMode) {
        useFixedSizeViewport = true;
        fixedViewportSizeEdit = newResolution;
        fixedViewportSize = newResolution;
    } else {
        sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
        int currentWindowWidth = window->getWidth();
        int currentWindowHeight = window->getHeight();
        if (newResolution.x > 0 && newResolution.y > 0 && currentWindowWidth != newResolution.x
                && currentWindowHeight != newResolution.y) {
            window->setWindowSize(newResolution.x, newResolution.y);
        }
    }

    // 1.1. Handle the new tiling mode for SSBO accesses.
    LineRenderer::setNewTilingMode(
            newState.tilingWidth, newState.tilingHeight,
            newState.useMortonCodeForTiling);

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
        if (useDockSpaceMode) {
            if (dataViews.empty()) {
                addNewDataView();
            }
            RenderingMode newRenderingMode = newState.renderingMode;
            setRenderer(
                    dataViews[0]->sceneData, dataViews[0]->oldRenderingMode, newRenderingMode,
                    dataViews[0]->lineRenderer, 0);
            dataViews[0]->renderingMode = newRenderingMode;
            dataViews[0]->updateCameraMode();
        } else {
            renderingMode = newState.renderingMode;
            setRenderer(sceneData, oldRenderingMode, renderingMode, lineRenderer, 0);
        }
    }

    // 2.2. Set the new renderer settings.
    bool reloadGatherShader = false;
    std::vector<bool> reloadGatherShaderDataViewList;
    if (useDockSpaceMode) {
        for (DataViewPtr& dataView : dataViews) {
            bool reloadGatherShaderLocal = reloadGatherShader;
            if (dataView->lineRenderer) {
                dataView->lineRenderer->setNewState(newState);
                reloadGatherShaderLocal |= dataView->lineRenderer->setNewSettings(newState.rendererSettings);
                reloadGatherShaderDataViewList.push_back(reloadGatherShaderLocal);
            }
        }
    } else {
        if (lineRenderer) {
            lineRenderer->setNewState(newState);
            reloadGatherShader |= lineRenderer->setNewSettings(newState.rendererSettings);
        }
    }

    // 3. Load the correct data set file.
    if (newState.dataSetDescriptor != lastState.dataSetDescriptor) {
        selectedDataSetIndex = 0;
        std::string nameLower = sgl::toLowerCopy(newState.dataSetDescriptor.name);
        for (size_t i = 0; i < dataSetInformationList.size(); i++) {
            if (sgl::toLowerCopy(dataSetInformationList.at(i)->name) == nameLower) {
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
            loadLineDataSet(newState.dataSetDescriptor.filenames, true);
        } else {
            if (newState.dataSetDescriptor.type == DATA_SET_TYPE_STRESS_LINES
                    && newState.dataSetDescriptor.enabledFileIndices.size() == 3) {
                LineDataStress::setUseMajorPS(newState.dataSetDescriptor.enabledFileIndices.at(0));
                LineDataStress::setUseMediumPS(newState.dataSetDescriptor.enabledFileIndices.at(1));
                LineDataStress::setUseMinorPS(newState.dataSetDescriptor.enabledFileIndices.at(2));
            }
            loadLineDataSet(getSelectedLineDataSetFilenames(), true);
        }
    }

    // 4. Pass state change to filters to handle internally necessary state changes.
    for (LineFilter* filter : dataFilters) {
        filter->setNewState(newState);
    }
    for (size_t i = 0; i < newState.filterSettings.size(); i++) {
        dataFilters.at(i)->setNewSettings(newState.filterSettings.at(i));
    }

    // 5. Pass state change to renderers to handle internally necessary state changes.
    if (lineData) {
        reloadGatherShader |= lineData->setNewSettings(newState.dataSetSettings);
    }

    // 6. Reload the gather shader if necessary.
    if (useDockSpaceMode) {
        size_t idx = 0;
        for (DataViewPtr& dataView : dataViews) {
            bool reloadGatherShaderLocal = reloadGatherShader || reloadGatherShaderDataViewList.at(idx);
            if (dataView->lineRenderer && reloadGatherShaderLocal) {
                dataView->lineRenderer->reloadGatherShaderExternal();
            }
            idx++;
        }
    } else {
        if (lineRenderer && reloadGatherShader) {
            lineRenderer->reloadGatherShaderExternal();
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
        device->waitIdle();
        delete newLineRenderer;
        newLineRenderer = nullptr;
    }

    if (oldRenderingMode != newRenderingMode) {
        // User depth buffer with higher accuracy when rendering lines opaquely.
        if (newRenderingMode == RENDERING_MODE_OPAQUE
                || newRenderingMode == RENDERING_MODE_DEFERRED_SHADING
                || oldRenderingMode == RENDERING_MODE_OPAQUE
                || oldRenderingMode == RENDERING_MODE_DEFERRED_SHADING) {
            VkFormat depthFormat;
            if (newRenderingMode == RENDERING_MODE_OPAQUE
                    || newRenderingMode == RENDERING_MODE_DEFERRED_SHADING) {
                depthFormat = device->getSupportedDepthFormat();
            } else {
                depthFormat = device->getSupportedDepthStencilFormat();
            }
            if (useDockSpaceMode) {
                dataViews.at(dataViewIndex)->sceneDepthTextureVkFormat = depthFormat;
                sceneDepthTextureVkFormat = depthFormat;
            } else {
                sceneDepthTextureVkFormat = depthFormat;
            }
            scheduleRecreateSceneFramebuffer();
        }
        oldRenderingMode = newRenderingMode;
    }

    if (newRenderingMode == RENDERING_MODE_OPAQUE) {
        newLineRenderer = new OpaqueLineRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_DEFERRED_SHADING) {
        newLineRenderer = new DeferredRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_PER_PIXEL_LINKED_LIST) {
        newLineRenderer = new PerPixelLinkedListLineRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_MLAB) {
        newLineRenderer = new MLABRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_OPACITY_OPTIMIZATION) {
        if (sgl::AppSettings::get()->getPrimaryDevice()->getPhysicalDeviceFeatures().geometryShader) {
            newLineRenderer = new OpacityOptimizationRenderer(&sceneDataRef, transferFunctionWindow);
        } else {
            std::string warningText =
                    std::string() + "The selected renderer \"" + RENDERING_MODE_NAMES[newRenderingMode] + "\" is not "
                    + "supported on this hardware due to the missing geometry shader physical device feature.";
            onUnsupportedRendererSelected(warningText, sceneDataRef, newRenderingMode, newLineRenderer);
        }
    } else if (newRenderingMode == RENDERING_MODE_DEPTH_COMPLEXITY) {
        newLineRenderer = new DepthComplexityRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_MBOIT) {
        newLineRenderer = new MBOITRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_MLAB_BUCKETS) {
        newLineRenderer = new MLABBucketRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_WBOIT) {
        if (sgl::AppSettings::get()->getPrimaryDevice()->getPhysicalDeviceFeatures().independentBlend) {
            newLineRenderer = new WBOITRenderer(&sceneDataRef, transferFunctionWindow);
        } else {
            std::string warningText =
                    std::string() + "The selected renderer \"" + RENDERING_MODE_NAMES[newRenderingMode] + "\" is not "
                    + "supported on this hardware due to the missing independentBlend physical device feature.";
            onUnsupportedRendererSelected(warningText, sceneDataRef, newRenderingMode, newLineRenderer);
        }
    } else if (newRenderingMode == RENDERING_MODE_DEPTH_PEELING) {
        newLineRenderer = new DepthPeelingRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_ATOMIC_LOOP_64) {
        if (sgl::AppSettings::get()->getPrimaryDevice()->getPhysicalDeviceShaderAtomicInt64Features().shaderBufferInt64Atomics
                || sgl::AppSettings::get()->getPrimaryDevice()->getPhysicalDeviceVulkan12Features().shaderBufferInt64Atomics) {
            newLineRenderer = new AtomicLoop64Renderer(&sceneDataRef, transferFunctionWindow);
        } else {
            std::string warningText =
                    std::string() + "The selected renderer \"" + RENDERING_MODE_NAMES[newRenderingMode] + "\" is not "
                    + "supported on this hardware due to missing 64-bit integer atomics support.";
            onUnsupportedRendererSelected(warningText, sceneDataRef, newRenderingMode, newLineRenderer);
        }
    } else if (newRenderingMode == RENDERING_MODE_VULKAN_RAY_TRACER) {
        if (sgl::AppSettings::get()->getPrimaryDevice()->getRayTracingPipelineSupported()) {
            newLineRenderer = new VulkanRayTracer(&sceneDataRef, transferFunctionWindow);
        } else {
            std::string warningText =
                    std::string() + "The selected renderer \"" + RENDERING_MODE_NAMES[newRenderingMode] + "\" is not "
                    + "supported on this hardware due to missing Vulkan ray pipelines.";
            onUnsupportedRendererSelected(warningText, sceneDataRef, newRenderingMode, newLineRenderer);
        }
    } else if (newRenderingMode == RENDERING_MODE_VOXEL_RAY_CASTING) {
        newLineRenderer = new VoxelRayCastingRenderer(&sceneDataRef, transferFunctionWindow);
    }
#ifdef USE_OSPRAY
    else if (newRenderingMode == RENDERING_MODE_OSPRAY_RAY_TRACER) {
        newLineRenderer = new OsprayRenderer(&sceneDataRef, transferFunctionWindow);
    }
#endif
    else if (newRenderingMode == RENDERING_MODE_LINE_DENSITY_MAP_RENDERER) {
        newLineRenderer = new LineDensityMapRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_VOLUMETRIC_PATH_TRACER) {
        newLineRenderer = new VolumetricPathTracingRenderer(&sceneDataRef, transferFunctionWindow);
    } else if (newRenderingMode == RENDERING_MODE_SPHERICAL_HEAT_MAP_RENDERER) {
        newLineRenderer = new SphericalHeatMapRenderer(&sceneDataRef, transferFunctionWindow);
    } else {
        int idx = std::clamp(int(newRenderingMode), 0, IM_ARRAYSIZE(RENDERING_MODE_NAMES) - 1);
        std::string warningText =
                std::string() + "The selected renderer \"" + RENDERING_MODE_NAMES[idx] + "\" is not "
                + "supported in this build configuration or incompatible with this system.";
        onUnsupportedRendererSelected(warningText, sceneDataRef, newRenderingMode, newLineRenderer);
    }

    newLineRenderer->initialize();
    newLineRenderer->setUseLinearRGB(useLinearRGB);
    newLineRenderer->setFileDialogInstance(fileDialogInstance);

    if (*sceneDataRef.sceneTexture) {
        newLineRenderer->onResolutionChanged();
    }

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

void MainApp::onUnsupportedRendererSelected(
        const std::string& warningText,
        SceneData& sceneDataRef, RenderingMode& newRenderingMode, LineRenderer*& newLineRenderer) {
    sgl::Logfile::get()->writeWarning(
            "Warning in MainApp::setRenderer: " + warningText, false);
    auto handle = sgl::dialog::openMessageBox(
            "Unsupported Renderer", warningText, sgl::dialog::Icon::WARNING);
    nonBlockingMsgBoxHandles.push_back(handle);
    newRenderingMode = RENDERING_MODE_OPAQUE;
    newLineRenderer = new OpaqueLineRenderer(&sceneDataRef, transferFunctionWindow);
}

void MainApp::resolutionChanged(sgl::EventPtr event) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    SciVisApp::resolutionChanged(event);
    if (!useDockSpaceMode) {
        auto* window = sgl::AppSettings::get()->getMainWindow();
        viewportWidth = uint32_t(window->getWidth());
        viewportHeight = uint32_t(window->getHeight());
        if (lineRenderer != nullptr) {
            lineRenderer->onResolutionChanged();
        }
    }
}

void MainApp::updateColorSpaceMode() {
    SciVisApp::updateColorSpaceMode();
    transferFunctionWindow.setUseLinearRGB(useLinearRGB);
    if (lineData) {
        lineData->setUseLinearRGB(useLinearRGB);
    }
    if (useDockSpaceMode) {
        for (DataViewPtr& dataView : dataViews) {
            dataView->useLinearRGB = useLinearRGB;
            dataView->viewportWidth = 0;
            dataView->viewportHeight = 0;
            if (dataView->lineRenderer) {
                dataView->lineRenderer->setUseLinearRGB(useLinearRGB);
            }
        }
    } else {
        if (lineRenderer) {
            lineRenderer->setUseLinearRGB(useLinearRGB);
        }
    }
}

void MainApp::render() {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    if (usePerformanceMeasurementMode) {
        performanceMeasurer->beginRenderFunction();
    }

    if (scheduledRecreateSceneFramebuffer) {
        device->waitIdle();
        sgl::vk::Swapchain* swapchain = sgl::AppSettings::get()->getSwapchain();
        createSceneFramebuffer();
        if (swapchain && sgl::AppSettings::get()->getUseGUI()) {
            sgl::ImGuiWrapper::get()->setVkRenderTarget(compositedTextureVk->getImageView());
            sgl::ImGuiWrapper::get()->onResolutionChanged();
        }
        if (videoWriter) {
            videoWriter->onSwapchainRecreated();
        }
        if (!useDockSpaceMode && lineRenderer) {
            lineRenderer->onResolutionChanged();
        }
        scheduledRecreateSceneFramebuffer = false;
    }

    if (visualizeSeedingProcess) {
        if (useDockSpaceMode) {
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_OPAQUE
                        || dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER) {
                    dataView->lineRenderer->notifyReRenderTriggeredExternally();
                    dataView->reRender = true;
                }
            }
        }
    }

    SciVisApp::preRender();
    if (useDockSpaceMode) {
        for (DataViewPtr& dataView : dataViews) {
            dataView->saveScreenshotDataIfAvailable();
        }
    }

    if (!useDockSpaceMode) {
        prepareVisualizationPipeline();

        componentOtherThanRendererNeedsReRender = reRender;
        if (lineData != nullptr) {
            bool lineDataNeedsReRender = lineData->needsReRender();
            reRender = reRender || lineDataNeedsReRender;
            componentOtherThanRendererNeedsReRender = componentOtherThanRendererNeedsReRender || lineDataNeedsReRender;
        }
    }

    if (!useDockSpaceMode) {
        if (lineRenderer != nullptr) {
            reRender |= lineRenderer->needsReRender();
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

    if (useDockSpaceMode && !dataViews.empty() && !uiOnScreenshot && recording && !isFirstRecordingFrame) {
        auto dataView = dataViews.at(0);
        if (dataView->viewportWidth > 0 && dataView->viewportHeight > 0) {
            rendererVk->transitionImageLayout(
                    dataView->compositedTextureVk->getImage(),
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            videoWriter->pushFramebufferImage(dataView->compositedTextureVk->getImage());
            rendererVk->transitionImageLayout(
                    dataView->compositedTextureVk->getImage(),
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        }
    }
}

void MainApp::renderGui() {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    focusedWindowIndex = -1;
    mouseHoverWindowIndex = -1;

#ifdef SGL_INPUT_API_V2
    if (sgl::Keyboard->keyPressed(ImGuiKey_O) && sgl::Keyboard->getModifier(ImGuiKey_ModCtrl)) {
        openFileDialog();
    }
#else
    if (sgl::Keyboard->keyPressed(SDLK_o) && (sgl::Keyboard->getModifier() & (KMOD_LCTRL | KMOD_RCTRL)) != 0) {
        openFileDialog();
    }
#endif

    if (IGFD_DisplayDialog(
            fileDialogInstance,
            "ChooseDataSetFile", ImGuiWindowFlags_NoCollapse,
            sgl::ImGuiWrapper::get()->getScaleDependentSize(1000, 580),
            ImVec2(FLT_MAX, FLT_MAX))) {
        if (IGFD_IsOk(fileDialogInstance)) {
            std::string filePathName = IGFD_GetFilePathNameString(fileDialogInstance);
            std::string filePath = IGFD_GetCurrentPathString(fileDialogInstance);
            std::string filter = IGFD_GetCurrentFilterString(fileDialogInstance);
            std::string userDatas;
            if (IGFD_GetUserDatas(fileDialogInstance)) {
                userDatas = std::string((const char*)IGFD_GetUserDatas(fileDialogInstance));
            }
            auto selection = IGFD_GetSelection(fileDialogInstance);

            // Is this line data set or a volume data file for the scattering line tracer?
            std::string currentPath = IGFD_GetCurrentPathString(fileDialogInstance);
            std::string filename = currentPath;
            if (!filename.empty() && filename.back() != '/' && filename.back() != '\\') {
                filename += "/";
            }
            if (selection.count != 0) {
                filename += selection.table[0].fileName;
            }
            IGFD_Selection_DestroyContent(&selection);

            loadDataFromFile(filename);
        }
        IGFD_CloseDialog(fileDialogInstance);
    }

    if (useDockSpaceMode) {
        if (isFirstFrame && dataViews.size() == 1) {
            if (dataViews.front()->renderingMode == RENDERING_MODE_NONE) {
                initializeFirstDataView();
            }
        }

        static bool isProgramStartup = true;
        ImGuiID dockSpaceId = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
        if (isProgramStartup) {
            ImGuiDockNode* centralNode = ImGui::DockBuilderGetNode(dockSpaceId);
            if (centralNode->IsEmpty()) {
                auto* window = sgl::AppSettings::get()->getMainWindow();
                //const ImVec2 dockSpaceSize = ImGui::GetMainViewport()->Size;//ImGui::GetContentRegionAvail();
                const ImVec2 dockSpaceSize(float(window->getWidth()), float(window->getHeight()));
                ImGui::DockBuilderSetNodeSize(dockSpaceId, dockSpaceSize);

                ImGuiID dockLeftId, dockMainId;
                ImGui::DockBuilderSplitNode(
                        dockSpaceId, ImGuiDir_Left, 0.29f, &dockLeftId, &dockMainId);
                ImGui::DockBuilderDockWindow("Opaque Renderer (1)###data_view_0", dockMainId);

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
        }

        renderGuiMenuBar();

        if (showPropertyEditor) {
            renderGuiPropertyEditorWindow();
        }

        prepareVisualizationPipeline();

        componentOtherThanRendererNeedsReRender = reRender;
        if (lineData != nullptr) {
            bool lineDataNeedsReRender = lineData->needsReRender();
            reRender = reRender || lineDataNeedsReRender;
            componentOtherThanRendererNeedsReRender = componentOtherThanRendererNeedsReRender || lineDataNeedsReRender;
        }

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
                                //dataView->resize(1, 1); // Use arbitrary size for initialization.
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
                // SetNextWindowFocus doesn't really seem to work... Needs more investigation.
                //if (isFirstFrame && i == 0) {
                //    ImGui::SetNextWindowFocus();
                //    focusedWindowIndex = i;
                //}
                if (ImGui::Begin(windowName.c_str(), &isViewOpen)) {
                    if (ImGui::IsWindowFocused()) {
                        focusedWindowIndex = i;
                    }
                    sgl::ImGuiWrapper::get()->setWindowViewport(i, ImGui::GetWindowViewport());
                    sgl::ImGuiWrapper::get()->setWindowPosAndSize(i, ImGui::GetWindowPos(), ImGui::GetWindowSize());

                    ImVec2 sizeContent = ImGui::GetContentRegionAvail();
                    if (useFixedSizeViewport) {
                        sizeContent = ImVec2(float(fixedViewportSize.x), float(fixedViewportSize.y));
                    }
                    if (int(sizeContent.x) != int(dataView->viewportWidth)
                            || int(sizeContent.y) != int(dataView->viewportHeight)) {
                        rendererVk->getDevice()->waitIdle();
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
                        //if (dataView->lineRenderer && dataView->lineRenderer->getRenderingMode() == ) {
                        if (dataView->lineRenderer) {
                            dataView->updateCameraMode();
                        }

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
                            std::string screenshotFilename =
                                    saveDirectoryScreenshots + saveFilenameScreenshots
                                    + "_" + sgl::toString(screenshotNumber);
                            if (dataViews.size() > 1) {
                                screenshotFilename += "_view" + sgl::toString(i);
                            }
                            screenshotFilename += ".png";

                            dataView->screenshotReadbackHelper->setScreenshotTransparentBackground(
                                    screenshotTransparentBackground);
                            dataView->saveScreenshot(screenshotFilename);
                            screenshot = false;

                            printNow = false;
                            screenshot = true;
                        }

                        if (isViewOpen) {
                            ImTextureID textureId = dataView->getImGuiTextureId();
                            ImGui::Image(
                                    textureId, sizeContent,
                                    ImVec2(0, 0), ImVec2(1, 1));
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
        if (isFirstFrame) {
            isFirstFrame = false;
        }
        if (!uiOnScreenshot && screenshot) {
            screenshot = false;
            screenshotNumber++;
        }
        reRender = false;
    } else {
        if (showPropertyEditor) {
            renderGuiPropertyEditorWindow();
        }

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
        videoWriter->setRenderer(rendererVk);
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

    // TODO: Remove option?
    /*newDockSpaceMode = useDockSpaceMode;
    if (propertyEditor.addCheckbox("Use Docking Mode", &newDockSpaceMode)) {
        scheduledDockSpaceModeChange = true;
    }*/

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

    if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES && renderingMode == RENDERING_MODE_OPAQUE) {
        if (useDockSpaceMode) {
            bool rendererSupportsVisualizingSeedingProcess = false;
            for (DataViewPtr& dataView : dataViews) {
                if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_OPAQUE
                        || dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER) {
                    rendererSupportsVisualizingSeedingProcess = true;
                }
            }

            if (rendererSupportsVisualizingSeedingProcess) {
                if (propertyEditor.addCheckbox("Visualize Seeding Process", &visualizeSeedingProcess)) {
                    LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
                    lineDataStress->setShallRenderSeedingProcess(visualizeSeedingProcess);
                    for (DataViewPtr& dataView : dataViews) {
                        if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_OPAQUE) {
                            OpaqueLineRenderer* opaqueLineRenderer = static_cast<OpaqueLineRenderer*>(
                                    dataView->lineRenderer);
                            opaqueLineRenderer->setVisualizeSeedingProcess(true);
                        } else if (dataView->lineRenderer->getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER) {
                            VulkanRayTracer* vulkanRayTracer = static_cast<VulkanRayTracer*>(dataView->lineRenderer);
                            vulkanRayTracer->setVisualizeSeedingProcess(true);
                        }
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
            if (renderingMode == RENDERING_MODE_OPAQUE) {
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
    DataViewPtr dataView = std::make_shared<DataView>(&sceneData);
    dataView->useLinearRGB = useLinearRGB;
    dataView->clearColor = clearColor;
    dataViews.push_back(dataView);
}

void MainApp::initializeFirstDataView() {
    DataViewPtr dataView = dataViews.back();
    dataView->renderingMode = RENDERING_MODE_OPAQUE;
    //dataView->renderingMode = RENDERING_MODE_WBOIT;
    //dataView->resize(1, 1); // Use arbitrary size for initialization.
    setRenderer(
            dataViews[0]->sceneData, dataViews[0]->oldRenderingMode,
            dataViews[0]->renderingMode, dataViews[0]->lineRenderer, 0);
    dataView->updateCameraMode();
    prepareVisualizationPipeline();
}

void MainApp::openFileDialog() {
    selectedDataSetIndex = 0;
    if (fileDialogDirectory.empty() || !sgl::FileUtils::get()->directoryExists(fileDialogDirectory)) {
        fileDialogDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
        if (!sgl::FileUtils::get()->exists(fileDialogDirectory)) {
            fileDialogDirectory = sgl::AppSettings::get()->getDataDirectory();
        }
    }
    IGFD_OpenModal(
            fileDialogInstance,
            "ChooseDataSetFile", "Choose a File",
            ".*,.obj,.dat,.binlines,.nc,.vtk,.vti,.vts,.vtr,.bin,.stress,.carti,.am,.field,.grib,.grb,.raw,.xyz,.nvdb,"
            ".bobj,.stl",
            fileDialogDirectory.c_str(),
            "", 1, nullptr,
            ImGuiFileDialogFlags_None);
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
                        if (selectedDataSetIndex == 1) {
                            streamlineTracingRequester->setShowWindow(true);
                        } else if (selectedDataSetIndex == 2) {
                            stressLineTracingRequester->setShowWindow(true);
                        } else if (selectedDataSetIndex == 3) {
                            scatteringLineTracingRequester->setShowWindow(true);
                        }
                    }
                }

                if (dataSetInformationRoot) {
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
#ifdef USE_PYTHON
            if (ImGui::MenuItem("Replay Widget", nullptr, replayWidget.getShowWindow())) {
                replayWidget.setShowWindow(!replayWidget.getShowWindow());
            }
#endif
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Tools")) {
            if (ImGui::MenuItem("Print Camera State")) {
                std::cout << "Position: (" << camera->getPosition().x << ", " << camera->getPosition().y
                          << ", " << camera->getPosition().z << ")" << std::endl;
                std::cout << "Look At: (" << camera->getLookAtLocation().x << ", " << camera->getLookAtLocation().y
                          << ", " << camera->getLookAtLocation().z << ")" << std::endl;
                std::cout << "Yaw: " << camera->getYaw() << std::endl;
                std::cout << "Pitch: " << camera->getPitch() << std::endl;
                std::cout << "FoVy: " << (camera->getFOVy() / sgl::PI * 180.0f) << std::endl;
            }

            deviceSelector->renderGuiMenu();

            ImGui::EndMenu();
        }

        bool isRendererComputationRunning = false;
        for (DataViewPtr& dataView : dataViews) {
            isRendererComputationRunning =
                    isRendererComputationRunning
                    || (dataView->lineRenderer && dataView->lineRenderer->getIsComputationRunning());
            if (isRendererComputationRunning) {
                break;
            }
        }

        if (lineDataRequester.getIsProcessingRequest()
                || streamlineTracingRequester->getIsProcessingRequest()
                || stressLineTracingRequester->getIsProcessingRequest()
                || scatteringLineTracingRequester->getIsProcessingRequest()
                || isRendererComputationRunning) {
            float windowContentRegionWidth =
                    ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x;
            ImGui::SetCursorPosX(windowContentRegionWidth - ImGui::GetTextLineHeight());
            ImGui::ProgressSpinner(
                    "##progress-spinner", -1.0f, -1.0f, 4.0f,
                    ImVec4(0.1f, 0.5f, 1.0f, 1.0f));
        }

        ImGui::EndMainMenuBar();
    }

    deviceSelector->renderGuiDialog();
    if (deviceSelector->getShallRestartApp()) {
        quit();
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
            ImGui::InputText("##datasetfilenamelabel", &customDataSetFileName);
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
                rendererVk->getDevice()->waitIdle();
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
            std::string buttonName = "X###x_view" + std::to_string(i);
            if (ImGui::Button(buttonName.c_str())) {
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
                            prepareVisualizationPipeline();
                            dataView->updateCameraMode();
                            reRender = true;
                        }
                    }
                    propertyEditor.addEndCombo();
                }

                if (dataViews.size() > 1
                        && propertyEditor.addCheckbox("Sync with Global Camera", &dataView->syncWithParentCamera)) {
                    dataView->reRender = true;
                    if (dataView->lineRenderer) {
                        dataView->lineRenderer->notifyReRenderTriggeredExternally();
                    }
                }

                if (dataView->lineRenderer) {
                    dataView->lineRenderer->renderGuiPropertyEditorNodesParent(propertyEditor);
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
                lineRenderer->renderGuiPropertyEditorNodesParent(propertyEditor);
                propertyEditor.endNode();
            }
        }
    }
}

void MainApp::update(float dt) {
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    sgl::SciVisApp::update(dt);

    for (int i = 0; i < int(nonBlockingMsgBoxHandles.size()); i++) {
        auto& handle = nonBlockingMsgBoxHandles.at(i);
        if (handle->ready(0)) {
            nonBlockingMsgBoxHandles.erase(nonBlockingMsgBoxHandles.begin() + i);
            i--;
        }
    }

    // TODO: Remove option?
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
            if (std::abs(camera->getFOVy() - replayWidget.getCameraFovy()) > 1e-6f) {
                camera->setFOVy(replayWidget.getCameraFovy());
                fovDegree = camera->getFOVy() / sgl::PI * 180.0f;
            }
            if (camera->getLookAtLocation() != replayWidget.getLookAtLocation()) {
                camera->setLookAtLocation(replayWidget.getLookAtLocation());
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
        hasMoved();

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
                if (dataView->lineRenderer && !dataView->lineRenderer->getUseCamera3d()) {
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

#ifdef SGL_INPUT_API_V2
        if (sgl::Keyboard->isKeyDown(ImGuiKey_U)) {
            transferFunctionWindow.setShowWindow(showSettingsWindow);
        }
#else
        if (sgl::Keyboard->isKeyDown(SDLK_u)) {
            transferFunctionWindow.setShowWindow(showSettingsWindow);
        }
#endif
    }

    if (!io.WantCaptureMouse || mouseHoverWindowIndex != -1) {
        if (useDockSpaceMode) {
            for (int i = 0; i < int(dataViews.size()); i++) {
                DataViewPtr& dataView = dataViews.at(i);
                if (i != mouseHoverWindowIndex) {
                    continue;
                }
                // 3D camera movement disabled for certain renderers.
                if (dataView->lineRenderer && !dataView->lineRenderer->getUseCamera3d()) {
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
                    dataView->syncCamera();
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

void MainApp::loadDataFromFile(const std::string& filename) {
    fileDialogDirectory = sgl::FileUtils::get()->getPathToFile(filename);

    std::string filenameLower = sgl::toLowerCopy(filename);
    bool isObjFile = sgl::endsWith(filenameLower, ".obj");
    bool isDatFile = sgl::endsWith(filenameLower, ".dat");
    bool isNcFile = sgl::endsWith(filenameLower, ".nc");

    /*
     * The program supports loading flow lines or triangle meshes from .dat files.
     */
    bool isObjTriangleMeshFile = false;
    if (isObjFile) {
        if (sgl::FileUtils::get()->exists(filename) && !sgl::FileUtils::get()->isDirectory(filename)) {
            std::ifstream objFile(filename);
            std::string lineString;
            std::getline(objFile, lineString);
            if (lineString.find("# Blender") != std::string::npos) {
                isObjTriangleMeshFile = true;
            }
            while (std::getline(objFile, lineString)) {
                if (sgl::startsWith(lineString, "f ")) {
                    isObjTriangleMeshFile = true;
                    break;
                }
                if (sgl::startsWith(lineString, "l ")) {
                    isObjTriangleMeshFile = false;
                    break;
                }
            }
            objFile.close();
        }
    }

    /*
     * The program supports two types of .dat file:
     * - .dat files containing a volume description.
     * - .dat files containing PSLs (DATA_SET_TYPE_STRESS_LINES).
     * We try to use a good heuristic for determining the right type.
     */
    bool isDatVolumeFile = false;
    if (isDatFile) {
        if (sgl::FileUtils::get()->exists(filename)) {
            // If the .dat file is > 100KiB, there is no way it can be a volume descriptor file.
            size_t fileSize = sgl::FileUtils::get()->getFileSizeInBytes(filename);
            if (fileSize < 100 * 1024) {
                // Loading the < 100kiB file should be cheap. Load it and check the type.
                uint8_t* bufferDat = nullptr;
                size_t lengthDat = 0;
                bool loadedDat = sgl::loadFileFromSource(
                        filename, bufferDat, lengthDat, false);
                if (!loadedDat) {
                    sgl::Logfile::get()->throwError(
                            "Error in MainApp::loadDataFromFile: Couldn't open file \"" + filename + "\".");
                }
                char* fileBuffer = reinterpret_cast<char*>(bufferDat);
                for (size_t charPtr = 0; charPtr < lengthDat; charPtr++) {
                    char currentChar = fileBuffer[charPtr];
                    if (currentChar == '\n' || currentChar == '\r') {
                        break;
                    }
                    // ':' is the key-value delimiter in .dat volume descriptor files.
                    if (currentChar == ':') {
                        isDatVolumeFile = true;
                        break;
                    }
                }
                delete[] bufferDat;
            }
        }
    }

    /*
     * The program supports two types of .dat file:
     * - .dat files containing a volume description.
     * - .dat files containing PSLs (DATA_SET_TYPE_STRESS_LINES).
     * We try to use a good heuristic for determining the right type.
     */
    bool isNcVolumeFile = false;
    if (isNcFile) {
        isNcVolumeFile = !getNetCdfFileStoresTrajectories(filename);
    }

    if (sgl::endsWith(filenameLower, ".vtk")
            || sgl::endsWith(filenameLower, ".vti")
            || sgl::endsWith(filenameLower, ".vts")
            || sgl::endsWith(filenameLower, ".vtr")
            || (isNcFile && isNcVolumeFile)
            || sgl::endsWith(filenameLower, ".am")
            || sgl::endsWith(filenameLower, ".bin")
            || sgl::endsWith(filenameLower, ".field")
#ifdef USE_ECCODES
            || sgl::endsWith(filenameLower, ".grib")
            || sgl::endsWith(filenameLower, ".grb")
#endif
            || (isDatFile && isDatVolumeFile)
            || sgl::endsWith(filenameLower, ".raw")) {
        selectedDataSetIndex = 1;
        streamlineTracingRequester->setDatasetFilename(filename);
        streamlineTracingRequester->setShowWindow(true);
    } else if (sgl::endsWith(filenameLower, ".stress")
               || sgl::endsWith(filenameLower, ".carti")) {
        selectedDataSetIndex = 2;
        stressLineTracingRequester->setDatasetFilename(filename);
        stressLineTracingRequester->setShowWindow(true);
    } else if (sgl::endsWith(filenameLower, ".xyz")
               || sgl::endsWith(filenameLower, ".nvdb")) {
        selectedDataSetIndex = 3;
        scatteringLineTracingRequester->setDatasetFilename(filename);
        scatteringLineTracingRequester->setShowWindow(true);
    } else {
        selectedDataSetIndex = 0;
        if ((isObjFile && !isObjTriangleMeshFile)
            || sgl::endsWith(filenameLower, ".binlines") || isNcFile) {
            dataSetType = DATA_SET_TYPE_FLOW_LINES;
        } else if (isDatFile) {
            dataSetType = DATA_SET_TYPE_STRESS_LINES;
        } else if ((isObjFile && isObjTriangleMeshFile)
                   || sgl::endsWith(filenameLower, ".bobj")
                   || sgl::endsWith(filenameLower, ".stl")) {
            dataSetType = DATA_SET_TYPE_TRIANGLE_MESH;
        } else {
            sgl::Logfile::get()->writeError("The selected file name has an unknown extension.");
        }
        customDataSetFileName = filename;
        loadLineDataSet(getSelectedLineDataSetFilenames());
    }
}

void MainApp::onFileDropped(const std::string& droppedFileName) {
    loadDataFromFile(droppedFileName);
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
    } else if (dataSetType == DATA_SET_TYPE_TRIANGLE_MESH) {
        TriangleMeshData* triangleMeshData = new TriangleMeshData(transferFunctionWindow);
        lineData = LineDataPtr(triangleMeshData);
    } else {
        sgl::Logfile::get()->writeError("Error in MainApp::loadLineDataSet: Invalid data set type.");
        return;
    }
    lineData->setFileDialogInstance(fileDialogInstance);

    if (blockingDataLoading) {
        bool dataLoaded = lineData->loadFromFile(fileNames, selectedDataSetInformation, transformationMatrixPtr);
        sgl::ColorLegendWidget::resetStandardSize();

        if (dataLoaded) {
            if (selectedDataSetInformation.hasCustomLineWidth) {
                LineRenderer::setLineWidth(selectedDataSetInformation.lineWidth);
            }

            this->lineData = lineData;
            lineData->onMainThreadDataInit();
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
        lineData->onMainThreadDataInit();
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
#ifdef TRACY_PROFILE_TRACING
    ZoneScoped;
#endif

    if (useDockSpaceMode) {
        if (lineData && !dataViews.empty()) {
            bool isPreviousNodeDirty = lineData->isDirty();
            filterData(isPreviousNodeDirty);
            if (isPreviousNodeDirty) {
                lineData->setTriangleRepresentationDirty();
                lineData->setIsDirty(true);
            }
            bool isTriangleRepresentationDirty = lineData->isTriangleRepresentationDirty();
            for (DataViewPtr& dataView : dataViews) {
                if (!dataView->lineRenderer) {
                    continue;
                }
                bool isTriangleRepresentationDirtyLocal =
                        isTriangleRepresentationDirty && dataView->lineRenderer->getIsTriangleRepresentationUsed();
                if (dataView->lineRenderer->isDirty() || isPreviousNodeDirty || isTriangleRepresentationDirtyLocal) {
                    rendererVk->getDevice()->waitIdle();
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
                rendererVk->getDevice()->waitIdle();
                lineRenderer->setLineData(lineData, newMeshLoaded);
            }
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
