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

#include <Utils/SciVis/SciVisApp.hpp>
#include <Graphics/Shader/Shader.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>

#include "Loaders/DataSetList.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "LineData/LineDataRequester.hpp"
#include "LineData/Filters/LineFilter.hpp"
#include "Renderers/SceneData.hpp"
#include "Renderers/AmbientOcclusion/AmbientOcclusionBaker.hpp"

#ifdef USE_PYTHON
#include "Widgets/ReplayWidget.hpp"
#endif

namespace sgl { namespace dialog {
class MsgBoxHandle;
typedef std::shared_ptr<MsgBoxHandle> MsgBoxHandlePtr;
}}

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;

class LineRenderer;
class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;
class DataView;
typedef std::shared_ptr<DataView> DataViewPtr;
class StreamlineTracingRequester;
class StressLineTracingRequester;
class ScatteringLineTracingRequester;

class MainApp : public sgl::SciVisApp {
public:
    /**
     * @param supportsRaytracing Whether raytracing via OpenGL-Vulkan interoperability is supported.
     */
    MainApp();
    ~MainApp() override;
    void render() override;
    void update(float dt) override;
    void resolutionChanged(sgl::EventPtr event) override;

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState);

protected:
    void renderGuiGeneralSettingsPropertyEditor() override;

private:
    /// Renders the GUI of the scene settings and all filters and renderers.
    void renderGui() override;
    /// Update the color space (linear RGB vs. sRGB).
    void updateColorSpaceMode() override;
    /// Called when the camera moved.
    void hasMoved() override;
    /// Callback when the camera was reset.
    void onCameraReset() override;

    void scheduleRecreateSceneFramebuffer();
    bool scheduledRecreateSceneFramebuffer = false;
    bool componentOtherThanRendererNeedsReRender = false;

    // Dock space mode.
    void renderGuiMenuBar();
    void renderGuiPropertyEditorBegin() override;
    void renderGuiPropertyEditorCustomNodes() override;
    void addNewDataView();
    void initializeFirstDataView();
    bool scheduledDockSpaceModeChange = false;
    bool newDockSpaceMode = false;
    int focusedWindowIndex = -1;
    int mouseHoverWindowIndex = -1;
    std::vector<DataViewPtr> dataViews;
    int hasMovedIndex = -1;
    bool isFirstFrame = true;

    /// Scene data (e.g., camera, main framebuffer, ...).
    uint32_t viewportWidth = 0;
    uint32_t viewportHeight = 0;
    SceneData sceneData;

    // This setting lets all data views use the same viewport resolution.
    bool useFixedSizeViewport = false;
    glm::ivec2 fixedViewportSizeEdit{ 2186, 1358 };
    glm::ivec2 fixedViewportSize{ 2186, 1358 };

    /// Scene data used in user interface.
    RenderingMode renderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;
    RenderingMode oldRenderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;

    // Data set GUI information.
    void loadAvailableDataSetInformation();
    std::vector<std::string> getSelectedLineDataSetFilenames();
    void openFileDialog();
    DataSetInformationPtr dataSetInformationRoot;
    std::vector<DataSetInformationPtr> dataSetInformationList; //< List of all leaves.
    std::vector<std::string> dataSetNames; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int selectedDataSetIndex = 0; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int currentlyLoadedDataSetIndex = -1;
    std::string customDataSetFileName;
    DataSetType dataSetType = DATA_SET_TYPE_NONE;
    bool visualizeSeedingProcess = false; ///< Only for stress line data.
    const float TIME_PER_SEED_POINT = 0.25f;
    ImGuiFileDialog* fileDialogInstance = nullptr;
    std::vector<sgl::dialog::MsgBoxHandlePtr> nonBlockingMsgBoxHandles;

    // Coloring & filtering dependent on importance criteria.
    sgl::TransferFunctionWindow transferFunctionWindow;

    // For making performance measurements.
    AutomaticPerformanceMeasurer *performanceMeasurer = nullptr;
    InternalState lastState;
    bool firstState = true;
    bool usesNewState = true;

#ifdef USE_PYTHON
    ReplayWidget replayWidget;
    bool replayWidgetRunning = false;
    bool realTimeReplayUpdates = false;
    bool updateTransferFunctionRange = false;
    glm::vec2 transferFunctionRange{};
#endif


    /// --- Visualization pipeline ---

    /// Loads line data from a file.
    void loadLineDataSet(const std::vector<std::string>& fileName, bool blockingDataLoading = false);
    /// Checks if an asynchronous loading request was finished.
    void checkLoadingRequestFinished();
    /// Reload the currently loaded data set.
    void reloadDataSet() override;
    /// Prepares the visualization pipeline for rendering.
    void prepareVisualizationPipeline();
    /// Returns the filtered mesh that is passed to the renderers.
    void filterData(bool& isDirty);
    /// Sets the used renderers.
    void setRenderer(
            SceneData& sceneDataRef, RenderingMode& oldRenderingMode, RenderingMode& newRenderingMode,
            LineRenderer*& newLineRenderer, int dataViewIndex);

    /// A list of filters that are applied sequentially on the data.
    std::vector<LineFilter*> dataFilters;

    LineRenderer* lineRenderer = nullptr;
    LineDataPtr lineData;
    LineDataRequester lineDataRequester;
    bool newMeshLoaded = true;
    sgl::AABB3 modelBoundingBox;
    void* zeromqContext = nullptr;
    StreamlineTracingRequester* streamlineTracingRequester = nullptr;
    StressLineTracingRequester* stressLineTracingRequester = nullptr;
    ScatteringLineTracingRequester* scatteringLineTracingRequester = nullptr;
    const int NUM_MANUAL_LOADERS = 4;
    DataSetInformation stressLineTracerDataSetInformation;

    bool optixInitialized = false;
};

#endif // MAINAPP_HPP