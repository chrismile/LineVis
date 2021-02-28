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
#include "LineData/Stress/StressLineTracingRequester.hpp"
#include "Renderers/SceneData.hpp"

class LineRenderer;
class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class MainApp : public sgl::SciVisApp {
public:
    MainApp();
    ~MainApp();
    void render();
    void update(float dt);
    void resolutionChanged(sgl::EventPtr event);

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState);

private:
    /// Renders the GUI of the scene settings and all filters and renderers.
    void renderGui();
    /// Renders the GUI for selecting an input file.
    void renderFileSelectionSettingsGui();
    /// Render the scene settings GUI, e.g. for setting the background clear color.
    void renderSceneSettingsGui();
    /// Update the color space (linear RGB vs. sRGB).
    void updateColorSpaceMode();
    // Called when the camera moved.
    void hasMoved();

    /// Scene data (e.g., camera, main framebuffer, ...).
    SceneData sceneData;

    /// Scene data used in user interface.
    RenderingMode renderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;

    // Data set GUI information.
    void loadAvailableDataSetInformation();
    std::vector<std::string> getSelectedMeshFilenames();
    std::vector<DataSetInformation> dataSetInformation;
    std::vector<std::string> dataSetNames; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int selectedDataSetIndex = 0; //< Contains "Local file..." at beginning, thus starts actually at 1.
    int currentlyLoadedDataSetIndex = -1;
    std::string customDataSetFileName;
    DataSetType dataSetType = DATA_SET_TYPE_NONE;

    // Coloring & filtering dependent on importance criteria.
    sgl::TransferFunctionWindow transferFunctionWindow;

    // For making performance measurements.
    AutomaticPerformanceMeasurer *performanceMeasurer = nullptr;
    InternalState lastState;
    bool firstState = true;
    bool usesNewState = true;


    /// --- Visualization pipeline ---

    /// Loads a hexahedral mesh from a file.
    void loadLineDataSet(const std::vector<std::string>& fileName);
    /// Checks if an asynchronous loading request was finished.
    void checkLoadingRequestFinished();
    /// Reload the currently loaded data set.
    virtual void reloadDataSet() override;
    /// Prepares the visualization pipeline for rendering.
    void prepareVisualizationPipeline();
    /// Returns the filtered mesh that is passed to the renderers.
    void filterData(bool& isDirty);
    /// Change the importance criterion used for coloring.
    void changeQualityMeasureType();
    /// Sets the used renderers.
    void setRenderer();

    /// A list of filters that are applied sequentially on the data.
    std::vector<LineFilter*> dataFilters;

    LineRenderer* lineRenderer = nullptr;
    LineDataPtr lineData;
    LineDataRequester lineDataRequester;
    bool newMeshLoaded = true;
    sgl::AABB3 modelBoundingBox;
    void* zeromqContext;
    StressLineTracingRequester* stressLineTracingRequester;
    DataSetInformation stressLineTracerDataSetInformation;
};

#endif // MAINAPP_HPP