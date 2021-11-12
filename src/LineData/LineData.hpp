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

#ifndef STRESSLINEVIS_LINEDATA_HPP
#define STRESSLINEVIS_LINEDATA_HPP

#include <memory>

#include <Graphics/Buffers/GeometryBuffer.hpp>
#include <Graphics/Shader/Shader.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/ColorLegendWidget.hpp>
#include "Utils/InternalState.hpp"
#include "Loaders/DataSetList.hpp"
#include "Loaders/TrajectoryFile.hpp"
#include "LineDataHeader.hpp"
#include "LineRenderData.hpp"

namespace sgl {
class PropertyEditor;
}

namespace sgl { namespace vk {
class RenderData;
typedef std::shared_ptr<RenderData> RenderDataPtr;
class BottomLevelAccelerationStructure;
typedef std::shared_ptr<BottomLevelAccelerationStructure> BottomLevelAccelerationStructurePtr;
class TopLevelAccelerationStructure;
typedef std::shared_ptr<TopLevelAccelerationStructure> TopLevelAccelerationStructurePtr;
}}

struct Trajectory;
typedef std::vector<Trajectory> Trajectories;

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class LineRenderer;

enum LineRasterizationRenderingTechnique {
    // Render screen-oriented bands, based on programmable vertex fetching or a geometry shader.
    LINE_RASTERIZATION_BAND_PROGRAMMABLE_FETCH,
    LINE_RASTERIZATION_BAND_GEOMETRY_SHADER,
    // Render tubes.
    LINE_RASTERIZATION_TUBE_GEOMETRY_SHADER,
    LINE_RASTERIZATION_TUBE_TRIANGLE_MESH
};
const char *const LINE_RASTERIZATION_ALL_TECHNIQUE_NAMES[] = {
        "Screen-aligned band (Prog. Fetch)", "Screen aligned band (GS)", "Tubes (GS)", "Tubes (Triangle Mesh)"
};
const int NUM_LINE_RASTERIZATION_TECHNIQUES_TOTAL =
        ((int)(sizeof(LINE_RASTERIZATION_ALL_TECHNIQUE_NAMES) / sizeof(*LINE_RASTERIZATION_ALL_TECHNIQUE_NAMES)));

class LineData {
public:
    LineData(sgl::TransferFunctionWindow &transferFunctionWindow, DataSetType dataSetType);
    virtual ~LineData();
    virtual void update(float dt) {}
    inline int getSelectedAttributeIndex() const { return selectedAttributeIndex; }
    void setSelectedAttributeIndex(int attributeIndex);
    void onTransferFunctionMapRebuilt();
    inline DataSetType getType() { return dataSetType; }
    /// Returns if the visualization mapping needs to be re-generated.
    inline bool isDirty() const { return dirty; }
    /// Returns if the triangle mesh visualization mapping needs to be re-generated.
    inline bool isTriangleRepresentationDirty() const { return triangleRepresentationDirty; }
    /// A renderer can signal that the triangle representation has changed.
    inline void setTriangleRepresentationDirty() { triangleRepresentationDirty = true; }
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { bool tmp = reRender; reRender = false; return tmp; }
    /// Do non-static settings that lead to a gather shader reload differ?
    virtual bool settingsDiffer(LineData* other) { return false; }
    /// Returns whether live visualization mapping updates can be used or whether the data set is too large.
    virtual bool getCanUseLiveUpdate(LineDataAccessType accessType) const;
    /**
     * A small data set has only a little amount of geometric data.
     * For large data sets, changing visualization mapping parameters triggering a rebuild of the internal
     * representation are not feasible to do in real-time and would result in considerable lag.
     */
    virtual bool getIsSmallDataSet() const=0;

    /// For changing performance measurement modes.
    virtual bool setNewState(const InternalState& newState) { return false; }
    /// For changing internal settings programmatically and not over the GUI.
    virtual bool setNewSettings(const SettingsMap& settings);

    /**
     * Load line data from the selected file(s).
     * @param fileNames The names of the files to load. More than one file makes primarily sense for, e.g., stress line
     * data with multiple principal stress directions.
     * @param dataSetInformation Metadata about the data set.
     * @param transformationMatrixPtr A transformation to apply to the loaded data (if any; can be nullptr).
     * @return Whether loading was successful.
     */
    virtual bool loadFromFile(
            const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
            glm::mat4* transformationMatrixPtr)=0;

    /**
     * Gets the file names that were used for loading.
     */
     inline const std::vector<std::string>& getFileNames() { return fileNames; }

    /**
     * Only has to be called by the main thread once after loading is finished.
     */
    virtual void recomputeHistogram()=0;

    // Statistics.
    inline const sgl::AABB3& getModelBoundingBox() { return modelBoundingBox; }
    inline const sgl::AABB3& getFocusBoundingBox() { return focusBoundingBox; }
    virtual size_t getNumAttributes()=0;
    virtual size_t getNumLines()=0;
    virtual size_t getNumLinePoints()=0;
    virtual size_t getNumLineSegments()=0;

    // Public interface for filtering trajectories.
    virtual void iterateOverTrajectories(std::function<void(const Trajectory&)> callback)=0;
    virtual void filterTrajectories(std::function<bool(const Trajectory&)> callback)=0;
    virtual void resetTrajectoryFilter()=0;

    // Get filtered line data (only containing points also shown when rendering).
    virtual Trajectories filterTrajectoryData()=0;
    virtual std::vector<std::vector<glm::vec3>> getFilteredLines()=0;

    // --- Retrieve data for rendering. Preferred way. ---
    virtual sgl::ShaderProgramPtr reloadGatherShader();
    virtual sgl::ShaderAttributesPtr getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader);
    virtual void setUniformGatherShaderData(sgl::ShaderProgramPtr& gatherShader);
    virtual void setUniformGatherShaderData_AllPasses();
    virtual void setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader);

    // --- Retrieve data for rendering. Only for renderers needing direct access! ---
    virtual TubeRenderData getTubeRenderData()=0;
    virtual TubeRenderDataProgrammableFetch getTubeRenderDataProgrammableFetch()=0;
    virtual TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization()=0;
    virtual BandRenderData getBandRenderData() { return {}; }
    virtual BandRenderData getTubeBandRenderData() { return {}; }

#ifdef USE_VULKAN_INTEROP
    // --- Retrieve data for rendering for Vulkan. ---
    virtual VulkanTubeTriangleRenderData getVulkanTubeTriangleRenderData(bool raytracing)=0;
    virtual VulkanTubeAabbRenderData getVulkanTubeAabbRenderData()=0;
    virtual VulkanHullTriangleRenderData getVulkanHullTriangleRenderData(bool raytracing);
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeTriangleTopLevelAS();
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeTriangleAndHullTopLevelAS();
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeAabbTopLevelAS();
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeAabbAndHullTopLevelAS();
    virtual std::map<std::string, std::string> getVulkanShaderPreprocessorDefines();
    virtual void setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData);
    virtual void updateVulkanUniformBuffers(sgl::vk::Renderer* renderer);
#endif

    // --- Retrieve triangle mesh on the CPU. ---
    virtual void getTriangleMesh(
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
            std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes)=0;
    virtual void getTriangleMesh(
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions)=0;

    // Retrieve simulation mesh outline (optional).
    inline bool hasSimulationMeshOutline() { return !simulationMeshOutlineVertexPositions.empty(); }
    sgl::ShaderProgramPtr reloadGatherShaderHull();
    sgl::ShaderAttributesPtr getGatherShaderAttributesHull(sgl::ShaderProgramPtr& gatherShader);
    void setUniformGatherShaderDataHull_Pass(sgl::ShaderProgramPtr& gatherShader);
    SimulationMeshOutlineRenderData getSimulationMeshOutlineRenderData();
    virtual bool shallRenderTransferFunctionWindow() { return true; }

    /**
     * For selecting options for the rendering technique (e.g., screen-oriented bands, tubes).
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiRenderer(bool isRasterizer);
    /**
     * For selecting options for the rendering technique (e.g., screen-oriented bands, tubes).
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiPropertyEditorNodesRenderer(sgl::PropertyEditor& propertyEditor, bool isRasterizer);
    /**
     * For line data settings.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiLineData(bool isRasterizer);
    /**
     * For changing other line rendering settings.
     */
    virtual bool renderGuiRenderingSettings();
    /**
     * For changing other line rendering settings.
     */
    virtual bool renderGuiRenderingSettingsPropertyEditor(sgl::PropertyEditor& propertyEditor);
    /**
     * For rendering a separate ImGui window.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiWindow(bool isRasterizer);
    /**
     * For rendering secondary ImGui windows (e.g., for transfer function widgets).
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiWindowSecondary(bool isRasterizer);
    /**
     * For rendering secondary, overlay ImGui windows.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiOverlay();
    /**
     * Renders the entries in the property editor.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor, bool isRasterizer);

    /// Certain GUI widgets might need the clear color.
    virtual void setClearColor(const sgl::Color& clearColor);
    /// Whether to use linear RGB when rendering.
    virtual void setUseLinearRGB(bool useLinearRGB) {}
    /// Set current rendering mode (e.g. for making visible certain UI options only for certain renderers).
    virtual void setLineRenderer(LineRenderer* lineRenderer) { this->lineRenderer = lineRenderer; }
    virtual void setRenderingMode(RenderingMode renderingMode) { this->renderingMode = renderingMode; }
    inline bool getShallRenderSimulationMeshBoundary() { return shallRenderSimulationMeshBoundary; }
    inline const std::string& getLineDataWindowName() const { return lineDataWindowName; }


    enum LinePrimitiveMode {
        LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH,
        LINE_PRIMITIVES_RIBBON_GEOMETRY_SHADER,
        LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER,
        LINE_PRIMITIVES_BAND, //< Only for stress lines for now.
        LINE_PRIMITIVES_TUBE_BAND, //< Only for stress lines for now.
        LINE_PRIMITIVES_TRIANGLE_MESH //< Not supported so far.
    };
    inline LinePrimitiveMode getLinePrimitiveMode() { return linePrimitiveMode; }
    inline bool useBands() {
        return linePrimitiveMode == LINE_PRIMITIVES_BAND || linePrimitiveMode == LINE_PRIMITIVES_TUBE_BAND;
    }

    /// This function should be called by sub-classes before accessing internal rendering data.
    virtual void rebuildInternalRepresentationIfNecessary();

protected:
    void loadSimulationMeshOutlineFromFile(
            const std::string& simulationMeshFilename, const sgl::AABB3& oldAABB, glm::mat4* transformationMatrixPtr);
    virtual void recomputeColorLegend();

    ///< The maximum number of line points to be considered a small data set (important, e.g., for live UI updates).
    const size_t SMALL_DATASET_LINE_POINTS_MAX = 10000;

    DataSetType dataSetType;
    sgl::AABB3 modelBoundingBox;
    sgl::AABB3 focusBoundingBox;
    std::vector<std::string> fileNames;
    std::vector<std::string> attributeNames;
    std::vector<glm::vec2> minMaxAttributeValues;
    int selectedAttributeIndex = 0; ///< Selected attribute/importance criterion index.
    int selectedAttributeIndexUi = 0;
    bool dirty = false; ///< Should be set to true if the representation changed.
    bool triangleRepresentationDirty = false; ///< Should be set to true if the triangle mesh representation changed.
    bool reRender = false;
    sgl::TransferFunctionWindow& transferFunctionWindow;

    // Color legend widgets for different attributes.
    bool shallRenderColorLegendWidgets = true;
    std::vector<sgl::ColorLegendWidget> colorLegendWidgets;

    // Rendering settings.
    RenderingMode renderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;
    LineRenderer* lineRenderer = nullptr;
    static LinePrimitiveMode linePrimitiveMode;
    static int tubeNumSubdivisions; ///< Number of tube subdivisions for LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER.
    std::vector<std::string> supportedRenderingModes;
    bool useCappedTubes = true;
    bool showLineDataWindow = true;
    std::string lineDataWindowName;

    /// Stores line point data if linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH.
    sgl::GeometryBufferPtr linePointDataSSBO;

#ifdef USE_VULKAN_INTEROP
    // Caches the rendering data when using Vulkan (as, e.g., the Vulkan ray tracer and AO baking could be used at the
    // same time).
    sgl::vk::BottomLevelAccelerationStructurePtr getTubeTriangleBottomLevelAS();
    sgl::vk::BottomLevelAccelerationStructurePtr getTubeAabbBottomLevelAS();
    sgl::vk::BottomLevelAccelerationStructurePtr getHullTriangleBottomLevelAS();
    VulkanTubeTriangleRenderData vulkanTubeTriangleRenderData;
    VulkanTubeAabbRenderData vulkanTubeAabbRenderData;
    VulkanHullTriangleRenderData vulkanHullTriangleRenderData;
    sgl::vk::BottomLevelAccelerationStructurePtr tubeTriangleBottomLevelAS;
    sgl::vk::BottomLevelAccelerationStructurePtr tubeAabbBottomLevelAS;
    sgl::vk::BottomLevelAccelerationStructurePtr hullTriangleBottomLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeTriangleTopLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeTriangleAndHullTopLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeAabbTopLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeAabbAndHullTopLevelAS;

    struct LineRenderSettings {
        float lineWidth = 0.0f;
        int32_t hasHullMesh = 0;
        float depthCueStrength = 0.0f;
        float ambientOcclusionStrength = 0.0f;

        // Ambient occlusion settings.
        uint32_t numAoTubeSubdivisions = 0;
        uint32_t numLineVertices = 0;
        uint32_t numParametrizationVertices = 0;
        uint32_t paddingLineSettings = 0;
    };
    struct HullRenderSettings {
        glm::vec4 color;
        glm::ivec3 padding;
        int32_t useShading;
    };

    // Uniform buffers with settings for rendering.
    LineRenderSettings lineRenderSettings;
    HullRenderSettings hullRenderSettings;
    sgl::vk::BufferPtr lineRenderSettingsBuffer;
    sgl::vk::BufferPtr hullRenderSettingsBuffer;
#endif

    // Optional.
    bool shallRenderSimulationMeshBoundary = false;
    glm::vec4 hullColor = glm::vec4(
            sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(0.5, 0.5, 0.5f)), 0.3f);
    float hullOpacity = 0.3f;
    bool hullUseShading = true;
    std::vector<uint32_t> simulationMeshOutlineTriangleIndices;
    std::vector<glm::vec3> simulationMeshOutlineVertexPositions;
    std::vector<glm::vec3> simulationMeshOutlineVertexNormals;
};

#endif //STRESSLINEVIS_LINEDATA_HPP
