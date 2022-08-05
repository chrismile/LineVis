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

#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/ColorLegendWidget.hpp>
#include <Graphics/Vulkan/Render/GraphicsPipeline.hpp>

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
class RasterData;
typedef std::shared_ptr<RasterData> RasterDataPtr;
class BottomLevelAccelerationStructure;
typedef std::shared_ptr<BottomLevelAccelerationStructure> BottomLevelAccelerationStructurePtr;
class TopLevelAccelerationStructure;
typedef std::shared_ptr<TopLevelAccelerationStructure> TopLevelAccelerationStructurePtr;
}}

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;

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
    friend class StreamlineTracingRequester;
public:
    LineData(sgl::TransferFunctionWindow &transferFunctionWindow, DataSetType dataSetType);
    virtual ~LineData();
    virtual void update(float dt) {}
    [[nodiscard]] inline int getSelectedAttributeIndex() const { return selectedAttributeIndex; }
    [[nodiscard]] inline const std::vector<std::string>& getAttributeNames() const { return attributeNames; }
    [[nodiscard]] inline size_t getNumAttributes() const { return attributeNames.size(); }
    void setSelectedAttributeIndex(int attributeIndex);
    void onTransferFunctionMapRebuilt();
    inline DataSetType getType() { return dataSetType; }
    /// Returns if the visualization mapping needs to be re-generated.
    [[nodiscard]] inline bool isDirty() const { return dirty; }
    /// Returns if the triangle mesh visualization mapping needs to be re-generated.
    [[nodiscard]] inline bool isTriangleRepresentationDirty() const { return triangleRepresentationDirty; }
    /// A renderer can signal that the triangle representation has changed.
    inline void setTriangleRepresentationDirty() { triangleRepresentationDirty = true; }
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { bool tmp = reRender; reRender = false; return tmp; }
    /// Do non-static settings that lead to a gather shader reload differ?
    virtual bool settingsDiffer(LineData* other) { return false; }
    /// Returns whether live visualization mapping updates can be used or whether the data set is too large.
    [[nodiscard]] virtual bool getCanUseLiveUpdate(LineDataAccessType accessType) const;
    /**
     * A small data set has only a little amount of geometric data.
     * For large data sets, changing visualization mapping parameters triggering a rebuild of the internal
     * representation are not feasible to do in real-time and would result in considerable lag.
     */
    [[nodiscard]] virtual bool getIsSmallDataSet() const=0;

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
    virtual size_t getBaseSizeInBytes()=0;

    // Public interface for filtering trajectories.
    virtual void iterateOverTrajectories(std::function<void(const Trajectory&)> callback)=0;
    virtual void iterateOverTrajectoriesNotFiltered(std::function<void(const Trajectory&)> callback)=0;
    virtual void filterTrajectories(std::function<bool(const Trajectory&)> callback)=0;
    virtual void resetTrajectoryFilter()=0;

    // Get filtered line data (only containing points also shown when rendering).
    virtual Trajectories filterTrajectoryData()=0;
    virtual std::vector<std::vector<glm::vec3>> getFilteredLines(LineRenderer* lineRenderer)=0;

    // --- Retrieve data for rendering. Preferred way. ---
    virtual std::vector<std::string> getShaderModuleNames();
    virtual void setGraphicsPipelineInfo(
            sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages);
    virtual void setRasterDataBindings(sgl::vk::RasterDataPtr& rasterData);

    // --- Retrieve data for rendering. Only for renderers needing direct access! ---
    virtual LinePassTubeRenderData getLinePassTubeRenderData()=0;
    virtual LinePassQuadsRenderDataProgrammablePull getLinePassQuadsRenderDataProgrammablePull()=0;
    virtual TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization()=0;
    virtual LinePassTubeRenderDataMeshShader getLinePassTubeRenderDataMeshShader()=0;
    virtual LinePassTubeRenderDataProgrammablePull getLinePassTubeRenderDataProgrammablePull()=0;
    virtual LinePassQuadsRenderData getLinePassQuadsRenderData() { return {}; }
    virtual TubeTriangleRenderData getLinePassTubeTriangleMeshRenderData(bool isRasterizer, bool vulkanRayTracing)=0;
    virtual TubeAabbRenderData getLinePassTubeAabbRenderData(bool isRasterizer)=0;
    virtual HullTriangleRenderData getVulkanHullTriangleRenderData(bool vulkanRayTracing);
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeTriangleTopLevelAS();
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeTriangleAndHullTopLevelAS();
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeAabbTopLevelAS();
    sgl::vk::TopLevelAccelerationStructurePtr getRayTracingTubeAabbAndHullTopLevelAS();
    inline void getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) {
        getVulkanShaderPreprocessorDefines(preprocessorDefines, true);
    }
    virtual void getVulkanShaderPreprocessorDefines(
            std::map<std::string, std::string>& preprocessorDefines, bool isRasterizer);
    virtual void setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData);
    virtual void updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer);

    // --- Retrieve triangle mesh on the CPU. ---
    virtual void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
            std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes)=0;
    virtual void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions)=0;

    // Retrieve simulation mesh outline (optional).
    inline bool hasSimulationMeshOutline() { return !simulationMeshOutlineVertexPositions.empty(); }
    SimulationMeshOutlineRenderData getSimulationMeshOutlineRenderData();
    virtual bool shallRenderTransferFunctionWindow() { return true; }

    /**
     * For selecting options for the rendering technique (e.g., screen-oriented bands, tubes).
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiPropertyEditorNodesRenderer(sgl::PropertyEditor& propertyEditor, LineRenderer* lineRenderer);
    /**
     * For changing other line rendering settings.
     */
    virtual bool renderGuiRenderingSettingsPropertyEditor(sgl::PropertyEditor& propertyEditor);
    /**
     * For rendering secondary ImGui windows (e.g., for transfer function widgets).
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiWindowSecondary();
    /**
     * For rendering secondary, overlay ImGui windows.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiOverlay();
    /**
     * Renders the entries in the property editor.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

    /// Certain GUI widgets might need the clear color.
    virtual void setClearColor(const sgl::Color& clearColor);
    /// Whether to use linear RGB when rendering.
    virtual void setUseLinearRGB(bool useLinearRGB) {}
    /// Set current rendering modes (e.g. for making visible certain UI options only for certain renderers).
    virtual void setLineRenderers(const std::vector<LineRenderer*>& lineRenderers);
    virtual void setRenderingModes(const std::vector<RenderingMode>& renderingModes) {}
    // Sets the global file dialog.
    void setFileDialogInstance(ImGuiFileDialog* fileDialogInstance);
    [[nodiscard]] inline bool getShallRenderSimulationMeshBoundary() const { return shallRenderSimulationMeshBoundary; }
    [[nodiscard]] inline const std::string& getLineDataWindowName() const { return lineDataWindowName; }
    /// Returns whether the gather shader needs to be reloaded.
    bool setUseCappedTubes(LineRenderer* lineRenderer, bool cappedTubes);
    [[nodiscard]] inline bool getUseCappedTubes() const { return useCappedTubes; }
    [[nodiscard]] static inline int getTubeNumSubdivisions() { return tubeNumSubdivisions; }

    enum LinePrimitiveMode {
        LINE_PRIMITIVES_QUADS_PROGRAMMABLE_PULL,
        LINE_PRIMITIVES_QUADS_GEOMETRY_SHADER,
        LINE_PRIMITIVES_TUBE_PROGRAMMABLE_PULL,
        LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER,
        LINE_PRIMITIVES_TUBE_TRIANGLE_MESH,
        LINE_PRIMITIVES_TUBE_MESH_SHADER,
        LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER,
        LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL,
        LINE_PRIMITIVES_TUBE_RIBBONS_GEOMETRY_SHADER,
        LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH,
        LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER,
    };
    static inline bool getLinePrimitiveModeUsesSingleVertexShaderInputs(LinePrimitiveMode mode) {
        return mode != LINE_PRIMITIVES_QUADS_PROGRAMMABLE_PULL
               && mode != LINE_PRIMITIVES_TUBE_PROGRAMMABLE_PULL
               && mode != LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL
               && mode != LINE_PRIMITIVES_TUBE_MESH_SHADER
               && mode != LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER
               && mode != LINE_PRIMITIVES_TUBE_TRIANGLE_MESH
               && mode != LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH;
    }
    static inline bool getLinePrimitiveModeSupportsLineMultiWidth(LinePrimitiveMode mode) {
        return mode == LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL
               || mode == LINE_PRIMITIVES_TUBE_RIBBONS_GEOMETRY_SHADER
               || mode == LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH
               || mode == LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER;
    }
    static bool getLinePrimitiveModeUsesGeometryShader(LinePrimitiveMode mode) {
        return mode == LINE_PRIMITIVES_QUADS_GEOMETRY_SHADER || mode == LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER
               || mode == LINE_PRIMITIVES_TUBE_RIBBONS_GEOMETRY_SHADER || mode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER;
    }
    static inline LinePrimitiveMode getLinePrimitiveMode() { return linePrimitiveMode; }
    static inline void setLinePrimitiveMode(LinePrimitiveMode mode) { linePrimitiveMode = mode; }
    static inline bool getUseBandRendering() {
        return linePrimitiveMode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER
                || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL
                || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_GEOMETRY_SHADER
                || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH
                || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER;
    }

    enum class RequestMode {
        NO_CACHE_SUPPORTED, TRIANGLES, AABBS, GEOMETRY_SHADER, PROGRAMMABLE_PULL, MESH_SHADER
    };

    static inline float getMinBandThickness() { return minBandThickness; }

    /// This function should be called by sub-classes before accessing internal rendering data.
    virtual void rebuildInternalRepresentationIfNecessary();

protected:
    void loadSimulationMeshOutlineFromFile(
            const std::string& simulationMeshFilename, const sgl::AABB3& oldAABB, glm::mat4* transformationMatrixPtr);
    virtual void recomputeColorLegend();
    int getAttributeNameIndex(const std::string& attributeName);
    bool updateLinePrimitiveMode(LineRenderer* lineRenderer);
    void removeOtherCachedDataTypes(RequestMode requestMode);

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
    ImGuiFileDialog* fileDialogInstance = nullptr;

    // Color legend widgets for different attributes.
    bool shallRenderColorLegendWidgets = true;
    std::vector<sgl::ColorLegendWidget> colorLegendWidgets;

    // Rendering settings.
    std::vector<LineRenderer*> lineRenderersCached;
    static LinePrimitiveMode linePrimitiveMode;
    static int tubeNumSubdivisions; ///< Number of tube subdivisions for LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER.
    std::vector<std::string> supportedRenderingModes;
    bool useCappedTubes = true;
    bool useHalos = true;
    bool showLineDataWindow = true;
    std::string lineDataWindowName;

    // If optional band data is provided.
    bool hasBandsData = false;
    static bool renderThickBands;
    static float minBandThickness;

    LinePrimitiveMode cacheLinePrimitiveMode = LinePrimitiveMode::LINE_PRIMITIVES_QUADS_GEOMETRY_SHADER;

    // Caches the triangle render data. This is useful, e.g., when the Vulkan ray tracer and ambient occlusion baking
    // are used at the same time.
    std::vector<sgl::vk::BottomLevelAccelerationStructurePtr> getTubeTriangleBottomLevelAS();
    sgl::vk::BottomLevelAccelerationStructurePtr getTubeAabbBottomLevelAS();
    sgl::vk::BottomLevelAccelerationStructurePtr getHullTriangleBottomLevelAS();
    void splitTriangleIndices(
            std::vector<uint32_t>& tubeTriangleIndices,
            const std::vector<TubeTriangleVertexData> &tubeTriangleVertexDataList);
    TubeTriangleRenderData vulkanTubeTriangleRenderData;
    TubeTriangleSplitData tubeTriangleSplitData;
    TubeAabbRenderData vulkanTubeAabbRenderData;
    HullTriangleRenderData vulkanHullTriangleRenderData;
    bool vulkanTubeTriangleRenderDataIsRayTracing = false;
    const size_t batchSizeLimit = 1024 * 1024 * 32;
    bool generateSplitTriangleData = false;
    std::vector<sgl::vk::BottomLevelAccelerationStructurePtr> tubeTriangleBottomLevelASes;
    sgl::vk::BottomLevelAccelerationStructurePtr tubeAabbBottomLevelAS;
    sgl::vk::BottomLevelAccelerationStructurePtr hullTriangleBottomLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeTriangleTopLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeTriangleAndHullTopLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeAabbTopLevelAS;
    sgl::vk::TopLevelAccelerationStructurePtr tubeAabbAndHullTopLevelAS;

    // Caches the line render data.
    LinePassTubeRenderDataProgrammablePull cachedRenderDataProgrammablePull;
    LinePassTubeRenderData cachedRenderDataGeometryShader;
    LinePassTubeRenderDataMeshShader cachedRenderDataMeshShader;

    // For deferred rendering.
    // Array of triangle meshlets (aabb, start idx, end idx)

    struct LineUniformData {
        // Camera data.
        glm::vec3 cameraPosition{};
        float fieldOfViewY = 1.0f;
        glm::mat4 viewMatrix{};
        glm::mat4 projectionMatrix{};
        glm::mat4 inverseViewMatrix{};
        glm::mat4 inverseProjectionMatrix{};
        glm::vec4 backgroundColor{};
        glm::vec4 foregroundColor{};

        // Line & band render settings.
        float lineWidth = 0.0f;
        float bandWidth = 0.0f;
        float minBandThickness = 1.0f;
        float depthCueStrength = 0.0f;
        float ambientOcclusionStrength = 0.0f;
        float ambientOcclusionGamma = 1.0f;
        float separatorBaseWidth = 0.2f;
        float helicityRotationFactor = 0.0f;

        // Pre-baked ambient occlusion settings (STATIC_AMBIENT_OCCLUSION_PREBAKING).
        uint32_t numAoTubeSubdivisions = 0;
        uint32_t numLineVertices = 0;
        uint32_t numParametrizationVertices = 0;

        // Multi-var & twist line settings.
        uint32_t numSubdivisionsBands = 6;

        // Hull render settings.
        glm::vec4 hullColor{};
        uint32_t hasHullMesh = 0;
        uint32_t hullUseShading = 0;

        // Antialiasing.glsl needs a viewport size. Change this value if downsampling/upscaling is used!
        glm::uvec2 viewportSize{};
    };

    // Uniform buffers with settings for rendering.
    LineUniformData lineUniformData = {};
    sgl::vk::BufferPtr lineUniformDataBuffer;

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
