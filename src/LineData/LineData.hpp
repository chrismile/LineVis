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

struct Trajectory;
typedef std::vector<Trajectory> Trajectories;

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class LineRenderer;

struct TubeRenderData {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
};

struct BandRenderData {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexOffsetLeftBuffer;
    sgl::GeometryBufferPtr vertexOffsetRightBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
};

/// For internal use of subclasses.
struct LinePointDataProgrammableFetch {
    glm::vec3 vertexPosition;
    float vertexAttribute;
    glm::vec3 vertexTangent;
    uint32_t principalStressIndex; ///< Padding in case of flow lines.
};

struct TubeRenderDataProgrammableFetch {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr linePointsBuffer;
    sgl::GeometryBufferPtr lineHierarchyLevelsBuffer; ///< Empty for flow lines.
};

struct TubeRenderDataOpacityOptimization {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::GeometryBufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
};

struct PointRenderData {
    sgl::GeometryBufferPtr vertexPositionBuffer;
};

struct SimulationMeshOutlineRenderData {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
};

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
    void setSelectedAttributeIndex(int qualityMeasureIdx);
    void onTransferFunctionMapRebuilt();
    inline DataSetType getType() { return dataSetType; }
    // Returns if the visualization mapping needs to be re-generated.
    inline bool isDirty() { return dirty; }
    // Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { bool tmp = reRender; reRender = false; return tmp; }
    // Do non-static settings that lead to a gather shader reload differ?
    virtual bool settingsDiffer(LineData* other) { return false; }

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

    // Statistics.
    inline const sgl::AABB3& getModelBoundingBox() { return modelBoundingBox; }
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
    virtual BandRenderData getBandRenderData() { return BandRenderData(); }

    // Retrieve simulation mesh outline (optional).
    inline bool hasSimulationMeshOutline() { return !simulationMeshOutlineVertexPositions.empty(); }
    sgl::ShaderProgramPtr reloadGatherShaderHull();
    sgl::ShaderAttributesPtr getGatherShaderAttributesHull(sgl::ShaderProgramPtr& gatherShader);
    void setUniformGatherShaderDataHull_Pass(sgl::ShaderProgramPtr& gatherShader);
    SimulationMeshOutlineRenderData getSimulationMeshOutlineRenderData();
    virtual bool shallRenderTransferFunctionWindow() { return true; }

    /**
     * For selecting rendering technique (e.g., screen-oriented bands, tubes) and other line data settings.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGui(bool isRasterizer);
    /**
     * For changing other line rendering settings.
     */
    virtual bool renderGuiRenderingSettings() { return false; }
    /**
     * For rendering a separate ImGui window.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiWindow(bool isRasterizer);
    /// Certain GUI widgets might need the clear color.
    virtual void setClearColor(const sgl::Color& clearColor);
    /// Whether to use linear RGB when rendering.
    virtual void setUseLinearRGB(bool useLinearRGB) {}
    /// Set current rendering mode (e.g. for making visible certain UI options only for certain renderers).
    virtual void setLineRenderer(LineRenderer* lineRenderer) { this->lineRenderer = lineRenderer; }
    virtual void setRenderingMode(RenderingMode renderingMode) { this->renderingMode = renderingMode; }
    inline bool getShallRenderSimulationMeshBoundary() { return shallRenderSimulationMeshBoundary; }

    enum LinePrimitiveMode {
        LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH,
        LINE_PRIMITIVES_RIBBON_GEOMETRY_SHADER,
        LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER,
        LINE_PRIMITIVES_BAND, //< Only for stress lines for now.
    };
    inline LinePrimitiveMode getLinePrimitiveMode() { return linePrimitiveMode; }

protected:
    void loadSimulationMeshOutlineFromFile(
            const std::string& simulationMeshFilename, const sgl::AABB3& oldAABB, glm::mat4* transformationMatrixPtr);
    void rebuildInternalRepresentationIfNecessary();
    virtual void recomputeHistogram()=0;
    virtual void recomputeColorLegend();

    DataSetType dataSetType;
    sgl::AABB3 modelBoundingBox;
    std::vector<std::string> attributeNames;
    std::vector<glm::vec2> minMaxAttributeValues;
    int selectedAttributeIndex = 0; ///< Selected attribute/importance criterion index.
    int selectedAttributeIndexUi = 0;
    bool dirty = false; ///< Should be set to true if the representation changed.
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

    /// Stores line point data if linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH.
    sgl::GeometryBufferPtr linePointDataSSBO;

    // Optional.
    bool shallRenderSimulationMeshBoundary = false;
    glm::vec4 hullColor = glm::vec4(
            sgl::TransferFunctionWindow::sRGBToLinearRGB(glm::vec3(0.5, 0.5, 0.5f)), 0.3f);
    float hullOpacity = 0.15f;
    bool hullUseShading = true;
    std::vector<uint32_t> simulationMeshOutlineTriangleIndices;
    std::vector<glm::vec3> simulationMeshOutlineVertexPositions;
    std::vector<glm::vec3> simulationMeshOutlineVertexNormals;
};

#endif //STRESSLINEVIS_LINEDATA_HPP
