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

#ifndef STRESSLINEVIS_LINEDATAFLOW_HPP
#define STRESSLINEVIS_LINEDATAFLOW_HPP

#include "Widgets/MultiVarTransferFunctionWindow.hpp"
#include "LineData.hpp"

class LineDataFlow : public LineData {
    friend class StreamlineTracingRequester;
public:
    explicit LineDataFlow(sgl::TransferFunctionWindow& transferFunctionWindow);
    ~LineDataFlow() override;
    bool settingsDiffer(LineData* other) override;
    void update(float dt) override;
    virtual void setTrajectoryData(const Trajectories& trajectories);
    [[nodiscard]] bool getIsSmallDataSet() const override;

    /// For changing internal settings programmatically and not over the GUI.
    bool setNewSettings(const SettingsMap& settings) override;

    /**
     * Load line data from the selected file(s).
     * @param fileNames The names of the files to load. More than one file makes primarily sense for, e.g., stress line
     * data with multiple principal stress directions.
     * @param dataSetInformation Metadata about the data set.
     * @param transformationMatrixPtr A transformation to apply to the loaded data (if any; can be nullptr).
     * @return Whether loading was successful.
     */
    bool loadFromFile(
            const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
            glm::mat4* transformationMatrixPtr) override;

    // Statistics.
    size_t getNumAttributes() override;
    size_t getNumLines() override;
    size_t getNumLinePoints() override;
    size_t getNumLineSegments() override;
    size_t getBaseSizeInBytes() override;

    // Public interface for filtering trajectories.
    void iterateOverTrajectories(std::function<void(const Trajectory&)> callback) override;
    void iterateOverTrajectoriesNotFiltered(std::function<void(const Trajectory&)> callback) override;
    void filterTrajectories(std::function<bool(const Trajectory&)> callback) override;
    void resetTrajectoryFilter() override;

    // Get filtered line data (only containing points also shown when rendering).
    Trajectories filterTrajectoryData() override;
    std::vector<std::vector<glm::vec3>> getFilteredLines(LineRenderer* lineRenderer) override;

    // --- Retrieve data for rendering. Preferred way. ---
    void setGraphicsPipelineInfo(
            sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) override;
    void setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData) override;
    void updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer) override;
    void setRasterDataBindings(sgl::vk::RasterDataPtr& rasterData) override;

    // --- Retrieve data for rendering. ---
    LinePassTubeRenderData getLinePassTubeRenderData() override;
    LinePassTubeRenderDataMeshShader getLinePassTubeRenderDataMeshShader() override;
    LinePassTubeRenderDataProgrammablePull getLinePassTubeRenderDataProgrammablePull() override;
    LinePassQuadsRenderData getLinePassQuadsRenderData() override;
    LinePassQuadsRenderDataProgrammablePull getLinePassQuadsRenderDataProgrammablePull() override;
    TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization() override;
    TubeTriangleRenderData getLinePassTubeTriangleMeshRenderData(bool isRasterizer, bool vulkanRayTracing) override;
    TubeAabbRenderData getLinePassTubeAabbRenderData(bool isRasterizer) override;
    void getVulkanShaderPreprocessorDefines(
            std::map<std::string, std::string>& preprocessorDefines, bool isRasterizer) override;

    // --- Retrieve triangle mesh on the CPU. ---
    void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
            std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes) override;
    void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) override;

    /**
     * Renders the entries in the property editor.
     * @return true if the gather shader needs to be reloaded.
     */
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;
    /**
     * For rendering secondary ImGui windows (e.g., for transfer function widgets).
     * @return true if the gather shader needs to be reloaded.
     */
    bool renderGuiWindowSecondary() override;
    /**
     * For rendering secondary, overlay ImGui windows.
     * @return true if the gather shader needs to be reloaded.
     */
    bool renderGuiOverlay() override;

    /// Certain GUI widgets might need the clear color.
    void setClearColor(const sgl::Color& clearColor) override;
    /// Whether to use linear RGB when rendering.
    void setUseLinearRGB(bool useLinearRGB) override;
    bool shallRenderTransferFunctionWindow() override { return !useMultiVarRendering; }

protected:
    void recomputeHistogram() override;
    void recomputeColorLegend() override;
    void recomputeWidgetPositions();

    /**
     * Function used by, e.g., @see getLinePassTubeRenderData, @see getLinePassTubeRenderDataMeshShader and
     * @see getLinePassTubeRenderDataProgrammablePull.
     * It encapsulates shared code for creating line vertex and index data.
     * @param indexOffsetFunctor A functor that returns the line index offset for the next points to be pushed.
     * @param pointPushFunctor A functor that accepts a new point.
     * @param pointPopFunctor A functor that gets called when the previous point needs to be removed. This usually
     * happens when a line doesn't have at least two valid points.
     * @param indicesPushFunctor A functor that creates line index data for the previously pushed points.
     */
    void getLinePassTubeRenderDataGeneral(
            const std::function<uint32_t()>& indexOffsetFunctor,
            const std::function<void(
                    const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent, float lineAttribute,
                    float lineRotation, uint32_t indexOffset, size_t lineIdx, size_t pointIdx)>& pointPushFunctor,
            const std::function<void()>& pointPopFunctor,
            const std::function<void(int numSegments, uint32_t indexOffset)>& indicesPushFunctor);

    Trajectories trajectories;
    size_t numTotalTrajectoryPoints = 0;
    std::vector<bool> filteredTrajectories;

    // Optional ribbon data.
    static bool useRibbons;
    std::vector<std::vector<glm::vec3>> ribbonsDirections;

    /*
     * If helicity data is stored for the trajectories, then the helicity can be used for drawing rotating bands
     * on the surface of the rendered tube primitives.
     */
    int helicityAttributeIndex = -1;
    bool hasHelicity = false;
    static bool useRotatingHelicityBands;
    float helicityRotationFactor = 1.0f;
    float maxHelicity = 0.0f;

    /**
     * Multi-var rendering can be used (at the moment only together with helicity data).
     */
    bool useMultiVarRendering = false;
    std::string comboValue = "";
    std::vector<uint32_t> isAttributeSelectedArray;
    MultiVarTransferFunctionWindow multiVarTransferFunctionWindow;
    // Uniform buffers with settings for rendering.
    struct MultiVarUniformData {
        uint32_t numSelectedAttributes{};
        uint32_t totalNumAttributes{};
        glm::uvec2 multiVarPadding{};
    };
    MultiVarUniformData multiVarUniformData;
    sgl::vk::BufferPtr multiVarUniformDataBuffer;
    sgl::vk::BufferPtr multiVarAttributeDataBuffer;
    std::vector<uint32_t> selectedAttributes;
    sgl::vk::BufferPtr multiVarSelectedAttributesBuffer;
};

#endif //STRESSLINEVIS_LINEDATAFLOW_HPP
