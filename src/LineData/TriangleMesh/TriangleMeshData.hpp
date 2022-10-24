/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#ifndef LINEVIS_TRIANGLEMESHDATA_HPP
#define LINEVIS_TRIANGLEMESHDATA_HPP

#include "Widgets/MultiVarTransferFunctionWindow.hpp"
#include "../LineData.hpp"

/**
 * LineVis also supports visualizing static triangle meshes. This class still is derived from the super-class
 * @see LineData.
 */
class TriangleMeshData : public LineData {
public:
    explicit TriangleMeshData(sgl::TransferFunctionWindow& transferFunctionWindow);
    ~TriangleMeshData() override;
    [[nodiscard]] bool getIsSmallDataSet() const override;
    // Called before the data is first used on the main thread.
    void onMainThreadDataInit() override;

    /**
     * Load the data from the selected file(s).
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
    size_t getNumAttributes() override { return vertexAttributesList.size(); }
    size_t getNumLines() override { return 0; }
    size_t getNumLinePoints() override { return 0; }
    size_t getNumLineSegments() override { return 0; }
    size_t getBaseSizeInBytes() override { return 0; }


    // Public interface for filtering trajectories.
    void iterateOverTrajectories(std::function<void(const Trajectory&)> callback) override {}
    void iterateOverTrajectoriesNotFiltered(std::function<void(const Trajectory&)> callback) override {}
    void filterTrajectories(std::function<bool(const Trajectory&)> callback) override {}
    void resetTrajectoryFilter() override {}

    // Get filtered line data (only containing points also shown when rendering).
    Trajectories filterTrajectoryData() override { return {}; }
    std::vector<std::vector<glm::vec3>> getFilteredLines(LineRenderer* lineRenderer) override { return {}; }

    // --- Retrieve data for rendering. Preferred way. ---
    void setGraphicsPipelineInfo(
            sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) override;
    void setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData) override;
    void updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer) override;
    void setRasterDataBindings(sgl::vk::RasterDataPtr& rasterData) override;

    // --- Retrieve data for rendering. ---
    LinePassTubeRenderData getLinePassTubeRenderData() override { return {}; }
    LinePassTubeRenderDataMeshShader getLinePassTubeRenderDataMeshShader() override { return {}; }
    LinePassTubeRenderDataProgrammablePull getLinePassTubeRenderDataProgrammablePull() override { return {}; }
    LinePassQuadsRenderData getLinePassQuadsRenderData() override { return {}; }
    LinePassQuadsRenderDataProgrammablePull getLinePassQuadsRenderDataProgrammablePull() override { return {}; }
    TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization() override { return {}; }
    TubeTriangleRenderData getLinePassTubeTriangleMeshRenderDataPayload(
            bool isRasterizer, bool vulkanRayTracing, TubeTriangleRenderDataPayloadPtr& payload) override;
    TubeAabbRenderData getLinePassTubeAabbRenderData(bool isRasterizer, bool ellipticTubes) override { return {}; }
    void getVulkanShaderPreprocessorDefines(
            std::map<std::string, std::string>& preprocessorDefines, bool isRasterizer) override;

    // --- Retrieve triangle mesh on the CPU. ---
    void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& _triangleIndices, std::vector<glm::vec3>& _vertexPositions,
            std::vector<glm::vec3>& _vertexNormals, std::vector<float>& _vertexAttributes) override;
    void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& _triangleIndices, std::vector<glm::vec3>& _vertexPositions) override;

    /**
     * Renders the entries in the property editor.
     * @return true if the gather shader needs to be reloaded.
     */
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

    MultiVarTransferFunctionWindow& getMultiVarTransferFunctionWindow();

protected:
    void recomputeHistogram() override;

    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<std::vector<float>> vertexAttributesList;

    ///< The maximum number of vertices/indices to be considered a small data set (important, e.g., for live UI updates).
    const size_t SMALL_DATASET_VERTICES_MAX = 1000000;
    const size_t SMALL_DATASET_INDICES_MAX = 1000000;

    // Rendering options.
    bool useBackfaceCulling = false;

    MultiVarTransferFunctionWindow multiVarTransferFunctionWindow;
};

#endif //LINEVIS_TRIANGLEMESHDATA_HPP
