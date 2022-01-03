/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020-2021, Christoph Neuhauser
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

#ifndef STRESSLINEVIS_LINEDATASTRESS_HPP
#define STRESSLINEVIS_LINEDATASTRESS_HPP

#include <array>

#include "LineData.hpp"
#include "Widgets/StressLineHierarchyMappingWidget.hpp"
#include "Widgets/MultiVarTransferFunctionWindow.hpp"

//const char *const DISTANCE_MEASURES[] = {
//        "Distance Exponential Kernel",
//        "Distance Squared Exponential Kernel"
//};
//const int NUM_DISTANCE_MEASURES = ((int)(sizeof(DISTANCE_MEASURES)/sizeof(*DISTANCE_MEASURES)));

class LineDataStress : public LineData {
public:
    explicit LineDataStress(sgl::TransferFunctionWindow &transferFunctionWindow);
    ~LineDataStress() override;
    bool settingsDiffer(LineData* other) override;
    [[nodiscard]] bool getIsSmallDataSet() const override;
    void update(float dt) override;

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

    void setStressTrajectoryData(
            const std::vector<Trajectories>& trajectoriesPs,
            const std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs);
    void setDegeneratePoints(
            const std::vector<glm::vec3>& degeneratePoints, std::vector<std::string>& attributeNames);
    /// Can be used to set what principal stress (PS) directions we want to display.
    void setUsedPsDirections(const std::vector<bool>& usedPsDirections);
    static inline bool getUsePrincipalStressDirectionIndex() { return usePrincipalStressDirectionIndex; }
    [[nodiscard]] inline bool getUseLineHierarchy() const { return useLineHierarchy; }
    inline bool getHasDegeneratePoints() { return !degeneratePoints.empty(); }
    inline MultiVarTransferFunctionWindow& getMultiVarTransferFunctionWindow() { return multiVarTransferFunctionWindow; }

    // Statistics.
    [[nodiscard]] size_t getNumStressDirections() const { return trajectoriesPs.size(); }
    size_t getNumAttributes() override;
    size_t getNumLines() override;
    size_t getNumLinePoints() override;
    size_t getNumLineSegments() override;

    // Public interface for filtering trajectories.
    void iterateOverTrajectories(std::function<void(const Trajectory&)> callback) override;
    void filterTrajectories(std::function<bool(const Trajectory&)> callback) override;
    void resetTrajectoryFilter() override;

    // Get filtered line data (only containing points also shown when rendering).
    Trajectories filterTrajectoryData() override;
    std::vector<std::vector<glm::vec3>> getFilteredLines(LineRenderer* lineRenderer) override;
    std::vector<Trajectories> filterTrajectoryPsData();
    /// Principal stress direction -> Line set index -> Point on line index.
    std::vector<std::vector<std::vector<glm::vec3>>> getFilteredPrincipalStressLines();

    // --- Retrieve data for rendering. Preferred way. ---
    sgl::ShaderProgramPtr reloadGatherShader() override;
    sgl::ShaderAttributesPtr getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader) override;
    void setUniformGatherShaderData_AllPasses() override;
    void setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader) override;

    // --- Retrieve data for rendering. ---
    TubeRenderData getTubeRenderData() override;
    TubeRenderDataProgrammableFetch getTubeRenderDataProgrammableFetch() override;
    TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization() override;
    PointRenderData getDegeneratePointsRenderData();
    BandRenderData getBandRenderData() override;

#ifdef USE_VULKAN_INTEROP
    // --- Retrieve data for rendering for Vulkan. ---
    VulkanTubeTriangleRenderData getVulkanTubeTriangleRenderData(LineRenderer* lineRenderer, bool raytracing) override;
    VulkanTubeAabbRenderData getVulkanTubeAabbRenderData(LineRenderer* lineRenderer) override;
    std::map<std::string, std::string> getVulkanShaderPreprocessorDefines() override;
    void setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData) override;
    void updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer) override;
#endif

    // --- Retrieve triangle mesh on the CPU. ---
    void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
            std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes) override;
    void getTriangleMesh(
            LineRenderer* lineRenderer,
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) override;

    /**
     * For selecting options for the rendering technique (e.g., screen-oriented bands, tubes).
     * @return true if the gather shader needs to be reloaded.
     */
    bool renderGuiPropertyEditorNodesRenderer(sgl::PropertyEditor& propertyEditor, LineRenderer* lineRenderer) override;
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
    /**
     * Renders the entries in the property editor.
     * @return true if the gather shader needs to be reloaded.
     */
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

    /// Certain GUI widgets might need the clear color.
    void setClearColor(const sgl::Color& clearColor) override;
    /// Whether to use linear RGB when rendering.
    void setUseLinearRGB(bool useLinearRGB) override;
    bool shallRenderTransferFunctionWindow() override { return !usePrincipalStressDirectionIndex; }

    /// Set current rendering modes (e.g. for making visible certain UI options only for certain renderers).
    void setRenderingModes(const std::vector<RenderingMode>& renderingModes) override;

    static inline void setUseMajorPS(bool val) { useMajorPS = val; }
    static inline void setUseMediumPS(bool val) { useMediumPS = val; }
    static inline void setUseMinorPS(bool val) { useMinorPS = val; }

    // The seed process can be rendered for the video.
    [[nodiscard]] inline bool getHasSeedPoints() const { return !seedPoints.empty(); }
    [[nodiscard]] inline bool getShallRenderSeedingProcess() const { return shallRenderSeedingProcess; }
    inline void setShallRenderSeedingProcess(bool shallRenderSeedingProcess) {
        this->shallRenderSeedingProcess = shallRenderSeedingProcess;
    }
    [[nodiscard]] inline int getNumSeedPoints() const { return int(seedPoints.size()); }
    [[nodiscard]] inline int getCurrentSeedIdx() const { return currentSeedIdx; }
    inline void setCurrentSeedIdx(int currentSeedIdx) { this->currentSeedIdx = currentSeedIdx; }
    [[nodiscard]] inline const glm::vec3& getCurrentSeedPosition() const { return seedPoints.at(currentSeedIdx); }

private:
    void recomputeHistogram() override;
    void recomputeColorLegend() override;
    void recomputeColorLegendPositions();

    // Should we show major, medium and/or minor principal stress lines?
    static bool useMajorPS, useMediumPS, useMinorPS;
    /// Should we use the principal direction ID for rendering?
    static bool usePrincipalStressDirectionIndex;

    // Principal stress lines (usually three line sets for three directions).
    std::vector<int> loadedPsIndices; ///< 0 = major, 1 = medium, 2 = minor.
    std::vector<Trajectories> trajectoriesPs;
    std::vector<StressTrajectoriesData> stressTrajectoriesDataPs;
    std::vector<glm::vec3> degeneratePoints;
    std::vector<bool> usedPsDirections; ///< What principal stress (PS) directions do we want to display?
    std::vector<std::vector<bool>> filteredTrajectoriesPs;
    std::vector<glm::vec2> minMaxAttributeValuesPs[3];
    size_t numTotalTrajectoryPoints = 0;
    int fileFormatVersion = 0;
    std::vector<std::vector<std::vector<glm::vec3>>> bandPointsUnsmoothedListLeftPs, bandPointsUnsmoothedListRightPs;
    std::vector<std::vector<std::vector<glm::vec3>>> bandPointsSmoothedListLeftPs, bandPointsSmoothedListRightPs;
    static std::array<bool, 3> psUseBands;
    static bool useSmoothedBands;
    enum class BandRenderMode {
        RIBBONS, EIGENVALUE_RATIO, HYPERSTREAMLINES
    };
    static BandRenderMode bandRenderMode;

    // Rendering mode settings.
    bool rendererSupportsTransparency = false;

    // Optional line hierarchy settings.
    void updateLineHierarchyHistogram();
    bool hasLineHierarchy = false;
    bool useLineHierarchy = false;
    enum class LineHierarchyType {
        GEO, PS, VM, LENGTH
    };
    static LineHierarchyType lineHierarchyType;
    static glm::vec3 lineHierarchySliderValues;

    // The seed process can be rendered for the video.
    bool shallRenderSeedingProcess = false;
    int currentSeedIdx = 0;
    std::vector<glm::vec3> seedPoints;

    /// Stores line point data if useProgrammableFetch is true.
    sgl::GeometryBufferPtr lineHierarchyLevelsSSBO;

    // Color legend widgets for different principal stress directions.
    StressLineHierarchyMappingWidget stressLineHierarchyMappingWidget;
    MultiVarTransferFunctionWindow multiVarTransferFunctionWindow;

#ifdef USE_VULKAN_INTEROP
    struct StressLineRenderSettings {
        glm::vec3 lineHierarchySlider{};
        float paddingStressLineSettings{};
        glm::ivec3 psUseBands{};
        int currentSeedIdx{};
    };

    // Uniform buffers with settings for rendering.
    StressLineRenderSettings stressLineRenderSettings;
    sgl::vk::BufferPtr stressLineRenderSettingsBuffer;
#endif

    // For computing distance do degenerate regions.
    /*const size_t NUM_DISTANCE_MEASURES = 2;
    enum DistanceMeasure {
        // Exponential kernel: f_1(x,y) = exp(-||x-y||_2 / l), l \in \mathbb{R}
        DISTANCE_MEASURE_EXPONENTIAL_KERNEL,
        // Squared exponential kernel: f_2(x,y) = exp(-||x-y||_2^2 / (2*l^2)), l \in \mathbb{R}
        DISTANCE_MEASURE_SQUARED_EXPONENTIAL_KERNEL //
    };
    DistanceMeasure distanceMeasure = DISTANCE_MEASURE_EXPONENTIAL_KERNEL;*/
};

#endif //STRESSLINEVIS_LINEDATASTRESS_HPP
