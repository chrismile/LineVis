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

#include "LineData.hpp"

class LineDataFlow : public LineData {
public:
    explicit LineDataFlow(sgl::TransferFunctionWindow &transferFunctionWindow);
    ~LineDataFlow() override;
    virtual void setTrajectoryData(const Trajectories& trajectories);
    bool getIsSmallDataSet() const override;

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

    // Public interface for filtering trajectories.
    void iterateOverTrajectories(std::function<void(const Trajectory&)> callback) override;
    void filterTrajectories(std::function<bool(const Trajectory&)> callback) override;
    void resetTrajectoryFilter() override;

    // Get filtered line data (only containing points also shown when rendering).
    Trajectories filterTrajectoryData() override;
    std::vector<std::vector<glm::vec3>> getFilteredLines() override;

    // --- Retrieve data for rendering. ---
    TubeRenderData getTubeRenderData() override;
    TubeRenderDataProgrammableFetch getTubeRenderDataProgrammableFetch() override;
    TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization() override;

#ifdef USE_VULKAN_INTEROP
    // --- Retrieve data for rendering for Vulkan. ---
    VulkanTubeTriangleRenderData getVulkanTubeTriangleRenderData(bool raytracing) override;
    VulkanTubeAabbRenderData getVulkanTubeAabbRenderData() override;
#endif

    // --- Retrieve triangle mesh on the CPU. ---
    void getTriangleMesh(
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
            std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes) override;
    void getTriangleMesh(
            std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) override;

protected:
    void recomputeHistogram() override;

    Trajectories trajectories;
    size_t numTotalTrajectoryPoints = 0;
    std::vector<bool> filteredTrajectories;
};

#endif //STRESSLINEVIS_LINEDATAFLOW_HPP
