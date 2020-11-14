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

#ifndef STRESSLINEVIS_LINEDATASTRESS_HPP
#define STRESSLINEVIS_LINEDATASTRESS_HPP

#include "LineData.hpp"

//const char *const DISTANCE_MEASURES[] = {
//        "Distance Exponential Kernel",
//        "Distance Squared Exponential Kernel"
//};
//const int NUM_DISTANCE_MEASURES = ((int)(sizeof(DISTANCE_MEASURES)/sizeof(*DISTANCE_MEASURES)));

class LineDataStress : public LineData {
public:
    LineDataStress(sgl::TransferFunctionWindow &transferFunctionWindow);
    ~LineDataStress();
    void setStressTrajectoryData(
            const std::vector<Trajectories>& trajectoriesPs,
            const std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs);
    void setDegeneratePoints(
            const std::vector<glm::vec3>& degeneratePoints, std::vector<std::string>& attributeNames);
    /// Can be used to set what principal stress (PS) directions we want to display.
    void setUsedPsDirections(const std::vector<bool>& usedPsDirections);

    // Statistics.
    virtual size_t getNumStressDirections() { return trajectoriesPs.size(); }
    virtual size_t getNumAttributes() override;
    virtual size_t getNumLines() override;
    virtual size_t getNumLinePoints() override;
    virtual size_t getNumLineSegments() override;

    // Get filtered line data (only containing points also shown when rendering).
    virtual Trajectories filterTrajectoryData() override;
    virtual std::vector<std::vector<glm::vec3>> getFilteredLines() override;
    std::vector<Trajectories> filterTrajectoryPsData();
    /// Principal stress direction -> Line set index -> Point on line index.
    std::vector<std::vector<std::vector<glm::vec3>>> getFilteredPrincipalStressLines();

    // --- Retrieve data for rendering. ---
    TubeRenderData getTubeRenderData();
    TubeRenderDataProgrammableFetch getTubeRenderDataProgrammableFetch();
    TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization();
    PointRenderData getDegeneratePointsRenderData();

private:
    virtual void recomputeHistogram() override;

    // Principal stress lines (usually three line sets for three directions).
    std::vector<Trajectories> trajectoriesPs;
    std::vector<StressTrajectoriesData> stressTrajectoriesDataPs;
    std::vector<glm::vec3> degeneratePoints;
    std::vector<bool> usedPsDirections; ///< What principal stress (PS) directions do we want to display?

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
