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

#ifndef TRAJECTORYFILE_HPP
#define TRAJECTORYFILE_HPP

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <Math/Geometry/AABB3.hpp>
#include <Utils/SciVis/ImportanceCriteria.hpp>

struct Trajectory {
    std::vector<glm::vec3> positions;
    std::vector<std::vector<float>> attributes;
};
typedef std::vector<Trajectory> Trajectories;

struct StressTrajectoryData {
    // Per line data.
    ///< Hierarchy levels between 0 and 1.
    std::vector<float> hierarchyLevels;

    // Per line point data.
    std::vector<float> majorPs;
    std::vector<float> mediumPs;
    std::vector<float> minorPs;
    std::vector<glm::vec3> majorPsDir;
    std::vector<glm::vec3> mediumPsDir;
    std::vector<glm::vec3> minorPsDir;
};
typedef std::vector<StressTrajectoryData> StressTrajectoriesData;

sgl::AABB3 computeTrajectoriesAABB3(const Trajectories& trajectories);
void normalizeTrajectoriesVertexPositions(
        Trajectories& trajectories, const glm::mat4* vertexTransformationMatrixPtr = nullptr);
void normalizeTrajectoriesVertexPositions(
        Trajectories& trajectories, const sgl::AABB3& aabb, const glm::mat4* vertexTransformationMatrixPtr = nullptr);
void normalizeVertexPositions(
        std::vector<glm::vec3>& vertexPositions, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr = nullptr);
void normalizeTrajectoriesVertexAttributes(Trajectories& trajectories);

sgl::AABB3 computeTrajectoriesPsAABB3(const std::vector<Trajectories>& trajectoriesPs);
void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const glm::mat4* vertexTransformationMatrixPtr = nullptr);
void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr = nullptr);
/// Normalize attributes per principal stress direction?
void normalizeTrajectoriesPsVertexAttributes_Total(std::vector<Trajectories>& trajectoriesPs);
/// Normalize across principal stress directions?
void normalizeTrajectoriesPsVertexAttributes_PerPs(std::vector<Trajectories>& trajectoriesPs);

/**
 * Selects @see loadTrajectoriesFromObj, @see loadTrajectoriesFromNetCdf or @see loadTrajectoriesFromBinLines depending
 * on the file endings and performs some normalization for special datasets (e.g. the rings dataset).
 * @param filename The name of the trajectory file to open.
 * @param attributeNames The names of the vertex attributes (if specified in the file). Left empty if not specified.
 * @param normalizeVertexPositions Whether to normalize the vertex positions.
 * @param normalizeAttributes Whether to normalize the list of attributes to the range [0,1].
 * @param vertexTransformationMatrixPtr Can be used to pass a transformation matrix for the vertex positions (optional).
 * @return The trajectories loaded from the file (empty if the file could not be opened).
 */
Trajectories loadFlowTrajectoriesFromFile(
        const std::string& filename, std::vector<std::string>& attributeNames,
        bool normalizeVertexPositions = true, bool normalizeAttributes = false,
        const glm::mat4* vertexTransformationMatrixPtr = nullptr);

/**
 * Uses @see loadStressTrajectoriesFromDat_v1 depending on the file endings and performs some normalization for special
 * datasets.
 * @param filenamesTrajectories The names of the principal stress trajectory files to open.
 * @param filenamesHierarchy The names of the line hierarchy files to open (optional; can be empty).
 * @param loadedPsIndices Which of the three principal stress directions (0 = major, 1 = medium, 2 = minor) were
 * loaded from the passed files.
 * @param trajectoriesPs The three trajectory sets loaded from the file (empty if the file(s) could not be opened).
 * @param stressTrajectoriesDataPs The principal stress data of the three trajectory sets loaded from the file (empty if
 * the file could not be opened).
 * @param bandPointsUnsmoothedListLeftPs The (unsmoothed) band points on the left band strand.
 * @param bandPointsUnsmoothedListRightPs The (unsmoothed) band points on the right band strand.
 * @param bandPointsSmoothedListLeftPs The (smoothed) band points on the left band strand.
 * @param bandPointsSmoothedListRightPs The (smoothed) band points on the right band strand.
 * @param simulationMeshOutlineTriangleIndices The triangle indices of the hull mesh (optional output).
 * @param simulationMeshOutlineVertexPositions The vertex positions of the hull mesh (optional output).
 * @param normalizeVertexPositions Whether to normalize the vertex positions.
 * @param normalizeAttributes Whether to normalize the list of attributes to the range [0,1].
 * @param oldAABB The old AABB before normalization is stored in the pointer (optional, can be nullptr).
 * @param vertexTransformationMatrixPtr Can be used to pass a transformation matrix for the vertex positions (optional).
 */
void loadStressTrajectoriesFromFile(
        const std::vector<std::string>& filenamesTrajectories, const std::vector<std::string>& filenamesHierarchy,
        int version, std::vector<int>& loadedPsIndices,
        std::vector<Trajectories>& trajectoriesPs, std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListRightPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListRightPs,
        std::vector<uint32_t>& simulationMeshOutlineTriangleIndices,
        std::vector<glm::vec3>& simulationMeshOutlineVertexPositions,
        bool normalizeVertexPositions = true, bool normalizeAttributes = false,
        sgl::AABB3* oldAABB = nullptr, const glm::mat4* vertexTransformationMatrixPtr = nullptr);

Trajectories loadTrajectoriesFromObj(const std::string& filename, std::vector<std::string>& attributeNames);

Trajectories loadTrajectoriesFromNetCdf(const std::string& filename);

Trajectories loadTrajectoriesFromBinLines(const std::string& filename);

#endif // TRAJECTORYFILE_HPP
