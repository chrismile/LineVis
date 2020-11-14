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

#ifndef PIXELSYNCOIT_TRAJECTORYFILE_HPP
#define PIXELSYNCOIT_TRAJECTORYFILE_HPP

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
void normalizeTrajectoriesVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const glm::mat4* vertexTransformationMatrixPtr = nullptr);
void normalizeTrajectoriesVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr = nullptr);
void normalizeTrajectoriesVertexAttributes(std::vector<Trajectories>& trajectoriesPs);

/**
 * Selects @see loadTrajectoriesFromObj, @see loadTrajectoriesFromNetCdf or @see loadTrajectoriesFromBinLines depending
 * on the file endings and performs some normalization for special datasets (e.g. the rings dataset).
 * @param filename The name of the trajectory file to open.
 * @param normalizeVertexPositions Whether to normalize the vertex positions.
 * @param normalizeAttributes Whether to normalize the list of attributes to the range [0,1].
 * @param vertexTransformationMatrixPtr Can be used to pass a transformation matrix for the vertex positions (optional).
 * @return The trajectories loaded from the file (empty if the file could not be opened).
 */
Trajectories loadFlowTrajectoriesFromFile(
        const std::string& filename, bool normalizeVertexPositions = true, bool normalizeAttributes = false,
        const glm::mat4* vertexTransformationMatrixPtr = nullptr);

/**
 * Uses @see loadStressTrajectoriesFromDat depending on the file endings and performs some normalization for special
 * datasets.
 * @param filenames The names of the principal stress trajectory files to open (usually three for all PS directions).
 * @param The three trajectory sets loaded from the file (empty if the file could not be opened).
 * @param The principal stress data of the three trajectory sets loaded from the file (empty if the file could not be
 * opened).
 * @param normalizeVertexPositions Whether to normalize the vertex positions.
 * @param normalizeAttributes Whether to normalize the list of attributes to the range [0,1].
 * @param oldAABB The old AABB before normalization is stored in the pointer (optional, can be nullptr).
 * @param vertexTransformationMatrixPtr Can be used to pass a transformation matrix for the vertex positions (optional).
 */
void loadStressTrajectoriesFromFile(
        const std::vector<std::string>& filenames,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        bool normalizeVertexPositions = true, bool normalizeAttributes = false,
        sgl::AABB3* oldAABB = nullptr, const glm::mat4* vertexTransformationMatrixPtr = nullptr);

Trajectories loadTrajectoriesFromObj(const std::string& filename);

Trajectories loadTrajectoriesFromNetCdf(const std::string& filename);

Trajectories loadTrajectoriesFromBinLines(const std::string& filename);

#endif //PIXELSYNCOIT_TRAJECTORYFILE_HPP
