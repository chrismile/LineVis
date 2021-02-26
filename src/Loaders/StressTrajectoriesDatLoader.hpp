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

#ifndef STRESSLINEVIS_STRESSTRAJECTORIESDATLOADER_HPP
#define STRESSLINEVIS_STRESSTRAJECTORIESDATLOADER_HPP

#include "TrajectoryFile.hpp"

/**
 * Loads principal stress lines from the specified files.
 * One .dat file can contain stress lines from either one or up to three principal stress directions.
 * @param filenamesTrajectories The names of the principal stress trajectory files to open.
 * @param filenamesHierarchy The names of the line hierarchy files to open (optional; can be empty).
 * @param loadedPsIndices Which of the three principal stress directions (0 = major, 1 = medium, 2 = minor) were
 * loaded from the passed files.
 * @param The three trajectory sets loaded from the file (empty if the file could not be opened).
 * @param The principal stress data of the three trajectory sets loaded from the file (empty if the file could not be
 * opened).
 */
void loadStressTrajectoriesFromDat_v1(
        const std::vector<std::string>& filenamesTrajectories,
        const std::vector<std::string>& filenamesHierarchy,
        std::vector<int>& loadedPsIndices,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs);

/**
 * Loads principal stress lines from the specified files.
 * One .dat file can contain stress lines from either one or up to three principal stress directions.
 * The .dat version 2 format stores band geometry data, and supplies pre-computed scalar field data.
 * @param filenamesTrajectories The names of the principal stress trajectory files to open.
 * @param loadedPsIndices Which of the three principal stress directions (0 = major, 1 = medium, 2 = minor) were
 * loaded from the passed files.
 * @param The three trajectory sets loaded from the file (empty if the file could not be opened).
 * @param The principal stress data of the three trajectory sets loaded from the file (empty if the file could not be
 * opened).
 * @param bandPointsListLeftPs The band points on the left band strand.
 * @param bandPointsListRightPs The band points on the right band strand.
 */
void loadStressTrajectoriesFromDat_v2(
        const std::vector<std::string>& filenamesTrajectories,
        std::vector<int>& loadedPsIndices,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListRightPs);


/**
 * Loads principal stress lines from the specified files.
 * One .dat file can contain stress lines from either one or up to three principal stress directions.
 * The .dat version 2 format stores band geometry data, and supplies pre-computed scalar field data
 * and simulation mesh hulls.
 * @param filenamesTrajectories The names of the principal stress trajectory files to open.
 * @param loadedPsIndices Which of the three principal stress directions (0 = major, 1 = medium, 2 = minor) were
 * loaded from the passed files.
 * @param The three trajectory sets loaded from the file (empty if the file could not be opened).
 * @param The principal stress data of the three trajectory sets loaded from the file (empty if the file could not be
 * opened).
 * @param bandPointsUnsmoothedListLeftPs The (unsmoothed) band points on the left band strand.
 * @param bandPointsUnsmoothedListRightPs The (unsmoothed) band points on the right band strand.
 * @param bandPointsSmoothedListLeftPs The (smoothed) band points on the left band strand.
 * @param bandPointsSmoothedListRightPs The (smoothed) band points on the right band strand.
 * @param simulationMeshOutlineTriangleIndices The triangle indices of the hull mesh (output).
 * @param simulationMeshOutlineVertexPositions The vertex positions of the hull mesh (output).
 */
void loadStressTrajectoriesFromDat_v3(
        const std::vector<std::string>& filenamesTrajectories,
        std::vector<int>& loadedPsIndices,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListRightPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListRightPs,
        std::vector<uint32_t>& simulationMeshOutlineTriangleIndices,
        std::vector<glm::vec3>& simulationMeshOutlineVertexPositions);

#endif //STRESSLINEVIS_STRESSTRAJECTORIESDATLOADER_HPP
