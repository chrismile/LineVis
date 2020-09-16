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

#define _FILE_OFFSET_BITS 64

#include <cstdio>
#include <iostream>

#include <Utils/File/Logfile.hpp>

#include "LineReader.hpp"
#include "StressTrajectoriesDatLoader.hpp"

void loadStressTrajectoriesFromDat(
        const std::vector<std::string>& filenames,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs) {
    trajectoriesPs.resize(filenames.size());
    stressTrajectoriesDataPs.resize(filenames.size());
    size_t geometryByteSize = 0;

    for (size_t psIdx = 0; psIdx < filenames.size(); psIdx++) {
        const std::string& filename = filenames.at(psIdx);
        Trajectories& trajectories = trajectoriesPs.at(psIdx);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(psIdx);

        LineReader lineReader(filename);
        uint32_t numLines = lineReader.readScalarLine<uint32_t>();
        trajectories.resize(numLines);
        stressTrajectoriesData.resize(numLines);
        for (uint32_t lineIdx = 0; lineIdx < numLines; lineIdx++) {
            Trajectory& trajectory = trajectories.at(lineIdx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(lineIdx);

            uint32_t lineLength = lineReader.readScalarLine<uint32_t>();
            trajectory.positions.reserve(lineLength);
            stressTrajectoryData.majorPs.reserve(lineLength);
            stressTrajectoryData.mediumPs.reserve(lineLength);
            stressTrajectoryData.minorPs.reserve(lineLength);
            stressTrajectoryData.majorPsDir.reserve(lineLength);
            stressTrajectoryData.mediumPsDir.reserve(lineLength);
            stressTrajectoryData.minorPsDir.reserve(lineLength);
            trajectory.attributes.resize(1);
            trajectory.attributes.front().reserve(lineLength);
            std::vector<float> positionData = lineReader.readVectorLine<float>(lineLength * 3);
            std::vector<float> psData = lineReader.readVectorLine<float>(lineLength * 12);
            std::vector<float> vonMisesData = lineReader.readVectorLine<float>(lineLength);

            for (uint32_t pointIdx = 0; pointIdx < lineLength; pointIdx++) {
                trajectory.positions.push_back(glm::vec3(
                        positionData.at(pointIdx * 3),
                        positionData.at(pointIdx * 3 + 1),
                        positionData.at(pointIdx * 3 + 2)));
                stressTrajectoryData.majorPs.push_back(psData.at(pointIdx * 12));
                stressTrajectoryData.majorPsDir.push_back(glm::vec3(
                        psData.at(pointIdx * 12 + 1),
                        psData.at(pointIdx * 12 + 2),
                        psData.at(pointIdx * 12 + 3)));
                stressTrajectoryData.mediumPs.push_back(psData.at(pointIdx * 12 + 4));
                stressTrajectoryData.mediumPsDir.push_back(glm::vec3(
                        psData.at(pointIdx * 12 + 5),
                        psData.at(pointIdx * 12 + 6),
                        psData.at(pointIdx * 12 + 7)));
                stressTrajectoryData.minorPs.push_back(psData.at(pointIdx * 12 + 8));
                stressTrajectoryData.minorPsDir.push_back(glm::vec3(
                        psData.at(pointIdx * 12 + 9),
                        psData.at(pointIdx * 12 + 10),
                        psData.at(pointIdx * 12 + 11)));
                trajectory.attributes.at(0).push_back(vonMisesData.at(pointIdx));
            }
        }

        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            geometryByteSize += trajectory.positions.size() * sizeof(float) * 3;
            geometryByteSize += stressTrajectoryData.majorPs.size() * sizeof(float);
            geometryByteSize += stressTrajectoryData.mediumPs.size() * sizeof(float);
            geometryByteSize += stressTrajectoryData.minorPs.size() * sizeof(float);
            geometryByteSize += stressTrajectoryData.majorPsDir.size() * sizeof(float) * 3;
            geometryByteSize += stressTrajectoryData.mediumPsDir.size() * sizeof(float) * 3;
            geometryByteSize += stressTrajectoryData.minorPsDir.size() * sizeof(float) * 3;
            for (const std::vector<float>& attributes : trajectory.attributes) {
                geometryByteSize += attributes.size() * sizeof(float);
            }
        }
    }

    std::cout << "Size of line geometry data (MiB): " << (geometryByteSize / (1024.0 * 1024.0)) << std::endl;
}
