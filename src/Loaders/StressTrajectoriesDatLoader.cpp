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

#include <cstdio>
#include <iostream>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <Utils/File/Logfile.hpp>
#include <Utils/File/LineReader.hpp>

#include "StressTrajectoriesDatLoader.hpp"

void loadStressLineHierarchyFromDat(
        const std::vector<std::string>& filenamesHierarchy,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs) {
    size_t psIdx = 0;
    for (size_t fileIdx = 0; fileIdx < filenamesHierarchy.size(); fileIdx++) {
        const std::string& filename = filenamesHierarchy.at(fileIdx);

        sgl::LineReader lineReader(filename);
        while (lineReader.isLineLeft()) {
            assert(psIdx < stressTrajectoriesDataPs.size());
            StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(psIdx);
            std::vector<std::string> linesInfo = lineReader.readVectorLine<std::string>();
            // Line metadata saved?
            uint32_t numLines = 0;
            if (linesInfo.size() == 1) {
                numLines = sgl::fromString<uint32_t>(linesInfo.at(0));
            } else if (linesInfo.size() == 2) {
                numLines = sgl::fromString<uint32_t>(linesInfo.at(1));
            } else {
                sgl::Logfile::get()->writeError(
                        std::string() + "ERROR in loadStressLineHierarchyFromDat: Invalid line metadata in file \""
                        + filename + "\".");
            }
            assert(stressTrajectoriesData.size() == numLines);
            for (uint32_t lineIdx = 0; lineIdx < numLines; lineIdx++) {
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(lineIdx);
                float lineHierarchyLevel = lineReader.readScalarLine<float>();
                stressTrajectoryData.hierarchyLevel = lineHierarchyLevel;
            }

            psIdx++;
        }
    }
}

void loadStressTrajectoriesFromDat(
        const std::vector<std::string>& filenamesTrajectories,
        const std::vector<std::string>& filenamesHierarchy,
        std::vector<int>& loadedPsIndices,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs) {
    trajectoriesPs.reserve(filenamesTrajectories.size());
    stressTrajectoriesDataPs.reserve(filenamesTrajectories.size());
    size_t geometryByteSize = 0;

    size_t psIdx = 0;
    for (size_t fileIdx = 0; fileIdx < filenamesTrajectories.size(); fileIdx++) {
        const std::string& filename = filenamesTrajectories.at(fileIdx);

        sgl::LineReader lineReader(filename);
        while (lineReader.isLineLeft()) {
            Trajectories trajectories;
            StressTrajectoriesData stressTrajectoriesData;
            std::vector<std::string> linesInfo = lineReader.readVectorLine<std::string>();
            // Line metadata saved?
            uint32_t numLines = 0;
            if (linesInfo.size() == 1) {
                numLines = sgl::fromString<uint32_t>(linesInfo.at(0));
            } else if (linesInfo.size() == 2) {
                numLines = sgl::fromString<uint32_t>(linesInfo.at(1));
                boost::algorithm::to_lower(linesInfo.at(0));
                if (boost::ends_with(linesInfo.at(0), "major")) {
                    loadedPsIndices.push_back(0);
                } else if (boost::ends_with(linesInfo.at(0), "medium")) {
                    loadedPsIndices.push_back(1);
                } else if (boost::ends_with(linesInfo.at(0), "minor")) {
                    loadedPsIndices.push_back(2);
                } else {
                    sgl::Logfile::get()->writeError(
                            std::string() + "ERROR in loadStressTrajectoriesFromDat_: "
                            + "Invalid principal stress identifier \"" + linesInfo.at(0) + "\".");
                }
            } else {
                sgl::Logfile::get()->writeError(
                        std::string() + "ERROR in loadStressTrajectoriesFromDat: Invalid line metadata in file \""
                        + filename + "\".");
            }
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
                trajectory.attributes.resize(2);
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
                    if (psIdx == 0) {
                        trajectory.attributes.at(1).push_back(std::abs(stressTrajectoryData.majorPs.back()));
                    } else if (psIdx == 1) {
                        trajectory.attributes.at(1).push_back(std::abs(stressTrajectoryData.mediumPs.back()));
                    } else {
                        trajectory.attributes.at(1).push_back(std::abs(stressTrajectoryData.minorPs.back()));
                    }
                }
            }

            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                geometryByteSize += trajectory.positions.size() * sizeof(float) * 3;
                geometryByteSize += sizeof(float); // hierarchy level
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

            trajectoriesPs.emplace_back(trajectories);
            stressTrajectoriesDataPs.emplace_back(stressTrajectoriesData);
            psIdx++;
        }
    }

    // Check if there's additional line hierarchy data.
    if (filenamesHierarchy.size() > 0) {
        loadStressLineHierarchyFromDat(filenamesHierarchy, stressTrajectoriesDataPs);
    }

    // Assume that all three PS directions are provided for the v1 .dat format.
    if (loadedPsIndices.size() == 0 && trajectoriesPs.size() == 3) {
        loadedPsIndices = {0, 1, 2};
    }

    std::cout << "Size of line geometry data (MiB): " << (geometryByteSize / (1024.0 * 1024.0)) << std::endl;
}

void loadStressTrajectoriesFromDat_v2(
        const std::vector<std::string>& filenamesTrajectories,
        std::vector<int>& loadedPsIndices,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsListRightPs) {
    trajectoriesPs.reserve(filenamesTrajectories.size());
    stressTrajectoriesDataPs.reserve(filenamesTrajectories.size());
    bandPointsListLeftPs.reserve(filenamesTrajectories.size());
    bandPointsListRightPs.reserve(filenamesTrajectories.size());
    size_t geometryByteSize = 0;

    size_t psIdx = 0;
    for (size_t fileIdx = 0; fileIdx < filenamesTrajectories.size(); fileIdx++) {
        const std::string& filename = filenamesTrajectories.at(fileIdx);

        sgl::LineReader lineReader(filename);
        while (lineReader.isLineLeft()) {
            Trajectories trajectories;
            StressTrajectoriesData stressTrajectoriesData;
            std::vector<std::vector<glm::vec3>> bandPointsListLeft;
            std::vector<std::vector<glm::vec3>> bandPointsListRight;
            std::vector<std::string> linesInfo = lineReader.readVectorLine<std::string>();
            // Line metadata saved?
            uint32_t numLines = 0;
            if (linesInfo.size() == 1) {
                numLines = sgl::fromString<uint32_t>(linesInfo.at(0));
            } else if (linesInfo.size() == 2) {
                boost::algorithm::to_lower(linesInfo.at(0));
                if (boost::ends_with(linesInfo.at(0), "major")) {
                    loadedPsIndices.push_back(0);
                } else if (boost::ends_with(linesInfo.at(0), "medium")) {
                    loadedPsIndices.push_back(1);
                } else if (boost::ends_with(linesInfo.at(0), "minor")) {
                    loadedPsIndices.push_back(2);
                } else {
                    sgl::Logfile::get()->writeError(
                            std::string() + "ERROR in loadStressTrajectoriesFromDat_v2: "
                            + "Invalid principal stress identifier \"" + linesInfo.at(0) + "\".");
                }
                numLines = sgl::fromString<uint32_t>(linesInfo.at(1));
            } else {
                sgl::Logfile::get()->writeError(
                        std::string() + "ERROR in loadStressTrajectoriesFromDat_v2: "
                        + "Invalid line metadata in file \"" + filename + "\".");
            }
            trajectories.resize(numLines);
            stressTrajectoriesData.resize(numLines);
            bandPointsListLeft.resize(numLines);
            bandPointsListRight.resize(numLines);
            for (uint32_t lineIdx = 0; lineIdx < numLines; lineIdx++) {
                Trajectory& trajectory = trajectories.at(lineIdx);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(lineIdx);
                std::vector<glm::vec3>& bandPointsLeft = bandPointsListLeft.at(lineIdx);
                std::vector<glm::vec3>& bandPointsRight = bandPointsListRight.at(lineIdx);

                std::vector<std::string> firstLineVector = lineReader.readVectorLine<std::string>();
                if (firstLineVector.size() != 2) {
                    sgl::Logfile::get()->writeError(
                            std::string() + "ERROR in loadStressTrajectoriesFromDat_v2: "
                            + "Invalid per line metadata in file \"" + filename + "\".");
                }
                uint32_t lineLength = sgl::fromString<uint32_t>(firstLineVector.at(0));
                float hierarchyLevel = sgl::fromString<float>(firstLineVector.at(1));
                stressTrajectoryData.hierarchyLevel = hierarchyLevel;
                trajectory.positions.reserve(lineLength);
                trajectory.attributes.resize(1);
                trajectory.attributes.front().reserve(lineLength);
                bandPointsLeft.reserve(lineLength);
                bandPointsRight.reserve(lineLength);
                std::vector<float> positionData = lineReader.readVectorLine<float>(lineLength * 3);
                std::vector<float> bandVertexData = lineReader.readVectorLine<float>(lineLength * 6);
                std::vector<float> scalarFieldData = lineReader.readVectorLine<float>(lineLength);

                for (uint32_t pointIdx = 0; pointIdx < lineLength; pointIdx++) {
                    trajectory.positions.push_back(glm::vec3(
                            positionData.at(pointIdx * 3),
                            positionData.at(pointIdx * 3 + 1),
                            positionData.at(pointIdx * 3 + 2)));
                    bandPointsLeft.push_back(glm::vec3(
                            bandVertexData.at(pointIdx * 6 + 0),
                            bandVertexData.at(pointIdx * 6 + 1),
                            bandVertexData.at(pointIdx * 6 + 2)));
                    bandPointsRight.push_back(glm::vec3(
                            bandVertexData.at(pointIdx * 6 + 3),
                            bandVertexData.at(pointIdx * 6 + 4),
                            bandVertexData.at(pointIdx * 6 + 5)));
                    trajectory.attributes.at(0).push_back(scalarFieldData.at(pointIdx));
                }
            }

            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                geometryByteSize += trajectory.positions.size() * sizeof(float) * 3;
                geometryByteSize += sizeof(float); // hierarchy level
                geometryByteSize += stressTrajectoryData.majorPs.size() * sizeof(float);
                geometryByteSize += stressTrajectoryData.mediumPs.size() * sizeof(float);
                geometryByteSize += stressTrajectoryData.minorPs.size() * sizeof(float);
                geometryByteSize += stressTrajectoryData.majorPsDir.size() * sizeof(float) * 3;
                geometryByteSize += stressTrajectoryData.mediumPsDir.size() * sizeof(float) * 3;
                geometryByteSize += stressTrajectoryData.minorPsDir.size() * sizeof(float) * 3;
                for (const std::vector<float>& attributes : trajectory.attributes) {
                    geometryByteSize += attributes.size() * sizeof(float);
                }
                geometryByteSize += bandPointsListLeft.at(trajectoryIdx).size() * sizeof(float) * 3;
                geometryByteSize += bandPointsListRight.at(trajectoryIdx).size() * sizeof(float) * 3;
            }

            trajectoriesPs.emplace_back(trajectories);
            stressTrajectoriesDataPs.emplace_back(stressTrajectoriesData);
            bandPointsListLeftPs.emplace_back(bandPointsListLeft);
            bandPointsListRightPs.emplace_back(bandPointsListRight);
            psIdx++;
        }
    }

    std::cout << "Size of line geometry data (MiB): " << (geometryByteSize / (1024.0 * 1024.0)) << std::endl;
}
