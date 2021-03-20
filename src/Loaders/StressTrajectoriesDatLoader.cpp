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

#include "Utils/TriangleNormals.hpp"
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
                stressTrajectoryData.hierarchyLevels.push_back(lineHierarchyLevel);
            }

            psIdx++;
        }
    }
}

void loadStressTrajectoriesFromDat_v1(
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
                        std::string() + "ERROR in loadStressTrajectoriesFromDat_v1: Invalid line metadata in file \""
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
                stressTrajectoryData.hierarchyLevels.push_back(hierarchyLevel);
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



void parseOutlineMeshHull(
        sgl::LineReader& lineReader,
        std::vector<uint32_t>& simulationMeshOutlineTriangleIndices,
        std::vector<glm::vec3>& simulationMeshOutlineVertexPositions) {
    std::vector<std::string> numVerticesLine = lineReader.readVectorLine<std::string>();
    if (numVerticesLine.size() != 2 || numVerticesLine.front() != "#Vertices") {
        sgl::Logfile::get()->writeError("Error in parseOutlineMeshHull: Invalid vertex information.");
    }
    uint32_t numVertices = sgl::fromString<uint32_t>(numVerticesLine.at(1));
    for (uint32_t vertexIdx = 0; vertexIdx < numVertices; vertexIdx++) {
        std::vector<float> vertexPosition = lineReader.readVectorLine<float>(3);
        assert(vertexPosition.size() == 3);
        simulationMeshOutlineVertexPositions.push_back(
                glm::vec3(vertexPosition.at(0), vertexPosition.at(1), vertexPosition.at(2)));
    }

    std::vector<std::string> numFacesLine = lineReader.readVectorLine<std::string>();
    if (numFacesLine.size() != 2 || numFacesLine.front() != "#Faces") {
        sgl::Logfile::get()->writeError("Error in parseOutlineMeshHull: Invalid face information.");
    }
    uint32_t numFaces = sgl::fromString<uint32_t>(numFacesLine.at(1));
    for (uint32_t faceIdx = 0; faceIdx < numFaces; faceIdx++) {
        std::vector<uint32_t> faceIndices = lineReader.readVectorLine<uint32_t>(4);
        assert(faceIndices.size() == 4);

        simulationMeshOutlineTriangleIndices.push_back(faceIndices.at(0));
        simulationMeshOutlineTriangleIndices.push_back(faceIndices.at(1));
        simulationMeshOutlineTriangleIndices.push_back(faceIndices.at(2));

        simulationMeshOutlineTriangleIndices.push_back(faceIndices.at(0));
        simulationMeshOutlineTriangleIndices.push_back(faceIndices.at(2));
        simulationMeshOutlineTriangleIndices.push_back(faceIndices.at(3));
    }
}

void loadStressTrajectoriesFromDat_v3(
        const std::vector<std::string>& filenamesTrajectories,
        std::vector<int>& loadedPsIndices, MeshType& meshType,
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListRightPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListRightPs,
        std::vector<uint32_t>& simulationMeshOutlineTriangleIndices,
        std::vector<glm::vec3>& simulationMeshOutlineVertexPositions) {
    trajectoriesPs.reserve(filenamesTrajectories.size());
    stressTrajectoriesDataPs.reserve(filenamesTrajectories.size());
    bandPointsUnsmoothedListLeftPs.reserve(filenamesTrajectories.size());
    bandPointsUnsmoothedListRightPs.reserve(filenamesTrajectories.size());
    bandPointsSmoothedListLeftPs.reserve(filenamesTrajectories.size());
    bandPointsSmoothedListRightPs.reserve(filenamesTrajectories.size());
    size_t geometryByteSize = 0;

    size_t psIdx = 0;
    for (size_t fileIdx = 0; fileIdx < filenamesTrajectories.size(); fileIdx++) {
        const std::string& filename = filenamesTrajectories.at(fileIdx);

        sgl::LineReader lineReader(filename);
        while (lineReader.isLineLeft()) {
            Trajectories trajectories;
            StressTrajectoriesData stressTrajectoriesData;
            std::vector<std::vector<glm::vec3>> bandPointsUnsmoothedListLeft;
            std::vector<std::vector<glm::vec3>> bandPointsUnsmoothedListRight;
            std::vector<std::vector<glm::vec3>> bandPointsSmoothedListLeft;
            std::vector<std::vector<glm::vec3>> bandPointsSmoothedListRight;
            std::vector<std::string> linesInfo = lineReader.readVectorLine<std::string>();

            if (linesInfo.front() == "#Outline") {
                if (linesInfo.size() == 1) {
                    meshType = MeshType::CARTESIAN;
                } else {
                    if (linesInfo.at(1) == "Cartesian") {
                        meshType = MeshType::CARTESIAN;
                    } else {
                        meshType = MeshType::UNSTRUCTURED;
                    }
                }
                parseOutlineMeshHull(
                        lineReader, simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions);
                continue;
            }

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
            bandPointsUnsmoothedListLeft.resize(numLines);
            bandPointsUnsmoothedListRight.resize(numLines);
            bandPointsSmoothedListLeft.resize(numLines);
            bandPointsSmoothedListRight.resize(numLines);
            for (uint32_t lineIdx = 0; lineIdx < numLines; lineIdx++) {
                Trajectory& trajectory = trajectories.at(lineIdx);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(lineIdx);
                std::vector<glm::vec3>& bandPointsUnsmoothedLeft = bandPointsUnsmoothedListLeft.at(lineIdx);
                std::vector<glm::vec3>& bandPointsUnsmoothedRight = bandPointsUnsmoothedListRight.at(lineIdx);
                std::vector<glm::vec3>& bandPointsSmoothedLeft = bandPointsSmoothedListLeft.at(lineIdx);
                std::vector<glm::vec3>& bandPointsSmoothedRight = bandPointsSmoothedListRight.at(lineIdx);

                std::vector<std::string> firstLineVector = lineReader.readVectorLine<std::string>();
                if (firstLineVector.size() == 0) {
                    sgl::Logfile::get()->writeError(
                            std::string() + "ERROR in loadStressTrajectoriesFromDat_v2: "
                            + "Invalid per line metadata in file \"" + filename + "\".");
                }
                uint32_t lineLength = sgl::fromString<uint32_t>(firstLineVector.at(0));

                // Add the hierarchy levels.
                for (int hierarchyIdx = 1; hierarchyIdx < std::max(int(firstLineVector.size()), 5); hierarchyIdx++) {
                    float hierarchyLevel = sgl::fromString<float>(firstLineVector.at(hierarchyIdx));
                    stressTrajectoryData.hierarchyLevels.push_back(hierarchyLevel);
                }
                if (firstLineVector.size() == 9) {
                    stressTrajectoryData.appearanceOrder = sgl::fromString<int>(firstLineVector.at(5)) - 1;
                    stressTrajectoryData.seedPosition = glm::vec3(
                            sgl::fromString<float>(firstLineVector.at(6)),
                            sgl::fromString<float>(firstLineVector.at(7)),
                            sgl::fromString<float>(firstLineVector.at(8)));
                }

                trajectory.positions.reserve(lineLength);
                bandPointsUnsmoothedLeft.reserve(lineLength);
                bandPointsUnsmoothedRight.reserve(lineLength);
                bandPointsSmoothedLeft.reserve(lineLength);
                bandPointsSmoothedRight.reserve(lineLength);
                std::vector<float> positionData = lineReader.readVectorLine<float>(
                        lineLength * 3);
                std::vector<float> bandVertexDataUnsmoothed = lineReader.readVectorLine<float>(
                        lineLength * 6);
                std::vector<float> bandVertexDataSmoothed = lineReader.readVectorLine<float>(
                        lineLength * 6);

                for (uint32_t pointIdx = 0; pointIdx < lineLength; pointIdx++) {
                    trajectory.positions.push_back(glm::vec3(
                            positionData.at(pointIdx * 3),
                            positionData.at(pointIdx * 3 + 1),
                            positionData.at(pointIdx * 3 + 2)));
                    bandPointsUnsmoothedLeft.push_back(glm::vec3(
                            bandVertexDataUnsmoothed.at(pointIdx * 6 + 0),
                            bandVertexDataUnsmoothed.at(pointIdx * 6 + 1),
                            bandVertexDataUnsmoothed.at(pointIdx * 6 + 2)));
                    bandPointsUnsmoothedRight.push_back(glm::vec3(
                            bandVertexDataUnsmoothed.at(pointIdx * 6 + 3),
                            bandVertexDataUnsmoothed.at(pointIdx * 6 + 4),
                            bandVertexDataUnsmoothed.at(pointIdx * 6 + 5)));
                    bandPointsSmoothedLeft.push_back(glm::vec3(
                            bandVertexDataSmoothed.at(pointIdx * 6 + 0),
                            bandVertexDataSmoothed.at(pointIdx * 6 + 1),
                            bandVertexDataSmoothed.at(pointIdx * 6 + 2)));
                    bandPointsSmoothedRight.push_back(glm::vec3(
                            bandVertexDataSmoothed.at(pointIdx * 6 + 3),
                            bandVertexDataSmoothed.at(pointIdx * 6 + 4),
                            bandVertexDataSmoothed.at(pointIdx * 6 + 5)));
                }

                trajectory.attributes.resize(8);
                for (int varIdx = 0; varIdx < 8; varIdx++) {
                    trajectory.attributes.at(varIdx).reserve(lineLength);
                    std::vector<float> scalarFieldData = lineReader.readVectorLine<float>(lineLength);
                    for (uint32_t pointIdx = 0; pointIdx < lineLength; pointIdx++) {
                        trajectory.attributes.at(varIdx).push_back(scalarFieldData.at(pointIdx));
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
                geometryByteSize += bandPointsUnsmoothedListLeft.at(trajectoryIdx).size() * sizeof(float) * 3;
                geometryByteSize += bandPointsUnsmoothedListRight.at(trajectoryIdx).size() * sizeof(float) * 3;
                geometryByteSize += bandPointsSmoothedListLeft.at(trajectoryIdx).size() * sizeof(float) * 3;
                geometryByteSize += bandPointsSmoothedListRight.at(trajectoryIdx).size() * sizeof(float) * 3;
            }

            trajectoriesPs.emplace_back(trajectories);
            stressTrajectoriesDataPs.emplace_back(stressTrajectoriesData);
            bandPointsUnsmoothedListLeftPs.emplace_back(bandPointsUnsmoothedListLeft);
            bandPointsUnsmoothedListRightPs.emplace_back(bandPointsUnsmoothedListRight);
            bandPointsSmoothedListLeftPs.emplace_back(bandPointsSmoothedListLeft);
            bandPointsSmoothedListRightPs.emplace_back(bandPointsSmoothedListRight);
            psIdx++;
        }
    }

    std::cout << "Size of line geometry data (MiB): " << (geometryByteSize / (1024.0 * 1024.0)) << std::endl;
}
