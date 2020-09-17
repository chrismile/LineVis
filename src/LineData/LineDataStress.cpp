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

#include <Graphics/Renderer.hpp>

#include "Loaders/TrajectoryFile.hpp"
#include "Renderers/Tubes/Tubes.hpp"

#include <Utils/File/Logfile.hpp>

#include "LineDataStress.hpp"

LineDataStress::LineDataStress(TransferFunctionWindow &transferFunctionWindow)
        : LineData(transferFunctionWindow, DATA_SET_TYPE_FLOW_LINES) {
}

LineDataStress::~LineDataStress() {
}

void LineDataStress::setStressTrajectoryData(
        const std::vector<Trajectories>& trajectoriesPs,
        const std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs) {
    this->trajectoriesPs = trajectoriesPs;
    this->stressTrajectoriesDataPs = stressTrajectoriesDataPs;

    sgl::Logfile::get()->writeInfo(
            std::string() + "Number of lines: " + std::to_string(getNumLines()));
    sgl::Logfile::get()->writeInfo(
            std::string() + "Number of line points: " + std::to_string(getNumLinePoints()));
    sgl::Logfile::get()->writeInfo(
            std::string() + "Number of line segments: " + std::to_string(getNumLineSegments()));

    dirty = true;
}

void LineDataStress::setUsedPsDirections(const std::vector<bool>& usedPsDirections) {
    this->usedPsDirections = usedPsDirections;
    dirty = true;
}

void LineDataStress::recomputeHistogram() {
    std::vector<float> attributeList;
    for (const Trajectories& trajectories : trajectoriesPs) {
        for (const Trajectory& trajectory : trajectories) {
            for (float val : trajectory.attributes.at(qualityMeasureIdx)) {
                attributeList.push_back(val);
            }
        }
    }
    transferFunctionWindow.computeHistogram(attributeList, 0.0f, 1.0f);
}

size_t LineDataStress::getNumAttributes() {
    size_t numAttributes = 0;
    if (!trajectoriesPs.empty() && !trajectoriesPs.front().empty()) {
        numAttributes = trajectoriesPs.front().front().attributes.size();
    }
    return numAttributes;
}

size_t LineDataStress::getNumLines() {
    size_t numLines = 0;
    for (Trajectories& trajectories : trajectoriesPs) {
        numLines += trajectories.size();
    }
    return numLines;
}

size_t LineDataStress::getNumLinePoints() {
    size_t numLinePoints = 0;
    for (Trajectories& trajectories : trajectoriesPs) {
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            numLinePoints += trajectory.positions.size();
        }
    }
    return numLinePoints;
}

size_t LineDataStress::getNumLineSegments() {
    size_t numLineSegments = 0;
    for (Trajectories& trajectories : trajectoriesPs) {
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            numLineSegments += trajectory.positions.size() - 1ull;
        }
    }
    return numLineSegments;
}


Trajectories LineDataStress::filterTrajectoryData() {
    Trajectories trajectoriesFiltered;

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        const Trajectories& trajectories = trajectoriesPs.at(psIdx);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        for (const Trajectory &trajectory : trajectories) {
            Trajectory trajectoryFiltered;
            size_t n = trajectory.positions.size();

            int numValidLinePoints = 0;
            for (size_t i = 0; i < n; i++) {
                glm::vec3 tangent, normal;
                if (i == 0) {
                    tangent = trajectory.positions[i + 1] - trajectory.positions[i];
                } else if (i == n - 1) {
                    tangent = trajectory.positions[i] - trajectory.positions[i - 1];
                } else {
                    tangent = (trajectory.positions[i + 1] - trajectory.positions[i - 1]);
                }
                float lineSegmentLength = glm::length(tangent);

                if (lineSegmentLength < 0.0001f) {
                    // In case the two vertices are almost identical, just skip this path line segment
                    continue;
                }

                trajectoryFiltered.positions.push_back(trajectory.positions.at(i));
                for (size_t attrIdx = 0; attrIdx < trajectory.attributes.size(); attrIdx++) {
                    trajectoryFiltered.attributes.at(attrIdx).push_back(trajectory.attributes.at(attrIdx).at(i));
                }
                numValidLinePoints++;
            }

            if (numValidLinePoints > 1) {
                trajectoriesFiltered.push_back(trajectoryFiltered);
            }
        }
    }
    return trajectoriesFiltered;
}

std::vector<std::vector<glm::vec3>> LineDataStress::getFilteredLines() {
    std::vector<std::vector<glm::vec3>> linesFiltered;
    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        const Trajectories& trajectories = trajectoriesPs.at(psIdx);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        for (const Trajectory &trajectory : trajectories) {
            std::vector<glm::vec3> lineFiltered;
            size_t n = trajectory.positions.size();

            int numValidLinePoints = 0;
            for (size_t i = 0; i < n; i++) {
                glm::vec3 tangent, normal;
                if (i == 0) {
                    tangent = trajectory.positions[i + 1] - trajectory.positions[i];
                } else if (i == n - 1) {
                    tangent = trajectory.positions[i] - trajectory.positions[i - 1];
                } else {
                    tangent = (trajectory.positions[i + 1] - trajectory.positions[i - 1]);
                }
                float lineSegmentLength = glm::length(tangent);

                if (lineSegmentLength < 0.0001f) {
                    // In case the two vertices are almost identical, just skip this path line segment
                    continue;
                }

                lineFiltered.push_back(trajectory.positions.at(i));
                numValidLinePoints++;
            }

            if (numValidLinePoints > 1) {
                linesFiltered.push_back(lineFiltered);
            }
        }
    }
    return linesFiltered;
}


std::vector<Trajectories> LineDataStress::filterTrajectoryPsData() {
    std::vector<Trajectories> trajectoriesPsFiltered;
    trajectoriesPsFiltered.reserve(trajectoriesPs.size());
    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        const Trajectories& trajectories = trajectoriesPs.at(psIdx);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories trajectoriesFiltered;
        trajectoriesFiltered.reserve(trajectories.size());
        for (const Trajectory& trajectory : trajectories) {
            Trajectory trajectoryFiltered;
            size_t n = trajectory.positions.size();

            int numValidLinePoints = 0;
            for (size_t i = 0; i < n; i++) {
                glm::vec3 tangent, normal;
                if (i == 0) {
                    tangent = trajectory.positions[i+1] - trajectory.positions[i];
                } else if (i == n - 1) {
                    tangent = trajectory.positions[i] - trajectory.positions[i-1];
                } else {
                    tangent = (trajectory.positions[i+1] - trajectory.positions[i-1]);
                }
                float lineSegmentLength = glm::length(tangent);

                if (lineSegmentLength < 0.0001f) {
                    // In case the two vertices are almost identical, just skip this path line segment
                    continue;
                }

                trajectoryFiltered.positions.push_back(trajectory.positions.at(i));
                for (size_t attrIdx = 0; attrIdx < trajectory.attributes.size(); attrIdx++) {
                    trajectoryFiltered.attributes.at(attrIdx).push_back(trajectory.attributes.at(attrIdx).at(i));
                }
                numValidLinePoints++;
            }

            if (numValidLinePoints > 1) {
                trajectoriesFiltered.push_back(trajectoryFiltered);
            }
        }

        trajectoriesPsFiltered.push_back(trajectoriesFiltered);
    }
    return trajectoriesPsFiltered;
}

std::vector<std::vector<std::vector<glm::vec3>>> LineDataStress::getFilteredPrincipalStressLines() {
    std::vector<std::vector<std::vector<glm::vec3>>> linesPsFiltered;
    linesPsFiltered.reserve(trajectoriesPs.size());
    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        const Trajectories& trajectories = trajectoriesPs.at(psIdx);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        std::vector<std::vector<glm::vec3>> linesFiltered;
        linesFiltered.reserve(trajectories.size());
        for (const Trajectory& trajectory : trajectories) {
            std::vector<glm::vec3> lineFiltered;
            size_t n = trajectory.positions.size();

            int numValidLinePoints = 0;
            for (size_t i = 0; i < n; i++) {
                glm::vec3 tangent, normal;
                if (i == 0) {
                    tangent = trajectory.positions[i+1] - trajectory.positions[i];
                } else if (i == n - 1) {
                    tangent = trajectory.positions[i] - trajectory.positions[i-1];
                } else {
                    tangent = (trajectory.positions[i+1] - trajectory.positions[i-1]);
                }
                float lineSegmentLength = glm::length(tangent);

                if (lineSegmentLength < 0.0001f) {
                    // In case the two vertices are almost identical, just skip this path line segment
                    continue;
                }

                lineFiltered.push_back(trajectory.positions.at(i));
                numValidLinePoints++;
            }

            if (numValidLinePoints > 1) {
                linesFiltered.push_back(lineFiltered);
            }
        }

        trajectoriesPs.push_back(trajectories);
    }

    return linesPsFiltered;
}


// --- Retrieve data for rendering. ---

TubeRenderData LineDataStress::getTubeRenderData() {
    rebuildInternalRepresentationIfNecessary();
    TubeRenderData tubeRenderData;

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;
    std::vector<uint32_t> vertexPrincipalStressIndices;

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(psIdx);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(qualityMeasureIdx);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
            }
        }

        size_t numVerticesOld = vertexPositions.size();
        createLineTubesRenderDataCPU(
                lineCentersList, lineAttributesList,
                lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);
        size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
        for (size_t i = 0; i < numVerticesAdded; i++) {
            vertexPrincipalStressIndices.push_back(psIdx);
        }
    }

    // Add the index buffer.
    tubeRenderData.indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*lineIndices.size(), (void*)&lineIndices.front(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);

    // Add the attribute buffer.
    tubeRenderData.vertexAttributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), (void*)&vertexAttributes.front(), sgl::VERTEX_BUFFER);

    // Add the normal buffer.
    tubeRenderData.vertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), (void*)&vertexNormals.front(), sgl::VERTEX_BUFFER);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = sgl::Renderer->createGeometryBuffer(
            vertexTangents.size()*sizeof(glm::vec3), (void*)&vertexTangents.front(), sgl::VERTEX_BUFFER);

    return tubeRenderData;
}

TubeRenderDataProgrammableFetch LineDataStress::getTubeRenderDataProgrammableFetch() {
    rebuildInternalRepresentationIfNecessary();
    TubeRenderDataProgrammableFetch tubeRenderData;

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;
    std::vector<uint32_t> vertexPrincipalStressIndices;

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(psIdx);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(qualityMeasureIdx);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
            }
        }

        size_t numVerticesOld = vertexPositions.size();
        createLineTubesRenderDataCPU(
                lineCentersList, lineAttributesList,
                lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);
        size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
        for (size_t i = 0; i < numVerticesAdded; i++) {
            vertexPrincipalStressIndices.push_back(psIdx);
        }
    }

    // 2. Construct the triangle topology for programmable fetching.
    std::vector<uint32_t> fetchIndices;
    fetchIndices.reserve(lineIndices.size()*3);
    // Iterate over all line segments
    for (size_t i = 0; i < lineIndices.size(); i += 2) {
        uint32_t base0 = lineIndices.at(i)*2;
        uint32_t base1 = lineIndices.at(i+1)*2;
        // 0,2,3,0,3,1
        fetchIndices.push_back(base0);
        fetchIndices.push_back(base1);
        fetchIndices.push_back(base1+1);
        fetchIndices.push_back(base0);
        fetchIndices.push_back(base1+1);
        fetchIndices.push_back(base0+1);
    }
    tubeRenderData.indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t) * fetchIndices.size(), (void*)&fetchIndices.front(), sgl::INDEX_BUFFER);

    // 3. Add the point data for all line points.
    std::vector<LinePointDataProgrammableFetch> linePointData;
    linePointData.resize(vertexPositions.size());
    for (size_t i = 0; i < vertexPositions.size(); i++) {
        linePointData.at(i).vertexPosition = vertexPositions.at(i);
        linePointData.at(i).vertexAttribute = vertexAttributes.at(i);
        linePointData.at(i).vertexTangent = vertexTangents.at(i);
        linePointData.at(i).principalStressIndex = vertexPrincipalStressIndices.at(i);
    }

    tubeRenderData.linePointsBuffer = sgl::Renderer->createGeometryBuffer(
            linePointData.size() * sizeof(LinePointDataProgrammableFetch), (void*)&linePointData.front(),
            sgl::SHADER_STORAGE_BUFFER);

    return tubeRenderData;
}

TubeRenderDataOpacityOptimization LineDataStress::getTubeRenderDataOpacityOptimization() {
    rebuildInternalRepresentationIfNecessary();
    TubeRenderDataOpacityOptimization tubeRenderData;

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;
    std::vector<uint32_t> vertexPrincipalStressIndices;

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(psIdx);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(qualityMeasureIdx);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
            }
        }

        size_t numVerticesOld = vertexPositions.size();
        createLineTubesRenderDataCPU(
                lineCentersList, lineAttributesList,
                lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);
        size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
        for (size_t i = 0; i < numVerticesAdded; i++) {
            vertexPrincipalStressIndices.push_back(psIdx);
        }
    }

    // Add the index buffer.
    tubeRenderData.indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*lineIndices.size(), (void*)&lineIndices.front(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);

    // Add the attribute buffer.
    tubeRenderData.vertexAttributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), (void*)&vertexAttributes.front(), sgl::VERTEX_BUFFER);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = sgl::Renderer->createGeometryBuffer(
            vertexTangents.size()*sizeof(glm::vec3), (void*)&vertexTangents.front(), sgl::VERTEX_BUFFER);

    return tubeRenderData;
}
