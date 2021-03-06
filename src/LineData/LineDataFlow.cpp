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

#include "LineDataFlow.hpp"

LineDataFlow::LineDataFlow(sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineData(transferFunctionWindow, DATA_SET_TYPE_FLOW_LINES) {
    // Bands not supported by flow lines at the moment.
    if (linePrimitiveMode == LINE_PRIMITIVES_BAND || linePrimitiveMode == LINE_PRIMITIVES_TUBE_BAND) {
        linePrimitiveMode = LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH;
    }
    lineDataWindowName = "Line Data (Flow)";
}

LineDataFlow::~LineDataFlow() {
}

bool LineDataFlow::loadFromFile(
        const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
        glm::mat4* transformationMatrixPtr) {
    this->fileNames = fileNames;
    attributeNames = dataSetInformation.attributeNames;
    Trajectories trajectories;
    trajectories = loadFlowTrajectoriesFromFile(
            fileNames.front(), attributeNames, true,
            false, transformationMatrixPtr);
    bool dataLoaded = !trajectories.empty();

    if (dataLoaded) {
        for (size_t attrIdx = attributeNames.size(); attrIdx < getNumAttributes(); attrIdx++) {
            attributeNames.push_back(std::string() + "Attribute #" + std::to_string(attrIdx + 1));
        }
        setTrajectoryData(trajectories);
        modelBoundingBox = computeTrajectoriesAABB3(trajectories);
        //recomputeHistogram(); ///< Called after data is loaded using LineDataRequester.
    }

    return dataLoaded;
}

void LineDataFlow::setTrajectoryData(const Trajectories& trajectories) {
    this->trajectories = trajectories;

    sgl::Logfile::get()->writeInfo(
            std::string() + "Number of lines: " + std::to_string(getNumLines()));
    sgl::Logfile::get()->writeInfo(
            std::string() + "Number of line points: " + std::to_string(getNumLinePoints()));
    sgl::Logfile::get()->writeInfo(
            std::string() + "Number of line segments: " + std::to_string(getNumLineSegments()));

    colorLegendWidgets.clear();
    colorLegendWidgets.resize(attributeNames.size());
    for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
        colorLegendWidgets.at(i).setPositionIndex(0, 1);
    }

    minMaxAttributeValues.clear();
    for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
        float minAttr = std::numeric_limits<float>::max();
        float maxAttr = std::numeric_limits<float>::lowest();
        for (const Trajectory& trajectory : trajectories) {
            for (float val : trajectory.attributes.at(i)) {
                minAttr = std::min(minAttr, val);
                maxAttr = std::max(maxAttr, val);
            }
        }
        minMaxAttributeValues.push_back(glm::vec2(minAttr, maxAttr));
        colorLegendWidgets[i].setAttributeMinValue(minAttr);
        colorLegendWidgets[i].setAttributeMaxValue(maxAttr);
        colorLegendWidgets[i].setAttributeDisplayName(
                std::string() + attributeNames.at(i));
    }
    //normalizeTrajectoriesVertexAttributes(this->trajectories);

    dirty = true;
}

void LineDataFlow::recomputeHistogram() {
    assert(colorLegendWidgets.size() == attributeNames.size());

    std::vector<float> attributeList;
    for (const Trajectory& trajectory : trajectories) {
        for (float val : trajectory.attributes.at(selectedAttributeIndex)) {
            attributeList.push_back(val);
        }
    }
    glm::vec2 minMaxAttributes = minMaxAttributeValues.at(selectedAttributeIndex);
    transferFunctionWindow.computeHistogram(attributeList, minMaxAttributes.x, minMaxAttributes.y);

    recomputeColorLegend();
}

size_t LineDataFlow::getNumAttributes() {
    size_t numAttributes = 0;
    if (!trajectories.empty()) {
        numAttributes = trajectories.front().attributes.size();
    }
    return numAttributes;
}

size_t LineDataFlow::getNumLines() {
    return trajectories.size();
}

size_t LineDataFlow::getNumLinePoints() {
    size_t numLinePoints = 0;
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        numLinePoints += trajectory.positions.size();
    }
    return numLinePoints;
}

size_t LineDataFlow::getNumLineSegments() {
    size_t numLineSegments = 0;
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        numLineSegments += trajectory.positions.size() - 1ull;
    }
    return numLineSegments;
}


void LineDataFlow::iterateOverTrajectories(std::function<void(const Trajectory&)> callback) {
    for (const Trajectory& trajectory : trajectories) {
        callback(trajectory);
    }
}

void LineDataFlow::filterTrajectories(std::function<bool(const Trajectory&)> callback) {
    size_t trajectoryIdx = 0;
    for (const Trajectory& trajectory : trajectories) {
        if (callback(trajectory)) {
            filteredTrajectories.at(trajectoryIdx) = true;
        }
        trajectoryIdx++;
    }
}

void LineDataFlow::resetTrajectoryFilter()  {
    if (filteredTrajectories.empty()) {
        filteredTrajectories.resize(trajectories.size(), false);
    } else {
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            filteredTrajectories.at(trajectoryIdx) = false;
        }
    }
}

Trajectories LineDataFlow::filterTrajectoryData() {
    Trajectories trajectoriesFiltered;
    trajectoriesFiltered.reserve(trajectories.size());
    size_t trajectoryIndex = 0;
    for (const Trajectory& trajectory : trajectories) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIndex)) {
            trajectoryIndex++;
            continue;
        }

        Trajectory trajectoryFiltered;
        trajectoryFiltered.attributes.resize(attributeNames.size());
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

        trajectoryIndex++;
    }
    return trajectoriesFiltered;
}

std::vector<std::vector<glm::vec3>> LineDataFlow::getFilteredLines() {
    std::vector<std::vector<glm::vec3>> linesFiltered;
    linesFiltered.reserve(trajectories.size());
    size_t trajectoryIndex = 0;
    for (const Trajectory& trajectory : trajectories) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIndex)) {
            trajectoryIndex++;
            continue;
        }

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

        trajectoryIndex++;
    }
    return linesFiltered;
}


// --- Retrieve data for rendering. ---

TubeRenderData LineDataFlow::getTubeRenderData() {
    rebuildInternalRepresentationIfNecessary();

    std::vector<std::vector<glm::vec3>> lineCentersList;
    std::vector<std::vector<float>> lineAttributesList;
    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;

    lineCentersList.resize(trajectories.size());
    lineAttributesList.resize(trajectories.size());
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }

        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
        assert(attributes.size() == trajectory.positions.size());
        std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
        std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            lineCenters.push_back(trajectory.positions.at(i));
            lineAttributes.push_back(attributes.at(i));
        }
    }

    createLineTubesRenderDataCPU(
            lineCentersList, lineAttributesList,
            lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);

    TubeRenderData tubeRenderData;

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

TubeRenderDataProgrammableFetch LineDataFlow::getTubeRenderDataProgrammableFetch() {
    rebuildInternalRepresentationIfNecessary();
    TubeRenderDataProgrammableFetch tubeRenderData;

    // 1. Compute all tangents.
    std::vector<std::vector<glm::vec3>> lineCentersList;
    std::vector<std::vector<float>> lineAttributesList;
    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;

    lineCentersList.resize(trajectories.size());
    lineAttributesList.resize(trajectories.size());
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }

        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
        assert(attributes.size() == trajectory.positions.size());
        std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
        std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            lineCenters.push_back(trajectory.positions.at(i));
            lineAttributes.push_back(attributes.at(i));
        }
    }

    createLineTubesRenderDataCPU(
            lineCentersList, lineAttributesList,
            lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);

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
        linePointData.at(i).principalStressIndex = 0;
    }

    tubeRenderData.linePointsBuffer = sgl::Renderer->createGeometryBuffer(
            linePointData.size() * sizeof(LinePointDataProgrammableFetch), (void*)&linePointData.front(),
            sgl::SHADER_STORAGE_BUFFER);

    return tubeRenderData;
}

TubeRenderDataOpacityOptimization LineDataFlow::getTubeRenderDataOpacityOptimization() {
    rebuildInternalRepresentationIfNecessary();

    std::vector<std::vector<glm::vec3>> lineCentersList;
    std::vector<std::vector<float>> lineAttributesList;
    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;

    lineCentersList.resize(trajectories.size());
    lineAttributesList.resize(trajectories.size());
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }

        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
        assert(attributes.size() == trajectory.positions.size());
        std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
        std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            lineCenters.push_back(trajectory.positions.at(i));
            lineAttributes.push_back(attributes.at(i));
        }
    }

    createLineTubesRenderDataCPU(
            lineCentersList, lineAttributesList,
            lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);

    TubeRenderDataOpacityOptimization tubeRenderData;

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
