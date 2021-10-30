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

#include <Utils/File/Logfile.hpp>
#include <Graphics/Renderer.hpp>

#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#endif

#include "Loaders/TrajectoryFile.hpp"
#include "Renderers/LineRenderer.hpp"
#include "Renderers/Tubes/Tubes.hpp"

#include "LineDataFlow.hpp"

LineDataFlow::LineDataFlow(sgl::TransferFunctionWindow& transferFunctionWindow)
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
        setTrajectoryData(trajectories);
    }

    return dataLoaded;
}

void LineDataFlow::setTrajectoryData(const Trajectories& trajectories) {
    this->trajectories = trajectories;

    for (size_t attrIdx = attributeNames.size(); attrIdx < getNumAttributes(); attrIdx++) {
        attributeNames.push_back(std::string() + "Attribute #" + std::to_string(attrIdx + 1));
    }

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
        minMaxAttributeValues.emplace_back(minAttr, maxAttr);
        colorLegendWidgets[i].setAttributeMinValue(minAttr);
        colorLegendWidgets[i].setAttributeMaxValue(maxAttr);
        colorLegendWidgets[i].setAttributeDisplayName(
                std::string() + attributeNames.at(i));
    }
    //normalizeTrajectoriesVertexAttributes(this->trajectories);

    numTotalTrajectoryPoints = 0;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(+: numTotalTrajectoryPoints) shared(trajectories) default(none)
#endif
    for (size_t i = 0; i < trajectories.size(); i++) {
        const Trajectory& trajectory = trajectories.at(i);
        numTotalTrajectoryPoints += trajectory.positions.size();
    }

    modelBoundingBox = computeTrajectoriesAABB3(trajectories);

    dirty = true;
}

bool LineDataFlow::getIsSmallDataSet() const {
    return numTotalTrajectoryPoints <= SMALL_DATASET_LINE_POINTS_MAX;
}

void LineDataFlow::recomputeHistogram() {
    std::vector<float> attributeList;
    for (const Trajectory& trajectory : trajectories) {
        for (float val : trajectory.attributes.at(selectedAttributeIndex)) {
            attributeList.push_back(val);
        }
    }
    glm::vec2 minMaxAttributes = minMaxAttributeValues.at(selectedAttributeIndex);
    transferFunctionWindow.computeHistogram(attributeList, minMaxAttributes.x, minMaxAttributes.y);
    selectedAttributeIndex = attributeNames.size() - 1;
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
    rebuildInternalRepresentationIfNecessary();

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
            glm::vec3 tangent;
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
    rebuildInternalRepresentationIfNecessary();

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
            glm::vec3 tangent;
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
            sizeof(uint32_t)*lineIndices.size(), lineIndices.data(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);

    // Add the attribute buffer.
    tubeRenderData.vertexAttributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), vertexAttributes.data(), sgl::VERTEX_BUFFER);

    // Add the normal buffer.
    tubeRenderData.vertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), vertexNormals.data(), sgl::VERTEX_BUFFER);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = sgl::Renderer->createGeometryBuffer(
            vertexTangents.size()*sizeof(glm::vec3), vertexTangents.data(), sgl::VERTEX_BUFFER);

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
            sizeof(uint32_t) * fetchIndices.size(), fetchIndices.data(), sgl::INDEX_BUFFER);

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
            linePointData.size() * sizeof(LinePointDataProgrammableFetch), linePointData.data(),
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
            sizeof(uint32_t)*lineIndices.size(), lineIndices.data(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), vertexPositions.data(), sgl::VERTEX_BUFFER);

    // Add the attribute buffer.
    tubeRenderData.vertexAttributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), vertexAttributes.data(), sgl::VERTEX_BUFFER);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = sgl::Renderer->createGeometryBuffer(
            vertexTangents.size()*sizeof(glm::vec3), vertexTangents.data(), sgl::VERTEX_BUFFER);

    return tubeRenderData;
}


#ifdef USE_VULKAN_INTEROP
VulkanTubeTriangleRenderData LineDataFlow::getVulkanTubeTriangleRenderData(bool raytracing) {
    rebuildInternalRepresentationIfNecessary();
    if (vulkanTubeTriangleRenderData.vertexBuffer) {
        return vulkanTubeTriangleRenderData;
    }

    std::vector<std::vector<glm::vec3>> lineCentersList;

    std::vector<uint32_t> tubeTriangleIndices;
    std::vector<glm::vec3> lineTangents;
    std::vector<glm::vec3> lineNormals;
    std::vector<TubeTriangleVertexData> tubeTriangleVertexDataList;
    std::vector<LinePointReference> linePointReferences;
    std::vector<TubeLinePointData> tubeTriangleLinePointDataList;

    lineCentersList.resize(trajectories.size());
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }
        lineCentersList.at(trajectoryIdx) = trajectories.at(trajectoryIdx).positions;
    }

    if (useCappedTubes) {
        createCappedTriangleTubesRenderDataCPU(
                lineCentersList, LineRenderer::getLineWidth() * 0.5f, tubeNumSubdivisions, false,
                tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                0, lineTangents, lineNormals);
    } else {
        createTriangleTubesRenderDataCPU(
                lineCentersList, LineRenderer::getLineWidth() * 0.5f, tubeNumSubdivisions,
                tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                0, lineTangents, lineNormals);
    }

    tubeTriangleLinePointDataList.resize(linePointReferences.size());
    for (size_t i = 0; i < linePointReferences.size(); i++) {
        LinePointReference& linePointReference = linePointReferences.at(i);
        TubeLinePointData& tubeTriangleLinePointData = tubeTriangleLinePointDataList.at(i);
        Trajectory& trajectory = trajectories.at(linePointReference.trajectoryIndex);
        std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);

        tubeTriangleLinePointData.linePosition = trajectory.positions.at(linePointReference.linePointIndex);
        tubeTriangleLinePointData.lineAttribute = attributes.at(linePointReference.linePointIndex);
        tubeTriangleLinePointData.lineTangent = lineTangents.at(i);
        tubeTriangleLinePointData.lineNormal = lineNormals.at(i);
    }


    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    vulkanTubeTriangleRenderData = {};

    if (tubeTriangleIndices.empty()) {
        return vulkanTubeTriangleRenderData;
    }

    uint32_t indexBufferFlags = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    uint32_t vertexBufferFlags = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    if (raytracing) {
        indexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
        vertexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
    }

    vulkanTubeTriangleRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeTriangleIndices.size() * sizeof(uint32_t), tubeTriangleIndices.data(),
            indexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeTriangleRenderData.vertexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeTriangleVertexDataList.size() * sizeof(TubeTriangleVertexData),
            tubeTriangleVertexDataList.data(),
            vertexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeTriangleRenderData.linePointBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeTriangleLinePointDataList.size() * sizeof(TubeLinePointData),
            tubeTriangleLinePointDataList.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    return vulkanTubeTriangleRenderData;
}

VulkanTubeAabbRenderData LineDataFlow::getVulkanTubeAabbRenderData() {
    rebuildInternalRepresentationIfNecessary();
    if (vulkanTubeAabbRenderData.indexBuffer) {
        return vulkanTubeAabbRenderData;
    }

    glm::vec3 lineWidthOffset(LineRenderer::getLineWidth() * 0.5f);

    std::vector<uint32_t> lineSegmentPointIndices;
    std::vector<sgl::AABB3> lineSegmentAabbs;
    std::vector<TubeLinePointData> tubeLinePointDataList;
    lineSegmentPointIndices.reserve(getNumLineSegments() * 2);
    lineSegmentAabbs.reserve(getNumLineSegments());
    tubeLinePointDataList.reserve(getNumLinePoints());
    uint32_t lineSegmentIndexCounter = 0;
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }
        Trajectory& trajectory = trajectories.at(trajectoryIdx);

        glm::vec3 lastLineNormal(1.0f, 0.0f, 0.0f);
        size_t numValidLinePoints = 0;
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            glm::vec3 tangent;
            if (i == 0) {
                tangent = trajectory.positions[i + 1] - trajectory.positions[i];
            } else if (i + 1 == trajectory.positions.size()) {
                tangent = trajectory.positions[i] - trajectory.positions[i - 1];
            } else {
                tangent = trajectory.positions[i + 1] - trajectory.positions[i - 1];
            }
            float lineSegmentLength = glm::length(tangent);

            if (lineSegmentLength < 0.0001f) {
                // In case the two vertices are almost identical, just skip this path line segment.
                continue;
            }
            tangent = glm::normalize(tangent);

            glm::vec3 helperAxis = lastLineNormal;
            if (glm::length(glm::cross(helperAxis, tangent)) < 0.01f) {
                // If tangent == lastNormal
                helperAxis = glm::vec3(0.0f, 1.0f, 0.0f);
                if (glm::length(glm::cross(helperAxis, tangent)) < 0.01f) {
                    // If tangent == helperAxis
                    helperAxis = glm::vec3(0.0f, 0.0f, 1.0f);
                }
            }
            glm::vec3 normal = glm::normalize(helperAxis - glm::dot(helperAxis, tangent) * tangent); // Gram-Schmidt
            lastLineNormal = normal;

            TubeLinePointData linePointData{};
            linePointData.linePosition = trajectory.positions.at(i);
            linePointData.lineAttribute = trajectory.attributes.at(selectedAttributeIndex).at(i);
            linePointData.lineTangent = tangent;
            linePointData.lineNormal = lastLineNormal;
            tubeLinePointDataList.push_back(linePointData);

            numValidLinePoints++;
        }

        if (numValidLinePoints == 1) {
            // Only one vertex left -> output nothing (tube consisting only of one point).
            tubeLinePointDataList.pop_back();
            continue;
        }

        for (size_t pointIdx = 1; pointIdx < numValidLinePoints; pointIdx++) {
            lineSegmentPointIndices.push_back(lineSegmentIndexCounter + pointIdx - 1);
            lineSegmentPointIndices.push_back(lineSegmentIndexCounter + pointIdx);

            const glm::vec3& pt0 = tubeLinePointDataList.at(lineSegmentIndexCounter + pointIdx - 1).linePosition;
            const glm::vec3& pt1 = tubeLinePointDataList.at(lineSegmentIndexCounter + pointIdx).linePosition;

            sgl::AABB3 aabb;
            aabb.min = glm::min(pt0, pt1) - lineWidthOffset;
            aabb.max = glm::max(pt0, pt1) + lineWidthOffset;
            lineSegmentAabbs.push_back(aabb);
        }
        lineSegmentIndexCounter += uint32_t(numValidLinePoints);
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    uint32_t indexBufferFlags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT
            | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
            | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
    uint32_t vertexBufferFlags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
            | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
            | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;

    vulkanTubeAabbRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineSegmentPointIndices.size() * sizeof(uint32_t), lineSegmentPointIndices.data(),
            indexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeAabbRenderData.aabbBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineSegmentAabbs.size() * sizeof(VkAabbPositionsKHR), lineSegmentAabbs.data(),
            vertexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeAabbRenderData.linePointBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeLinePointDataList.size() * sizeof(TubeLinePointData),
            tubeLinePointDataList.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    return vulkanTubeAabbRenderData;
}
#endif

void LineDataFlow::getTriangleMesh(
        std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes) {
    rebuildInternalRepresentationIfNecessary();

    std::vector<std::vector<glm::vec3>> lineCentersList;

    std::vector<glm::vec3> lineTangents;
    std::vector<glm::vec3> lineNormals;
    std::vector<TubeTriangleVertexData> tubeTriangleVertexDataList;
    std::vector<LinePointReference> linePointReferences;

    lineCentersList.resize(trajectories.size());
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }
        lineCentersList.at(trajectoryIdx) = trajectories.at(trajectoryIdx).positions;
    }

    if (useCappedTubes) {
        createCappedTriangleTubesRenderDataCPU(
                lineCentersList, LineRenderer::getLineWidth() * 0.5f, tubeNumSubdivisions, false,
                triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                0, lineTangents, lineNormals);
    } else {
        createTriangleTubesRenderDataCPU(
                lineCentersList, LineRenderer::getLineWidth() * 0.5f, tubeNumSubdivisions,
                triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                0, lineTangents, lineNormals);
    }

    vertexPositions.reserve(tubeTriangleVertexDataList.size());
    vertexNormals.reserve(tubeTriangleVertexDataList.size());
    vertexAttributes.reserve(tubeTriangleVertexDataList.size());
    for (TubeTriangleVertexData& tubeTriangleVertexData : tubeTriangleVertexDataList) {
        vertexPositions.push_back(tubeTriangleVertexData.vertexPosition);
        vertexNormals.push_back(tubeTriangleVertexData.vertexNormal);

        LinePointReference& linePointReference = linePointReferences.at(
                tubeTriangleVertexData.vertexLinePointIndex & 0x7FFFFFFFu);
        Trajectory& trajectory = trajectories.at(linePointReference.trajectoryIndex);
        std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
        float attributeValue = attributes.at(linePointReference.linePointIndex);
        vertexAttributes.push_back(attributeValue);
    }
}

void LineDataFlow::getTriangleMesh(
        std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) {
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    getTriangleMesh(triangleIndices, vertexPositions, vertexNormals, vertexAttributes);
}
