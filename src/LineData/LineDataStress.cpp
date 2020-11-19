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
#include <Graphics/Shader/ShaderManager.hpp>

#include "Loaders/TrajectoryFile.hpp"
#include "Renderers/Tubes/Tubes.hpp"
#include "Renderers/OIT/OpacityOptimizationRenderer.hpp"

#include <Utils/File/Logfile.hpp>

#include "Loaders/DegeneratePointsDatLoader.hpp"
#include "SearchStructures/KdTree.hpp"
#include "LineDataStress.hpp"

bool LineDataStress::useMajorPS = true;
bool LineDataStress::useMediumPS = true;
bool LineDataStress::useMinorPS = true;
bool LineDataStress::usePrincipalStressDirectionIndex = false;

const char* const stressDirectionNames[] = { "Major", "Medium", "Minor" };

LineDataStress::LineDataStress(sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineData(transferFunctionWindow, DATA_SET_TYPE_STRESS_LINES) {
    setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
}

LineDataStress::~LineDataStress() {
}

void LineDataStress::setRenderingMode(RenderingMode renderingMode) {
    LineData::setRenderingMode(renderingMode);
    rendererSupportsTransparency = renderingMode != RENDERING_MODE_ALL_LINES_OPAQUE;
}

bool LineDataStress::renderGui(bool isRasterizer) {
    bool shallReloadGatherShader = LineData::renderGui(isRasterizer);

    bool usedPsChanged = false;
    usedPsChanged |= ImGui::Checkbox("Major", &useMajorPS); ImGui::SameLine();
    usedPsChanged |= ImGui::Checkbox("Medium", &useMediumPS); ImGui::SameLine();
    usedPsChanged |= ImGui::Checkbox("Minor", &useMinorPS);
    if (usedPsChanged) {
        setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
        shallReloadGatherShader = true;
    }
    if (ImGui::Checkbox("Use Principal Stress Direction Index", &usePrincipalStressDirectionIndex)) {
        dirty = true;
        shallReloadGatherShader = true;
    }

    bool recomputeOpacityOptimization = false;

    if (hasLineHierarchy) {
        if (ImGui::Checkbox("Use Hierarchy Culling", &useLineHierarchy)) {
            dirty = true;
            shallReloadGatherShader = true;
        }
        if (!rendererSupportsTransparency && useLineHierarchy) {
            for (int i = 0; i < 3; i++) {
                if (ImGui::SliderFloat(
                        stressDirectionNames[i], &lineHierarchySliderValues[i], 0.0f, 1.0f)) {
                    reRender = true;
                    recomputeOpacityOptimization = true;
                }
            }
        }
        if (rendererSupportsTransparency && useLineHierarchy) {
            for (int i = 0; i < 3; i++) {
                if (ImGui::SliderFloat2(
                        stressDirectionNames[i], &lineHierarchySliderValuesTransparency[i][0],
                        0.0f, 1.0f)) {
                    reRender = true;
                    recomputeOpacityOptimization = true;
                }
            }
        }
    }

    if (lineRenderer && renderingMode == RENDERING_MODE_OPACITY_OPTIMIZATION && recomputeOpacityOptimization) {
        static_cast<OpacityOptimizationRenderer*>(lineRenderer)->onHasMoved();
    }

    return shallReloadGatherShader;
}

bool LineDataStress::renderGuiWindow(bool isRasterizer) {
    colorLegendWidget.renderGui();
    return false;
}

void LineDataStress::setClearColor(const sgl::Color& clearColor) {
    colorLegendWidget.setClearColor(clearColor);
}

bool LineDataStress::loadFromFile(
        const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
        glm::mat4* transformationMatrixPtr) {
    hasLineHierarchy = useLineHierarchy = !dataSetInformation.filenamesStressLineHierarchy.empty();
    attributeNames = dataSetInformation.attributeNames;
    std::vector<Trajectories> trajectoriesPs;
    std::vector<StressTrajectoriesData> stressTrajectoriesDataPs;
    sgl::AABB3 oldAABB;
    loadStressTrajectoriesFromFile(
            fileNames, dataSetInformation.filenamesStressLineHierarchy, trajectoriesPs, stressTrajectoriesDataPs,
            true, true, &oldAABB, transformationMatrixPtr);
    bool dataLoaded = !trajectoriesPs.empty();

    if (dataLoaded) {
        setStressTrajectoryData(trajectoriesPs, stressTrajectoriesDataPs);
        if (!dataSetInformation.degeneratePointsFilename.empty()) {
            std::vector<glm::vec3> degeneratePoints;
            loadDegeneratePointsFromDat(
                    dataSetInformation.degeneratePointsFilename, degeneratePoints);
            normalizeVertexPositions(degeneratePoints, oldAABB, transformationMatrixPtr);
            setDegeneratePoints(degeneratePoints, attributeNames);
        }
        modelBoundingBox = computeTrajectoriesPsAABB3(trajectoriesPs);

        for (size_t attrIdx = attributeNames.size(); attrIdx < getNumAttributes(); attrIdx++) {
            attributeNames.push_back(std::string() + "Attribute #" + std::to_string(attrIdx + 1));
        }
        recomputeHistogram();
    }

    return dataLoaded;
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

// Exponential kernel: f_1(x,y) = exp(-||x-y||_2 / l), l \in \mathbb{R}
float exponentialKernel(const glm::vec3& pt0, const glm::vec3& pt1, float lengthScale) {
    glm::vec3 diff = pt0 - pt1;
    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    return std::exp(-dist / lengthScale);
}

// Squared exponential kernel: f_2(x,y) = exp(-||x-y||_2^2 / (2*l^2)), l \in \mathbb{R}
float squaredExponentialKernel(const glm::vec3& pt0, const glm::vec3& pt1, float lengthScale) {
    glm::vec3 diff = pt0 - pt1;
    float dist = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
    return std::exp(-dist / (2.0f * lengthScale * lengthScale));
}

void LineDataStress::setDegeneratePoints(
        const std::vector<glm::vec3>& degeneratePoints, std::vector<std::string>& attributeNames) {
    this->degeneratePoints = degeneratePoints;
    if (degeneratePoints.size() == 0) {
        return;
    }

    // Build a search structure on the degenerate points.
    KdTree kdTree;
    std::vector<IndexedPoint> indexedPoints;
    std::vector<IndexedPoint*> indexedPointsPointers;
    indexedPoints.resize(degeneratePoints.size());
    indexedPointsPointers.reserve(degeneratePoints.size());
    for (size_t i = 0; i < degeneratePoints.size(); i++) {
        IndexedPoint* indexedPoint = &indexedPoints.at(i);
        indexedPoint->index = i;
        indexedPoint->position = degeneratePoints.at(i);
        indexedPointsPointers.push_back(indexedPoint);
    }
    kdTree.build(indexedPointsPointers);

    // TODO: Dependent on AABB?
    const float lengthScale = 0.02f;

    // Find for all line points the distance to the closest degenerate point.
    for (Trajectories& trajectories : trajectoriesPs) {
        for (Trajectory& trajectory : trajectories) {
            std::vector<float> distanceMeasuresExponentialKernel;
            std::vector<float> distanceMeasuresSquaredExponentialKernel;
            distanceMeasuresExponentialKernel.resize(trajectory.positions.size());
            distanceMeasuresSquaredExponentialKernel.resize(trajectory.positions.size());
            #pragma omp parallel for shared(trajectory, kdTree, degeneratePoints, lengthScale, distanceMeasuresExponentialKernel, distanceMeasuresSquaredExponentialKernel) default(none)
            for (size_t linePointIdx = 0; linePointIdx < trajectory.positions.size(); linePointIdx++) {
                const glm::vec3& linePoint = trajectory.positions.at(linePointIdx);

                IndexedPoint* nearestNeighbor = kdTree.findNearestNeighbor(linePoint);
                float distanceExponentialKernel = exponentialKernel(
                        linePoint, nearestNeighbor->position, lengthScale);
                float distanceSquaredExponentialKernel = squaredExponentialKernel(
                        linePoint, nearestNeighbor->position, lengthScale);

                distanceMeasuresExponentialKernel.at(linePointIdx) = distanceExponentialKernel;
                distanceMeasuresSquaredExponentialKernel.at(linePointIdx) = distanceSquaredExponentialKernel;
            }
            trajectory.attributes.push_back(distanceMeasuresExponentialKernel);
            trajectory.attributes.push_back(distanceMeasuresSquaredExponentialKernel);
        }
    }

    attributeNames.push_back("Distance Exponential Kernel");
    attributeNames.push_back("Distance Squared Exponential Kernel");
}

void LineDataStress::setUsedPsDirections(const std::vector<bool>& usedPsDirections) {
    this->usedPsDirections = usedPsDirections;
    dirty = true;
}

void LineDataStress::recomputeHistogram() {
    std::vector<float> attributeList;
    for (const Trajectories& trajectories : trajectoriesPs) {
        for (const Trajectory& trajectory : trajectories) {
            for (float val : trajectory.attributes.at(selectedAttributeIndex)) {
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

sgl::ShaderProgramPtr LineDataStress::reloadGatherShader() {
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->addPreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX", "");
    }
    if (useLineHierarchy) {
        sgl::ShaderManager->addPreprocessorDefine("USE_LINE_HIERARCHY_LEVEL", "");
    }
    if (rendererSupportsTransparency) {
        sgl::ShaderManager->addPreprocessorDefine("USE_TRANSPARENCY", "");
    }
    sgl::ShaderProgramPtr gatherShader = LineData::reloadGatherShader();
    if (rendererSupportsTransparency) {
        sgl::ShaderManager->removePreprocessorDefine("USE_TRANSPARENCY");
    }
    if (useLineHierarchy) {
        sgl::ShaderManager->removePreprocessorDefine("USE_LINE_HIERARCHY_LEVEL");
    }
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->removePreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX");
    }
    return gatherShader;
}

TubeRenderData LineDataStress::getTubeRenderData() {
    rebuildInternalRepresentationIfNecessary();
    TubeRenderData tubeRenderData;

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;
    std::vector<uint32_t> vertexPrincipalStressIndices;
    std::vector<float> vertexLineHierarchyLevels;

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(psIdx);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(psIdx);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
                if (hasLineHierarchy) {
                    vertexLineHierarchyLevels.push_back(stressTrajectoryData.hierarchyLevel);
                }
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
            lineIndices.size()*sizeof(uint32_t), (void*)&lineIndices.front(), sgl::INDEX_BUFFER);

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

    // Add the principal stress index buffer.
    tubeRenderData.vertexPrincipalStressIndexBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPrincipalStressIndices.size()*sizeof(uint32_t),
            (void*)&vertexPrincipalStressIndices.front(), sgl::VERTEX_BUFFER);

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        tubeRenderData.vertexLineHierarchyLevelBuffer = sgl::Renderer->createGeometryBuffer(
                vertexLineHierarchyLevels.size()*sizeof(float),
                (void*)&vertexLineHierarchyLevels.front(), sgl::VERTEX_BUFFER);
    }

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
    std::vector<float> vertexLineHierarchyLevels;

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(psIdx);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(psIdx);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
                if (hasLineHierarchy) {
                    vertexLineHierarchyLevels.push_back(stressTrajectoryData.hierarchyLevel);
                }
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

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        tubeRenderData.lineHierarchyLevelsBuffer = sgl::Renderer->createGeometryBuffer(
                vertexLineHierarchyLevels.size()*sizeof(float),
                (void*)&vertexLineHierarchyLevels.front(), sgl::SHADER_STORAGE_BUFFER);
    }

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
    std::vector<float> vertexLineHierarchyLevels;

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(psIdx);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(psIdx);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
                if (hasLineHierarchy) {
                    vertexLineHierarchyLevels.push_back(stressTrajectoryData.hierarchyLevel);
                }
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
            lineIndices.size()*sizeof(uint32_t), (void*)&lineIndices.front(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);

    // Add the attribute buffer.
    tubeRenderData.vertexAttributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), (void*)&vertexAttributes.front(), sgl::VERTEX_BUFFER);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = sgl::Renderer->createGeometryBuffer(
            vertexTangents.size()*sizeof(glm::vec3), (void*)&vertexTangents.front(), sgl::VERTEX_BUFFER);

    // Add the principal stress index buffer.
    tubeRenderData.vertexPrincipalStressIndexBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPrincipalStressIndices.size()*sizeof(uint32_t),
            (void*)&vertexPrincipalStressIndices.front(), sgl::VERTEX_BUFFER);

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        tubeRenderData.vertexLineHierarchyLevelBuffer = sgl::Renderer->createGeometryBuffer(
                vertexLineHierarchyLevels.size()*sizeof(float),
                (void*)&vertexLineHierarchyLevels.front(), sgl::VERTEX_BUFFER);
    }

    return tubeRenderData;
}

PointRenderData LineDataStress::getDegeneratePointsRenderData() {
    PointRenderData renderData;
    renderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            degeneratePoints.size()*sizeof(glm::vec3), (void*)&degeneratePoints.front(),
            sgl::VERTEX_BUFFER);
    return renderData;
}

sgl::ShaderAttributesPtr LineDataStress::getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader) {
    sgl::ShaderAttributesPtr shaderAttributes;

    if (useProgrammableFetch) {
        TubeRenderDataProgrammableFetch tubeRenderData = this->getTubeRenderDataProgrammableFetch();
        linePointDataSSBO = tubeRenderData.linePointsBuffer;
        lineHierarchyLevelsSSBO = tubeRenderData.lineHierarchyLevelsBuffer;

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    } else {
        TubeRenderData tubeRenderData = this->getTubeRenderData();
        linePointDataSSBO = sgl::GeometryBufferPtr();
        lineHierarchyLevelsSSBO = sgl::GeometryBufferPtr();

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);

        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        shaderAttributes->addGeometryBuffer(
                tubeRenderData.vertexPositionBuffer, "vertexPosition",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexAttributeBuffer, "vertexAttribute",
                sgl::ATTRIB_FLOAT, 1);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexNormalBuffer, "vertexNormal",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexTangentBuffer, "vertexTangent",
                sgl::ATTRIB_FLOAT, 3);
        if (tubeRenderData.vertexPrincipalStressIndexBuffer) {
            shaderAttributes->addGeometryBufferOptional(
                    tubeRenderData.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex",
                    sgl::ATTRIB_UNSIGNED_INT,
                    1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);
        }
        if (tubeRenderData.vertexLineHierarchyLevelBuffer) {
            shaderAttributes->addGeometryBufferOptional(
                    tubeRenderData.vertexLineHierarchyLevelBuffer, "vertexLineHierarchyLevel",
                    sgl::ATTRIB_FLOAT, 1);
        }
    }

    return shaderAttributes;
}

void LineDataStress::setUniformGatherShaderData_AllPasses() {
    LineData::setUniformGatherShaderData_AllPasses();
    if (useLineHierarchy && useProgrammableFetch) {
        sgl::ShaderManager->bindShaderStorageBuffer(3, lineHierarchyLevelsSSBO);
    }
}

void LineDataStress::setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader) {
    LineData::setUniformGatherShaderData_Pass(gatherShader);
    if (useLineHierarchy) {
        if (!rendererSupportsTransparency) {
            gatherShader->setUniform("lineHierarchySlider", lineHierarchySliderValues);
        } else {
            lineHierarchySliderValuesLower[0] = lineHierarchySliderValuesTransparency[0][0];
            lineHierarchySliderValuesLower[1] = lineHierarchySliderValuesTransparency[1][0];
            lineHierarchySliderValuesLower[2] = lineHierarchySliderValuesTransparency[2][0];
            lineHierarchySliderValuesUpper[0] = lineHierarchySliderValuesTransparency[0][1];
            lineHierarchySliderValuesUpper[1] = lineHierarchySliderValuesTransparency[1][1];
            lineHierarchySliderValuesUpper[2] = lineHierarchySliderValuesTransparency[2][1];
            gatherShader->setUniform("lineHierarchySliderLower", lineHierarchySliderValuesLower);
            gatherShader->setUniform("lineHierarchySliderUpper", lineHierarchySliderValuesUpper);
        }
    }
}
