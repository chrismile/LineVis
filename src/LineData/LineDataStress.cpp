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

#include "Utils/TriangleNormals.hpp"
#include "Utils/MeshSmoothing.hpp"
#include "Loaders/DegeneratePointsDatLoader.hpp"
#include "SearchStructures/KdTree.hpp"
#include "Renderers/LineRenderer.hpp"
#include "LineDataStress.hpp"

bool LineDataStress::useMajorPS = true;
bool LineDataStress::useMediumPS = false;
bool LineDataStress::useMinorPS = true;
bool LineDataStress::usePrincipalStressDirectionIndex = true;
std::array<bool, 3> LineDataStress::psUseBands = {true, true, false};
bool LineDataStress::renderThickBands = true;
bool LineDataStress::useSmoothedBands = true;
LineDataStress::LineHierarchyType LineDataStress::lineHierarchyType = LineDataStress::LineHierarchyType::GEO;
glm::vec3 LineDataStress::lineHierarchySliderValues = glm::vec3(1.0f);

const char* const stressDirectionNames[] = { "Major", "Medium", "Minor" };
const char* const lineHierarchyTypeNames[] = { "geo-based", "PS-based", "vM-based", "Length-based" };

LineDataStress::LineDataStress(sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineData(transferFunctionWindow, DATA_SET_TYPE_STRESS_LINES),
          multiVarTransferFunctionWindow("stress", { "reds.xml", "greens.xml", "blues.xml" }) {
    colorLegendWidgets.resize(3);
    for (int psIdx = 0; psIdx < 3; psIdx++) {
        colorLegendWidgets[psIdx].setPositionIndex(psIdx, 3);
    }
    setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
    lineDataWindowName = "Line Data (Stress)";
}

LineDataStress::~LineDataStress() {
}

bool LineDataStress::settingsDiffer(LineData* other) {
    return useLineHierarchy != static_cast<LineDataStress*>(other)->useLineHierarchy || (hasBandsData != hasBandsData);
}

void LineDataStress::update(float dt) {
    if (rendererSupportsTransparency && useLineHierarchy) {
        stressLineHierarchyMappingWidget.update(dt);
    }
    if (usePrincipalStressDirectionIndex) {
        multiVarTransferFunctionWindow.update(dt);
    }
}

void LineDataStress::setRenderingMode(RenderingMode renderingMode) {
    LineData::setRenderingMode(renderingMode);
    rendererSupportsTransparency = renderingMode != RENDERING_MODE_ALL_LINES_OPAQUE;
}

bool LineDataStress::renderGuiRenderer(bool isRasterizer) {
    bool shallReloadGatherShader = LineData::renderGuiRenderer(isRasterizer);
    return shallReloadGatherShader;
}

bool LineDataStress::renderGuiLineData(bool isRasterizer) {
    bool shallReloadGatherShader = LineData::renderGuiLineData(isRasterizer);

    bool usedPsChanged = false;
    usedPsChanged |= ImGui::Checkbox("Major##usedpsmajor", &useMajorPS); ImGui::SameLine();
    usedPsChanged |= ImGui::Checkbox("Medium##usedpsmedium", &useMediumPS); ImGui::SameLine();
    usedPsChanged |= ImGui::Checkbox("Minor##usedpsminor", &useMinorPS);
    if (usedPsChanged) {
        setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
        shallReloadGatherShader = true;
    }

    bool recomputeOpacityOptimization = false;
    if (hasLineHierarchy) {
        if (fileFormatVersion >= 3) {
            if (ImGui::Combo(
                    "Line Hierarchy Type", (int*)&lineHierarchyType,
                    lineHierarchyTypeNames, IM_ARRAYSIZE(lineHierarchyTypeNames))) {
                updateLineHierarchyHistogram();
                dirty = true;
            }
        }
        if (!rendererSupportsTransparency && useLineHierarchy) {
            bool sliderChanged = false;
            for (int psIdx : loadedPsIndices) {
                if (ImGui::SliderFloat(
                        stressDirectionNames[psIdx], &lineHierarchySliderValues[psIdx], 0.0f, 1.0f)) {
                    reRender = true;
                    recomputeOpacityOptimization = true;
                    sliderChanged = true;
                }
            }
            if (sliderChanged) {
                bool useLineHierarchyNew = glm::any(
                        glm::lessThan(lineHierarchySliderValues, glm::vec3(1.0f)));
                if (useLineHierarchy != useLineHierarchyNew) {
                    dirty = true;
                    shallReloadGatherShader = true;
                }
            }
        }
    }

    if (useBands()) {
        ImGui::Text("Render as bands:");
        bool usedBandsChanged = false;
        usedBandsChanged |= ImGui::Checkbox("Major##bandsmajor", &psUseBands[0]); ImGui::SameLine();
        usedBandsChanged |= ImGui::Checkbox("Medium##bandsmedium", &psUseBands[1]); ImGui::SameLine();
        usedBandsChanged |= ImGui::Checkbox("Minor##bandsminor", &psUseBands[2]);
        if (usedBandsChanged) {
            dirty = true;
        }
    }

    if (ImGui::CollapsingHeader(
            "Advanced Settings", nullptr, 0)) {
        if (useBands()) {
            if (ImGui::Checkbox("Render Thick Bands", &renderThickBands)) {
                shallReloadGatherShader = true;
            }
            ImGui::SameLine();

            if (fileFormatVersion >= 3 && ImGui::Checkbox("Smoothed Bands", &useSmoothedBands)) {
                dirty = true;
            }
        }

        if (ImGui::Checkbox("Use Principal Stress Direction Index", &usePrincipalStressDirectionIndex)) {
            dirty = true;
            shallReloadGatherShader = true;
            recomputeColorLegend();
        }

    }

    if (lineRenderer && renderingMode == RENDERING_MODE_OPACITY_OPTIMIZATION && recomputeOpacityOptimization) {
        static_cast<OpacityOptimizationRenderer*>(lineRenderer)->onHasMoved();
    }

    return shallReloadGatherShader;
}

bool LineDataStress::renderGuiRenderingSettings() {
    if (useBands() && ImGui::SliderFloat(
            "Band Width", &LineRenderer::bandWidth, LineRenderer::MIN_LINE_WIDTH, LineRenderer::MAX_LINE_WIDTH,
            "%.4f")) {
        reRender = true;
    }
    return false;
}

bool LineDataStress::renderGuiWindowSecondary(bool isRasterizer) {
    if (rendererSupportsTransparency && useLineHierarchy) {
        bool hierarchyMappingChanged = stressLineHierarchyMappingWidget.renderGui();
        reRender = reRender || hierarchyMappingChanged;
        if (lineRenderer && renderingMode == RENDERING_MODE_OPACITY_OPTIMIZATION) {
            static_cast<OpacityOptimizationRenderer*>(lineRenderer)->onHasMoved();
        }
    }

    if (usePrincipalStressDirectionIndex && multiVarTransferFunctionWindow.renderGui()) {
        reRender = true;
        if (multiVarTransferFunctionWindow.getTransferFunctionMapRebuilt()) {
            onTransferFunctionMapRebuilt();
            if (lineRenderer) {
                lineRenderer->onTransferFunctionMapRebuilt();
            }
        }
    }

    if (usePrincipalStressDirectionIndex && shallRenderColorLegendWidgets) {
        for (int psIdx : loadedPsIndices) {
            colorLegendWidgets.at(psIdx).setAttributeMinValue(
                    multiVarTransferFunctionWindow.getSelectedRangeMin(psIdx));
            colorLegendWidgets.at(psIdx).setAttributeMaxValue(
                    multiVarTransferFunctionWindow.getSelectedRangeMax(psIdx));
            colorLegendWidgets.at(psIdx).renderGui();
        }
        return false;
    } else {
        return LineData::renderGuiWindowSecondary(isRasterizer);
    }
}

void LineDataStress::setClearColor(const sgl::Color& clearColor) {
    LineData::setClearColor(clearColor);
    stressLineHierarchyMappingWidget.setClearColor(clearColor);
    multiVarTransferFunctionWindow.setClearColor(clearColor);
}

void LineDataStress::setUseLinearRGB(bool useLinearRGB) {
    multiVarTransferFunctionWindow.setUseLinearRGB(useLinearRGB);
}

bool LineDataStress::loadFromFile(
        const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
        glm::mat4* transformationMatrixPtr) {
    this->fileNames = fileNames;
    hasLineHierarchy = useLineHierarchy = !dataSetInformation.filenamesStressLineHierarchy.empty();
    if (dataSetInformation.version >= 2) {
        hasLineHierarchy = useLineHierarchy = true;
    }
    attributeNames = dataSetInformation.attributeNames;

    std::vector<Trajectories> trajectoriesPs;
    std::vector<StressTrajectoriesData> stressTrajectoriesDataPs;
    sgl::AABB3 oldAABB;
    MeshType meshType = MeshType::CARTESIAN;
    loadStressTrajectoriesFromFile(
            fileNames, dataSetInformation.filenamesStressLineHierarchy, dataSetInformation.version, loadedPsIndices,
            meshType, trajectoriesPs, stressTrajectoriesDataPs,
            bandPointsUnsmoothedListLeftPs, bandPointsUnsmoothedListRightPs,
            bandPointsSmoothedListLeftPs, bandPointsSmoothedListRightPs,
            simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions,
            true, false, &oldAABB, transformationMatrixPtr);
    hasBandsData = !bandPointsUnsmoothedListLeftPs.empty();
    if (!simulationMeshOutlineTriangleIndices.empty()) {
        normalizeVertexPositions(simulationMeshOutlineVertexPositions, oldAABB, transformationMatrixPtr);
        if (meshType == MeshType::CARTESIAN) {
            laplacianSmoothing(simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions);
        }
        computeSmoothTriangleNormals(
                simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions,
                simulationMeshOutlineVertexNormals);
    }
    bool dataLoaded = !trajectoriesPs.empty();

    if (dataLoaded) {
        setStressTrajectoryData(trajectoriesPs, stressTrajectoriesDataPs);

        seedPoints.resize(getNumLines());
        for (StressTrajectoriesData& stressTrajectoriesData : stressTrajectoriesDataPs) {
            normalizeVertexPositions(degeneratePoints, oldAABB, transformationMatrixPtr);
            for (StressTrajectoryData& stressTrajectoryData : stressTrajectoriesData) {
                normalizeVertexPosition(stressTrajectoryData.seedPosition, oldAABB, transformationMatrixPtr);
                seedPoints.at(stressTrajectoryData.appearanceOrder) = stressTrajectoryData.seedPosition;
            }
        }

        if (!dataSetInformation.degeneratePointsFilename.empty()) {
            std::vector<glm::vec3> degeneratePoints;
            loadDegeneratePointsFromDat(
                    dataSetInformation.degeneratePointsFilename, degeneratePoints);
            normalizeVertexPositions(degeneratePoints, oldAABB, transformationMatrixPtr);
            setDegeneratePoints(degeneratePoints, attributeNames);
        }
        if (!dataSetInformation.meshFilename.empty() && simulationMeshOutlineTriangleIndices.empty()) {
            loadSimulationMeshOutlineFromFile(dataSetInformation.meshFilename, oldAABB, transformationMatrixPtr);
        }
        if (!simulationMeshOutlineTriangleIndices.empty()) {
            shallRenderSimulationMeshBoundary = true;
        }
        fileFormatVersion = dataSetInformation.version;
        modelBoundingBox = computeTrajectoriesPsAABB3(trajectoriesPs);

        std::vector<bool> usedPsDirections = {false, false, false};
        for (size_t i = 0; i < loadedPsIndices.size(); i++) {
            int psIdx = loadedPsIndices.at(i);
            colorLegendWidgets[psIdx].setPositionIndex(int(i), int(loadedPsIndices.size()));
            usedPsDirections.at(psIdx) = true;
        }
        setUsedPsDirections(usedPsDirections);

        // Use bands if possible.
        if (hasBandsData) {
            linePrimitiveMode = LINE_PRIMITIVES_TUBE_BAND;
            tubeNumSubdivisions = 8;
        } else if (useBands()) {
            linePrimitiveMode = LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH;
        }

        //recomputeHistogram(); ///< Called after data is loaded using LineDataRequester.
    }

    return dataLoaded;
}

void LineDataStress::setStressTrajectoryData(
        const std::vector<Trajectories>& trajectoriesPs,
        const std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs) {
    this->trajectoriesPs = trajectoriesPs;
    this->stressTrajectoriesDataPs = stressTrajectoriesDataPs;
    filteredTrajectoriesPs.resize(trajectoriesPs.size());
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
    colorLegendWidgets.resize(std::max(attributeNames.size(), size_t(3)));

    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
        float minAttrTotal = std::numeric_limits<float>::max();
        float maxAttrTotal = std::numeric_limits<float>::lowest();
        int i = 0;
        for (const Trajectories& trajectories : trajectoriesPs) {
            size_t psIdx = loadedPsIndices.at(i);
            float minAttr = std::numeric_limits<float>::max();
            float maxAttr = std::numeric_limits<float>::lowest();
            for (const Trajectory& trajectory : trajectories) {
                for (float val : trajectory.attributes.at(attrIdx)) {
                    minAttr = std::min(minAttr, val);
                    maxAttr = std::max(maxAttr, val);
                }
            }
            minMaxAttributeValuesPs[psIdx].push_back(glm::vec2(minAttr, maxAttr));
            minAttrTotal = std::min(minAttrTotal, minAttr);
            maxAttrTotal = std::max(maxAttrTotal, maxAttr);
            i++;
        }
        minMaxAttributeValues.push_back(glm::vec2(minAttrTotal, maxAttrTotal));
    }

    updateLineHierarchyHistogram();

    dirty = true;
}

void LineDataStress::updateLineHierarchyHistogram() {
    if (hasLineHierarchy) {
        for (size_t i = 0; i < loadedPsIndices.size(); i++) {
            int psIdx = loadedPsIndices.at(i);
            std::vector<float> lineHierarchyLevelValues;
            const StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);
            for (const StressTrajectoryData& stressTrajectoryData : stressTrajectoriesData) {
                lineHierarchyLevelValues.push_back(stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
            }
            stressLineHierarchyMappingWidget.setLineHierarchyLevelValues(psIdx, lineHierarchyLevelValues);
        }
    }
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
    size_t psIdx = 0;
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
        minMaxAttributeValuesPs[psIdx].push_back(glm::vec2(0.0f, 1.0f));
        minMaxAttributeValuesPs[psIdx].push_back(glm::vec2(0.0f, 1.0f));
        psIdx++;
    }

    minMaxAttributeValues.push_back(glm::vec2(0.0f, 1.0f));
    minMaxAttributeValues.push_back(glm::vec2(0.0f, 1.0f));
    attributeNames.push_back("Distance Exponential Kernel");
    attributeNames.push_back("Distance Squared Exponential Kernel");
}

void LineDataStress::setUsedPsDirections(const std::vector<bool>& usedPsDirections) {
    this->usedPsDirections = usedPsDirections;
    useMajorPS = usedPsDirections.at(0);
    useMediumPS = usedPsDirections.at(1);
    useMinorPS = usedPsDirections.at(2);
    if (std::find(loadedPsIndices.begin(), loadedPsIndices.end(), 0) == loadedPsIndices.end()) {
        useMajorPS = false;
    }
    if (std::find(loadedPsIndices.begin(), loadedPsIndices.end(), 1) == loadedPsIndices.end()) {
        useMediumPS = false;
    }
    if (std::find(loadedPsIndices.begin(), loadedPsIndices.end(), 2) == loadedPsIndices.end()) {
        useMinorPS = false;
    }
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
    glm::vec2 minMaxAttributes = minMaxAttributeValues.at(selectedAttributeIndex);
    transferFunctionWindow.computeHistogram(attributeList, minMaxAttributes.x, minMaxAttributes.y);

    std::vector<std::string> attrNamesMultiVarWindow;
    std::vector<std::vector<float>> attributesValuesMultiVarWindow;
    for (int psIdx = 0; psIdx < 3; psIdx++) {
        attrNamesMultiVarWindow.push_back(
                std::string() + attributeNames.at(selectedAttributeIndex) + " (" + stressDirectionNames[psIdx] + ")");
        std::vector<float> attributeValues;
        auto it = std::find(loadedPsIndices.begin(), loadedPsIndices.end(), psIdx);
        if (it != loadedPsIndices.end()) {
            Trajectories& trajectories = trajectoriesPs.at(std::distance(loadedPsIndices.begin(), it));
            for (Trajectory& trajectory : trajectories) {
                for (float& attrVal : trajectory.attributes.at(selectedAttributeIndex)) {
                    attributeValues.push_back(attrVal);
                }
            }
        } else {
            attributeValues = {0.0f, 1.0f};
        }
        attributesValuesMultiVarWindow.push_back(attributeValues);
    }
    multiVarTransferFunctionWindow.setAttributesValues(attrNamesMultiVarWindow, attributesValuesMultiVarWindow);

    recomputeColorLegend();
}

void LineDataStress::recomputeColorLegend() {
    if (usePrincipalStressDirectionIndex) {
        for (size_t i = 0; i < loadedPsIndices.size(); i++) {
            int psIdx = loadedPsIndices.at(i);
            colorLegendWidgets[psIdx].setAttributeDisplayName(
                    std::string() + attributeNames.at(selectedAttributeIndex)
                    + " (" + stressDirectionNames[psIdx] + ")");

            glm::vec2 minMaxAttributes = minMaxAttributeValuesPs[psIdx].at(selectedAttributeIndex);
            colorLegendWidgets[psIdx].setAttributeMinValue(minMaxAttributes.x);
            colorLegendWidgets[psIdx].setAttributeMaxValue(minMaxAttributes.y);
            colorLegendWidgets[psIdx].setPositionIndex(int(i), int(loadedPsIndices.size()));

            /*glm::vec3 baseColor;
            if (psIdx == 0) {
                baseColor = glm::vec3(1.0, 0.0, 0.0);
            } else if (psIdx == 1) {
                baseColor = glm::vec3(0.0, 1.0, 0.0);
            } else {
                baseColor = glm::vec3(0.0, 0.0, 1.0);
            }

            std::vector<sgl::Color> transferFunctionColorMap;
            transferFunctionColorMap.reserve(ColorLegendWidget::STANDARD_MAP_RESOLUTION);
            for (int i = 0; i < ColorLegendWidget::STANDARD_MAP_RESOLUTION; i++) {
                float posFloat = float(i) / float(ColorLegendWidget::STANDARD_MAP_RESOLUTION - 1);
                glm::vec3 color = mix(baseColor, glm::vec3(1.0), pow((1.0 - posFloat), 10.0) * 0.5);
                glm::vec3 color_sRGB = sgl::TransferFunctionWindow::linearRGBTosRGB(color);
                transferFunctionColorMap.push_back(color_sRGB);
            }
            colorLegendWidgets[psIdx].setTransferFunctionColorMap(transferFunctionColorMap);*/
            colorLegendWidgets[psIdx].setTransferFunctionColorMap(
                    multiVarTransferFunctionWindow.getTransferFunctionMap_sRGB(psIdx));
        }
    } else {
        colorLegendWidgets[selectedAttributeIndex].setAttributeDisplayName(
                std::string() + attributeNames.at(selectedAttributeIndex));
        glm::vec2 minMaxAttributes = minMaxAttributeValues.at(selectedAttributeIndex);
        colorLegendWidgets[selectedAttributeIndex].setAttributeMinValue(minMaxAttributes.x);
        colorLegendWidgets[selectedAttributeIndex].setAttributeMaxValue(minMaxAttributes.y);
        colorLegendWidgets[selectedAttributeIndex].setPositionIndex(0, 1);
        LineData::recomputeColorLegend();
    }
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


void LineDataStress::iterateOverTrajectories(std::function<void(const Trajectory&)> callback) {
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        for (const Trajectory& trajectory : trajectoriesPs.at(i)) {
            callback(trajectory);
        }
    }
}

void LineDataStress::filterTrajectories(std::function<bool(const Trajectory&)> callback) {
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        Trajectories& trajectories = trajectoriesPs.at(i);
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);

        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (callback(trajectories.at(trajectoryIdx))) {
                filteredTrajectories.at(trajectoryIdx) = true;
            }
        }
    }
}

void LineDataStress::resetTrajectoryFilter()  {
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        Trajectories & trajectories = trajectoriesPs.at(i);
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);

        if (filteredTrajectories.empty()) {
            filteredTrajectories.resize(trajectories.size(), false);
        } else {
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                filteredTrajectories.at(trajectoryIdx) = false;
            }
        }
    }
}

Trajectories LineDataStress::filterTrajectoryData() {
    Trajectories trajectoriesFiltered;

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        const Trajectories& trajectories = trajectoriesPs.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        size_t trajectoryIdx = 0;
        for (const Trajectory &trajectory : trajectories) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                trajectoryIdx++;
                continue;
            }

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

            trajectoryIdx++;
        }
    }
    return trajectoriesFiltered;
}

std::vector<std::vector<glm::vec3>> LineDataStress::getFilteredLines() {
    std::vector<std::vector<glm::vec3>> linesFiltered;
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        const Trajectories& trajectories = trajectoriesPs.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        size_t trajectoryIdx = 0;
        for (const Trajectory &trajectory : trajectories) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                trajectoryIdx++;
                continue;
            }

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

            trajectoryIdx++;
        }
    }
    return linesFiltered;
}


std::vector<Trajectories> LineDataStress::filterTrajectoryPsData() {
    std::vector<Trajectories> trajectoriesPsFiltered;
    trajectoriesPsFiltered.reserve(trajectoriesPs.size());
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        const Trajectories& trajectories = trajectoriesPs.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories trajectoriesFiltered;
        trajectoriesFiltered.reserve(trajectories.size());
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        size_t trajectoryIdx = 0;
        for (const Trajectory &trajectory : trajectories) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                trajectoryIdx++;
                continue;
            }

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

            trajectoryIdx++;
        }

        trajectoriesPsFiltered.push_back(trajectoriesFiltered);
    }
    return trajectoriesPsFiltered;
}

std::vector<std::vector<std::vector<glm::vec3>>> LineDataStress::getFilteredPrincipalStressLines() {
    std::vector<std::vector<std::vector<glm::vec3>>> linesPsFiltered;
    linesPsFiltered.reserve(trajectoriesPs.size());
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        const Trajectories& trajectories = trajectoriesPs.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        std::vector<std::vector<glm::vec3>> linesFiltered;
        linesFiltered.reserve(trajectories.size());
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        size_t trajectoryIdx = 0;
        for (const Trajectory &trajectory : trajectories) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                trajectoryIdx++;
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

            trajectoryIdx++;
        }

        trajectoriesPs.push_back(trajectories);
    }

    return linesPsFiltered;
}


// --- Retrieve data for rendering. ---

sgl::ShaderProgramPtr LineDataStress::reloadGatherShader() {
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->addPreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX", "");
        sgl::ShaderManager->addPreprocessorDefine("USE_MULTI_VAR_TRANSFER_FUNCTION", "");
    }
    if (useLineHierarchy) {
        sgl::ShaderManager->addPreprocessorDefine("USE_LINE_HIERARCHY_LEVEL", "");
    }
    if (rendererSupportsTransparency) {
        sgl::ShaderManager->addPreprocessorDefine("USE_TRANSPARENCY", "");
    }
    if (useBands() && renderThickBands) {
        sgl::ShaderManager->addPreprocessorDefine("BAND_RENDERING_THICK", "");
    }

    sgl::ShaderProgramPtr gatherShader = LineData::reloadGatherShader();

    if (useBands() && renderThickBands) {
        sgl::ShaderManager->removePreprocessorDefine("BAND_RENDERING_THICK");
    }
    if (rendererSupportsTransparency) {
        sgl::ShaderManager->removePreprocessorDefine("USE_TRANSPARENCY");
    }
    if (useLineHierarchy) {
        sgl::ShaderManager->removePreprocessorDefine("USE_LINE_HIERARCHY_LEVEL");
    }
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->removePreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX");
        sgl::ShaderManager->removePreprocessorDefine("USE_MULTI_VAR_TRANSFER_FUNCTION");
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
    std::vector<uint32_t> vertexLineAppearanceOrders;

    std::vector<std::vector<std::vector<glm::vec3>>> *bandPointsListRightPs;
    if (useBands()) {
        if (useSmoothedBands) {
            bandPointsListRightPs = &bandPointsSmoothedListRightPs;
        } else {
            bandPointsListRightPs = &bandPointsUnsmoothedListRightPs;
        }
    }

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        if (useBands() && psUseBands.at(psIdx)) {
            std::vector<std::vector<glm::vec3>> bandRightVectorList;
            std::vector<std::vector<glm::vec3>>& bandPointsListRight = bandPointsListRightPs->at(i);

            lineCentersList.resize(trajectories.size());
            lineAttributesList.resize(trajectories.size());
            bandRightVectorList.resize(trajectories.size());
            std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                    continue;
                }

                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
                std::vector<glm::vec3>& bandPointsRight = bandPointsListRight.at(trajectoryIdx);
                assert(attributes.size() == trajectory.positions.size());
                std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
                std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
                std::vector<glm::vec3>& bandRightVectors = bandRightVectorList.at(trajectoryIdx);
                for (size_t i = 0; i < trajectory.positions.size(); i++) {
                    lineCenters.push_back(trajectory.positions.at(i));
                    lineAttributes.push_back(attributes.at(i));
                    if (hasLineHierarchy) {
                        vertexLineHierarchyLevels.push_back(
                                stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
                    }
                    vertexLineAppearanceOrders.push_back(stressTrajectoryData.appearanceOrder);
                    bandRightVectors.push_back(bandPointsRight.at(i));
                }
            }

            size_t numVerticesOld = vertexPositions.size();

            for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
                const std::vector<glm::vec3> &lineCenters = lineCentersList.at(lineId);
                const std::vector<float> &lineAttributes = lineAttributesList.at(lineId);
                const std::vector<glm::vec3> &bandRightVectors = bandRightVectorList.at(lineId);
                assert(lineCenters.size() == lineAttributes.size());
                size_t n = lineCenters.size();
                size_t indexOffset = vertexPositions.size();

                if (n < 2) {
                    continue;
                }

                glm::vec3 lastLineNormal(1.0f, 0.0f, 0.0f);
                int numValidLinePoints = 0;
                for (size_t i = 0; i < n; i++) {
                    glm::vec3 tangent, normal;
                    if (i == 0) {
                        tangent = lineCenters[i+1] - lineCenters[i];
                    } else if (i == n - 1) {
                        tangent = lineCenters[i] - lineCenters[i-1];
                    } else {
                        tangent = (lineCenters[i+1] - lineCenters[i-1]);
                    }
                    float lineSegmentLength = glm::length(tangent);

                    if (lineSegmentLength < 0.0001f) {
                        // In case the two vertices are almost identical, just skip this path line segment
                        continue;
                    }
                    tangent = glm::normalize(tangent);

                    normal = glm::cross(bandRightVectors.at(i), tangent);

                    vertexPositions.push_back(lineCenters.at(i));
                    vertexNormals.push_back(normal);
                    vertexTangents.push_back(tangent);
                    vertexAttributes.push_back(lineAttributes.at(i));
                    numValidLinePoints++;
                }

                if (numValidLinePoints == 1) {
                    // Only one vertex left -> Output nothing (tube consisting only of one point).
                    vertexPositions.pop_back();
                    vertexNormals.pop_back();
                    vertexTangents.pop_back();
                    vertexAttributes.pop_back();
                    continue;
                }

                // Create indices
                for (int i = 0; i < numValidLinePoints-1; i++) {
                    lineIndices.push_back(indexOffset + i);
                    lineIndices.push_back(indexOffset + i + 1);
                }
            }

            size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
            for (size_t i = 0; i < numVerticesAdded; i++) {
                vertexPrincipalStressIndices.push_back(psIdx);
            }
        } else {
            // Compute all tangents.
            lineCentersList.resize(trajectories.size());
            lineAttributesList.resize(trajectories.size());
            std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                    continue;
                }

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
                        vertexLineHierarchyLevels.push_back(
                                stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
                    }
                    vertexLineAppearanceOrders.push_back(stressTrajectoryData.appearanceOrder);
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
    }

    if (useBands()) {
        for (size_t i = 0; i < trajectoriesPs.size(); i++) {
            // 1. Compute all tangents.
            std::vector<std::vector<glm::vec3>> lineCentersList;
            std::vector<std::vector<float>> lineAttributesList;

        }
    } else {
        for (size_t i = 0; i < trajectoriesPs.size(); i++) {
            int psIdx = loadedPsIndices.at(i);
            if (!usedPsDirections.at(psIdx)) {
                continue;
            }

            Trajectories& trajectories = trajectoriesPs.at(i);
            StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

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

    // Add the line appearance order buffer.
    tubeRenderData.vertexLineAppearanceOrderBuffer = sgl::Renderer->createGeometryBuffer(
            vertexLineAppearanceOrders.size()*sizeof(uint32_t),
            (void*)&vertexLineAppearanceOrders.front(), sgl::VERTEX_BUFFER);

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

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }

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
                    vertexLineHierarchyLevels.push_back(
                            stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
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

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

        // 1. Compute all tangents.
        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

        lineCentersList.resize(trajectories.size());
        lineAttributesList.resize(trajectories.size());
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }

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
                    vertexLineHierarchyLevels.push_back(
                            stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
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

BandRenderData LineDataStress::getBandRenderData() {
    rebuildInternalRepresentationIfNecessary();
    BandRenderData bandRenderData;

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<float> vertexAttributes;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<glm::vec3> vertexOffsetsLeft;
    std::vector<glm::vec3> vertexOffsetsRight;
    std::vector<uint32_t> vertexPrincipalStressIndices;
    std::vector<float> vertexLineHierarchyLevels;
    std::vector<uint32_t> vertexLineAppearanceOrders;

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

        if (psUseBands.at(psIdx)) {
            std::vector<std::vector<std::vector<glm::vec3>>>* bandPointsListLeftPs;
            std::vector<std::vector<std::vector<glm::vec3>>>* bandPointsListRightPs;
            if (useSmoothedBands) {
                bandPointsListLeftPs = &bandPointsSmoothedListLeftPs;
                bandPointsListRightPs = &bandPointsSmoothedListRightPs;
            } else {
                bandPointsListLeftPs = &bandPointsUnsmoothedListLeftPs;
                bandPointsListRightPs = &bandPointsUnsmoothedListRightPs;
            }
            std::vector<std::vector<glm::vec3>>& bandPointsListLeft = bandPointsListLeftPs->at(i);
            std::vector<std::vector<glm::vec3>>& bandPointsListRight = bandPointsListRightPs->at(i);

            std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                    continue;
                }

                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
                std::vector<glm::vec3>& bandPointsLeft = bandPointsListLeft.at(trajectoryIdx);
                std::vector<glm::vec3>& bandPointsRight = bandPointsListRight.at(trajectoryIdx);
                assert(attributes.size() == trajectory.positions.size());
                assert(attributes.size() == bandPointsLeft.size());
                assert(attributes.size() == bandPointsRight.size());

                if (trajectory.positions.size() < 2) {
                    continue;
                }

                size_t indexStart = vertexPositions.size();
                int n = int(trajectory.positions.size());
                for (size_t i = 0; i < trajectory.positions.size(); i++) {
                    vertexPositions.push_back(trajectory.positions.at(i));
                    vertexOffsetsLeft.push_back(bandPointsLeft.at(i));
                    vertexOffsetsRight.push_back(bandPointsRight.at(i));

                    glm::vec3 vertexTangent;
                    if (i == 0) {
                        vertexTangent = trajectory.positions[i+1] - trajectory.positions[i];
                    } else if (i == n - 1) {
                        vertexTangent = trajectory.positions[i] - trajectory.positions[i-1];
                    } else {
                        vertexTangent = trajectory.positions[i+1] - trajectory.positions[i-1];
                    }
                    vertexTangent = glm::normalize(vertexTangent);
                    vertexTangents.push_back(vertexTangent);

                    glm::vec3 vertexNormal = glm::normalize(glm::cross(
                            vertexTangent, bandPointsRight.at(i) - bandPointsLeft.at(i)));
                    vertexNormals.push_back(vertexNormal);

                    vertexAttributes.push_back(attributes.at(i));
                    vertexPrincipalStressIndices.push_back(psIdx);
                    if (hasLineHierarchy) {
                        vertexLineHierarchyLevels.push_back(
                                stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
                    }
                    vertexLineAppearanceOrders.push_back(stressTrajectoryData.appearanceOrder);
                }

                for (size_t i = 0; i < trajectory.positions.size() - 1; i++) {
                    lineIndices.push_back(indexStart + i);
                    lineIndices.push_back(indexStart + i + 1);
                }
            }
        } else {
            // 1. Compute all tangents.
            std::vector<std::vector<glm::vec3>> lineCentersList;
            std::vector<std::vector<float>> lineAttributesList;

            lineCentersList.resize(trajectories.size());
            lineAttributesList.resize(trajectories.size());
            std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                    continue;
                }

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
                        vertexLineHierarchyLevels.push_back(
                                stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
                    }
                    vertexLineAppearanceOrders.push_back(stressTrajectoryData.appearanceOrder);
                }
            }

            size_t numVerticesOld = vertexPositions.size();
            createLineTubesRenderDataCPU(
                    lineCentersList, lineAttributesList,
                    lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);
            size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
            for (size_t i = 0; i < numVerticesAdded; i++) {
                vertexPrincipalStressIndices.push_back(psIdx);
                vertexOffsetsLeft.push_back(glm::vec3(0.0f));
                vertexOffsetsRight.push_back(glm::vec3(0.0f));
            }
        }
    }

    // Add the index buffer.
    bandRenderData.indexBuffer = sgl::Renderer->createGeometryBuffer(
            lineIndices.size()*sizeof(uint32_t), (void*)&lineIndices.front(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    bandRenderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);

    // Add the attribute buffer.
    bandRenderData.vertexAttributeBuffer = sgl::Renderer->createGeometryBuffer(
            vertexAttributes.size()*sizeof(float), (void*)&vertexAttributes.front(), sgl::VERTEX_BUFFER);

    // Add the normal buffer.
    bandRenderData.vertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), (void*)&vertexNormals.front(), sgl::VERTEX_BUFFER);

    // Add the tangent buffer.
    bandRenderData.vertexTangentBuffer = sgl::Renderer->createGeometryBuffer(
            vertexTangents.size()*sizeof(glm::vec3), (void*)&vertexTangents.front(), sgl::VERTEX_BUFFER);

    // Add the left vertex offset buffer.
    bandRenderData.vertexOffsetLeftBuffer = sgl::Renderer->createGeometryBuffer(
            vertexOffsetsLeft.size()*sizeof(glm::vec3), (void*)&vertexOffsetsLeft.front(), sgl::VERTEX_BUFFER);

    // Add the right vertex offset buffer.
    bandRenderData.vertexOffsetRightBuffer = sgl::Renderer->createGeometryBuffer(
            vertexOffsetsRight.size()*sizeof(glm::vec3), (void*)&vertexOffsetsRight.front(), sgl::VERTEX_BUFFER);

    // Add the principal stress index buffer.
    bandRenderData.vertexPrincipalStressIndexBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPrincipalStressIndices.size()*sizeof(uint32_t),
            (void*)&vertexPrincipalStressIndices.front(), sgl::VERTEX_BUFFER);

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        bandRenderData.vertexLineHierarchyLevelBuffer = sgl::Renderer->createGeometryBuffer(
                vertexLineHierarchyLevels.size()*sizeof(float),
                (void*)&vertexLineHierarchyLevels.front(), sgl::VERTEX_BUFFER);
    }

    // Add the line appearance order buffer.
    bandRenderData.vertexLineAppearanceOrderBuffer = sgl::Renderer->createGeometryBuffer(
            vertexLineAppearanceOrders.size()*sizeof(uint32_t),
            (void*)&vertexLineAppearanceOrders.front(), sgl::VERTEX_BUFFER);

    return bandRenderData;
}

sgl::ShaderAttributesPtr LineDataStress::getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader) {
    sgl::ShaderAttributesPtr shaderAttributes;

    if (linePrimitiveMode == LINE_PRIMITIVES_BAND) {
        BandRenderData tubeRenderData = this->getBandRenderData();
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
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexOffsetLeftBuffer, "vertexOffsetLeft",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexOffsetRightBuffer, "vertexOffsetRight",
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
        if (tubeRenderData.vertexLineAppearanceOrderBuffer) {
            shaderAttributes->addGeometryBufferOptional(
                    tubeRenderData.vertexLineAppearanceOrderBuffer, "vertexLineAppearanceOrder",
                    sgl::ATTRIB_UNSIGNED_INT, 1);
        }
    } else if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
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
        if (tubeRenderData.vertexLineAppearanceOrderBuffer) {
            shaderAttributes->addGeometryBufferOptional(
                    tubeRenderData.vertexLineAppearanceOrderBuffer, "vertexLineAppearanceOrder",
                    sgl::ATTRIB_UNSIGNED_INT,
                    1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);
        }
    }

    return shaderAttributes;
}

void LineDataStress::setUniformGatherShaderData_AllPasses() {
    LineData::setUniformGatherShaderData_AllPasses();
    if (useLineHierarchy && linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
        sgl::ShaderManager->bindShaderStorageBuffer(3, lineHierarchyLevelsSSBO);
    }

    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->bindShaderStorageBuffer(9, multiVarTransferFunctionWindow.getMinMaxSsbo());
    }
}

void LineDataStress::setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader) {
    if (!usePrincipalStressDirectionIndex) {
        LineData::setUniformGatherShaderData_Pass(gatherShader);
    } else {
        gatherShader->setUniformOptional(
                "transferFunctionTexture",
                multiVarTransferFunctionWindow.getTransferFunctionMapTexture(), 0);
    }
    gatherShader->setUniformOptional("currentSeedIdx", int32_t(currentSeedIdx));

    if (useLineHierarchy) {
        if (!rendererSupportsTransparency) {
            gatherShader->setUniform("lineHierarchySlider", glm::vec3(1.0f) - lineHierarchySliderValues);
        } else {
            gatherShader->setUniformOptional(
                    "lineHierarchyImportanceMap",
                    stressLineHierarchyMappingWidget.getHierarchyMappingTexture(), 1);
        }
    }

    if (useBands()) {
        gatherShader->setUniform("bandWidth", LineRenderer::bandWidth);
        gatherShader->setUniform(
                "psUseBands", glm::ivec3(psUseBands[0], psUseBands[1], psUseBands[2]));
    }
}
