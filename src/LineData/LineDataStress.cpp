/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020-2021, Christoph Neuhauser
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

#ifdef USE_EIGEN
#include <Eigen/Eigenvalues>
#endif

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Loaders/TrajectoryFile.hpp"
#include "Renderers/Tubes/Tubes.hpp"
#include "Renderers/OIT/OpacityOptimizationRenderer.hpp"

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
std::array<bool, 3> LineDataStress::psUseBands = {false, false, true};
bool LineDataStress::useSmoothedBands = false;
LineDataStress::BandRenderMode LineDataStress::bandRenderMode = LineDataStress::BandRenderMode::RIBBONS;
LineDataStress::LineHierarchyType LineDataStress::lineHierarchyType = LineDataStress::LineHierarchyType::GEO;
glm::vec3 LineDataStress::lineHierarchySliderValues = glm::vec3(1.0f);

const char* const stressDirectionNames[] = { "Major", "Medium", "Minor" };
const char* const lineHierarchyTypeNames[] = { "GEO-based", "PS-based", "vM-based", "Length-based" };
const char* const bandRenderModeNames[] = { "Ribbons", "Eigenvalue Ratio", "Hyperstreamlines" };

LineDataStress::LineDataStress(sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineData(transferFunctionWindow, DATA_SET_TYPE_STRESS_LINES),
          multiVarTransferFunctionWindow(
                  "stress",
                  { "qualitative-ocher.xml", "qualitative-emerald.xml", "qualitative-pale-lilac.xml" }) {
    colorLegendWidgets.resize(3);
    for (int psIdx = 0; psIdx < 3; psIdx++) {
        colorLegendWidgets[psIdx].setPositionIndex(psIdx, 3);
    }
    setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
    lineDataWindowName = "Line Data (Stress)";

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    stressLineUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(StressLineUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

LineDataStress::~LineDataStress() = default;

bool LineDataStress::settingsDiffer(LineData* other) {
    return useLineHierarchy != static_cast<LineDataStress*>(other)->useLineHierarchy
            || (hasBandsData != static_cast<LineDataStress*>(other)->hasBandsData);
}

bool LineDataStress::getIsSmallDataSet() const {
    return numTotalTrajectoryPoints <= SMALL_DATASET_LINE_POINTS_MAX;
}

void LineDataStress::update(float dt) {
    if (rendererSupportsTransparency && useLineHierarchy) {
        stressLineHierarchyMappingWidget.update(dt);
    }
    if (usePrincipalStressDirectionIndex) {
        multiVarTransferFunctionWindow.update(dt);
    }
}

void LineDataStress::setRenderingModes(const std::vector<RenderingMode>& renderingModes) {
    LineData::setRenderingModes(renderingModes);
    rendererSupportsTransparency = std::all_of(renderingModes.cbegin(), renderingModes.cend(), [](RenderingMode mode){
        return mode != RENDERING_MODE_ALL_LINES_OPAQUE
               && mode != RENDERING_MODE_VULKAN_RAY_TRACER
               && mode != RENDERING_MODE_VOXEL_RAY_CASTING
               && mode != RENDERING_MODE_OSPRAY_RAY_TRACER;
    });
}

bool LineDataStress::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = LineData::setNewSettings(settings);

    bool usedPsChanged = false;
    usedPsChanged |= settings.getValueOpt("major_on", useMajorPS);
    usedPsChanged |= settings.getValueOpt("medium_on", useMediumPS);
    usedPsChanged |= settings.getValueOpt("minor_on", useMinorPS);
    if (usedPsChanged) {
        setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
        shallReloadGatherShader = true;
    }

    bool recomputeOpacityOptimization = false;
    if (hasLineHierarchy) {
        bool sliderChanged = false;
        const char* const stressDirectionLodNames[] = { "major_lod", "medium_lod", "minor_lod" };
        for (int psIdx : loadedPsIndices) {
            if (settings.getValueOpt(
                    stressDirectionLodNames[psIdx], lineHierarchySliderValues[psIdx])) {
                reRender = true;
                recomputeOpacityOptimization = true;
                sliderChanged = true;
            }
        }
        if (sliderChanged) {
            bool useLineHierarchyNew = glm::any(
                    glm::lessThan(lineHierarchySliderValues, glm::vec3(1.0f)));
            if (useLineHierarchy != useLineHierarchyNew) {
                shallReloadGatherShader = true;
            }
            triangleRepresentationDirty = true;
        }
    }

    if (getUseBandRendering()) {
        bool usedBandsChanged = false;
        usedBandsChanged |= settings.getValueOpt("major_use_bands", psUseBands[0]);
        usedBandsChanged |= settings.getValueOpt("medium_use_bands", psUseBands[1]);
        usedBandsChanged |= settings.getValueOpt("minor_use_bands", psUseBands[2]);
        if (usedBandsChanged) {
            dirty = true;
        }
    }

    if (getUseBandRendering()) {
        if (fileFormatVersion >= 3 && settings.getValueOpt("smoothed_bands", useSmoothedBands)) {
            dirty = true;
        }

#ifdef USE_EIGEN
        std::string bandRenderingModeName;
        if (fileFormatVersion >= 3 && settings.getValueOpt("band_render_mode", bandRenderingModeName)) {
            int i;
            for (i = 0; i < IM_ARRAYSIZE(bandRenderModeNames); i++) {
                if (bandRenderingModeName == bandRenderModeNames[i]) {
                    bandRenderMode = BandRenderMode(i);
                    dirty = true;
                    shallReloadGatherShader = true;
                    break;
                }
            }
            if (i == IM_ARRAYSIZE(bandRenderModeNames)) {
                sgl::Logfile::get()->writeError(
                        "Error in LineDataStress::setNewSettings: Unknown band rendering mode name \""
                        + bandRenderingModeName + "\".");
            }
        }
#endif


        if (bandRenderMode == BandRenderMode::RIBBONS) {
            if (settings.getValueOpt("thick_bands", renderThickBands)) {
                shallReloadGatherShader = true;
            }

            if (settings.getValueOpt("min_band_thickness", minBandThickness)) {
                shallReloadGatherShader = true;
            }
        }
    }

    if (settings.getValueOpt("use_principal_stress_direction_index", usePrincipalStressDirectionIndex)) {
        dirty = true;
        shallReloadGatherShader = true;
        recomputeColorLegend();
    }

    if (recomputeOpacityOptimization) {
        sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT));
    }

    return shallReloadGatherShader;
}

bool LineDataStress::renderGuiPropertyEditorNodesRenderer(
        sgl::PropertyEditor& propertyEditor, LineRenderer* lineRenderer) {
    bool shallReloadGatherShader = LineData::renderGuiPropertyEditorNodesRenderer(propertyEditor, lineRenderer);
    return shallReloadGatherShader;
}

bool LineDataStress::renderGuiWindowSecondary() {
    if (rendererSupportsTransparency && useLineHierarchy) {
        bool hierarchyMappingChanged = stressLineHierarchyMappingWidget.renderGui();
        reRender = reRender || hierarchyMappingChanged;
        sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT));
    }

    if (usePrincipalStressDirectionIndex && multiVarTransferFunctionWindow.renderGui()) {
        reRender = true;
        if (multiVarTransferFunctionWindow.getTransferFunctionMapRebuilt()) {
            onTransferFunctionMapRebuilt();
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
        }
    }

    return LineData::renderGuiWindowSecondary();
}

bool LineDataStress::renderGuiOverlay() {
    if (usePrincipalStressDirectionIndex && shallRenderColorLegendWidgets) {
        for (int psIdx : loadedPsIndices) {
            colorLegendWidgets.at(psIdx).setAttributeMinValue(
                    multiVarTransferFunctionWindow.getSelectedRangeMin(psIdx));
            colorLegendWidgets.at(psIdx).setAttributeMaxValue(
                    multiVarTransferFunctionWindow.getSelectedRangeMax(psIdx));
            if (usedPsDirections[psIdx]) {
                colorLegendWidgets.at(psIdx).renderGui();
            }
        }
        return false;
    } else {
        return LineData::renderGuiOverlay();
    }
}

bool LineDataStress::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = LineData::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.beginNode("Used PS Directions")) {
        bool usedPsChanged = false;
        usedPsChanged |= propertyEditor.addCheckbox("Major##usedpsmajor", &useMajorPS);
        usedPsChanged |= propertyEditor.addCheckbox("Medium##usedpsmedium", &useMediumPS);
        usedPsChanged |= propertyEditor.addCheckbox("Minor##usedpsminor", &useMinorPS);
        if (usedPsChanged) {
            setUsedPsDirections({useMajorPS, useMediumPS, useMinorPS});
            shallReloadGatherShader = true;
        }
        propertyEditor.endNode();
    }

    bool recomputeOpacityOptimization = false;
    if (hasLineHierarchy) {
        if (fileFormatVersion >= 3) {
            if (propertyEditor.addCombo(
                    "Line Hierarchy Type", (int*)&lineHierarchyType,
                    lineHierarchyTypeNames, IM_ARRAYSIZE(lineHierarchyTypeNames))) {
                updateLineHierarchyHistogram();
                dirty = true;
            }
        }
        if (!rendererSupportsTransparency && useLineHierarchy) {
            bool canUseLiveUpdate = getCanUseLiveUpdate(LineDataAccessType::TRIANGLE_MESH);
            bool sliderChanged = false;
            for (int psIdx : loadedPsIndices) {
                ImGui::EditMode editMode = propertyEditor.addSliderFloatEdit(
                        stressDirectionNames[psIdx], &lineHierarchySliderValues[psIdx], 0.0f, 1.0f);
                if ((canUseLiveUpdate && editMode != ImGui::EditMode::NO_CHANGE)
                    || (!canUseLiveUpdate && editMode == ImGui::EditMode::INPUT_FINISHED)) {
                    reRender = true;
                    recomputeOpacityOptimization = true;
                    sliderChanged = true;
                }
            }
            if (sliderChanged) {
                bool useLineHierarchyNew = glm::any(
                        glm::lessThan(lineHierarchySliderValues, glm::vec3(1.0f)));

                bool isRasterizer = std::any_of(
                        lineRenderersCached.cbegin(), lineRenderersCached.cend(), [](LineRenderer* lineRenderer){
                            return lineRenderer->getIsRasterizer();
                        });

                if (isRasterizer) {
                    if (useLineHierarchy != useLineHierarchyNew) {
                        shallReloadGatherShader = true;
                    }
                }
                triangleRepresentationDirty = true;
            }
        }
    }

    if (getUseBandRendering()) {
        if (propertyEditor.beginNode("Render as Bands")) {
            bool usedBandsChanged = false;
            usedBandsChanged |= propertyEditor.addCheckbox("Major##bandsmajor", &psUseBands[0]);
            usedBandsChanged |= propertyEditor.addCheckbox("Medium##bandsmedium", &psUseBands[1]);
            usedBandsChanged |= propertyEditor.addCheckbox("Minor##bandsminor", &psUseBands[2]);
            if (usedBandsChanged) {
                dirty = true;
            }
            propertyEditor.endNode();
        }
    }

    if (propertyEditor.beginNode("Advanced Settings")) {
        if (getUseBandRendering()) {
            if (fileFormatVersion >= 3 && propertyEditor.addCheckbox("Smoothed Bands", &useSmoothedBands)) {
                dirty = true;
            }

#ifdef USE_EIGEN
            bool showBandRenderingMode = std::any_of(
                    lineRenderersCached.cbegin(), lineRenderersCached.cend(),
                    [](LineRenderer* lineRenderer) {
                        return lineRenderer->getIsRasterizer()
                               || lineRenderer->getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER
                               || lineRenderer->getRenderingMode() == RENDERING_MODE_OSPRAY_RAY_TRACER;
                    });
            if (showBandRenderingMode && fileFormatVersion >= 3 && propertyEditor.addCombo(
                    "Band Rendering Mode", (int*)&bandRenderMode,
                    bandRenderModeNames, IM_ARRAYSIZE(bandRenderModeNames))) {
                dirty = true;
                shallReloadGatherShader = true;
            }
#endif

            if (bandRenderMode == BandRenderMode::RIBBONS) {
                if (propertyEditor.addCheckbox("Render Thick Bands", &renderThickBands)) {
                    shallReloadGatherShader = true;
                }

                if (renderThickBands && propertyEditor.addSliderFloat(
                        "Min. Band Thickness", &minBandThickness, 0.01f, 1.0f)) {
                    triangleRepresentationDirty = true;
                    shallReloadGatherShader = true;
                }
            }
        }

        if (propertyEditor.addCheckbox(
                "Use Principal Stress Direction Index", &usePrincipalStressDirectionIndex)) {
            dirty = true;
            shallReloadGatherShader = true;
            recomputeColorLegend();
        }

        propertyEditor.endNode();
    }

    if (recomputeOpacityOptimization) {
        sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT));
    }

    return shallReloadGatherShader;
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

#ifdef USE_EIGEN
    if (dataSetInformation.version >= 3) {
        attributeNames.emplace_back("Major Stress");
        attributeNames.emplace_back("Medium Stress");
        attributeNames.emplace_back("Minor Stress");
        attributeNames.emplace_back("Degeneracy Measure");
    }
#endif

    std::vector<Trajectories> trajectoriesPs;
    std::vector<StressTrajectoriesData> stressTrajectoriesDataPs;
    sgl::AABB3 oldAABB;
    MeshType meshType = MeshType::CARTESIAN;
    loadStressTrajectoriesFromFile(
            fileNames, dataSetInformation.filenamesStressLineHierarchy,
            dataSetInformation.version, loadedPsIndices, meshType, trajectoriesPs, stressTrajectoriesDataPs,
            bandPointsUnsmoothedListLeftPs, bandPointsUnsmoothedListRightPs,
            bandPointsSmoothedListLeftPs, bandPointsSmoothedListRightPs,
            simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions,
            true, false, &oldAABB,
            transformationMatrixPtr);
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
        focusBoundingBox = modelBoundingBox;

        std::vector<bool> usedPsDirections = { false, false, false };
        for (size_t i = 0; i < loadedPsIndices.size(); i++) {
            int psIdx = loadedPsIndices.at(i);
            colorLegendWidgets[psIdx].setPositionIndex(int(i), int(loadedPsIndices.size()));
            usedPsDirections.at(psIdx) = true;
        }
        setUsedPsDirections(usedPsDirections);

        // Disable bands for minor PSLs if first PS direction is not used.
        psUseBands = {false, false, true};
        if (!usedPsDirections.at(0)) {
            psUseBands.at(2) = false;
        }

        // Use bands if possible.
        if (hasBandsData && !getUseBandRendering()) {
            linePrimitiveMode = LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL;
        } else if (!hasBandsData && getUseBandRendering()) {
            linePrimitiveMode = LINE_PRIMITIVES_TUBE_PROGRAMMABLE_PULL;
        }
        if (hasBandsData) {
            tubeNumSubdivisions = std::max(tubeNumSubdivisions, 8);
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
            minMaxAttributeValuesPs[psIdx].emplace_back(minAttr, maxAttr);
            minAttrTotal = std::min(minAttrTotal, minAttr);
            maxAttrTotal = std::max(maxAttrTotal, maxAttr);
            i++;
        }
        minMaxAttributeValues.emplace_back(minAttrTotal, maxAttrTotal);
    }

    updateLineHierarchyHistogram();

    numTotalTrajectoryPoints = 0;
    for (const Trajectories& trajectories : trajectoriesPs) {
#if _OPENMP >= 201107
        #pragma omp parallel for reduction(+: numTotalTrajectoryPoints) shared(trajectories) default(none)
#endif
        for (size_t i = 0; i < trajectories.size(); i++) {
            const Trajectory& trajectory = trajectories.at(i);
            numTotalTrajectoryPoints += trajectory.positions.size();
        }
    }

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
    if (degeneratePoints.empty()) {
        return;
    }

    // Build a search structure on the degenerate points.
    KdTree<ptrdiff_t> kdTree;
    std::vector<std::pair<glm::vec3, ptrdiff_t>> pointsAndIndices(degeneratePoints.size());
    for (size_t i = 0; i < degeneratePoints.size(); i++) {
        pointsAndIndices.at(i) = std::make_pair(degeneratePoints.at(i), ptrdiff_t(i));
    }
    kdTree.build(pointsAndIndices);

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
#if _OPENMP >= 201107
            #pragma omp parallel for shared(trajectory, kdTree, degeneratePoints, \
            distanceMeasuresExponentialKernel, distanceMeasuresSquaredExponentialKernel) \
            firstprivate(lengthScale) default(none)
#endif
            for (size_t linePointIdx = 0; linePointIdx < trajectory.positions.size(); linePointIdx++) {
                const glm::vec3& linePoint = trajectory.positions.at(linePointIdx);

                auto nearestNeighbor = kdTree.findNearestNeighbor(linePoint);
                float distanceExponentialKernel = exponentialKernel(
                        linePoint, nearestNeighbor.value().first, lengthScale);
                float distanceSquaredExponentialKernel = squaredExponentialKernel(
                        linePoint, nearestNeighbor.value().first, lengthScale);

                distanceMeasuresExponentialKernel.at(linePointIdx) = distanceExponentialKernel;
                distanceMeasuresSquaredExponentialKernel.at(linePointIdx) = distanceSquaredExponentialKernel;
            }
            trajectory.attributes.push_back(distanceMeasuresExponentialKernel);
            trajectory.attributes.push_back(distanceMeasuresSquaredExponentialKernel);
        }
        minMaxAttributeValuesPs[psIdx].emplace_back(0.0f, 1.0f);
        minMaxAttributeValuesPs[psIdx].emplace_back(0.0f, 1.0f);
        psIdx++;
    }

    minMaxAttributeValues.emplace_back(0.0f, 1.0f);
    minMaxAttributeValues.emplace_back(0.0f, 1.0f);
    attributeNames.emplace_back("Distance Exponential Kernel");
    attributeNames.emplace_back("Distance Squared Exponential Kernel");
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
    recomputeColorLegendPositions();
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

void LineDataStress::recomputeColorLegendPositions() {
    int totalDisplayedPsIndices = 0;
    for (size_t i = 0; i < loadedPsIndices.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (usedPsDirections[psIdx]) {
            totalDisplayedPsIndices++;
        }
    }

    int positionIndex = 0;
    for (size_t i = 0; i < loadedPsIndices.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        colorLegendWidgets[psIdx].setPositionIndex(positionIndex, totalDisplayedPsIndices);
        if (usedPsDirections[psIdx]) {
            positionIndex++;
        }
    }
}

void LineDataStress::recomputeColorLegend() {
    if (usePrincipalStressDirectionIndex) {
        int totalDisplayedPsIndices = 0;
        for (size_t i = 0; i < loadedPsIndices.size(); i++) {
            int psIdx = loadedPsIndices.at(i);
            if (usedPsDirections[psIdx]) {
                totalDisplayedPsIndices++;
            }
        }

        int positionIndex = 0;
        for (size_t i = 0; i < loadedPsIndices.size(); i++) {
            int psIdx = loadedPsIndices.at(i);
            colorLegendWidgets[psIdx].setAttributeDisplayName(
                    std::string() + attributeNames.at(selectedAttributeIndex)
                    + " (" + stressDirectionNames[psIdx] + ")");

            glm::vec2 minMaxAttributes = minMaxAttributeValuesPs[psIdx].at(selectedAttributeIndex);
            colorLegendWidgets[psIdx].setAttributeMinValue(minMaxAttributes.x);
            colorLegendWidgets[psIdx].setAttributeMaxValue(minMaxAttributes.y);
            colorLegendWidgets[psIdx].setPositionIndex(positionIndex, totalDisplayedPsIndices);

            if (usedPsDirections[psIdx]) {
                positionIndex++;
            }

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
        recomputeColorLegendPositions();
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

size_t LineDataStress::getBaseSizeInBytes() {
    size_t baseSizeInBytes = 0;
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        const Trajectories& trajectories = trajectoriesPs.at(i);
        const StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            const Trajectory& trajectory = trajectories.at(trajectoryIdx);
            const StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            baseSizeInBytes += trajectory.positions.size() * sizeof(glm::vec3);
            for (size_t attributeIdx = 0; attributeIdx < trajectory.attributes.size(); attributeIdx++) {
                baseSizeInBytes += trajectory.attributes.at(attributeIdx).size() * sizeof(float);
            }
            baseSizeInBytes += stressTrajectoryData.hierarchyLevels.size() * sizeof(float);
            baseSizeInBytes += sizeof(int); // appearanceOrder
            baseSizeInBytes += sizeof(glm::vec3); // seedPosition
            baseSizeInBytes += stressTrajectoryData.majorPs.size() * sizeof(float);
            baseSizeInBytes += stressTrajectoryData.mediumPs.size() * sizeof(float);
            baseSizeInBytes += stressTrajectoryData.minorPs.size() * sizeof(float);
            baseSizeInBytes += stressTrajectoryData.majorPsDir.size() * sizeof(glm::vec3);
            baseSizeInBytes += stressTrajectoryData.mediumPsDir.size() * sizeof(glm::vec3);
            baseSizeInBytes += stressTrajectoryData.minorPsDir.size() * sizeof(glm::vec3);
        }
    }
    return baseSizeInBytes;
}


void LineDataStress::iterateOverTrajectories(std::function<void(const Trajectory&)> callback) {
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        for (const Trajectory& trajectory : trajectoriesPs.at(i)) {
            callback(trajectory);
        }
    }
}

void LineDataStress::iterateOverTrajectoriesNotFiltered(std::function<void(const Trajectory&)> callback) {
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        const Trajectories& trajectories = trajectoriesPs.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        size_t trajectoryIdx = 0;
        for (const Trajectory& trajectory : trajectories) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                trajectoryIdx++;
                continue;
            }
            callback(trajectory);
            trajectoryIdx++;
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
        Trajectories& trajectories = trajectoriesPs.at(i);
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
    rebuildInternalRepresentationIfNecessary();

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
            trajectoryFiltered.attributes.resize(attributeNames.size());
            size_t n = trajectory.positions.size();

            int numValidLinePoints = 0;
            for (size_t i = 0; i < n; i++) {
                glm::vec3 tangent;
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

std::vector<std::vector<glm::vec3>> LineDataStress::getFilteredLines(LineRenderer* lineRenderer) {
    rebuildInternalRepresentationIfNecessary();

    std::vector<std::vector<glm::vec3>> linesFiltered;
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        const Trajectories& trajectories = trajectoriesPs.at(i);
        const StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);
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
            if ((!lineRenderer || !lineRenderer->isRasterizer) && lineHierarchySliderValues[psIdx]
                    < 1.0 - stressTrajectoriesData.at(trajectoryIdx).hierarchyLevels.at(int(lineHierarchyType))) {
                trajectoryIdx++;
                continue;
            }

            std::vector<glm::vec3> lineFiltered;
            size_t n = trajectory.positions.size();

            int numValidLinePoints = 0;
            for (size_t i = 0; i < n; i++) {
                glm::vec3 tangent;
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
    rebuildInternalRepresentationIfNecessary();

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

            trajectoryIdx++;
        }

        trajectoriesPs.push_back(trajectories);
    }

    return linesPsFiltered;
}


// --- Retrieve data for rendering. ---
void LineDataStress::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    LineData::setGraphicsPipelineInfo(pipelineInfo, shaderStages);

    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER) {
        pipelineInfo.setVertexBufferBindingByLocationIndex("vertexOffsetLeft", sizeof(glm::vec3));
        pipelineInfo.setVertexBufferBindingByLocationIndex("vertexOffsetRight", sizeof(glm::vec3));
        pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
                "vertexPrincipalStressIndex", sizeof(uint32_t));
        if (hasLineHierarchy) {
            pipelineInfo.setVertexBufferBindingByLocationIndex(
                    "vertexLineHierarchyLevel", sizeof(float));
        }
        pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
                "vertexLineAppearanceOrder", sizeof(uint32_t));
    } else if (getLinePrimitiveModeUsesSingleVertexShaderInputs(linePrimitiveMode)) {
        pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
                "vertexPrincipalStressIndex", sizeof(uint32_t));
        if (hasLineHierarchy) {
            pipelineInfo.setVertexBufferBindingByLocationIndex(
                    "vertexLineHierarchyLevel", sizeof(float));
        }
        pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
                "vertexLineAppearanceOrder", sizeof(uint32_t));
        pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
                "vertexMajorStress", sizeof(float));
        pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
                "vertexMediumStress", sizeof(float));
        pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
                "vertexMinorStress", sizeof(float));
    }
}

void LineDataStress::setRasterDataBindings(sgl::vk::RasterDataPtr& rasterData) {
    setVulkanRenderDataDescriptors(rasterData);

    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER) {
        LinePassQuadsRenderData tubeRenderData = this->getLinePassQuadsRenderData();
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setVertexBuffer(tubeRenderData.vertexPositionBuffer, "vertexPosition");
        rasterData->setVertexBuffer(tubeRenderData.vertexAttributeBuffer, "vertexAttribute");
        rasterData->setVertexBuffer(tubeRenderData.vertexNormalBuffer, "vertexNormal");
        rasterData->setVertexBuffer(tubeRenderData.vertexTangentBuffer, "vertexTangent");
        rasterData->setVertexBuffer(tubeRenderData.vertexOffsetLeftBuffer, "vertexOffsetLeft");
        rasterData->setVertexBuffer(tubeRenderData.vertexOffsetRightBuffer, "vertexOffsetRight");

        if (tubeRenderData.vertexPrincipalStressIndexBuffer) {
            rasterData->setVertexBufferOptional(
                    tubeRenderData.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex");
        }
        if (tubeRenderData.vertexLineHierarchyLevelBuffer) {
            rasterData->setVertexBufferOptional(
                    tubeRenderData.vertexLineHierarchyLevelBuffer, "vertexLineHierarchyLevel");
        }
        if (tubeRenderData.vertexLineAppearanceOrderBuffer) {
            rasterData->setVertexBufferOptional(
                    tubeRenderData.vertexLineAppearanceOrderBuffer, "vertexLineAppearanceOrder");
        }
    } else if (linePrimitiveMode == LINE_PRIMITIVES_QUADS_PROGRAMMABLE_PULL) {
        LinePassQuadsRenderDataProgrammablePull tubeRenderData = this->getLinePassQuadsRenderDataProgrammablePull();
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setStaticBuffer(tubeRenderData.linePointsBuffer, "LinePoints");
        if (useLineHierarchy) {
            rasterData->setStaticBuffer(tubeRenderData.lineHierarchyLevelsBuffer, "LineHierarchyLevels");
        }
    } else if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_PROGRAMMABLE_PULL
               || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL) {
        LinePassTubeRenderDataProgrammablePull tubeRenderData = this->getLinePassTubeRenderDataProgrammablePull();
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setStaticBuffer(tubeRenderData.linePointDataBuffer, "LinePointDataBuffer");
        if (tubeRenderData.stressLinePointDataBuffer) {
            rasterData->setStaticBuffer(
                    tubeRenderData.stressLinePointDataBuffer,
                    "StressLinePointDataBuffer");
        }
        if (tubeRenderData.stressLinePointPrincipalStressDataBuffer
                && linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL) {
            rasterData->setStaticBuffer(
                    tubeRenderData.stressLinePointPrincipalStressDataBuffer,
                    "StressLinePointPrincipalStressDataBuffer");
        }
    } else if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_MESH_SHADER
               || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER) {
        LinePassTubeRenderDataMeshShader tubeRenderData = this->getLinePassTubeRenderDataMeshShader();
        if (!tubeRenderData.meshletDataBuffer) {
            return;
        }
        rasterData->setStaticBuffer(tubeRenderData.meshletDataBuffer, "MeshletDataBuffer");
        rasterData->setStaticBuffer(tubeRenderData.linePointDataBuffer, "LinePointDataBuffer");
        if (tubeRenderData.stressLinePointDataBuffer) {
            rasterData->setStaticBuffer(
                    tubeRenderData.stressLinePointDataBuffer,
                    "StressLinePointDataBuffer");
        }
        if (tubeRenderData.stressLinePointPrincipalStressDataBuffer
                && linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER) {
            rasterData->setStaticBuffer(
                    tubeRenderData.stressLinePointPrincipalStressDataBuffer,
                    "StressLinePointPrincipalStressDataBuffer");
        }
        rasterData->setMeshTasks(tubeRenderData.numMeshlets, 0);
    } else if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_TRIANGLE_MESH
               || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH) {
        TubeTriangleRenderData tubeRenderData = this->getLinePassTubeTriangleMeshRenderData(
                true, false);
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setStaticBuffer(tubeRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
        rasterData->setStaticBuffer(tubeRenderData.linePointDataBuffer, "LinePointDataBuffer");
        if (tubeRenderData.stressLinePointDataBuffer) {
            rasterData->setStaticBuffer(
                    tubeRenderData.stressLinePointDataBuffer,
                    "StressLinePointDataBuffer");
        }
        if (tubeRenderData.stressLinePointPrincipalStressDataBuffer
                && linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH) {
            rasterData->setStaticBuffer(
                    tubeRenderData.stressLinePointPrincipalStressDataBuffer,
                    "StressLinePointPrincipalStressDataBuffer");
        }
    } else {
        LinePassTubeRenderData tubeRenderData = this->getLinePassTubeRenderData();
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setVertexBuffer(tubeRenderData.vertexPositionBuffer, "vertexPosition");
        rasterData->setVertexBuffer(tubeRenderData.vertexAttributeBuffer, "vertexAttribute");
        rasterData->setVertexBufferOptional(tubeRenderData.vertexNormalBuffer, "vertexNormal");
        rasterData->setVertexBuffer(tubeRenderData.vertexTangentBuffer, "vertexTangent");

        if (tubeRenderData.vertexPrincipalStressIndexBuffer) {
            rasterData->setVertexBuffer(
                    tubeRenderData.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex");
        }
        if (tubeRenderData.vertexLineHierarchyLevelBuffer) {
            rasterData->setVertexBufferOptional(
                    tubeRenderData.vertexLineHierarchyLevelBuffer, "vertexLineHierarchyLevel");
        }
        if (tubeRenderData.vertexLineAppearanceOrderBuffer) {
            rasterData->setVertexBufferOptional(
                    tubeRenderData.vertexLineAppearanceOrderBuffer, "vertexLineAppearanceOrder");
        }
        if (tubeRenderData.vertexMajorStressBuffer) {
            rasterData->setVertexBufferOptional(tubeRenderData.vertexMajorStressBuffer, "vertexMajorStress");
        }
        if (tubeRenderData.vertexMediumStressBuffer) {
            rasterData->setVertexBufferOptional(tubeRenderData.vertexMediumStressBuffer, "vertexMediumStress");
        }
        if (tubeRenderData.vertexMinorStressBuffer) {
            rasterData->setVertexBufferOptional(tubeRenderData.vertexMinorStressBuffer, "vertexMinorStress");
        }
    }
}

void LineDataStress::getLinePassTubeRenderDataGeneral(
        const std::function<uint32_t()>& indexOffsetFunctor,
        const std::function<void(
                const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent, float lineAttribute,
                uint32_t indexOffset, int principalStressIndex, float lineHierarchyLevel, int lineAppearanceOrder,
                float majorStress, float mediumStress, float minorStress)>& pointPushFunctor,
        const std::function<void()>& pointPopFunctor,
        const std::function<void(int numSegments, uint32_t indexOffset)>& indicesPushFunctor) {
    std::vector<std::vector<std::vector<glm::vec3>>>* bandPointsListRightPs;
    if (getUseBandRendering()) {
        if (useSmoothedBands) {
            bandPointsListRightPs = &bandPointsSmoothedListRightPs;
        } else {
            bandPointsListRightPs = &bandPointsUnsmoothedListRightPs;
        }
    }

#ifdef USE_EIGEN
    int majorStressIdx = -1;
    int mediumStressIdx = -1;
    int minorStressIdx = -1;

    if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
            && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        majorStressIdx = getAttributeNameIndex("Major Stress");
        mediumStressIdx = getAttributeNameIndex("Medium Stress");
        minorStressIdx = getAttributeNameIndex("Minor Stress");
    }
#endif

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

        std::vector<std::vector<glm::vec3>> lineCentersList;
        std::vector<std::vector<float>> lineAttributesList;

#ifdef USE_EIGEN
        std::vector<std::vector<float>> lineMajorStressesList;
        std::vector<std::vector<float>> lineMediumStressesList;
        std::vector<std::vector<float>> lineMinorStressesList;
#endif

        if (getUseBandRendering() && psUseBands.at(psIdx)) {
            std::vector<std::vector<glm::vec3>> bandRightVectorList;
            std::vector<std::vector<glm::vec3>>& bandPointsListRight = bandPointsListRightPs->at(i);

#ifdef USE_EIGEN
            lineMajorStressesList.resize(trajectories.size());
            lineMediumStressesList.resize(trajectories.size());
            lineMinorStressesList.resize(trajectories.size());
#endif

            lineCentersList.resize(trajectories.size());
            lineAttributesList.resize(trajectories.size());
            bandRightVectorList.resize(trajectories.size());
            std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                    continue;
                }

                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                //StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
                std::vector<glm::vec3>& bandPointsRight = bandPointsListRight.at(trajectoryIdx);
                assert(attributes.size() == trajectory.positions.size());
                std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
                std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
                std::vector<glm::vec3>& bandRightVectors = bandRightVectorList.at(trajectoryIdx);

#ifdef USE_EIGEN
                std::vector<float>& lineMajorStresses = lineMajorStressesList.at(trajectoryIdx);
                std::vector<float>& lineMediumStresses = lineMediumStressesList.at(trajectoryIdx);
                std::vector<float>& lineMinorStresses = lineMinorStressesList.at(trajectoryIdx);
#endif

                for (size_t j = 0; j < trajectory.positions.size(); j++) {
                    lineCenters.push_back(trajectory.positions.at(j));
                    lineAttributes.push_back(attributes.at(j));
                    bandRightVectors.push_back(bandPointsRight.at(j));

#ifdef USE_EIGEN
                    if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                            && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                        lineMajorStresses.push_back(trajectory.attributes.at(majorStressIdx).at(j));
                        lineMediumStresses.push_back(trajectory.attributes.at(mediumStressIdx).at(j));
                        lineMinorStresses.push_back(trajectory.attributes.at(minorStressIdx).at(j));
                    }
#endif
                }
            }

            for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
                const std::vector<glm::vec3>& lineCenters = lineCentersList.at(lineId);
                const std::vector<float>& lineAttributes = lineAttributesList.at(lineId);
                const std::vector<glm::vec3>& bandRightVectors = bandRightVectorList.at(lineId);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(lineId);
                assert(lineCenters.size() == lineAttributes.size());
                size_t n = lineCenters.size();
                uint32_t indexOffset = indexOffsetFunctor();

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


                    float lineHierarchyLevel = 0.0f;
                    float majorStress = 1.0f, mediumStress = 1.0f, minorStress = 1.0f;
                    if (hasLineHierarchy) {
                        lineHierarchyLevel = stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType));
                    }
#ifdef USE_EIGEN
                    if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                            && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                        majorStress = lineMajorStressesList.at(lineId).at(i);
                        mediumStress = lineMediumStressesList.at(lineId).at(i);
                        minorStress = lineMinorStressesList.at(lineId).at(i);
                    }
#endif
                    pointPushFunctor(
                            lineCenters.at(i), normal, tangent, lineAttributes.at(i),
                            indexOffset,
                            psIdx, lineHierarchyLevel, stressTrajectoryData.appearanceOrder,
                            majorStress, mediumStress, minorStress);
                    numValidLinePoints++;
                }

                if (numValidLinePoints == 1) {
                    // Only one vertex left -> Output nothing (tube consisting only of one point).
                    pointPopFunctor();
                }
                if (numValidLinePoints <= 1) {
                    continue;
                }

                // Create indices.
                int numSegments = numValidLinePoints - 1;
                indicesPushFunctor(numSegments, indexOffset);
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
                //StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
                assert(attributes.size() == trajectory.positions.size());
                std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
                std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
                for (size_t i = 0; i < trajectory.positions.size(); i++) {
                    lineCenters.push_back(trajectory.positions.at(i));
                    lineAttributes.push_back(attributes.at(i));
                }
            }

            for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
                const std::vector<glm::vec3>& lineCenters = lineCentersList.at(lineId);
                const std::vector<float>& lineAttributes = lineAttributesList.at(lineId);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(lineId);
                assert(lineCenters.size() == lineAttributes.size());
                size_t n = lineCenters.size();
                uint32_t indexOffset = indexOffsetFunctor();

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

                    glm::vec3 helperAxis = lastLineNormal;
                    if (glm::length(glm::cross(helperAxis, tangent)) < 0.01f) {
                        // If tangent == lastNormal
                        helperAxis = glm::vec3(0.0f, 1.0f, 0.0f);
                        if (glm::length(glm::cross(helperAxis, normal)) < 0.01f) {
                            // If tangent == helperAxis
                            helperAxis = glm::vec3(0.0f, 0.0f, 1.0f);
                        }
                    }
                    normal = glm::normalize(helperAxis - tangent * glm::dot(helperAxis, tangent)); // Gram-Schmidt
                    lastLineNormal = normal;

                    float lineHierarchyLevel = 0.0f;
                    float majorStress = 1.0f, mediumStress = 1.0f, minorStress = 1.0f;
                    if (hasLineHierarchy) {
                        lineHierarchyLevel = stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType));
                    }
                    pointPushFunctor(
                            lineCenters.at(i), normal, tangent, lineAttributes.at(i),
                            indexOffset,
                            psIdx, lineHierarchyLevel, stressTrajectoryData.appearanceOrder,
                            majorStress, mediumStress, minorStress);
                    numValidLinePoints++;
                }

                if (numValidLinePoints == 1) {
                    // Only one vertex left -> Output nothing (tube consisting only of one point).
                    pointPopFunctor();
                }
                if (numValidLinePoints <= 1) {
                    continue;
                }

                // Create indices.
                int numSegments = numValidLinePoints - 1;
                indicesPushFunctor(numSegments, indexOffset);
            }
        }
    }
}

LinePassTubeRenderData LineDataStress::getLinePassTubeRenderData() {
    rebuildInternalRepresentationIfNecessary();

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;
    std::vector<uint32_t> vertexPrincipalStressIndices;
    std::vector<float> vertexLineHierarchyLevels;
    std::vector<uint32_t> vertexLineAppearanceOrders;

#ifdef USE_EIGEN
    std::vector<float> vertexMajorStresses;
    std::vector<float> vertexMediumStresses;
    std::vector<float> vertexMinorStresses;
#endif

    getLinePassTubeRenderDataGeneral(
            [&]() {
                return uint32_t(vertexPositions.size());
            },
            [&](const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent, float lineAttribute,
                    uint32_t indexOffset, int principalStressIndex, float lineHierarchyLevel, int lineAppearanceOrder,
                    float majorStress, float mediumStress, float minorStress) {
                vertexPositions.push_back(lineCenter);
                vertexNormals.push_back(normal);
                vertexTangents.push_back(tangent);
                vertexAttributes.push_back(lineAttribute);
                vertexPrincipalStressIndices.push_back(uint32_t(principalStressIndex));
                if (hasLineHierarchy) {
                    vertexLineHierarchyLevels.push_back(lineHierarchyLevel);
                }
                vertexLineAppearanceOrders.push_back(lineAppearanceOrder);
#ifdef USE_EIGEN
                if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                        && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                    vertexMajorStresses.push_back(majorStress);
                    vertexMediumStresses.push_back(mediumStress);
                    vertexMinorStresses.push_back(minorStress);
                }
#endif
            },
            [&] {
                vertexPositions.pop_back();
                vertexNormals.pop_back();
                vertexTangents.pop_back();
                vertexAttributes.pop_back();
                vertexPrincipalStressIndices.pop_back();
                vertexLineHierarchyLevels.pop_back();
                vertexLineAppearanceOrders.pop_back();
#ifdef USE_EIGEN
                if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                        && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                    vertexMajorStresses.pop_back();
                    vertexMediumStresses.pop_back();
                    vertexMinorStresses.pop_back();
                }
#endif
            },
            [&](int numSegments, uint32_t indexOffset) {
                for (int j = 0; j < numSegments; j++) {
                    lineIndices.push_back(indexOffset + j);
                    lineIndices.push_back(indexOffset + j + 1);
                }
            }
    );

    if (lineIndices.empty()) {
        return {};
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    LinePassTubeRenderData tubeRenderData;

    // Add the index buffer.
    tubeRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineIndices.size() * sizeof(uint32_t), lineIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPositions.size() * sizeof(glm::vec3), vertexPositions.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the attribute buffer.
    tubeRenderData.vertexAttributeBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexAttributes.size() * sizeof(float), vertexAttributes.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the normal buffer.
    tubeRenderData.vertexNormalBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexNormals.size() * sizeof(glm::vec3), vertexNormals.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexTangents.size() * sizeof(glm::vec3), vertexTangents.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the principal stress index buffer.
    tubeRenderData.vertexPrincipalStressIndexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPrincipalStressIndices.size() * sizeof(uint32_t), vertexPrincipalStressIndices.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        tubeRenderData.vertexLineHierarchyLevelBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexLineHierarchyLevels.size() * sizeof(float), vertexLineHierarchyLevels.data(),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    // Add the line appearance order buffer.
    tubeRenderData.vertexLineAppearanceOrderBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexLineAppearanceOrders.size() * sizeof(uint32_t), vertexLineAppearanceOrders.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

#ifdef USE_EIGEN
    if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
            && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        tubeRenderData.vertexMajorStressBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexMajorStresses.size() * sizeof(float), vertexMajorStresses.data(),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
        tubeRenderData.vertexMediumStressBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexMediumStresses.size() * sizeof(float), vertexMediumStresses.data(),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
        tubeRenderData.vertexMinorStressBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexMinorStresses.size() * sizeof(float), vertexMinorStresses.data(),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }
#endif

    return tubeRenderData;
}

LinePassQuadsRenderDataProgrammablePull LineDataStress::getLinePassQuadsRenderDataProgrammablePull() {
    rebuildInternalRepresentationIfNecessary();
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    LinePassQuadsRenderDataProgrammablePull tubeRenderData;

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
            //StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
            }
        }


        size_t numVerticesOld = vertexPositions.size();
        std::vector<uint32_t> validLineIndices;
        std::vector<uint32_t> numValidLineVertices;
        createLineTubesRenderDataCPU(
                lineCentersList, lineAttributesList,
                lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes,
                validLineIndices, numValidLineVertices);
        for (size_t idx = 0; idx < validLineIndices.size(); idx++) {
            uint32_t trajectoryIdx = validLineIndices.at(idx);
            uint32_t numValidPoints = numValidLineVertices.at(idx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            if (hasLineHierarchy) {
                for (uint32_t counter = 0; counter < numValidPoints; counter++) {
                    vertexLineHierarchyLevels.push_back(
                            stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
                }
            }
        }
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
    if (fetchIndices.empty()) {
        return {};
    }
    tubeRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, fetchIndices.size() * sizeof(uint32_t), fetchIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // 3. Add the point data for all line points.
    std::vector<LinePassQuadsLinePointDataProgrammablePull> linePointData;
    linePointData.resize(vertexPositions.size());
    for (size_t i = 0; i < vertexPositions.size(); i++) {
        linePointData.at(i).vertexPosition = vertexPositions.at(i);
        linePointData.at(i).vertexAttribute = vertexAttributes.at(i);
        linePointData.at(i).vertexTangent = vertexTangents.at(i);
        linePointData.at(i).principalStressIndex = vertexPrincipalStressIndices.at(i);
    }

    tubeRenderData.linePointsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, linePointData.size() * sizeof(LinePassQuadsLinePointDataProgrammablePull), linePointData.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        tubeRenderData.lineHierarchyLevelsBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexLineHierarchyLevels.size() * sizeof(float), vertexLineHierarchyLevels.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    return tubeRenderData;
}

TubeRenderDataOpacityOptimization LineDataStress::getTubeRenderDataOpacityOptimization() {
    rebuildInternalRepresentationIfNecessary();
    TubeRenderDataOpacityOptimization tubeRenderData;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

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
            //StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            for (size_t i = 0; i < trajectory.positions.size(); i++) {
                lineCenters.push_back(trajectory.positions.at(i));
                lineAttributes.push_back(attributes.at(i));
            }
        }

        size_t numVerticesOld = vertexPositions.size();
        std::vector<uint32_t> validLineIndices;
        std::vector<uint32_t> numValidLineVertices;
        createLineTubesRenderDataCPU(
                lineCentersList, lineAttributesList,
                lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes,
                validLineIndices, numValidLineVertices);
        for (size_t idx = 0; idx < validLineIndices.size(); idx++) {
            uint32_t trajectoryIdx = validLineIndices.at(idx);
            uint32_t numValidPoints = numValidLineVertices.at(idx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
            if (hasLineHierarchy) {
                for (uint32_t counter = 0; counter < numValidPoints; counter++) {
                    vertexLineHierarchyLevels.push_back(
                            stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
                }
            }
        }
        size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
        for (size_t i = 0; i < numVerticesAdded; i++) {
            vertexPrincipalStressIndices.push_back(psIdx);
        }
    }


    if (lineIndices.empty()) {
        return {};
    }

    // Add the index buffer.
    tubeRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineIndices.size() * sizeof(uint32_t), lineIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPositions.size() * sizeof(glm::vec3), vertexPositions.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the attribute buffer.
    tubeRenderData.vertexAttributeBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexAttributes.size() * sizeof(float), vertexAttributes.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexTangents.size() * sizeof(glm::vec3), vertexTangents.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the principal stress index buffer.
    tubeRenderData.vertexPrincipalStressIndexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPrincipalStressIndices.size() * sizeof(uint32_t), vertexPrincipalStressIndices.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        tubeRenderData.vertexLineHierarchyLevelBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexLineHierarchyLevels.size() * sizeof(float), vertexLineHierarchyLevels.data(),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    return tubeRenderData;
}

PointRenderData LineDataStress::getDegeneratePointsRenderData() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    if (degeneratePoints.empty()) {
        return {};
    }

    PointRenderData renderData;
    renderData.vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, degeneratePoints.size() * sizeof(glm::vec3), degeneratePoints.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    return renderData;
}

LinePassTubeRenderDataMeshShader LineDataStress::getLinePassTubeRenderDataMeshShader() {
    rebuildInternalRepresentationIfNecessary();

    // We can emit a maximum of 64 vertices/primitives from the mesh shader.
    int numLineSegmentsPerMeshlet = 64 / tubeNumSubdivisions - 1;
    std::vector<MeshletData> meshlets;
    MeshletData meshlet{};
    std::vector<LinePointDataUnified> linePoints;
    std::vector<StressLinePointDataUnified> stressLinePoints;
#ifdef USE_EIGEN
    std::vector<StressLinePointPrincipalStressDataUnified> stressLinePointsPrincipalStress;
#endif

    getLinePassTubeRenderDataGeneral(
            [&]() {
                return uint32_t(linePoints.size());
            },
            [&](const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent, float lineAttribute,
                    uint32_t indexOffset, int principalStressIndex, float lineHierarchyLevel, int lineAppearanceOrder,
                    float majorStress, float mediumStress, float minorStress) {
                LinePointDataUnified linePointData;
                linePointData.linePosition = lineCenter;
                linePointData.lineNormal = normal;
                linePointData.lineTangent = tangent;
                linePointData.lineAttribute = lineAttribute;
                linePointData.lineStartIndex = indexOffset;
                linePoints.push_back(linePointData);

                StressLinePointDataUnified stressLinePointData;
                stressLinePointData.linePrincipalStressIndex = uint32_t(principalStressIndex);
                stressLinePointData.lineLineAppearanceOrder = lineAppearanceOrder;
                stressLinePointData.lineLineHierarchyLevel = lineHierarchyLevel;
                stressLinePoints.push_back(stressLinePointData);

#ifdef USE_EIGEN
                if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                        && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                    StressLinePointPrincipalStressDataUnified stressLinePointPrincipalStressData;
                    stressLinePointPrincipalStressData.lineMajorStress = majorStress;
                    stressLinePointPrincipalStressData.lineMediumStress = mediumStress;
                    stressLinePointPrincipalStressData.lineMinorStress = minorStress;
                    stressLinePointsPrincipalStress.push_back(stressLinePointPrincipalStressData);
                }
#endif
            },
            [&] {
                linePoints.pop_back();
                stressLinePoints.pop_back();
#ifdef USE_EIGEN
                if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                        && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                    stressLinePointsPrincipalStress.pop_back();
                }
#endif
            },
            [&](int numSegments, uint32_t indexOffset) {
                int numSegmentsLeft = numSegments;
                int numMeshlets = sgl::iceil(numSegments, numLineSegmentsPerMeshlet);
                for (int j = 0; j < numMeshlets; j++) {
                    meshlet.linePointIndexStart = indexOffset + j * numLineSegmentsPerMeshlet;
                    meshlet.numLinePoints =
                            (numSegmentsLeft > numLineSegmentsPerMeshlet ? numLineSegmentsPerMeshlet : numSegmentsLeft) + 1;
                    meshlets.push_back(meshlet);
                    numSegmentsLeft -= numLineSegmentsPerMeshlet;
                }
            }
    );

    if (meshlets.empty()) {
        return {};
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    LinePassTubeRenderDataMeshShader renderData{};
    renderData.numMeshlets = uint32_t(meshlets.size());

    // Add the meshlet data buffer.
    renderData.meshletDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, meshlets.size() * sizeof(MeshletData), meshlets.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the line point data buffer.
    renderData.linePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, linePoints.size() * sizeof(LinePointDataUnified), linePoints.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the stress line point data buffer.
    renderData.stressLinePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, stressLinePoints.size() * sizeof(StressLinePointDataUnified), stressLinePoints.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

#ifdef USE_EIGEN
    if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER
            && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        renderData.stressLinePointPrincipalStressDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, stressLinePointsPrincipalStress.size() * sizeof(StressLinePointPrincipalStressDataUnified),
                stressLinePointsPrincipalStress.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }
#endif

    return renderData;
}

LinePassTubeRenderDataProgrammablePull LineDataStress::getLinePassTubeRenderDataProgrammablePull() {
    rebuildInternalRepresentationIfNecessary();

    std::vector<uint32_t> triangleIndices;
    std::vector<LinePointDataUnified> linePoints;
    std::vector<StressLinePointDataUnified> stressLinePoints;
#ifdef USE_EIGEN
    std::vector<StressLinePointPrincipalStressDataUnified> stressLinePointsPrincipalStress;
#endif

    getLinePassTubeRenderDataGeneral(
            [&]() {
                return uint32_t(linePoints.size());
            },
            [&](const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent, float lineAttribute,
                    uint32_t indexOffset, int principalStressIndex, float lineHierarchyLevel, int lineAppearanceOrder,
                    float majorStress, float mediumStress, float minorStress) {
                LinePointDataUnified linePointData;
                linePointData.linePosition = lineCenter;
                linePointData.lineNormal = normal;
                linePointData.lineTangent = tangent;
                linePointData.lineAttribute = lineAttribute;
                linePointData.lineStartIndex = indexOffset;
                linePoints.push_back(linePointData);

                StressLinePointDataUnified stressLinePointData;
                stressLinePointData.linePrincipalStressIndex = uint32_t(principalStressIndex);
                stressLinePointData.lineLineAppearanceOrder = lineAppearanceOrder;
                stressLinePointData.lineLineHierarchyLevel = lineHierarchyLevel;
                stressLinePoints.push_back(stressLinePointData);

#ifdef USE_EIGEN
                if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                        && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                    StressLinePointPrincipalStressDataUnified stressLinePointPrincipalStressData;
                    stressLinePointPrincipalStressData.lineMajorStress = majorStress;
                    stressLinePointPrincipalStressData.lineMediumStress = mediumStress;
                    stressLinePointPrincipalStressData.lineMinorStress = minorStress;
                    stressLinePointsPrincipalStress.push_back(stressLinePointPrincipalStressData);
                }
#endif
            },
            [&] {
                linePoints.pop_back();
                stressLinePoints.pop_back();
#ifdef USE_EIGEN
                if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
                        && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                    stressLinePointsPrincipalStress.pop_back();
                }
#endif
            },
            [&](int numSegments, uint32_t indexOffset) {
                for (int j = 0; j < numSegments; j++) {
                    uint32_t indexOffsetCurrent = (indexOffset + j) * tubeNumSubdivisions;
                    uint32_t indexOffsetNext = (indexOffset + j + 1) * tubeNumSubdivisions;
                    for (int k = 0; k < tubeNumSubdivisions; k++) {
                        int kNext = (k + 1) % tubeNumSubdivisions;

                        triangleIndices.push_back(indexOffsetCurrent + k);
                        triangleIndices.push_back(indexOffsetCurrent + kNext);
                        triangleIndices.push_back(indexOffsetNext + k);

                        triangleIndices.push_back(indexOffsetNext + k);
                        triangleIndices.push_back(indexOffsetCurrent + kNext);
                        triangleIndices.push_back(indexOffsetNext + kNext);
                    }
                }
            }
    );

    if (triangleIndices.empty()) {
        return {};
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    LinePassTubeRenderDataProgrammablePull renderData;

    // Add the index buffer.
    renderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, triangleIndices.size() * sizeof(uint32_t), triangleIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the line point data buffer.
    renderData.linePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, linePoints.size() * sizeof(LinePointDataUnified), linePoints.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the stress line point data buffer.
    renderData.stressLinePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, stressLinePoints.size() * sizeof(StressLinePointDataUnified), stressLinePoints.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

#ifdef USE_EIGEN
    if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL
            && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        renderData.stressLinePointPrincipalStressDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, stressLinePointsPrincipalStress.size() * sizeof(StressLinePointPrincipalStressDataUnified),
                stressLinePointsPrincipalStress.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }
#endif

    return renderData;
}

LinePassQuadsRenderData LineDataStress::getLinePassQuadsRenderData() {
    rebuildInternalRepresentationIfNecessary();
    LinePassQuadsRenderData bandRenderData;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

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

                uint32_t indexStart = uint32_t(vertexPositions.size());
                int n = int(trajectory.positions.size());
                int numValidLinePoints = 0;
                for (int i = 0; i < int(trajectory.positions.size()); i++) {
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
                    numValidLinePoints++;
                }

                if (numValidLinePoints == 1) {
                    // Only one vertex left -> Output nothing (tube consisting only of one point).
                    vertexPositions.pop_back();
                    vertexNormals.pop_back();
                    vertexTangents.pop_back();
                    vertexAttributes.pop_back();
                    vertexLineHierarchyLevels.pop_back();
                    vertexLineAppearanceOrders.pop_back();
                }
                if (numValidLinePoints <= 1) {
                    continue;
                }

                for (int i = 0; i < numValidLinePoints - 1; i++) {
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
                //StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
                assert(attributes.size() == trajectory.positions.size());
                std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
                std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
                for (size_t i = 0; i < trajectory.positions.size(); i++) {
                    lineCenters.push_back(trajectory.positions.at(i));
                    lineAttributes.push_back(attributes.at(i));
                }
            }

            size_t numVerticesOld = vertexPositions.size();
            std::vector<uint32_t> validLineIndices;
            std::vector<uint32_t> numValidLineVertices;
            createLineTubesRenderDataCPU(
                    lineCentersList, lineAttributesList,
                    lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes,
                    validLineIndices, numValidLineVertices);
            for (size_t idx = 0; idx < validLineIndices.size(); idx++) {
                uint32_t trajectoryIdx = validLineIndices.at(idx);
                uint32_t numValidPoints = numValidLineVertices.at(idx);
                StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);
                if (hasLineHierarchy) {
                    for (uint32_t counter = 0; counter < numValidPoints; counter++) {
                        vertexLineHierarchyLevels.push_back(
                                stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType)));
                    }
                }
                for (uint32_t counter = 0; counter < numValidPoints; counter++) {
                    vertexLineAppearanceOrders.push_back(stressTrajectoryData.appearanceOrder);
                }
            }
            size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
            for (size_t i = 0; i < numVerticesAdded; i++) {
                vertexPrincipalStressIndices.push_back(psIdx);
                vertexOffsetsLeft.emplace_back(0.0f);
                vertexOffsetsRight.emplace_back(0.0f);
            }
        }
    }


    if (lineIndices.empty()) {
        return {};
    }

    // Add the index buffer.
    bandRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineIndices.size() * sizeof(uint32_t), lineIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the position buffer.
    bandRenderData.vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPositions.size() * sizeof(glm::vec3), vertexPositions.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the attribute buffer.
    bandRenderData.vertexAttributeBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexAttributes.size() * sizeof(float), vertexAttributes.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the normal buffer.
    bandRenderData.vertexNormalBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexNormals.size() * sizeof(glm::vec3), vertexNormals.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the tangent buffer.
    bandRenderData.vertexTangentBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexTangents.size() * sizeof(glm::vec3), vertexTangents.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the vertex offset left buffer.
    bandRenderData.vertexOffsetLeftBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexOffsetsLeft.size() * sizeof(glm::vec3), vertexOffsetsLeft.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the vertex offset right buffer.
    bandRenderData.vertexOffsetRightBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexOffsetsRight.size() * sizeof(glm::vec3), vertexOffsetsRight.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the principal stress index buffer.
    bandRenderData.vertexPrincipalStressIndexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPrincipalStressIndices.size() * sizeof(uint32_t), vertexPrincipalStressIndices.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (hasLineHierarchy) {
        // Add the line hierarchy level buffer.
        bandRenderData.vertexLineHierarchyLevelBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexLineHierarchyLevels.size() * sizeof(float), vertexLineHierarchyLevels.data(),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    // Add the line appearance order buffer.
    bandRenderData.vertexLineAppearanceOrderBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexLineAppearanceOrders.size() * sizeof(uint32_t), vertexLineAppearanceOrders.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    return bandRenderData;
}

TubeTriangleRenderData LineDataStress::getLinePassTubeTriangleMeshRenderData(bool isRasterizer, bool vulkanRayTracing) {
    rebuildInternalRepresentationIfNecessary();
    if (vulkanTubeTriangleRenderData.vertexBuffer && vulkanTubeTriangleRenderDataIsRayTracing == vulkanRayTracing) {
        return vulkanTubeTriangleRenderData;
    }

    std::vector<uint32_t> tubeTriangleIndices;
    std::vector<TubeTriangleVertexData> tubeTriangleVertexDataList;
    std::vector<LinePointDataUnified> tubeTriangleLinePointDataList;
    std::vector<StressLinePointDataUnified> tubeTriangleStressLinePointDataList;
#ifdef USE_EIGEN
    std::vector<StressLinePointPrincipalStressDataUnified> tubeTriangleStressLinePointPrincipalStressDataList;
#endif

#ifdef USE_EIGEN
    int majorStressIdx = -1;
    int mediumStressIdx = -1;
    int minorStressIdx = -1;

    if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        majorStressIdx = getAttributeNameIndex("Major Stress");
        mediumStressIdx = getAttributeNameIndex("Medium Stress");
        minorStressIdx = getAttributeNameIndex("Minor Stress");
    }
#endif

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

        std::vector<std::vector<glm::vec3>> lineCentersList;
        lineCentersList.resize(trajectories.size());
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }
            if (!isRasterizer && lineHierarchySliderValues[psIdx]
                    < 1.0 - stressTrajectoriesData.at(trajectoryIdx).hierarchyLevels.at(int(lineHierarchyType))) {
                continue;
            }
            lineCentersList.at(trajectoryIdx) = trajectories.at(trajectoryIdx).positions;
        }

        std::vector<LinePointReference> linePointReferences;
        std::vector<glm::vec3> lineTangents;
        std::vector<glm::vec3> lineNormals;
        if (getUseBandRendering() && psUseBands.at(psIdx)) {
            std::vector<std::vector<glm::vec3>> bandPointsListRight;
            bandPointsListRight.resize(trajectories.size());
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                    continue;
                }
                if (useSmoothedBands) {
                    bandPointsListRight.at(trajectoryIdx) = bandPointsSmoothedListRightPs.at(i).at(trajectoryIdx);
                } else {
                    bandPointsListRight.at(trajectoryIdx) = bandPointsUnsmoothedListRightPs.at(i).at(trajectoryIdx);
                }
            }

            std::vector<uint32_t> linePrincipalStressIndexList;
            linePrincipalStressIndexList.resize(trajectories.size(), uint32_t(psIdx));

#ifdef USE_EIGEN
            std::vector<std::vector<float>> lineMajorStressesList;
            std::vector<std::vector<float>> lineMediumStressesList;
            std::vector<std::vector<float>> lineMinorStressesList;
            if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                lineMajorStressesList.resize(trajectories.size());
                lineMediumStressesList.resize(trajectories.size());
                lineMinorStressesList.resize(trajectories.size());
                for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                    if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                        continue;
                    }
                    const auto& trajectory = trajectories.at(trajectoryIdx);
                    size_t numPoints = trajectory.positions.size();
                    std::vector<float>& lineMajorStresses = lineMajorStressesList.at(trajectoryIdx);
                    std::vector<float>& lineMediumStresses = lineMediumStressesList.at(trajectoryIdx);
                    std::vector<float>& lineMinorStresses = lineMinorStressesList.at(trajectoryIdx);
                    lineMajorStresses.reserve(numPoints);
                    lineMediumStresses.reserve(numPoints);
                    lineMinorStresses.reserve(numPoints);
                    for (size_t ptIdx = 0; ptIdx < numPoints; ptIdx++) {
                        lineMajorStresses.push_back(trajectory.attributes.at(majorStressIdx).at(ptIdx));
                        lineMediumStresses.push_back(trajectory.attributes.at(mediumStressIdx).at(ptIdx));
                        lineMinorStresses.push_back(trajectory.attributes.at(minorStressIdx).at(ptIdx));
                    }
                }
            }
#endif

#ifdef USE_EIGEN
            if (bandRenderMode == LineDataStress::BandRenderMode::RIBBONS) {
#endif
                float binormalRadius = LineRenderer::getBandWidth() * 0.5f;
                float normalRadius = binormalRadius * minBandThickness;
                if (useCappedTubes) {
                    createCappedTriangleEllipticTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, normalRadius,
                            binormalRadius, tubeNumSubdivisions,
                            false, tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(tubeTriangleLinePointDataList.size()), lineTangents, lineNormals);
                } else {
                    createTriangleEllipticTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, normalRadius,
                            binormalRadius, tubeNumSubdivisions,
                            tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(tubeTriangleLinePointDataList.size()), lineTangents, lineNormals);
                }
#ifdef USE_EIGEN
            } else {
                float radius = LineRenderer::getBandWidth() * 0.5f;
                bool hyperstreamline = bandRenderMode == LineDataStress::BandRenderMode::HYPERSTREAMLINES;
                if (useCappedTubes) {
                    createCappedTrianglePrincipalStressTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, linePrincipalStressIndexList,
                            lineMajorStressesList, lineMediumStressesList, lineMinorStressesList,
                            radius, tubeNumSubdivisions, false,
                            hyperstreamline, tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(tubeTriangleLinePointDataList.size()), lineTangents, lineNormals);
                } else {
                    createTrianglePrincipalStressTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, linePrincipalStressIndexList,
                            lineMajorStressesList, lineMediumStressesList, lineMinorStressesList,
                            radius, tubeNumSubdivisions, hyperstreamline,
                            tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(tubeTriangleLinePointDataList.size()), lineTangents, lineNormals);
                }
            }
#endif
        } else {
            if (useCappedTubes) {
                createCappedTriangleTubesRenderDataCPU(
                        lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                        tubeNumSubdivisions, false,
                        tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                        uint32_t(tubeTriangleLinePointDataList.size()), lineTangents, lineNormals);
            } else {
                createTriangleTubesRenderDataCPU(
                        lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                        tubeNumSubdivisions,
                        tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                        uint32_t(tubeTriangleLinePointDataList.size()), lineTangents, lineNormals);
            }
        }

        size_t offset = tubeTriangleLinePointDataList.size();
        tubeTriangleLinePointDataList.resize(offset + linePointReferences.size());
        tubeTriangleStressLinePointDataList.resize(offset + linePointReferences.size());
#ifdef USE_EIGEN
        if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
            tubeTriangleStressLinePointPrincipalStressDataList.resize(offset + linePointReferences.size());
        }
#endif
        uint32_t lineStartIndex = 0;
        uint32_t lastTrajectoryIndex = 0;
        for (size_t ptIdx = 0; ptIdx < linePointReferences.size(); ptIdx++) {
            LinePointReference& linePointReference = linePointReferences.at(ptIdx);
            LinePointDataUnified& tubeTriangleLinePointData = tubeTriangleLinePointDataList.at(offset + ptIdx);
            StressLinePointDataUnified& tubeTriangleStressLinePointData =
                    tubeTriangleStressLinePointDataList.at(offset + ptIdx);
            Trajectory& trajectory = trajectories.at(linePointReference.trajectoryIndex);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(linePointReference.trajectoryIndex);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);

            tubeTriangleLinePointData.linePosition = trajectory.positions.at(linePointReference.linePointIndex);
            tubeTriangleLinePointData.lineAttribute = attributes.at(linePointReference.linePointIndex);
            tubeTriangleLinePointData.lineTangent = lineTangents.at(ptIdx);
            tubeTriangleLinePointData.lineNormal = lineNormals.at(ptIdx);

            if (lastTrajectoryIndex != linePointReference.trajectoryIndex) {
                lastTrajectoryIndex = linePointReference.trajectoryIndex;
                lineStartIndex = uint32_t(i);
            }
            tubeTriangleLinePointData.lineStartIndex = lineStartIndex;

            tubeTriangleStressLinePointData.linePrincipalStressIndex = uint32_t(psIdx);
            tubeTriangleStressLinePointData.lineLineHierarchyLevel =
                    stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType));
            tubeTriangleStressLinePointData.lineLineAppearanceOrder = uint32_t(stressTrajectoryData.appearanceOrder);

#ifdef USE_EIGEN
            if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                StressLinePointPrincipalStressDataUnified& stressLinePointPrincipalStressData =
                        tubeTriangleStressLinePointPrincipalStressDataList.at(offset + ptIdx);
                stressLinePointPrincipalStressData.lineMajorStress =
                        trajectory.attributes.at(majorStressIdx).at(linePointReference.linePointIndex);
                stressLinePointPrincipalStressData.lineMediumStress =
                        trajectory.attributes.at(mediumStressIdx).at(linePointReference.linePointIndex);
                stressLinePointPrincipalStressData.lineMinorStress =
                        trajectory.attributes.at(minorStressIdx).at(linePointReference.linePointIndex);
            }
#endif
        }

        lineCentersList.clear();
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    vulkanTubeTriangleRenderData = {};
    vulkanTubeTriangleRenderDataIsRayTracing = vulkanRayTracing;

    if (tubeTriangleIndices.empty()) {
        return vulkanTubeTriangleRenderData;
    }

    if (generateSplitTriangleData) {
        splitTriangleIndices(tubeTriangleIndices, tubeTriangleVertexDataList);
    }

    uint32_t indexBufferFlags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    uint32_t vertexBufferFlags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    if (vulkanRayTracing) {
        indexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
        vertexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
    }

    vulkanTubeTriangleRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeTriangleIndices.size() * sizeof(uint32_t), tubeTriangleIndices.data(),
            indexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeTriangleRenderData.vertexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeTriangleVertexDataList.size() * sizeof(TubeTriangleVertexData),
            tubeTriangleVertexDataList.data(),
            vertexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeTriangleRenderData.linePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeTriangleLinePointDataList.size() * sizeof(LinePointDataUnified),
            tubeTriangleLinePointDataList.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeTriangleRenderData.stressLinePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeTriangleStressLinePointDataList.size() * sizeof(StressLinePointDataUnified),
            tubeTriangleStressLinePointDataList.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

#ifdef USE_EIGEN
    if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        vulkanTubeTriangleRenderData.stressLinePointPrincipalStressDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, tubeTriangleStressLinePointPrincipalStressDataList.size() * sizeof(StressLinePointPrincipalStressDataUnified),
                tubeTriangleStressLinePointPrincipalStressDataList.data(),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
                | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    }
#endif

    if (generateSplitTriangleData) {
        std::vector<uint32_t> instanceTriangleIndexOffsets;
        uint32_t batchIndexBufferOffset = 0;
        for (const uint32_t& batchNumIndices : tubeTriangleSplitData.numBatchIndices) {
            instanceTriangleIndexOffsets.push_back(batchIndexBufferOffset);
            batchIndexBufferOffset += uint32_t(batchNumIndices) / 3u;
        }
        vulkanTubeTriangleRenderData.instanceTriangleIndexOffsetBuffer = std::make_shared<sgl::vk::Buffer>(
                device, instanceTriangleIndexOffsets.size() * sizeof(uint32_t),
                instanceTriangleIndexOffsets.data(),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    return vulkanTubeTriangleRenderData;
}

TubeAabbRenderData LineDataStress::getLinePassTubeAabbRenderData(bool isRasterizer) {
    rebuildInternalRepresentationIfNecessary();
    if (vulkanTubeAabbRenderData.aabbBuffer) {
        return vulkanTubeAabbRenderData;
    }

    //glm::vec3 lineWidthOffset(std::max(LineRenderer::getLineWidth() * 0.5f, LineRenderer::getBandWidth() * 0.5f));
    glm::vec3 lineWidthOffset(LineRenderer::getLineWidth() * 0.5f);

    std::vector<uint32_t> lineSegmentPointIndices;
    std::vector<sgl::AABB3> lineSegmentAabbs;
    std::vector<LinePointDataUnified> tubeLinePointDataList;
    std::vector<StressLinePointDataUnified> tubeStressLinePointDataList;
#ifdef USE_EIGEN
    std::vector<StressLinePointPrincipalStressDataUnified> tubeStressLinePointPrincipalStressDataList;
#endif

#ifdef USE_EIGEN
    int majorStressIdx = -1;
    int mediumStressIdx = -1;
    int minorStressIdx = -1;

    if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        majorStressIdx = getAttributeNameIndex("Major Stress");
        mediumStressIdx = getAttributeNameIndex("Medium Stress");
        minorStressIdx = getAttributeNameIndex("Minor Stress");
    }
#endif

    lineSegmentPointIndices.reserve(getNumLineSegments() * 2);
    lineSegmentAabbs.reserve(getNumLineSegments());
    const size_t numLinePointsEstimated = getNumLinePoints();
    tubeLinePointDataList.reserve(numLinePointsEstimated);
    tubeStressLinePointDataList.reserve(numLinePointsEstimated);
    tubeStressLinePointPrincipalStressDataList.reserve(numLinePointsEstimated);
    uint32_t lineSegmentIndexCounter = 0;
    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories &trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData &stressTrajectoriesData = stressTrajectoriesDataPs.at(i);
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);

        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }
            if (!isRasterizer && lineHierarchySliderValues[psIdx]
                    < 1.0 - stressTrajectoriesData.at(trajectoryIdx).hierarchyLevels.at(int(lineHierarchyType))) {
                continue;
            }
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            StressTrajectoryData& stressTrajectoryData = stressTrajectoriesData.at(trajectoryIdx);

            glm::vec3 lastLineNormal(1.0f, 0.0f, 0.0f);
            uint32_t numValidLinePoints = 0;
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

                LinePointDataUnified linePointData{};
                linePointData.linePosition = trajectory.positions.at(i);
                linePointData.lineAttribute = trajectory.attributes.at(selectedAttributeIndex).at(i);
                linePointData.lineTangent = tangent;
                linePointData.lineNormal = lastLineNormal;
                tubeLinePointDataList.push_back(linePointData);

                StressLinePointDataUnified stressLinePointData{};
                stressLinePointData.lineLineHierarchyLevel = stressTrajectoryData.hierarchyLevels.at(int(lineHierarchyType));
                stressLinePointData.lineLineAppearanceOrder = uint32_t(stressTrajectoryData.appearanceOrder);
                stressLinePointData.linePrincipalStressIndex = uint32_t(psIdx);
                tubeStressLinePointDataList.push_back(stressLinePointData);

#ifdef USE_EIGEN
                if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                    StressLinePointPrincipalStressDataUnified stressLinePointPrincipalStressData{};
                    stressLinePointPrincipalStressData.lineMajorStress =
                            trajectory.attributes.at(majorStressIdx).at(i);
                    stressLinePointPrincipalStressData.lineMediumStress =
                            trajectory.attributes.at(mediumStressIdx).at(i);
                    stressLinePointPrincipalStressData.lineMinorStress =
                            trajectory.attributes.at(minorStressIdx).at(i);
                    tubeStressLinePointPrincipalStressDataList.push_back(stressLinePointPrincipalStressData);
                }
#endif

                numValidLinePoints++;
            }

            if (numValidLinePoints == 1) {
                // Only one vertex left -> output nothing (tube consisting only of one point).
                tubeLinePointDataList.pop_back();
            }
            if (numValidLinePoints <= 1) {
                continue;
            }

            for (uint32_t pointIdx = 1; pointIdx < numValidLinePoints; pointIdx++) {
                lineSegmentPointIndices.push_back(lineSegmentIndexCounter + pointIdx - 1);
                lineSegmentPointIndices.push_back(lineSegmentIndexCounter + pointIdx);

                const glm::vec3& pt0 = tubeLinePointDataList.at(lineSegmentIndexCounter + pointIdx - 1).linePosition;
                const glm::vec3& pt1 = tubeLinePointDataList.at(lineSegmentIndexCounter + pointIdx).linePosition;

                sgl::AABB3 aabb;
                aabb.min = glm::min(pt0, pt1) - lineWidthOffset;
                aabb.max = glm::max(pt0, pt1) + lineWidthOffset;
                lineSegmentAabbs.push_back(aabb);
            }
            lineSegmentIndexCounter += numValidLinePoints;
        }
    }

    if (lineSegmentIndexCounter == 0) {
        return {};
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    vulkanTubeAabbRenderData = {};

    if (lineSegmentPointIndices.empty()) {
        return vulkanTubeAabbRenderData;
    }

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

    vulkanTubeAabbRenderData.linePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeLinePointDataList.size() * sizeof(LinePointDataUnified),
            tubeLinePointDataList.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanTubeAabbRenderData.stressLinePointDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, tubeStressLinePointDataList.size() * sizeof(StressLinePointDataUnified),
            tubeStressLinePointDataList.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

#ifdef USE_EIGEN
    if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        vulkanTubeAabbRenderData.stressLinePointPrincipalStressDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, tubeStressLinePointPrincipalStressDataList.size() * sizeof(StressLinePointPrincipalStressDataUnified),
                tubeStressLinePointPrincipalStressDataList.data(),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
                | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    }
#endif

    return vulkanTubeAabbRenderData;
}

void LineDataStress::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string>& preprocessorDefines, bool isRasterizer) {
    LineData::getVulkanShaderPreprocessorDefines(preprocessorDefines, isRasterizer);
    preprocessorDefines.insert(std::make_pair("STRESS_LINE_DATA", ""));

    bool useBandsDefine;
    if (isRasterizer) {
        useBandsDefine = getUseBandRendering();
    } else {
        useBandsDefine = std::any_of(
                psUseBands.cbegin(), psUseBands.cend(), [] (bool useBand) { return useBand; });
    }
    if (useBandsDefine) {
        preprocessorDefines.insert(std::make_pair("USE_BANDS", ""));
    }

    if (usePrincipalStressDirectionIndex) {
        preprocessorDefines.insert(std::make_pair("USE_PRINCIPAL_STRESS_DIRECTION_INDEX", ""));
        preprocessorDefines.insert(std::make_pair("USE_MULTI_VAR_TRANSFER_FUNCTION", ""));
    }
    if (useLineHierarchy) {
        preprocessorDefines.insert(std::make_pair("USE_LINE_HIERARCHY_LEVEL", ""));
    }
    if (rendererSupportsTransparency) {
        preprocessorDefines.insert(std::make_pair("USE_TRANSPARENCY", ""));
    }

    if (isRasterizer && getUseBandRendering() && bandRenderMode == BandRenderMode::RIBBONS) {
        if (renderThickBands) {
            preprocessorDefines.insert(std::make_pair("BAND_RENDERING_THICK", ""));
            preprocessorDefines.insert(std::make_pair("MIN_THICKNESS", std::to_string(minBandThickness)));
        } else {
            preprocessorDefines.insert(std::make_pair("MIN_THICKNESS", std::to_string(1e-2f)));
        }
    }

#ifdef USE_EIGEN
    if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
            && bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        preprocessorDefines.insert(std::make_pair("USE_PRINCIPAL_STRESSES", ""));
    }
    if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
            && bandRenderMode == LineDataStress::BandRenderMode::EIGENVALUE_RATIO) {
        preprocessorDefines.insert(std::make_pair("USE_NORMAL_STRESS_RATIO_TUBES", ""));
    }
    if (getLinePrimitiveModeSupportsLineMultiWidth(linePrimitiveMode)
            && bandRenderMode == LineDataStress::BandRenderMode::HYPERSTREAMLINES) {
        preprocessorDefines.insert(std::make_pair("USE_HYPERSTREAMLINES", ""));
    }
#endif
}

void LineDataStress::setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData) {
    LineData::setVulkanRenderDataDescriptors(renderData);

    renderData->setStaticBufferOptional(
            stressLineUniformDataBuffer, "StressLineUniformDataBuffer");

    if (usePrincipalStressDirectionIndex
            && renderData->getShaderStages()->hasDescriptorBinding(0, "transferFunctionTexture")) {
        const sgl::vk::DescriptorInfo& descriptorInfo = renderData->getShaderStages()->getDescriptorInfoByName(
                0, "transferFunctionTexture");
        if (descriptorInfo.image.arrayed == 1) {
            renderData->setStaticTexture(
                    multiVarTransferFunctionWindow.getTransferFunctionMapTextureVulkan(),
                    "transferFunctionTexture");
            renderData->setStaticBuffer(
                    multiVarTransferFunctionWindow.getMinMaxSsboVulkan(), "MinMaxBuffer");
        }
    }
    if (useLineHierarchy && rendererSupportsTransparency
            && renderData->getShaderStages()->hasDescriptorBinding(0, "lineHierarchyImportanceMap")) {
        renderData->setStaticTexture(
                stressLineHierarchyMappingWidget.getHierarchyMappingTexture(),
                "lineHierarchyImportanceMap");
    }
}

void LineDataStress::updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer) {
    LineData::updateVulkanUniformBuffers(lineRenderer, renderer);

    stressLineUniformData.lineHierarchySlider = glm::vec3(1.0f) - lineHierarchySliderValues;
    if (getUseBandRendering()) {
        stressLineUniformData.psUseBands = glm::ivec3(psUseBands[0], psUseBands[1], psUseBands[2]);
    } else {
        stressLineUniformData.psUseBands = glm::ivec3(0, 0, 0);
    }
    stressLineUniformData.currentSeedIdx = int32_t(currentSeedIdx);

    stressLineUniformDataBuffer->updateData(
            sizeof(StressLineUniformData), &stressLineUniformData, renderer->getVkCommandBuffer());
}

void LineDataStress::getTriangleMesh(
        LineRenderer* lineRenderer,
        std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals, std::vector<float>& vertexAttributes) {
    rebuildInternalRepresentationIfNecessary();

    std::vector<TubeTriangleVertexData> tubeTriangleVertexDataList;
    size_t numTubeTriangleLinePoints = 0;

#ifdef USE_EIGEN
    int majorStressIdx = -1;
    int mediumStressIdx = -1;
    int minorStressIdx = -1;

    if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
        majorStressIdx = getAttributeNameIndex("Major Stress");
        mediumStressIdx = getAttributeNameIndex("Medium Stress");
        minorStressIdx = getAttributeNameIndex("Minor Stress");
    }
#endif

    for (size_t i = 0; i < trajectoriesPs.size(); i++) {
        int psIdx = loadedPsIndices.at(i);
        if (!usedPsDirections.at(psIdx)) {
            continue;
        }

        Trajectories& trajectories = trajectoriesPs.at(i);
        StressTrajectoriesData& stressTrajectoriesData = stressTrajectoriesDataPs.at(i);

        size_t offsetVertices = tubeTriangleVertexDataList.size();

        std::vector<std::vector<glm::vec3>> lineCentersList;
        lineCentersList.resize(trajectories.size());
        std::vector<bool>& filteredTrajectories = filteredTrajectoriesPs.at(i);
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }
            if ((!lineRenderer || !lineRenderer->isRasterizer) && lineHierarchySliderValues[psIdx]
                    < 1.0 - stressTrajectoriesData.at(trajectoryIdx).hierarchyLevels.at(int(lineHierarchyType))) {
                continue;
            }
            lineCentersList.at(trajectoryIdx) = trajectories.at(trajectoryIdx).positions;
        }

        std::vector<LinePointReference> linePointReferences;
        std::vector<glm::vec3> lineTangents;
        std::vector<glm::vec3> lineNormals;
        if (getUseBandRendering() && psUseBands.at(psIdx)) {
            std::vector<std::vector<glm::vec3>> bandPointsListRight;
            bandPointsListRight.resize(trajectories.size());
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                    continue;
                }
                if (useSmoothedBands) {
                    bandPointsListRight.at(trajectoryIdx) = bandPointsSmoothedListRightPs.at(i).at(trajectoryIdx);
                } else {
                    bandPointsListRight.at(trajectoryIdx) = bandPointsUnsmoothedListRightPs.at(i).at(trajectoryIdx);
                }
            }

            std::vector<uint32_t> linePrincipalStressIndexList;
            linePrincipalStressIndexList.resize(trajectories.size(), uint32_t(psIdx));

#ifdef USE_EIGEN
            std::vector<std::vector<float>> lineMajorStressesList;
            std::vector<std::vector<float>> lineMediumStressesList;
            std::vector<std::vector<float>> lineMinorStressesList;
            if (bandRenderMode != LineDataStress::BandRenderMode::RIBBONS) {
                lineMajorStressesList.resize(trajectories.size());
                lineMediumStressesList.resize(trajectories.size());
                lineMinorStressesList.resize(trajectories.size());
                for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                    if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                        continue;
                    }
                    const auto& trajectory = trajectories.at(trajectoryIdx);
                    size_t numPoints = trajectory.positions.size();
                    std::vector<float>& lineMajorStresses = lineMajorStressesList.at(trajectoryIdx);
                    std::vector<float>& lineMediumStresses = lineMediumStressesList.at(trajectoryIdx);
                    std::vector<float>& lineMinorStresses = lineMinorStressesList.at(trajectoryIdx);
                    lineMajorStresses.reserve(numPoints);
                    lineMediumStresses.reserve(numPoints);
                    lineMinorStresses.reserve(numPoints);
                    for (size_t ptIdx = 0; ptIdx < numPoints; ptIdx++) {
                        lineMajorStresses.push_back(trajectory.attributes.at(majorStressIdx).at(ptIdx));
                        lineMediumStresses.push_back(trajectory.attributes.at(mediumStressIdx).at(ptIdx));
                        lineMinorStresses.push_back(trajectory.attributes.at(minorStressIdx).at(ptIdx));
                    }
                }
            }
#endif

#ifdef USE_EIGEN
            if (bandRenderMode == LineDataStress::BandRenderMode::RIBBONS) {
#endif
                float binormalRadius = LineRenderer::getBandWidth() * 0.5f;
                float normalRadius = binormalRadius * minBandThickness;
                if (useCappedTubes) {
                    createCappedTriangleEllipticTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, normalRadius,
                            binormalRadius, tubeNumSubdivisions,
                            false, triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(numTubeTriangleLinePoints), lineTangents, lineNormals);
                } else {
                    createTriangleEllipticTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, normalRadius,
                            binormalRadius, tubeNumSubdivisions,
                            triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(numTubeTriangleLinePoints), lineTangents, lineNormals);
                }
#ifdef USE_EIGEN
            } else {
                float radius = LineRenderer::getBandWidth() * 0.5f;
                bool hyperstreamline = bandRenderMode == LineDataStress::BandRenderMode::HYPERSTREAMLINES;
                if (useCappedTubes) {
                    createCappedTrianglePrincipalStressTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, linePrincipalStressIndexList,
                            lineMajorStressesList, lineMediumStressesList, lineMinorStressesList,
                            radius, tubeNumSubdivisions, false,
                            hyperstreamline, triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(numTubeTriangleLinePoints), lineTangents, lineNormals);
                } else {
                    createTrianglePrincipalStressTubesRenderDataCPU(
                            lineCentersList, bandPointsListRight, linePrincipalStressIndexList,
                            lineMajorStressesList, lineMediumStressesList, lineMinorStressesList,
                            radius, tubeNumSubdivisions, hyperstreamline,
                            triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                            uint32_t(numTubeTriangleLinePoints), lineTangents, lineNormals);
                }
            }
#endif
        } else {
            if (useCappedTubes) {
                createCappedTriangleTubesRenderDataCPU(
                        lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                        tubeNumSubdivisions, false,
                        triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                        uint32_t(numTubeTriangleLinePoints), lineTangents, lineNormals);
            } else {
                createTriangleTubesRenderDataCPU(
                        lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                        tubeNumSubdivisions,
                        triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                        uint32_t(numTubeTriangleLinePoints), lineTangents, lineNormals);
            }
        }

        size_t offset = numTubeTriangleLinePoints;
        numTubeTriangleLinePoints = offset + linePointReferences.size();

        for (size_t vertexIdx = offsetVertices; vertexIdx < tubeTriangleVertexDataList.size(); vertexIdx++) {
            TubeTriangleVertexData& tubeTriangleVertexData = tubeTriangleVertexDataList.at(vertexIdx);
            vertexPositions.push_back(tubeTriangleVertexData.vertexPosition);
            vertexNormals.push_back(tubeTriangleVertexData.vertexNormal);

            LinePointReference& linePointReference = linePointReferences.at(
                    (tubeTriangleVertexData.vertexLinePointIndex & 0x7FFFFFFFu) - offset);
            Trajectory& trajectory = trajectories.at(linePointReference.trajectoryIndex);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            float attributeValue = attributes.at(linePointReference.linePointIndex);
            vertexAttributes.push_back(attributeValue);
        }

        lineCentersList.clear();
    }
}

void LineDataStress::getTriangleMesh(
        LineRenderer* lineRenderer, std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) {
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    getTriangleMesh(lineRenderer, triangleIndices, vertexPositions, vertexNormals, vertexAttributes);
}
