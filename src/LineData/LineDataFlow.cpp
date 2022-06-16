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

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include <ImGui/imgui_custom.h>

#include "Loaders/TrajectoryFile.hpp"
#include "Renderers/LineRenderer.hpp"
#include "Renderers/Tubes/Tubes.hpp"
#include "LineDataFlow.hpp"

bool LineDataFlow::useRibbons = true;
bool LineDataFlow::useRotatingHelicityBands = false;
bool LineDataFlow::useUniformTwistLineWidth = true;
float LineDataFlow::separatorWidth = 0.2f;

LineDataFlow::LineDataFlow(sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineData(transferFunctionWindow, DATA_SET_TYPE_FLOW_LINES),
          multiVarTransferFunctionWindow("multivar", {
                  "reds.xml", "blues.xml", "greens.xml", "purples.xml", "oranges.xml", "pinks.xml", "golds.xml",
                  "dark-blues.xml" }) {
    lineDataWindowName = "Line Data (Flow)";

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    multiVarUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(MultiVarUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

LineDataFlow::~LineDataFlow() = default;

bool LineDataFlow::settingsDiffer(LineData* other) {
    return hasBandsData != static_cast<LineDataFlow*>(other)->hasBandsData;
}

void LineDataFlow::update(float dt) {
    if (useMultiVarRendering) {
        multiVarTransferFunctionWindow.update(dt);
    }
}

bool LineDataFlow::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = LineData::renderGuiPropertyEditorNodes(propertyEditor);

    if (!useRotatingHelicityBands && getUseBandRendering()) {
        if (propertyEditor.addCheckbox("Render as Bands", &useRibbons)) {
            dirty = true;
            shallReloadGatherShader = true;
        }
    }

    if (hasHelicity || getUseBandRendering()) {
        if (propertyEditor.beginNode("Advanced Settings")) {
            if (!useRotatingHelicityBands && getUseBandRendering()) {
                if (propertyEditor.addCheckbox("Render Thick Bands", &renderThickBands)) {
                    shallReloadGatherShader = true;
                }

                if (renderThickBands && propertyEditor.addSliderFloat(
                        "Min. Band Thickness", &minBandThickness, 0.01f, 1.0f)) {
                    triangleRepresentationDirty = true;
                    shallReloadGatherShader = true;
                }
            }

            if (hasHelicity) {
                if (propertyEditor.addCheckbox("Show Helicity Bands", &useRotatingHelicityBands)) {
                    if (useRotatingHelicityBands) {
                        useRibbons = false;
                    }
                    dirty = true;
                    shallReloadGatherShader = true;
                }
                if (useRotatingHelicityBands) {
                    if (propertyEditor.addSliderFloatEdit(
                            "Helicity Factor", &helicityRotationFactor,
                            0.0f, 2.0f) == ImGui::EditMode::INPUT_FINISHED) {
                        dirty = true;
                    }
                }
            }

            if (useRotatingHelicityBands && propertyEditor.addSliderInt(
                    "Band Subdivisions", reinterpret_cast<int*>(&numSubdivisionsBands),
                    2, 16)) {
                reRender = true;
                setNumSubdivisionsManually = true;
            }

            if (useRotatingHelicityBands && propertyEditor.addCheckbox(
                    "Uniform Twist Lines", &useUniformTwistLineWidth)) {
                reRender = true;
                shallReloadGatherShader = true;
            }

            if (propertyEditor.addCheckbox("Multi-Var Rendering", &useMultiVarRendering)) {
                dirty = true;
                shallReloadGatherShader = true;
                recomputeColorLegend();
                recomputeWidgetPositions();
                if (!setNumSubdivisionsManually) {
                    numSubdivisionsBands = useMultiVarRendering ? 8 : 6;
                }
            }

            if (useMultiVarRendering) {
                bool itemHasChanged = false;
                std::vector<std::string> comboSelVec(0);
                if (propertyEditor.addBeginCombo(
                        "Variables", comboValue, ImGuiComboFlags_NoArrowButton)) {
                    for (size_t v = 0; v < isAttributeSelectedArray.size(); ++v) {
                        if (ImGui::Selectable(
                                attributeNames.at(v).c_str(),
                                reinterpret_cast<bool*>(&isAttributeSelectedArray[v]),
                                ImGuiSelectableFlags_::ImGuiSelectableFlags_DontClosePopups)) {
                            itemHasChanged = true;
                        }

                        if (static_cast<bool>(isAttributeSelectedArray.at(v))) {
                            ImGui::SetItemDefaultFocus();
                            comboSelVec.push_back(attributeNames.at(v));
                        }
                    }

                    comboValue = "";
                    for (size_t v = 0; v < comboSelVec.size(); ++v) {
                        comboValue += comboSelVec[v];
                        if (comboSelVec.size() > 1 && v + 1 != comboSelVec.size()) {
                            comboValue += ",";
                        }
                    }

                    if (itemHasChanged) {
                        selectedAttributes.clear();
                        for (size_t varIdx = 0; varIdx < isAttributeSelectedArray.size(); ++varIdx) {
                            if (isAttributeSelectedArray.at(varIdx) != 0) {
                                selectedAttributes.push_back(varIdx);
                            }
                        }
                        recomputeWidgetPositions();
                        reRender = true;
                    }

                    propertyEditor.addEndCombo();
                }
            }

            if ((useMultiVarRendering || useRotatingHelicityBands) && propertyEditor.addSliderFloat(
                    "Separator Width", &separatorWidth, 0.0f, 1.0f)) {
                reRender = true;
            }

            propertyEditor.endNode();
        }
    }

    return shallReloadGatherShader;
}

bool LineDataFlow::renderGuiWindowSecondary()  {
    if (useMultiVarRendering && multiVarTransferFunctionWindow.renderGui()) {
        reRender = true;
        if (multiVarTransferFunctionWindow.getTransferFunctionMapRebuilt()) {
            onTransferFunctionMapRebuilt();
            sgl::EventManager::get()->triggerEvent(std::make_shared<sgl::Event>(
                    ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT));
        }
    }

    return LineData::renderGuiWindowSecondary();
}

bool LineDataFlow::renderGuiOverlay() {
    bool shallReloadGatherShader = false;
    if (useMultiVarRendering && shallRenderColorLegendWidgets) {
        for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
            if (isAttributeSelectedArray.at(i)) {
                colorLegendWidgets.at(i).setAttributeMinValue(
                        multiVarTransferFunctionWindow.getSelectedRangeMin(int(i)));
                colorLegendWidgets.at(i).setAttributeMaxValue(
                        multiVarTransferFunctionWindow.getSelectedRangeMax(int(i)));
                colorLegendWidgets.at(i).renderGui();
            }
        }
    } else {
        shallReloadGatherShader = LineData::renderGuiOverlay() || shallReloadGatherShader;
    }
    return shallReloadGatherShader;
}

void LineDataFlow::setClearColor(const sgl::Color& clearColor) {
    LineData::setClearColor(clearColor);
    multiVarTransferFunctionWindow.setClearColor(clearColor);
}

void LineDataFlow::setUseLinearRGB(bool useLinearRGB) {
    multiVarTransferFunctionWindow.setUseLinearRGB(useLinearRGB);
}

void LineDataFlow::recomputeWidgetPositions() {
    if (!useMultiVarRendering) {
        for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
            colorLegendWidgets.at(i).setPositionIndex(0, 1);
        }
        return;
    }

    int numWidgetsVisible = 0;
    for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
        if (isAttributeSelectedArray.at(i)) {
            numWidgetsVisible++;
        }
    }
    int positionCounter = 0;
    for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
        colorLegendWidgets.at(i).setPositionIndex(positionCounter, numWidgetsVisible);
        if (isAttributeSelectedArray.at(i)) {
            positionCounter++;
        }
    }
}

bool LineDataFlow::loadFromFile(
        const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
        glm::mat4* transformationMatrixPtr) {
    this->fileNames = fileNames;
    attributeNames = dataSetInformation.attributeNames;
    BinLinesData binLinesData = loadFlowTrajectoriesFromFile(
            fileNames.front(), attributeNames, true,
            false, transformationMatrixPtr);
    trajectories = {};
    bool dataLoaded = !binLinesData.trajectories.empty();

    if (dataLoaded) {
        attributeNames = binLinesData.attributeNames;
        ribbonsDirections = binLinesData.ribbonsDirections;
        shallRenderSimulationMeshBoundary = !binLinesData.simulationMeshOutlineTriangleIndices.empty();
        simulationMeshOutlineTriangleIndices = binLinesData.simulationMeshOutlineTriangleIndices;
        simulationMeshOutlineVertexPositions = binLinesData.simulationMeshOutlineVertexPositions;
        simulationMeshOutlineVertexNormals = binLinesData.simulationMeshOutlineVertexNormals;
        onAttributeNamesSet();
        setTrajectoryData(binLinesData.trajectories);
    }

    return dataLoaded;
}

void LineDataFlow::onAttributeNamesSet() {
    isAttributeSelectedArray.clear();
    isAttributeSelectedArray.resize(attributeNames.size(), 0);
    if (!attributeNames.empty()) {
        sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
        multiVarSelectedAttributesBuffer = std::make_shared<sgl::vk::Buffer>(
                device, sizeof(uint32_t) * attributeNames.size(),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }
}

void LineDataFlow::setTrajectoryData(const Trajectories& trajectories) {
    hasBandsData = !ribbonsDirections.empty();
    useRibbons = !useRotatingHelicityBands && hasBandsData;
    if (ribbonsDirections.empty()
            && (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER || linePrimitiveMode == LINE_PRIMITIVES_TUBE_RIBBONS_GEOMETRY_SHADER)) {
        linePrimitiveMode = LINE_PRIMITIVES_QUADS_PROGRAMMABLE_PULL;
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

    this->trajectories = trajectories;

    if (getNumAttributes() == 0) {
        for (Trajectory& trajectory : this->trajectories) {
            trajectory.attributes.resize(1);
            trajectory.attributes.front().resize(trajectory.positions.size());
        }
    }

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
        for (const Trajectory& trajectory : this->trajectories) {
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

    helicityAttributeIndex = -1;
    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
        std::string attributeNameLower = boost::to_lower_copy(attributeNames.at(attrIdx));
        if (attributeNameLower.find("helicity") != std::string::npos) {
            helicityAttributeIndex = int(attrIdx);
            break;
        }
    }
    if (helicityAttributeIndex != -1) {
        hasHelicity = true;
        //useRotatingHelicityBands = true;
        //useRibbons = false;
        glm::vec2 minMaxHelicity = minMaxAttributeValues.at(helicityAttributeIndex);
        maxHelicity = std::max(std::abs(minMaxHelicity.x), std::abs(minMaxHelicity.y));
    }

    numTotalTrajectoryPoints = 0;
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(+: numTotalTrajectoryPoints) shared(trajectories) default(none)
#endif
    for (size_t i = 0; i < trajectories.size(); i++) {
        const Trajectory& trajectory = trajectories.at(i);
        numTotalTrajectoryPoints += trajectory.positions.size();
    }

    modelBoundingBox = computeTrajectoriesAABB3(this->trajectories);
    focusBoundingBox = modelBoundingBox;

    dirty = true;
}

bool LineDataFlow::getIsSmallDataSet() const {
    return numTotalTrajectoryPoints <= SMALL_DATASET_LINE_POINTS_MAX;
}

bool LineDataFlow::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = LineData::setNewSettings(settings);

    if (getUseBandRendering() && settings.getValueOpt("use_ribbons", useRibbons)) {
        if (settings.getValueOpt("use_ribbons", useRibbons)) {
            dirty = true;
        }

        if (settings.getValueOpt("thick_bands", renderThickBands)) {
            shallReloadGatherShader = true;
        }

        if (settings.getValueOpt("min_band_thickness", minBandThickness)) {
            shallReloadGatherShader = true;
        }

        if (settings.getValueOpt("rotating_helicity_bands", useRotatingHelicityBands)) {
            if (useRotatingHelicityBands) {
                useRibbons = false;
            }
            dirty = true;
            shallReloadGatherShader = true;
        }
        if (settings.getValueOpt("separator_width", separatorWidth)) {
            reRender = true;
        }
        if (settings.getValueOpt("use_uniform_twist_line_width", useUniformTwistLineWidth)) {
            shallReloadGatherShader = true;
        }
        if (settings.getValueOpt("use_multi_var_rendering", useMultiVarRendering)) {
            dirty = true;
            shallReloadGatherShader = true;
            recomputeColorLegend();
            recomputeWidgetPositions();
        }
    }

    return shallReloadGatherShader;
}

void LineDataFlow::recomputeHistogram() {
    std::vector<float> attributeList;
    if (!attributeNames.empty()) {
        for (const Trajectory& trajectory : trajectories) {
            for (float val : trajectory.attributes.at(selectedAttributeIndex)) {
                attributeList.push_back(val);
            }
        }
    }
    glm::vec2 minMaxAttributes;
    if (selectedAttributeIndex < int(minMaxAttributeValues.size())) {
        minMaxAttributes = minMaxAttributeValues.at(selectedAttributeIndex);
    } else {
        minMaxAttributes = glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
    }
    transferFunctionWindow.computeHistogram(
            attributeList, minMaxAttributes.x, minMaxAttributes.y);

    const size_t numAttributes = attributeNames.size();
    std::vector<std::vector<float>> attributesList(numAttributes);
    for (const Trajectory& trajectory : trajectories) {
        for (size_t attrIdx = 0; attrIdx < numAttributes; attrIdx++) {
            std::vector<float>& attributeList = attributesList.at(attrIdx);
            for (float val : trajectory.attributes.at(attrIdx)) {
                attributeList.push_back(val);
            }
        }
    }
    multiVarTransferFunctionWindow.setAttributesValues(attributeNames, attributesList);

    recomputeColorLegend();
}

void LineDataFlow::recomputeColorLegend() {
    if (useMultiVarRendering) {
        for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
            std::vector<sgl::Color16> transferFunctionColorMap =
                    multiVarTransferFunctionWindow.getTransferFunctionMap_sRGB(int(i));
            colorLegendWidgets.at(i).setTransferFunctionColorMap(transferFunctionColorMap);
        }
    } else {
        LineData::recomputeColorLegend();
    }
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

size_t LineDataFlow::getBaseSizeInBytes() {
    size_t baseSizeInBytes = 0;
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        const Trajectory& trajectory = trajectories.at(trajectoryIdx);
        baseSizeInBytes += trajectory.positions.size() * sizeof(glm::vec3);
        for (size_t attributeIdx = 0; attributeIdx < trajectory.attributes.size(); attributeIdx++) {
            baseSizeInBytes += trajectory.attributes.at(attributeIdx).size() * sizeof(float);
        }
    }
    return baseSizeInBytes;
}


void LineDataFlow::iterateOverTrajectories(std::function<void(const Trajectory&)> callback) {
    for (const Trajectory& trajectory : trajectories) {
        callback(trajectory);
    }
}

void LineDataFlow::iterateOverTrajectoriesNotFiltered(std::function<void(const Trajectory&)> callback) {
    size_t trajectoryIndex = 0;
    for (const Trajectory& trajectory : trajectories) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIndex)) {
            trajectoryIndex++;
            continue;
        }
        callback(trajectory);
        trajectoryIndex++;
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

std::vector<std::vector<glm::vec3>> LineDataFlow::getFilteredLines(LineRenderer* lineRenderer) {
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


// --- Retrieve data for rendering. Preferred way. ---
void LineDataFlow::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    LineData::setGraphicsPipelineInfo(pipelineInfo, shaderStages);

    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER) {
        pipelineInfo.setVertexBufferBindingByLocationIndex("vertexOffsetLeft", sizeof(glm::vec3));
        pipelineInfo.setVertexBufferBindingByLocationIndex("vertexOffsetRight", sizeof(glm::vec3));
    } else if (getLinePrimitiveModeUsesSingleVertexShaderInputs(linePrimitiveMode)) {
        if (useRotatingHelicityBands) {
            pipelineInfo.setVertexBufferBindingByLocationIndex("vertexRotation", sizeof(float));
        }
    }
}

void LineDataFlow::setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData) {
    LineData::setVulkanRenderDataDescriptors(renderData);

    if (useMultiVarRendering) {
        renderData->setStaticBufferOptional(
                multiVarUniformDataBuffer, "MultiVarUniformDataBuffer");
        renderData->setStaticBufferOptional(
                multiVarSelectedAttributesBuffer, "AttributeSelectionMapBuffer");
    }

    if (useMultiVarRendering
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
}

void LineDataFlow::updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer) {
    if (useMultiVarRendering || useRotatingHelicityBands) {
        lineUniformData.numSubdivisionsBands = numSubdivisionsBands;
        lineUniformData.separatorBaseWidth = separatorWidth;
    }

    LineData::updateVulkanUniformBuffers(lineRenderer, renderer);

    if (useMultiVarRendering) {
        multiVarUniformData.numSelectedAttributes = uint32_t(selectedAttributes.size());
        multiVarUniformData.totalNumAttributes = uint32_t(attributeNames.size());
        multiVarUniformDataBuffer->updateData(
                sizeof(MultiVarUniformData), &multiVarUniformData,
                renderer->getVkCommandBuffer());
        if (!selectedAttributes.empty()) {
            multiVarSelectedAttributesBuffer->updateData(
                    sizeof(uint32_t) * selectedAttributes.size(), selectedAttributes.data(),
                    renderer->getVkCommandBuffer());
        }
    }
}

void LineDataFlow::setRasterDataBindings(sgl::vk::RasterDataPtr& rasterData) {
    setVulkanRenderDataDescriptors(rasterData);

    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER) {
        LinePassQuadsRenderData tubeRenderData = this->getLinePassQuadsRenderData();
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setVertexBuffer(tubeRenderData.vertexPositionBuffer, "vertexPosition");
        rasterData->setVertexBuffer(tubeRenderData.vertexAttributeBuffer, "vertexAttribute");
        rasterData->setVertexBufferOptional(tubeRenderData.vertexNormalBuffer, "vertexNormal");
        rasterData->setVertexBuffer(tubeRenderData.vertexTangentBuffer, "vertexTangent");
        rasterData->setVertexBuffer(tubeRenderData.vertexOffsetLeftBuffer, "vertexOffsetLeft");
        rasterData->setVertexBuffer(tubeRenderData.vertexOffsetRightBuffer, "vertexOffsetRight");
    } else if (getLinePrimitiveModeUsesSingleVertexShaderInputs(linePrimitiveMode)) {
        LinePassTubeRenderData tubeRenderData = this->getLinePassTubeRenderData();
        if (!tubeRenderData.indexBuffer) {
            return;
        }
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setVertexBuffer(tubeRenderData.vertexPositionBuffer, "vertexPosition");
        rasterData->setVertexBuffer(tubeRenderData.vertexAttributeBuffer, "vertexAttribute");
        rasterData->setVertexBufferOptional(tubeRenderData.vertexNormalBuffer, "vertexNormal");
        rasterData->setVertexBuffer(tubeRenderData.vertexTangentBuffer, "vertexTangent");
        if (useRotatingHelicityBands) {
            rasterData->setVertexBuffer(tubeRenderData.vertexRotationBuffer, "vertexRotation");
            if (useUniformTwistLineWidth) {
                rasterData->setStaticBuffer(tubeRenderData.vertexPositionBuffer, "LinePositionsBuffer");
                rasterData->setStaticBuffer(tubeRenderData.vertexRotationBuffer, "LineRotationsBuffer");
            }
        }
        if (useMultiVarRendering) {
            rasterData->setStaticBuffer(
                    tubeRenderData.multiVarAttributeDataBuffer, "AttributeDataArrayBuffer");
        }
    } else {
        LineData::setRasterDataBindings(rasterData);
    }
}



// --- Retrieve data for rendering. ---

LinePassQuadsRenderDataProgrammablePull LineDataFlow::getLinePassQuadsRenderDataProgrammablePull() {
    rebuildInternalRepresentationIfNecessary();
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    LinePassQuadsRenderDataProgrammablePull tubeRenderData;

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
        linePointData.at(i).principalStressIndex = 0;
    }

    tubeRenderData.linePointsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, linePointData.size() * sizeof(LinePassQuadsLinePointDataProgrammablePull), linePointData.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

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

    if (lineIndices.empty()) {
        return {};
    }

    TubeRenderDataOpacityOptimization tubeRenderData;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

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

    return tubeRenderData;
}

void LineDataFlow::getLinePassTubeRenderDataGeneral(
        const std::function<uint32_t()>& indexOffsetFunctor,
        const std::function<void(
                const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent, float lineAttribute,
                float lineRotation, uint32_t indexOffset, size_t lineIdx, size_t pointIdx)>& pointPushFunctor,
        const std::function<void()>& pointPopFunctor,
        const std::function<void(int numSegments, uint32_t indexOffset)>& indicesPushFunctor) {
    std::vector<std::vector<glm::vec3>> lineCentersList;
    std::vector<std::vector<float>> lineAttributesList;
    std::vector<std::vector<float>> lineRotationsList; //< Used if useRotatingHelicityBands is set to true.

    lineCentersList.resize(trajectories.size());
    lineAttributesList.resize(trajectories.size());
    if (!useRotatingHelicityBands && getUseBandRendering() && useRibbons && hasBandsData) {
        std::vector<std::vector<glm::vec3>> ribbonDirectionsList;

        ribbonDirectionsList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }

            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            std::vector<glm::vec3>& ribbonDirections = ribbonsDirections.at(trajectoryIdx);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            std::vector<glm::vec3>& ribbonDirectionVectors = ribbonDirectionsList.at(trajectoryIdx);

            for (size_t j = 0; j < trajectory.positions.size(); j++) {
                lineCenters.push_back(trajectory.positions.at(j));
                lineAttributes.push_back(attributes.at(j));
                ribbonDirectionVectors.push_back(ribbonDirections.at(j));
            }
        }

        for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
            const std::vector<glm::vec3>& lineCenters = lineCentersList.at(lineId);
            const std::vector<float>& lineAttributes = lineAttributesList.at(lineId);
            const std::vector<glm::vec3>& ribbonDirectionVectors = ribbonDirectionsList.at(lineId);
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

                normal = glm::cross(ribbonDirectionVectors.at(i), tangent);

                pointPushFunctor(
                        lineCenters.at(i), normal, tangent, lineAttributes.at(i),
                        0.0f, indexOffset, lineId, i);
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
    } else if (useRotatingHelicityBands) {
        lineRotationsList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }

            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            std::vector<float>& helicities = trajectory.attributes.at(helicityAttributeIndex);
            assert(attributes.size() == trajectory.positions.size());
            std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
            std::vector<float>& lineAttributes = lineAttributesList.at(trajectoryIdx);
            std::vector<float>& lineRotations = lineRotationsList.at(trajectoryIdx);

            float rotation = 0.0f;
            for (size_t j = 0; j < trajectory.positions.size(); j++) {
                lineCenters.push_back(trajectory.positions.at(j));
                lineAttributes.push_back(attributes.at(j));
                lineRotations.push_back(rotation);
                float lineSegmentLength = 0.0f;
                if (j < trajectory.positions.size() - 1) {
                    lineSegmentLength = glm::length(trajectory.positions.at(j + 1) - trajectory.positions.at(j));
                }
                rotation += helicities.at(j) / maxHelicity * sgl::PI * helicityRotationFactor * lineSegmentLength / 0.005f;
            }
        }

        for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
            const std::vector<glm::vec3>& lineCenters = lineCentersList.at(lineId);
            const std::vector<float>& lineAttributes = lineAttributesList.at(lineId);
            const std::vector<float>& lineRotations = lineRotationsList.at(lineId);
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

                pointPushFunctor(
                        lineCenters.at(i), normal, tangent, lineAttributes.at(i),
                        lineRotations.at(i), indexOffset, lineId, i);
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

        for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
            const std::vector<glm::vec3>& lineCenters = lineCentersList.at(lineId);
            const std::vector<float>& lineAttributes = lineAttributesList.at(lineId);
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

                pointPushFunctor(
                        lineCenters.at(i), normal, tangent, lineAttributes.at(i),
                        0.0f, indexOffset, lineId, i);
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

LinePassTubeRenderData LineDataFlow::getLinePassTubeRenderData() {
    rebuildInternalRepresentationIfNecessary();

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<float> vertexAttributes;
    std::vector<float> vertexRotations; //< Used if useRotatingHelicityBands is set to true.
    std::vector<float> multiVarAttributeData;

    getLinePassTubeRenderDataGeneral(
            [&]() {
                return uint32_t(vertexPositions.size());
            },
            [&](const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent,
                    float lineAttribute, float lineRotation, uint32_t indexOffset,
                    size_t lineIdx, size_t pointIdx) {
                vertexPositions.push_back(lineCenter);
                vertexNormals.push_back(normal);
                vertexTangents.push_back(tangent);
                vertexAttributes.push_back(lineAttribute);
                vertexRotations.push_back(lineRotation);

                if (useMultiVarRendering) {
                    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                        multiVarAttributeData.push_back(
                                trajectories.at(lineIdx).attributes.at(attrIdx).at(pointIdx));
                    }
                }
            },
            [&] {
                vertexPositions.pop_back();
                vertexNormals.pop_back();
                vertexTangents.pop_back();
                vertexAttributes.pop_back();
                vertexRotations.pop_back();

                if (useMultiVarRendering) {
                    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                        multiVarAttributeData.pop_back();
                    }
                }
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

    VkBufferUsageFlags flags = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    if (useRotatingHelicityBands && useUniformTwistLineWidth) {
        flags |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    }

    // Add the index buffer.
    tubeRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, lineIndices.size() * sizeof(uint32_t), lineIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexPositions.size() * sizeof(glm::vec3), vertexPositions.data(),
            flags, VMA_MEMORY_USAGE_GPU_ONLY);

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

    if (useRotatingHelicityBands) {
        tubeRenderData.vertexRotationBuffer = std::make_shared<sgl::vk::Buffer>(
                device, vertexRotations.size() * sizeof(float), vertexRotations.data(),
                flags, VMA_MEMORY_USAGE_GPU_ONLY);
    }

    if (useMultiVarRendering) {
        tubeRenderData.multiVarAttributeDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, multiVarAttributeData.size() * sizeof(float), multiVarAttributeData.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    return tubeRenderData;
}

LinePassTubeRenderDataMeshShader LineDataFlow::getLinePassTubeRenderDataMeshShader() {
    rebuildInternalRepresentationIfNecessary();

    // We can emit a maximum of 64 vertices/primitives from the mesh shader.
    int numLineSegmentsPerMeshlet = 64 / tubeNumSubdivisions - 1;
    std::vector<MeshletData> meshlets;
    MeshletData meshlet{};
    std::vector<LinePointDataUnified> linePoints;
    std::vector<float> multiVarAttributeData;

    getLinePassTubeRenderDataGeneral(
            [&]() {
                return uint32_t(linePoints.size());
            },
            [&](const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent,
                    float lineAttribute, float lineRotation, uint32_t indexOffset,
                    size_t lineIdx, size_t pointIdx) {
                LinePointDataUnified linePointData;
                linePointData.linePosition = lineCenter;
                linePointData.lineNormal = normal;
                linePointData.lineTangent = tangent;
                linePointData.lineAttribute = lineAttribute;
                linePointData.lineRotation = lineRotation;
                linePointData.lineStartIndex = indexOffset;
                linePoints.push_back(linePointData);

                if (useMultiVarRendering) {
                    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                        multiVarAttributeData.push_back(
                                trajectories.at(lineIdx).attributes.at(attrIdx).at(pointIdx));
                    }
                }
            },
            [&] {
                linePoints.pop_back();

                if (useMultiVarRendering) {
                    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                        multiVarAttributeData.pop_back();
                    }
                }
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

    if (useMultiVarRendering) {
        renderData.multiVarAttributeDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, multiVarAttributeData.size() * sizeof(float), multiVarAttributeData.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    return renderData;
}

LinePassTubeRenderDataProgrammablePull LineDataFlow::getLinePassTubeRenderDataProgrammablePull() {
    rebuildInternalRepresentationIfNecessary();

    std::vector<uint32_t> triangleIndices;
    std::vector<LinePointDataUnified> linePoints;
    std::vector<float> multiVarAttributeData;

    getLinePassTubeRenderDataGeneral(
            [&]() {
                return uint32_t(linePoints.size());
            },
            [&](const glm::vec3& lineCenter, const glm::vec3& normal, const glm::vec3& tangent,
                    float lineAttribute, float lineRotation, uint32_t indexOffset,
                    size_t lineIdx, size_t pointIdx) {
                LinePointDataUnified linePointData;
                linePointData.linePosition = lineCenter;
                linePointData.lineNormal = normal;
                linePointData.lineTangent = tangent;
                linePointData.lineAttribute = lineAttribute;
                linePointData.lineRotation = lineRotation;
                linePointData.lineStartIndex = indexOffset;
                linePoints.push_back(linePointData);

                if (useMultiVarRendering) {
                    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                        multiVarAttributeData.push_back(
                                trajectories.at(lineIdx).attributes.at(attrIdx).at(pointIdx));
                    }
                }
            },
            [&] {
                linePoints.pop_back();

                if (useMultiVarRendering) {
                    for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                        multiVarAttributeData.pop_back();
                    }
                }
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
    LinePassTubeRenderDataProgrammablePull renderData{};

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

    if (useMultiVarRendering) {
        renderData.multiVarAttributeDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, multiVarAttributeData.size() * sizeof(float), multiVarAttributeData.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    return renderData;
}

LinePassQuadsRenderData LineDataFlow::getLinePassQuadsRenderData() {
    rebuildInternalRepresentationIfNecessary();
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    LinePassQuadsRenderData bandRenderData;

    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<float> vertexAttributes;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<glm::vec3> vertexOffsetsLeft;
    std::vector<glm::vec3> vertexOffsetsRight;

    if (useRibbons && hasBandsData) {
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }

            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);
            std::vector<glm::vec3>& ribbonDirections = ribbonsDirections.at(trajectoryIdx);
            assert(attributes.size() == trajectory.positions.size());
            assert(attributes.size() == ribbonDirections.size());

            if (trajectory.positions.size() < 2) {
                continue;
            }

            auto indexStart = uint32_t(vertexPositions.size());
            int n = int(trajectory.positions.size());
            int numValidLinePoints = 0;
            for (int i = 0; i < int(trajectory.positions.size()); i++) {
                auto ribbonDirection = ribbonDirections.at(i);
                vertexPositions.push_back(trajectory.positions.at(i));
                vertexOffsetsLeft.push_back(-ribbonDirection);
                vertexOffsetsRight.push_back(ribbonDirection);

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

                glm::vec3 vertexNormal = glm::normalize(glm::cross(vertexTangent, ribbonDirection));
                vertexNormals.push_back(vertexNormal);

                vertexAttributes.push_back(attributes.at(i));
                numValidLinePoints++;
            }

            if (numValidLinePoints == 1) {
                // Only one vertex left -> Output nothing (tube consisting only of one point).
                vertexPositions.pop_back();
                vertexOffsetsLeft.pop_back();
                vertexOffsetsRight.pop_back();
                vertexNormals.pop_back();
                vertexTangents.pop_back();
                vertexAttributes.pop_back();
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
        size_t numVerticesAdded = vertexPositions.size() - numVerticesOld;
        for (size_t i = 0; i < numVerticesAdded; i++) {
            vertexOffsetsLeft.emplace_back(0.0f);
            vertexOffsetsRight.emplace_back(0.0f);
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

    return bandRenderData;
}


TubeTriangleRenderData LineDataFlow::getLinePassTubeTriangleMeshRenderData(bool isRasterizer, bool vulkanRayTracing) {
    rebuildInternalRepresentationIfNecessary();
    if (vulkanTubeTriangleRenderData.vertexBuffer && vulkanTubeTriangleRenderDataIsRayTracing == vulkanRayTracing) {
        return vulkanTubeTriangleRenderData;
    }

    std::vector<std::vector<glm::vec3>> lineCentersList;

    std::vector<uint32_t> tubeTriangleIndices;
    std::vector<glm::vec3> lineTangents;
    std::vector<glm::vec3> lineNormals;
    std::vector<TubeTriangleVertexData> tubeTriangleVertexDataList;
    std::vector<LinePointReference> linePointReferences;
    std::vector<LinePointDataUnified> tubeTriangleLinePointDataList;
    std::vector<float> multiVarAttributeData;

    lineCentersList.resize(trajectories.size());
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }
        lineCentersList.at(trajectoryIdx) = trajectories.at(trajectoryIdx).positions;
    }

    if (getUseBandRendering() && useRibbons && hasBandsData) {
        std::vector<std::vector<glm::vec3>> ribbonDirectionsList;
        ribbonDirectionsList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }
            ribbonDirectionsList.at(trajectoryIdx) = ribbonsDirections.at(trajectoryIdx);
        }

        float binormalRadius = LineRenderer::getBandWidth() * 0.5f;
        float normalRadius = binormalRadius * minBandThickness;
        if (useCappedTubes) {
            createCappedTriangleEllipticTubesRenderDataCPU(
                    lineCentersList, ribbonDirectionsList, normalRadius,
                    binormalRadius, tubeNumSubdivisions,
                    false, tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        } else {
            createTriangleEllipticTubesRenderDataCPU(
                    lineCentersList, ribbonDirectionsList, normalRadius,
                    binormalRadius, tubeNumSubdivisions,
                    tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        }
    } else {
        if (useCappedTubes) {
            createCappedTriangleTubesRenderDataCPU(
                    lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                    tubeNumSubdivisions, false,
                    tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        } else {
            createTriangleTubesRenderDataCPU(
                    lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                    tubeNumSubdivisions,
                    tubeTriangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        }
    }

    tubeTriangleLinePointDataList.resize(linePointReferences.size());
    if (useMultiVarRendering) {
        multiVarAttributeData.reserve(attributeNames.size() * linePointReferences.size());
    }
    float rotation = 0.0f; //< Used if useRotatingHelicityBands is set to true.
    uint32_t lineStartIndex = 0;
    uint32_t lastTrajectoryIndex = 0;
    for (size_t i = 0; i < linePointReferences.size(); i++) {
        LinePointReference& linePointReference = linePointReferences.at(i);
        LinePointDataUnified& tubeTriangleLinePointData = tubeTriangleLinePointDataList.at(i);
        Trajectory& trajectory = trajectories.at(linePointReference.trajectoryIndex);
        std::vector<float>& attributes = trajectory.attributes.at(selectedAttributeIndex);

        tubeTriangleLinePointData.linePosition = trajectory.positions.at(linePointReference.linePointIndex);
        tubeTriangleLinePointData.lineAttribute = attributes.at(linePointReference.linePointIndex);
        tubeTriangleLinePointData.lineTangent = lineTangents.at(i);
        tubeTriangleLinePointData.lineNormal = lineNormals.at(i);

        if (lastTrajectoryIndex != linePointReference.trajectoryIndex) {
            lastTrajectoryIndex = linePointReference.trajectoryIndex;
            lineStartIndex = uint32_t(i);
        }
        tubeTriangleLinePointData.lineStartIndex = lineStartIndex;

        if (useRotatingHelicityBands) {
            tubeTriangleLinePointData.lineRotation = rotation;
            float helicity = trajectory.attributes.at(helicityAttributeIndex).at(
                    linePointReference.linePointIndex);
            float lineSegmentLength = 0.0f;
            if (i < linePointReferences.size() - 1) {
                LinePointReference& nextLinePointReference = linePointReferences.at(i + 1);
                if (linePointReference.trajectoryIndex == nextLinePointReference.trajectoryIndex) {
                    lineSegmentLength = glm::length(
                            trajectory.positions.at(nextLinePointReference.linePointIndex)
                            - trajectory.positions.at(linePointReference.linePointIndex));
                }
            }
            rotation += helicity / maxHelicity * sgl::PI * helicityRotationFactor * lineSegmentLength / 0.005f;
        }

        if (useMultiVarRendering) {
            for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                multiVarAttributeData.push_back(
                        trajectory.attributes.at(attrIdx).at(linePointReference.linePointIndex));
            }
        }
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
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (useMultiVarRendering) {
        vulkanTubeTriangleRenderData.multiVarAttributeDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, multiVarAttributeData.size() * sizeof(float), multiVarAttributeData.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

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

TubeAabbRenderData LineDataFlow::getLinePassTubeAabbRenderData(bool isRasterizer) {
    rebuildInternalRepresentationIfNecessary();
    if (vulkanTubeAabbRenderData.indexBuffer) {
        return vulkanTubeAabbRenderData;
    }

    glm::vec3 lineWidthOffset(LineRenderer::getLineWidth() * 0.5f);

    std::vector<uint32_t> lineSegmentPointIndices;
    std::vector<sgl::AABB3> lineSegmentAabbs;
    std::vector<LinePointDataUnified> tubeLinePointDataList;
    std::vector<float> multiVarAttributeData;
    lineSegmentPointIndices.reserve(getNumLineSegments() * 2);
    lineSegmentAabbs.reserve(getNumLineSegments());
    tubeLinePointDataList.reserve(getNumLinePoints());
    if (useMultiVarRendering) {
        multiVarAttributeData.reserve(attributeNames.size() * getNumLinePoints());
    }

    uint32_t lineSegmentIndexCounter = 0;
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
            continue;
        }
        Trajectory& trajectory = trajectories.at(trajectoryIdx);

        glm::vec3 lastLineNormal(1.0f, 0.0f, 0.0f);
        uint32_t numValidLinePoints = 0;
        float rotation = 0.0f; //< Used if useRotatingHelicityBands is set to true.
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            glm::vec3 tangent;
            if (i == 0) {
                tangent = trajectory.positions[i + 1] - trajectory.positions[i];
            } else if (i + 1 == trajectory.positions.size()) {
                tangent = trajectory.positions[i] - trajectory.positions[i - 1];
            } else {
                tangent = trajectory.positions[i + 1] - trajectory.positions[i - 1];
            }
            float tangentLength = glm::length(tangent);

            if (tangentLength < 0.0001f) {
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
            if (useRotatingHelicityBands) {
                linePointData.lineRotation = rotation;
                float helicity = trajectory.attributes.at(helicityAttributeIndex).at(i);

                float lineSegmentLength = 0.0f;
                if (i < trajectory.positions.size() - 1) {
                    lineSegmentLength = glm::length(trajectory.positions.at(i + 1) - trajectory.positions.at(i));
                }
                rotation += helicity / maxHelicity * sgl::PI * helicityRotationFactor * lineSegmentLength / 0.005f;
            }
            tubeLinePointDataList.push_back(linePointData);

            if (useMultiVarRendering) {
                for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                    multiVarAttributeData.push_back(trajectory.attributes.at(attrIdx).at(i));
                }
            }

            numValidLinePoints++;
        }

        if (numValidLinePoints == 1) {
            // Only one vertex left -> output nothing (tube consisting only of one point).
            tubeLinePointDataList.pop_back();

            if (useMultiVarRendering) {
                for (size_t attrIdx = 0; attrIdx < attributeNames.size(); attrIdx++) {
                    multiVarAttributeData.pop_back();
                }
            }
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
        lineSegmentIndexCounter += uint32_t(numValidLinePoints);
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

    if (useMultiVarRendering) {
        vulkanTubeTriangleRenderData.multiVarAttributeDataBuffer = std::make_shared<sgl::vk::Buffer>(
                device, multiVarAttributeData.size() * sizeof(float), multiVarAttributeData.data(),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    return vulkanTubeAabbRenderData;
}

void LineDataFlow::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string>& preprocessorDefines, bool isRasterizer) {
    LineData::getVulkanShaderPreprocessorDefines(preprocessorDefines, isRasterizer);
    if (useRibbons && hasBandsData && !useRotatingHelicityBands && (!isRasterizer || getUseBandRendering())) {
        preprocessorDefines.insert(std::make_pair("USE_BANDS", ""));
        if (renderThickBands) {
            preprocessorDefines.insert(std::make_pair("BAND_RENDERING_THICK", ""));
            preprocessorDefines.insert(std::make_pair("MIN_THICKNESS", std::to_string(minBandThickness)));
        } else {
            preprocessorDefines.insert(std::make_pair("MIN_THICKNESS", std::to_string(1e-2f)));
        }
    }
    if (useRotatingHelicityBands) {
        preprocessorDefines.insert(std::make_pair("USE_ROTATING_HELICITY_BANDS", ""));
        if (useUniformTwistLineWidth) {
            preprocessorDefines.insert(std::make_pair("UNIFORM_HELICITY_BAND_WIDTH", ""));
        }
    }
    bool isQuadsMode =
            linePrimitiveMode == LINE_PRIMITIVES_QUADS_PROGRAMMABLE_PULL
            || linePrimitiveMode == LINE_PRIMITIVES_QUADS_GEOMETRY_SHADER
            || linePrimitiveMode == LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER;
    if (useMultiVarRendering && (!isRasterizer || !isQuadsMode)) {
        preprocessorDefines.insert(std::make_pair("USE_MULTI_VAR_RENDERING", ""));
        preprocessorDefines.insert(std::make_pair("USE_MULTI_VAR_TRANSFER_FUNCTION", ""));
    }
}

void LineDataFlow::getTriangleMesh(
        LineRenderer* lineRenderer,
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

    if (getUseBandRendering() && useRibbons && hasBandsData) {
        std::vector<std::vector<glm::vec3>> ribbonDirectionsList;
        ribbonDirectionsList.resize(trajectories.size());
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            if (!filteredTrajectories.empty() && filteredTrajectories.at(trajectoryIdx)) {
                continue;
            }
            ribbonDirectionsList.at(trajectoryIdx) = ribbonsDirections.at(trajectoryIdx);
        }

        float binormalRadius = LineRenderer::getBandWidth() * 0.5f;
        float normalRadius = binormalRadius * minBandThickness;
        if (useCappedTubes) {
            createCappedTriangleEllipticTubesRenderDataCPU(
                    lineCentersList, ribbonDirectionsList, normalRadius,
                    binormalRadius, tubeNumSubdivisions,
                    false, triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        } else {
            createTriangleEllipticTubesRenderDataCPU(
                    lineCentersList, ribbonDirectionsList, normalRadius,
                    binormalRadius, tubeNumSubdivisions,
                    triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        }
    } else {
        if (useCappedTubes) {
            createCappedTriangleTubesRenderDataCPU(
                    lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                    tubeNumSubdivisions, false,
                    triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        } else {
            createTriangleTubesRenderDataCPU(
                    lineCentersList, LineRenderer::getLineWidth() * 0.5f,
                    tubeNumSubdivisions,
                    triangleIndices, tubeTriangleVertexDataList, linePointReferences,
                    0, lineTangents, lineNormals);
        }
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
        LineRenderer* lineRenderer,
        std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions) {
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    getTriangleMesh(lineRenderer, triangleIndices, vertexPositions, vertexNormals, vertexAttributes);
}
