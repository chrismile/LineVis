/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <tracy/Tracy.hpp>

#include <Utils/AppSettings.hpp>
#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include "Utils/IndexMesh.hpp"
#include "Utils/MeshSmoothing.hpp"
#include "Utils/TriangleNormals.hpp"
#include "../LineDataFlow.hpp"
#include "StreamlineSeeder.hpp"
#include "StreamlineTracingGrid.hpp"
#include "Loader/StructuredGridVtkLoader.hpp"
#include "Loader/VtkXmlLoader.hpp"
#include "Loader/NetCdfLoader.hpp"
#include "Loader/AmiraMeshLoader.hpp"
#include "Loader/RbcBinFileLoader.hpp"
#include "Loader/FieldFileLoader.hpp"
#include "Loader/DatRawFileLoader.hpp"
#ifdef USE_ECCODES
#include "Loader/GribLoader.hpp"
#endif
#include "Loaders/BinLinesLoader.hpp"
#include "StreamlineTracingRequester.hpp"

StreamlineTracingRequester::StreamlineTracingRequester(sgl::TransferFunctionWindow& transferFunctionWindow)
        : transferFunctionWindow(transferFunctionWindow) {
    streamlineSeeders.insert(std::make_pair(
            StreamlineSeedingStrategy::VOLUME, StreamlineSeederPtr(new StreamlineVolumeSeeder)));
    streamlineSeeders.insert(std::make_pair(
            StreamlineSeedingStrategy::PLANE, StreamlineSeederPtr(new StreamlinePlaneSeeder)));
    streamlineSeeders.insert(std::make_pair(
            StreamlineSeedingStrategy::MAX_HELICITY_FIRST, StreamlineSeederPtr(
                    new StreamlineMaxHelicityFirstSeeder)));
    guiTracingSettings.seeder = streamlineSeeders[guiTracingSettings.streamlineSeedingStrategy];

    lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
    loadGridDataSetList();
    requesterThread = std::thread(&StreamlineTracingRequester::mainLoop, this);
}

StreamlineTracingRequester::~StreamlineTracingRequester() {
    join();

    if (cachedGrid) {
        delete cachedGrid;
        cachedGrid = nullptr;
    }

    requestLineData = {};
    replyLineData = {};
}

void StreamlineTracingRequester::loadGridDataSetList() {
    gridDataSetNames.clear();
    gridDataSetFilenames.clear();
    gridDataSetNames.emplace_back("Local file...");
    gridDataSetNames.emplace_back("Arnold-Beltrami-Childress flow generator");

    std::string filename = lineDataSetsDirectory + "flow_grids.json";
    if (sgl::FileUtils::get()->exists(filename)) {
        // Parse the passed JSON file.
        std::ifstream jsonFileStream(filename.c_str());
        Json::CharReaderBuilder builder;
        JSONCPP_STRING errorString;
        Json::Value root;
        if (!parseFromStream(builder, jsonFileStream, &root, &errorString)) {
            sgl::Logfile::get()->writeError(errorString);
            return;
        }
        jsonFileStream.close();

        Json::Value sources = root["grids"];
        for (Json::Value::const_iterator sourceIt = sources.begin(); sourceIt != sources.end(); ++sourceIt) {
            DataSetInformation dataSetInformation;
            Json::Value source = *sourceIt;

            gridDataSetNames.push_back(source["name"].asString());
            gridDataSetFilenames.push_back(source["filename"].asString());

            GridDataSetMetaData gridDataSetMetaData;
            if (source.isMember("time")) {
                auto timeElement = source["time"];
                if (timeElement.isString()) {
                    std::string timeString = timeElement.asString();
                    std::vector<std::string> timeStringVector;
                    sgl::splitStringWhitespace(timeString, timeStringVector);
                    if (timeStringVector.size() == 1) {
                        gridDataSetMetaData.time = sgl::fromString<int>(timeStringVector.at(0));
                    } else if (timeStringVector.size() == 2) {
                        std::string dateStringRaw;
                        for (char c : timeStringVector.at(0)) {
                            if (int(c) >= int('0') && int(c) <= '9') {
                                dateStringRaw += c;
                            }
                        }
                        std::string timeStringRaw;
                        for (char c : timeStringVector.at(1)) {
                            if (int(c) >= int('0') && int(c) <= '9') {
                                timeStringRaw += c;
                            }
                        }
                        gridDataSetMetaData.date = sgl::fromString<int>(dateStringRaw);
                        gridDataSetMetaData.time = sgl::fromString<int>(timeStringRaw);
                    }
                } else {
                    gridDataSetMetaData.time = timeElement.asInt();
                }
            }
            if (source.isMember("data_date") && source.isMember("data_time")) {
                auto dataDateElement = source["data_date"];
                auto dataTimeElement = source["data_time"];
                gridDataSetMetaData.date = dataDateElement.asInt();
                gridDataSetMetaData.time = dataTimeElement.asInt();
            }
            if (source.isMember("scale")) {
                auto scaleElement = source["scale"];
                if (scaleElement.isDouble()) {
                    gridDataSetMetaData.scale = glm::vec3(scaleElement.asFloat());
                } else if (scaleElement.isArray()) {
                    int dim = 0;
                    for (const auto& scaleDimElement : scaleElement) {
                        gridDataSetMetaData.scale[dim] = scaleDimElement.asFloat();
                        dim++;
                    }
                }
            }
            if (source.isMember("axes")) {
                auto axesElement = source["axes"];
                int dim = 0;
                for (const auto& axisElement : axesElement) {
                    gridDataSetMetaData.axes[dim] = axisElement.asInt();
                    dim++;
                }
            }
            if (source.isMember("subsampling_factor")) {
                gridDataSetMetaData.subsamplingFactor = source["subsampling_factor"].asInt();
                gridDataSetMetaData.susamplingFactorSet = true;
            }
            if (source.isMember("velocity_field_name")) {
                gridDataSetMetaData.velocityFieldName = source["velocity_field_name"].asString();
            }
            gridDataSetsMetaData.push_back(gridDataSetMetaData);
        }
    }
}

void StreamlineTracingRequester::renderGui() {
    if (!showWindow) {
        return;
    }

    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSizeLocation(
            sgl::LOCATION_RIGHT | sgl::LOCATION_BOTTOM, 22, 22, 780, 1190);
    if (ImGui::Begin("Streamline Tracer", &showWindow)) {
        {
            std::lock_guard<std::mutex> replyLock(gridInfoMutex);
            if (newGridLoaded) {
                newGridLoaded = false;
                for (auto& it : streamlineSeeders) {
                    it.second->setNewGridBox(gridBox);
                }
            }
        }

        bool changed = false;
        if (ImGui::Combo(
                "Data Set", &selectedGridDataSetIndex, gridDataSetNames.data(),
                int(gridDataSetNames.size()))) {
            if (selectedGridDataSetIndex == 1) {
                guiTracingSettings.exportPath = lineDataSetsDirectory + "abc_flow.binlines";
                changed = true;
            }
            if (selectedGridDataSetIndex >= 2) {
                const std::string pathString = gridDataSetFilenames.at(selectedGridDataSetIndex - 2);
                bool isAbsolutePath = sgl::FileUtils::get()->getIsPathAbsolute(pathString);
                if (isAbsolutePath) {
                    gridDataSetFilename = pathString;
                } else {
                    gridDataSetFilename = lineDataSetsDirectory + pathString;
                }
                const auto& metaData = gridDataSetsMetaData.at(selectedGridDataSetIndex - 2);
                guiTracingSettings.gridSubsamplingFactor = metaData.subsamplingFactor;
                guiTracingSettings.exportPath =
                        sgl::FileUtils::get()->removeExtension(gridDataSetFilename) + ".binlines";
                changed = true;
            }
        }
        if (selectedGridDataSetIndex == 0) {
            ImGui::InputText("##griddatasetfilenamelabel", &gridDataSetFilename);
            ImGui::SameLine();
            if (ImGui::Button("Load File")) {
                changed = true;
            }
        }
        if (selectedGridDataSetIndex == 1 && abcFlowGenerator.renderGui()) {
            changed = true;
        }

        if (ImGui::Combo(
                "Primitive Type", (int*)&guiTracingSettings.flowPrimitives,
                FLOW_PRIMITIVE_NAMES, IM_ARRAYSIZE(FLOW_PRIMITIVE_NAMES))) {
            changed = true;
        }

        if (!guiTracingSettings.seeder->getIsRegular()) {
            if (ImGui::SliderIntPowerOfTwoEdit(
                    "#Primitives", &guiTracingSettings.numPrimitives,
                    1, 1048576) == ImGui::EditMode::INPUT_FINISHED) {
                changed = true;
            }
        }

        if (ImGui::Combo(
                "Seeding Strategy", (int*)&guiTracingSettings.streamlineSeedingStrategy,
                STREAMLINE_SEEDING_STRATEGY_NAMES, IM_ARRAYSIZE(STREAMLINE_SEEDING_STRATEGY_NAMES))) {
            guiTracingSettings.seeder = streamlineSeeders[guiTracingSettings.streamlineSeedingStrategy];
            changed = true;
        }

        if (guiTracingSettings.seeder->renderGui()) {
            changed = true;
        }

        if (ImGui::CollapsingHeader(
                "Advanced Settings", nullptr, ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::SliderFloatEdit(
                    "Time Step Scale", &guiTracingSettings.timeStepScale, 0.1f, 10.0f,
                    "%.1f", ImGuiSliderFlags_Logarithmic) == ImGui::EditMode::INPUT_FINISHED) {
                changed = true;
            }
            if (ImGui::SliderIntPowerOfTwoEdit(
                    "Grid Subsampling", &guiTracingSettings.gridSubsamplingFactor,
                    1, 8) == ImGui::EditMode::INPUT_FINISHED) {
                changed = true;
            }
            if (ImGui::SliderIntEdit(
                    "Max. #Iterations", &guiTracingSettings.maxNumIterations,
                    10, 10000) == ImGui::EditMode::INPUT_FINISHED) {
                changed = true;
            }
            if (ImGui::SliderFloatEdit(
                    "Termination Dist.", &guiTracingSettings.terminationDistance, 0.01f, 100.0f,
                    "%.2f", ImGuiSliderFlags_Logarithmic) == ImGui::EditMode::INPUT_FINISHED) {
                changed = true;
            }
            if ((guiTracingSettings.loopCheckMode == LoopCheckMode::START_POINT
                    || guiTracingSettings.loopCheckMode == LoopCheckMode::ALL_POINTS) && ImGui::SliderFloatEdit(
                    "Term. Dist. Self", &guiTracingSettings.terminationDistanceSelf, 0.01f, 100.0f,
                    "%.2f", ImGuiSliderFlags_Logarithmic) == ImGui::EditMode::INPUT_FINISHED) {
                changed = true;
            }
            if (ImGui::SliderFloatEdit(
                    "Min. Line Length", &guiTracingSettings.minimumLength, 0.001f, 2.0f,
                    "%.3f") == ImGui::EditMode::INPUT_FINISHED) {
                changed = true;
            }
            if (guiTracingSettings.streamlineSeedingStrategy == StreamlineSeedingStrategy::MAX_HELICITY_FIRST) {
                if (ImGui::SliderFloatEdit(
                        "Min. Separation Dist.", &guiTracingSettings.minimumSeparationDistance,
                        0.001f, 0.2f, "%.3f") == ImGui::EditMode::INPUT_FINISHED) {
                    changed = true;
                }
                if (ImGui::Combo(
                        "Distance Check", (int*)&guiTracingSettings.terminationCheckType,
                        TERMINATION_CHECK_TYPE_NAMES, IM_ARRAYSIZE(TERMINATION_CHECK_TYPE_NAMES))) {
                    changed = true;
                }
            }
            if (ImGui::Combo(
                    "Loop Check Mode", (int*)&guiTracingSettings.loopCheckMode,
                    LOOP_CHECK_MODE_NAMES, IM_ARRAYSIZE(LOOP_CHECK_MODE_NAMES))) {
                changed = true;
            }
            if (ImGui::Checkbox("Show Boundary Mesh", &guiTracingSettings.showSimulationGridOutline)) {
                changed = true;
            }
            if (guiTracingSettings.showSimulationGridOutline) {
                if (ImGui::Checkbox("Smooth Boundary", &guiTracingSettings.smoothedSimulationGridOutline)) {
                    std::lock_guard<std::mutex> lock(cachedGridMetadataMutex);
                    cachedSimulationMeshOutlineTriangleIndices.clear();
                    cachedSimulationMeshOutlineVertexPositions.clear();
                    cachedSimulationMeshOutlineVertexNormals.clear();
                    changed = true;
                }
            }
            if (ImGui::Combo(
                    "Integration Method", (int*)&guiTracingSettings.integrationMethod,
                    STREAMLINE_INTEGRATION_METHOD_NAMES,
                    IM_ARRAYSIZE(STREAMLINE_INTEGRATION_METHOD_NAMES))) {
                changed = true;
            }
            if (ImGui::Combo(
                    "Integration Direction", (int*)&guiTracingSettings.integrationDirection,
                    STREAMLINE_INTEGRATION_DIRECTION_NAMES,
                    IM_ARRAYSIZE(STREAMLINE_INTEGRATION_DIRECTION_NAMES))) {
                changed = true;
            }
            {
                std::lock_guard<std::mutex> lock(cachedGridMetadataMutex);
                if (cachedGridVectorFieldNames.size() > 1) {
                    if (ImGui::Combo(
                            "Vector Field", (int*)&guiTracingSettings.vectorFieldIndex,
                            cachedGridVectorFieldNames.data(),
                            int(cachedGridVectorFieldNames.size()))) {
                        changed = true;
                    }
                }
            }

            if (guiTracingSettings.flowPrimitives == FlowPrimitives::STREAMRIBBONS) {
                if (ImGui::Checkbox("Use Helicity Ribbons", &guiTracingSettings.useHelicity)) {
                    changed = true;
                }
                if (guiTracingSettings.useHelicity && ImGui::Checkbox(
                        "Normalized Velocity", &guiTracingSettings.useNormalizedVelocity)) {
                    changed = true;
                }
                ImGui::SameLine();
                if (guiTracingSettings.useHelicity && ImGui::Checkbox(
                        "Normalized Vorticity", &guiTracingSettings.useNormalizedVorticity)) {
                    changed = true;
                }
                if (guiTracingSettings.useHelicity && ImGui::SliderFloatEdit(
                        "Max. Helicity Twist", &guiTracingSettings.maxHelicityTwist,
                        0.0f, 1.0f) == ImGui::EditMode::INPUT_FINISHED) {
                    changed = true;
                }
                if (guiTracingSettings.useHelicity && ImGui::SliderFloat3Edit(
                        "Init. Ribbon Dir.", &guiTracingSettings.initialRibbonDirection.x,
                        0.0f, 1.0f) == ImGui::EditMode::INPUT_FINISHED) {
                    changed = true;
                }
                if (guiTracingSettings.useHelicity && ImGui::SliderIntPowerOfTwoEdit(
                        "Seeding Subsampling", &guiTracingSettings.seedingSubsamplingFactor,
                        1, 8) == ImGui::EditMode::INPUT_FINISHED) {
                    changed = true;
                }
            }

            if (ImGui::Checkbox("Export to Disk", &guiTracingSettings.exportToDisk)) {
                changed = true;
            }
            if (guiTracingSettings.exportToDisk) {
                ImGui::InputText("##linesexportpath", &guiTracingSettings.exportPath);
            }
        }

        if (changed) {
            requestNewData();
        }
    }
    ImGui::End();
}

void StreamlineTracingRequester::setLineTracerSettings(const SettingsMap& settings) {
    bool changed = false;

    std::string datasetName;
    if (settings.getValueOpt("dataset", datasetName)) {
        for (int i = 0; i < int(gridDataSetNames.size()); i++) {
            if (datasetName == gridDataSetNames.at(i)) {
                selectedGridDataSetIndex = i + 2;
                const std::string pathString = gridDataSetFilenames.at(selectedGridDataSetIndex - 2);
                bool isAbsolutePath = sgl::FileUtils::get()->getIsPathAbsolute(pathString);
                if (isAbsolutePath) {
                    gridDataSetFilename = pathString;
                } else {
                    gridDataSetFilename = lineDataSetsDirectory + pathString;
                }
                guiTracingSettings.exportPath =
                        sgl::FileUtils::get()->removeExtension(gridDataSetFilename) + ".binlines";
                changed = true;
                break;
            }
        }
    }

    std::string flowPrimitivesName;
    if (settings.getValueOpt("flow_primitives", flowPrimitivesName)) {
        int i;
        for (i = 0; i < IM_ARRAYSIZE(FLOW_PRIMITIVE_NAMES); i++) {
            if (boost::to_lower_copy(flowPrimitivesName)
                    == boost::to_lower_copy(std::string(FLOW_PRIMITIVE_NAMES[i]))) {
                guiTracingSettings.flowPrimitives = FlowPrimitives(i);
                changed = true;
                break;
            }
        }
        if (i == IM_ARRAYSIZE(FLOW_PRIMITIVE_NAMES)) {
            sgl::Logfile::get()->writeError(
                    "Error in StreamlineTracingRequester::setLineTracerSettings: Unknown flow primitive type \""
                    + flowPrimitivesName + "\".");
        }
    }

    std::string seedingStrategyName;
    if (settings.getValueOpt("seeding_strategy", seedingStrategyName)) {
        int i;
        for (i = 0; i < IM_ARRAYSIZE(STREAMLINE_SEEDING_STRATEGY_NAMES); i++) {
            if (boost::to_lower_copy(seedingStrategyName)
                    == boost::to_lower_copy(std::string(STREAMLINE_SEEDING_STRATEGY_NAMES[i]))) {
                guiTracingSettings.streamlineSeedingStrategy = StreamlineSeedingStrategy(i);
                changed = true;
                break;
            }
        }
        if (i == IM_ARRAYSIZE(STREAMLINE_SEEDING_STRATEGY_NAMES)) {
            sgl::Logfile::get()->writeError(
                    "Error in StreamlineTracingRequester::setLineTracerSettings: Unknown seeding strategy name \""
                    + seedingStrategyName + "\".");
        }
    }

    std::string integrationMethodName;
    if (settings.getValueOpt("integration_method", integrationMethodName)) {
        int i;
        for (i = 0; i < IM_ARRAYSIZE(STREAMLINE_INTEGRATION_METHOD_NAMES); i++) {
            if (boost::to_lower_copy(integrationMethodName)
                    == boost::to_lower_copy(std::string(STREAMLINE_INTEGRATION_METHOD_NAMES[i]))) {
                guiTracingSettings.integrationMethod = StreamlineIntegrationMethod(i);
                changed = true;
                break;
            }
        }
        if (i == IM_ARRAYSIZE(STREAMLINE_INTEGRATION_METHOD_NAMES)) {
            sgl::Logfile::get()->writeError(
                    "Error in StreamlineTracingRequester::setLineTracerSettings: Unknown integration method name \""
                    + integrationMethodName + "\".");
        }
    }

    std::string integrationDirectionName;
    if (settings.getValueOpt("integration_direction", integrationDirectionName)) {
        int i;
        for (i = 0; i < IM_ARRAYSIZE(STREAMLINE_INTEGRATION_DIRECTION_NAMES); i++) {
                if (boost::to_lower_copy(integrationDirectionName)
                == boost::to_lower_copy(std::string(STREAMLINE_INTEGRATION_DIRECTION_NAMES[i]))) {
                guiTracingSettings.integrationDirection = StreamlineIntegrationDirection(i);
                changed = true;
                break;
            }
        }
        if (i == IM_ARRAYSIZE(STREAMLINE_INTEGRATION_DIRECTION_NAMES)) {
            sgl::Logfile::get()->writeError(
                    "Error in StreamlineTracingRequester::setLineTracerSettings: Unknown integration direction \""
                    + integrationDirectionName + "\".");
        }
    }

    std::string vectorFieldName;
    if (settings.getValueOpt("vector_field", vectorFieldName)) {
        std::lock_guard<std::mutex> lock(cachedGridMetadataMutex);
        int i;
        for (i = 0; i < int(cachedGridVectorFieldNames.size()); i++) {
            if (boost::to_lower_copy(vectorFieldName)
                    == boost::to_lower_copy(std::string(STREAMLINE_INTEGRATION_DIRECTION_NAMES[i]))) {
                guiTracingSettings.vectorFieldIndex = i;
                changed = true;
                break;
            }
        }
        if (i == int(cachedGridVectorFieldNames.size())) {
            sgl::Logfile::get()->writeError(
                    "Error in StreamlineTracingRequester::setLineTracerSettings: Unknown vector field name \""
                    + vectorFieldName + "\".");
        }
    }

    std::string terminationCheckTypeName;
    if (settings.getValueOpt("termination_check_type", terminationCheckTypeName)) {
        int i;
        for (i = 0; i < IM_ARRAYSIZE(TERMINATION_CHECK_TYPE_NAMES); i++) {
            if (boost::to_lower_copy(terminationCheckTypeName)
                == boost::to_lower_copy(std::string(TERMINATION_CHECK_TYPE_NAMES[i]))) {
                guiTracingSettings.terminationCheckType = TerminationCheckType(i);
                changed = true;
                break;
            }
        }
        if (i == IM_ARRAYSIZE(TERMINATION_CHECK_TYPE_NAMES)) {
            sgl::Logfile::get()->writeError(
                    "Error in StreamlineTracingRequester::setLineTracerSettings: Unknown termination check type \""
                    + terminationCheckTypeName + "\".");
        }
    }

    std::string loopCheckModeName;
    if (settings.getValueOpt("loop_check_mode", loopCheckModeName)) {
        int i;
        for (i = 0; i < IM_ARRAYSIZE(LOOP_CHECK_MODE_NAMES); i++) {
            if (boost::to_lower_copy(loopCheckModeName)
                == boost::to_lower_copy(std::string(LOOP_CHECK_MODE_NAMES[i]))) {
                guiTracingSettings.loopCheckMode = LoopCheckMode(i);
                changed = true;
                break;
            }
        }
        if (i == IM_ARRAYSIZE(LOOP_CHECK_MODE_NAMES)) {
            sgl::Logfile::get()->writeError(
                    "Error in StreamlineTracingRequester::setLineTracerSettings: Unknown loop check mode \""
                    + loopCheckModeName + "\".");
        }
    }

    changed |= settings.getValueOpt("num_primitives", guiTracingSettings.numPrimitives);
    changed |= settings.getValueOpt("time_step_scale", guiTracingSettings.timeStepScale);
    changed |= settings.getValueOpt("grid_subsampling_factor", guiTracingSettings.gridSubsamplingFactor);
    changed |= settings.getValueOpt("max_num_iterations", guiTracingSettings.maxNumIterations);
    changed |= settings.getValueOpt("termination_distance", guiTracingSettings.terminationDistance);
    changed |= settings.getValueOpt("termination_distance_self", guiTracingSettings.terminationDistanceSelf);
    changed |= settings.getValueOpt("min_line_length", guiTracingSettings.minimumLength);
    changed |= settings.getValueOpt("min_separation_distance", guiTracingSettings.minimumSeparationDistance);
    changed |= settings.getValueOpt("show_boundary_mesh", guiTracingSettings.showSimulationGridOutline);
    changed |= settings.getValueOpt("smooth_boundary", guiTracingSettings.smoothedSimulationGridOutline);
    changed |= settings.getValueOpt("use_helicity", guiTracingSettings.useHelicity);
    changed |= settings.getValueOpt("max_helicity_twist", guiTracingSettings.maxHelicityTwist);
    changed |= settings.getValueOpt("initial_ribbon_direction", guiTracingSettings.initialRibbonDirection);
    changed |= settings.getValueOpt("seeding_subsampling_factor", guiTracingSettings.seedingSubsamplingFactor);

    settings.getValueOpt("export_to_disk", guiTracingSettings.exportToDisk);
    settings.getValueOpt("export_path", guiTracingSettings.exportPath);

    changed |= guiTracingSettings.seeder->setNewSettings(settings);

    if (changed) {
        requestNewData();
    }
}

void StreamlineTracingRequester::setDatasetFilename(const std::string& newDatasetFilename) {
    bool isDataSetInList = false;
    for (int i = 0; i < int(gridDataSetFilenames.size()); i++) {
        auto newDataSetPath = boost::filesystem::absolute(newDatasetFilename);
        auto currentDataSetPath = boost::filesystem::absolute(
                lineDataSetsDirectory + gridDataSetFilenames.at(i));
        if (boost::filesystem::equivalent(newDataSetPath, currentDataSetPath)) {
            selectedGridDataSetIndex = i + 2;
            const std::string pathString = gridDataSetFilenames.at(selectedGridDataSetIndex - 2);
            bool isAbsolutePath = sgl::FileUtils::get()->getIsPathAbsolute(pathString);
            if (isAbsolutePath) {
                gridDataSetFilename = pathString;
            } else {
                gridDataSetFilename = lineDataSetsDirectory + pathString;
            }
            guiTracingSettings.exportPath =
                    sgl::FileUtils::get()->removeExtension(gridDataSetFilename) + ".binlines";
            isDataSetInList = true;
            break;
        }
    }

    if (!isDataSetInList) {
        selectedGridDataSetIndex = 0;
        gridDataSetFilename = newDatasetFilename;
    }

    requestNewData();
}

void StreamlineTracingRequester::requestNewData() {
    if (selectedGridDataSetIndex != 1 && gridDataSetFilename.empty()) {
        return;
    }

    StreamlineTracingSettings request = guiTracingSettings;
    request.seeder = StreamlineSeederPtr(guiTracingSettings.seeder->copy());
    request.isAbcDataSet = selectedGridDataSetIndex == 1;
    if (selectedGridDataSetIndex == 1) {
        request.dataSourceFilename = "ABC_" + abcFlowGenerator.getSettingsString();
        request.abcFlowGenerator = abcFlowGenerator;
    } else {
        request.dataSourceFilename = boost::filesystem::absolute(gridDataSetFilename).generic_string();
    }
    if (selectedGridDataSetIndex >= 2) {
        request.gridDataSetMetaData = gridDataSetsMetaData.at(selectedGridDataSetIndex - 2);
    } else {
        request.gridDataSetMetaData = {};
    }
    request.gridDataSetMetaData.useNormalizedVelocity = guiTracingSettings.useNormalizedVelocity;
    request.gridDataSetMetaData.useNormalizedVorticity = guiTracingSettings.useNormalizedVorticity;

    queueRequestStruct(request);
}

bool StreamlineTracingRequester::getHasNewData(DataSetInformation& dataSetInformation, LineDataPtr& lineData) {
    if (getReply(lineData)) {
        return true;
    }
    return false;
}

void StreamlineTracingRequester::queueRequestStruct(const StreamlineTracingSettings& request) {
    {
        std::lock_guard<std::mutex> lock(requestMutex);
        workerTracingSettings = request;
        requestLineData = LineDataPtr(new LineDataFlow(transferFunctionWindow));
        hasRequest = true;
    }
    hasRequestConditionVariable.notify_all();
}

bool StreamlineTracingRequester::getReply(LineDataPtr& lineData) {
    bool hasReply;
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        hasReply = this->hasReply;
        if (hasReply) {
            lineData = replyLineData;
        }

        // Now, new requests can be worked on.
        this->hasReply = false;
        // this->replyMessage.clear();
        this->replyLineData = LineDataPtr();
    }
    hasReplyConditionVariable.notify_all();
    return hasReply;
}

void StreamlineTracingRequester::join() {
    if (!programIsFinished) {
        {
            std::lock_guard<std::mutex> lockRequest(requestMutex);
            programIsFinished = true;
            hasRequest = true;

            {
                std::lock_guard<std::mutex> lockReply(replyMutex);
                this->hasReply = false;
                // this->replyMessage.clear();
            }
            hasReplyConditionVariable.notify_all();
        }
        hasRequestConditionVariable.notify_all();
        if (requesterThread.joinable()) {
            requesterThread.join();
        }
    }
}

void StreamlineTracingRequester::mainLoop() {
#ifdef TRACY_ENABLE
    tracy::SetThreadName("StreamlineTracingRequester");
#endif

    while (true) {
        std::unique_lock<std::mutex> requestLock(requestMutex);
        hasRequestConditionVariable.wait(requestLock, [this] { return hasRequest; });

        if (programIsFinished) {
            break;
        }

        if (hasRequest) {
            StreamlineTracingSettings request = workerTracingSettings;
            std::shared_ptr<LineDataFlow> lineData = std::static_pointer_cast<LineDataFlow>(requestLineData);
            hasRequest = false;
            isProcessingRequest = true;
            requestLock.unlock();

            traceLines(request, lineData);

            std::lock_guard<std::mutex> replyLock(replyMutex);
            hasReply = true;

            replyLineData = lineData;
            isProcessingRequest = false;
        }
    }
}

void StreamlineTracingRequester::traceLines(
        StreamlineTracingSettings& request, std::shared_ptr<LineDataFlow>& lineData) {
    if (cachedGridFilename != request.dataSourceFilename || !(cachedGridMetaData == request.gridDataSetMetaData)
            || cachedGridSubsamplingFactor != request.gridSubsamplingFactor) {
        if (cachedGrid) {
            delete cachedGrid;
            cachedGrid = nullptr;
        }
        cachedGridFilename = request.dataSourceFilename;
        cachedGridMetaData = request.gridDataSetMetaData;
        cachedGridSubsamplingFactor = request.gridSubsamplingFactor;

        cachedGrid = new StreamlineTracingGrid;
        if (request.gridDataSetMetaData.axes != glm::ivec3(0, 1, 2)) {
            cachedGrid->setTransposeAxes(request.gridDataSetMetaData.axes);
        }
        if (request.gridSubsamplingFactor != 1) {
            cachedGrid->setGridSubsamplingFactor(request.gridSubsamplingFactor);
        }
        if (request.isAbcDataSet) {
            request.abcFlowGenerator.load(request.gridDataSetMetaData, cachedGrid);
        } else if (boost::ends_with(request.dataSourceFilename, ".vtk")) {
            StructuredGridVtkLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        } else if (boost::ends_with(request.dataSourceFilename, ".vti")
                || boost::ends_with(request.dataSourceFilename, ".vts")) {
            VtkXmlLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        } else if (boost::ends_with(request.dataSourceFilename, ".nc")) {
            NetCdfLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        } else if (boost::ends_with(request.dataSourceFilename, ".am")) {
            AmiraMeshLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        } else if (boost::ends_with(request.dataSourceFilename, ".bin")) {
            RbcBinFileLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        } else if (boost::ends_with(request.dataSourceFilename, ".field")) {
            FieldFileLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        } else if (boost::ends_with(request.dataSourceFilename, ".dat")
                || boost::ends_with(request.dataSourceFilename, ".raw")) {
            DatRawFileLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        }
#ifdef USE_ECCODES
        else if (boost::ends_with(request.dataSourceFilename, ".grib")
                || boost::ends_with(request.dataSourceFilename, ".grb")) {
            GribLoader::load(
                    request.dataSourceFilename, request.gridDataSetMetaData,
                    cachedGrid);
        }
#endif

        {
            std::lock_guard<std::mutex> replyLock(gridInfoMutex);
            newGridLoaded = true;
            gridBox = cachedGrid->getBox();
        }

        std::lock_guard<std::mutex> lock(cachedGridMetadataMutex);
        cachedSimulationMeshOutlineTriangleIndices.clear();
        cachedSimulationMeshOutlineVertexPositions.clear();
        cachedSimulationMeshOutlineVertexNormals.clear();
        cachedGridVectorFieldNames = cachedGrid->getVectorFieldNames();
    }
    if (guiTracingSettings.showSimulationGridOutline && cachedSimulationMeshOutlineVertexPositions.empty()) {
        std::vector<uint32_t> simulationMeshOutlineTriangleIndices;
        std::vector<glm::vec3> simulationMeshOutlineVertexPositions;
        std::vector<glm::vec3> simulationMeshOutlineVertexNormals;

        cachedGrid->computeSimulationBoundaryMesh(
                simulationMeshOutlineTriangleIndices,
                simulationMeshOutlineVertexPositions);
        if (guiTracingSettings.smoothedSimulationGridOutline) {
            laplacianSmoothing(
                    simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions);
        }
        normalizeVertexPositions(
                simulationMeshOutlineVertexPositions, gridBox, nullptr);
        computeSmoothTriangleNormals(
                simulationMeshOutlineTriangleIndices,
                simulationMeshOutlineVertexPositions,
                simulationMeshOutlineVertexNormals);

        std::lock_guard<std::mutex> lock(cachedGridMetadataMutex);
        cachedSimulationMeshOutlineTriangleIndices = simulationMeshOutlineTriangleIndices;
        cachedSimulationMeshOutlineVertexPositions = simulationMeshOutlineVertexPositions;
        cachedSimulationMeshOutlineVertexNormals = simulationMeshOutlineVertexNormals;
    }

    Trajectories trajectories;
    if (guiTracingSettings.flowPrimitives == FlowPrimitives::STREAMLINES) {
        cachedGrid->traceStreamlines(request, trajectories);
    } else if (guiTracingSettings.flowPrimitives == FlowPrimitives::STREAMRIBBONS) {
        std::vector<std::vector<glm::vec3>> ribbonsDirections;
        cachedGrid->traceStreamribbons(request, trajectories, ribbonsDirections);
        lineData->ribbonsDirections = ribbonsDirections;
    }
    normalizeTrajectoriesVertexPositions(trajectories, gridBox, nullptr);

    if (guiTracingSettings.showSimulationGridOutline) {
        lineData->shallRenderSimulationMeshBoundary = true;
        lineData->simulationMeshOutlineTriangleIndices = cachedSimulationMeshOutlineTriangleIndices;
        lineData->simulationMeshOutlineVertexPositions = cachedSimulationMeshOutlineVertexPositions;
        lineData->simulationMeshOutlineVertexNormals = cachedSimulationMeshOutlineVertexNormals;
    }

    if (workerTracingSettings.exportToDisk) {
        BinLinesData binLinesData;
        binLinesData.trajectories = trajectories;
        binLinesData.verticesNormalized = true;
        binLinesData.attributeNames = cachedGrid->getScalarFieldNames();
        binLinesData.ribbonsDirections = lineData->ribbonsDirections;
        binLinesData.simulationMeshOutlineTriangleIndices = cachedSimulationMeshOutlineTriangleIndices;
        binLinesData.simulationMeshOutlineVertexPositions = cachedSimulationMeshOutlineVertexPositions;
        binLinesData.simulationMeshOutlineVertexNormals = cachedSimulationMeshOutlineVertexNormals;
        saveTrajectoriesAsBinLines(workerTracingSettings.exportPath, binLinesData);
    }

    lineData->fileNames = { request.dataSourceFilename };
    lineData->attributeNames = cachedGrid->getScalarFieldNames();
    lineData->onAttributeNamesSet();
    lineData->setTrajectoryData(trajectories);
}
