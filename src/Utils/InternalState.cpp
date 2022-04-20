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

#include "Renderers/OIT/SyncMode.hpp"
#include "LineData/LineData.hpp"
#include "InternalState.hpp"

void getTestModesOpaque(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;
    state.name = "Opaque";
    states.push_back(state);
}

void getTestModesDepthComplexity(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_DEPTH_COMPLEXITY;
    state.name = "Depth Complexity";
    states.push_back(state);
}

void getTestModesPerPixelLinkedLists(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_PER_PIXEL_LINKED_LIST;
    state.name = "PPLL";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesOpacityOptimization(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_PER_PIXEL_LINKED_LIST;
    state.name = "Opacity Optimization";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesMlab(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_MLAB;

    state.name = "MLAB (No Sync)";
    state.rendererSettings = { SettingsMap({
            { "syncMode", std::to_string((int)NO_SYNC) }
    })};
    states.push_back(state);

    state.name = "MLAB (Spinlock)";
    state.rendererSettings = { SettingsMap({
            { "syncMode", std::to_string((int)SYNC_SPINLOCK) }
    })};
    states.push_back(state);

    state.name = "MLAB (Unordered Interlock)";
    state.rendererSettings = { SettingsMap({
            { "syncMode", std::to_string((int)SYNC_FRAGMENT_SHADER_INTERLOCK) },
            { "useOrderedFragmentShaderInterlock", "false" }
    })};
    states.push_back(state);

    state.name = "MLAB (Ordered Interlock)";
    state.rendererSettings = { SettingsMap({
            { "syncMode", std::to_string((int)SYNC_FRAGMENT_SHADER_INTERLOCK) },
            { "useOrderedFragmentShaderInterlock", "true" }
    })};
    states.push_back(state);
}

void getTestModesOITForDataSet(std::vector<InternalState>& states, InternalState state) {
    //getTestModesDepthComplexity(states, state);
    //getTestModesPerPixelLinkedLists(states, state);
    //getTestModesOpacityOptimization(states, state);
    //getTestModesOpacityOptimization(states, state);
    getTestModesOpaque(states, state);
    getTestModesDepthComplexity(states, state);
    getTestModesPerPixelLinkedLists(states, state);
    getTestModesMlab(states, state);
}

std::vector<InternalState> getTestModesOIT()
{
    std::vector<InternalState> states;
    //std::vector<glm::ivec2> windowResolutions = {
    //        glm::ivec2(1280, 720), glm::ivec2(1920, 1080), glm::ivec2(2560, 1440) };
    std::vector<glm::ivec2> windowResolutions = { glm::ivec2(1920, 1080) };
    //std::vector<glm::ivec2> windowResolutions = { glm::ivec2(2560, 1440) };
    //std::vector<glm::ivec2> windowResolutions = { glm::ivec2(2186, 1358) };
    std::vector<DataSetDescriptor> dataSetDescriptors = {
            //DataSetDescriptor("Aneurysm"),
            DataSetDescriptor("Femur (Vis2021)"),
    };
    std::vector<std::string> transferFunctionNames = {
            //"Transparent_Aneurysm.xml",
            "Standard.xml",
    };
    InternalState state;

    /*for (size_t i = 0; i < windowResolutions.size(); i++) {
        state.windowResolution = windowResolutions.at(i);
        for (size_t j = 0; j < dataSetDescriptors.size(); j++) {
            state.dataSetDescriptor = dataSetDescriptors.at(j);
            if (transferFunctionNames.size() > 0) {
                state.transferFunctionName = transferFunctionNames.at(j);
            }
            getTestModesOITForDataSet(states, state);
        }
    }*/
    for (size_t i = 0; i < dataSetDescriptors.size(); i++) {
        state.dataSetDescriptor = dataSetDescriptors.at(i);
        for (size_t j = 0; j < windowResolutions.size(); j++) {
            state.windowResolution = windowResolutions.at(j);
            if (!transferFunctionNames.empty()) {
                state.transferFunctionName = transferFunctionNames.at(i);
            }
            getTestModesOITForDataSet(states, state);
        }
    }

    // Append model name to state name if more than one model is loaded
    if (!dataSetDescriptors.empty() || windowResolutions.size() > 1) {
        for (InternalState& state : states) {
            state.name =
                    sgl::toString(state.windowResolution.x) + "x" + sgl::toString(state.windowResolution.y)
                    + " " + state.dataSetDescriptor.name + " " + state.name;
        }
    }

    return states;
}


void setLinePrimitiveMode(InternalState& state, LineData::LinePrimitiveMode linePrimitiveMode) {
    state.dataSetSettings = { SettingsMap(
            {
                    { "linePrimitiveModeIndex", std::to_string(int(linePrimitiveMode)) }
            })};
}
void getTestModesRasterization(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_ALL_LINES_OPAQUE;

    state.name = "Rasterization (Quads Programmable Pull)";
    setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_QUADS_PROGRAMMABLE_PULL);
    states.push_back(state);

    state.name = "Rasterization (Geometry Shader)";
    setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER);
    states.push_back(state);

    state.name = "Rasterization (Mesh Shader)";
    setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_MESH_SHADER);
    states.push_back(state);
}

void getTestModesVulkanRayTracing(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_VULKAN_RAY_TRACER;
    state.name = "Depth Complexity";
    states.push_back(state);
}

void getTestModesVoxelRayCasting(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_VOXEL_RAY_CASTING;
    state.name = "PPLL";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesOspray(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_OSPRAY_RAY_TRACER;
    state.name = "Opacity Optimization";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesOpaqueRenderingForDataSet(std::vector<InternalState>& states, InternalState state) {
    getTestModesRasterization(states, state);
    //getTestModesVulkanRayTracing(states, state);
    //getTestModesVoxelRayCasting(states, state);
    //getTestModesOspray(states, state);
}

std::vector<InternalState> getTestModesOpaqueRendering()
{
    std::vector<InternalState> states;
    std::vector<glm::ivec2> windowResolutions = { glm::ivec2(1920, 1080) };
    std::vector<DataSetDescriptor> dataSetDescriptors = {
            //DataSetDescriptor("Aneurysm"),
            DataSetDescriptor("Convection Rolls"),
            //DataSetDescriptor("Femur (Vis2021)"),
    };
    std::vector<std::string> transferFunctionNames = {
            //"Standard.xml"
    };
    InternalState state;

    for (size_t i = 0; i < dataSetDescriptors.size(); i++) {
        state.dataSetDescriptor = dataSetDescriptors.at(i);
        for (size_t j = 0; j < windowResolutions.size(); j++) {
            state.windowResolution = windowResolutions.at(j);
            if (!transferFunctionNames.empty()) {
                state.transferFunctionName = transferFunctionNames.at(i);
            }
            getTestModesOpaqueRenderingForDataSet(states, state);
        }
    }

    // Append model name to state name if more than one model is loaded
    if (!dataSetDescriptors.empty() || windowResolutions.size() > 1) {
        for (InternalState& state : states) {
            state.name =
                    sgl::toString(state.windowResolution.x) + "x" + sgl::toString(state.windowResolution.y)
                    + " " + state.dataSetDescriptor.name + " " + state.name;
        }
    }

    return states;
}

std::vector<InternalState> getTestModes() {
    return getTestModesOpaqueRendering();
}
