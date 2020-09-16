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

#include "InternalState.hpp"

void getTestModesDepthComplexity(std::vector<InternalState> &states, InternalState state) {
    state.renderingMode = RENDERING_MODE_DEPTH_COMPLEXITY;
    state.name = "Depth Complexity";
    states.push_back(state);
}

void getTestModesPerPixelLinkedLists(std::vector<InternalState> &states, InternalState state) {
    state.renderingMode = RENDERING_MODE_PER_PIXEL_LINKED_LIST;
    state.name = "PPLL";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesOpacityOptimization(std::vector<InternalState> &states, InternalState state) {
    state.renderingMode = RENDERING_MODE_PER_PIXEL_LINKED_LIST;
    state.name = "Opacity Optimization";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesPaperForMesh(std::vector<InternalState> &states, InternalState state) {
    getTestModesDepthComplexity(states, state);
    getTestModesPerPixelLinkedLists(states, state);
    getTestModesOpacityOptimization(states, state);
}

std::vector<InternalState> getTestModesPaper()
{
    std::vector<InternalState> states;
    std::vector<glm::ivec2> windowResolutions = {
            glm::ivec2(1280, 720), glm::ivec2(1920, 1080), glm::ivec2(2560, 1440) };
    //std::vector<glm::ivec2> windowResolutions = { glm::ivec2(2560, 1440) };
    //std::vector<glm::ivec2> windowResolutions = { glm::ivec2(2186, 1358) };
    std::vector<DataSetDescriptor> dataSetDescriptors = {
            DataSetDescriptor("Rings"),
    };
    std::vector<std::string> transferFunctionNames = {
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
            getTestModesPaperForMesh(states, state);
        }
    }*/
    for (size_t i = 0; i < dataSetDescriptors.size(); i++) {
        state.dataSetDescriptor = dataSetDescriptors.at(i);
        for (size_t j = 0; j < windowResolutions.size(); j++) {
            state.windowResolution = windowResolutions.at(j);
            if (transferFunctionNames.size() > 0) {
                state.transferFunctionName = transferFunctionNames.at(i);
            }
            getTestModesPaperForMesh(states, state);
        }
    }

    // Append model name to state name if more than one model is loaded
    if (dataSetDescriptors.size() >= 1 || windowResolutions.size() > 1) {
        for (InternalState &state : states) {
            state.name =
                    sgl::toString(state.windowResolution.x) + "x" + sgl::toString(state.windowResolution.y)
                    + " " + state.dataSetDescriptor.name + " " + state.name;
        }
    }

    return states;
}
