/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser, Michael Kern
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

#ifndef STRESSLINEVIS_MULTIVARWINDOW_HPP
#define STRESSLINEVIS_MULTIVARWINDOW_HPP

#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <Graphics/Color.hpp>
#include <ImGui/ImGuiWrapper.hpp>

class MultiVarWindow {
public:
    MultiVarWindow();

    /// Render GUI elements.
    bool renderGui();

    void setAttributes(
            const std::vector<std::vector<float>>& _variables,
            const std::vector<std::string>& _names);

    inline void setClearColor(const sgl::Color& _clearColor) {
        clearColor = _clearColor;
    }
    inline void setShowWindow(const bool _showWindow ) {
        showWindow = _showWindow ;
    }

protected:
    bool showWindow;
    int32_t variableIndex;
    sgl::Color clearColor;
    int32_t histogramRes;
    std::vector<std::vector<float>> attributes;
    std::vector<std::string> names;
    std::vector<std::vector<float>> histograms;
    std::vector<glm::vec2> variablesMinMax;

    void computeHistograms();
    // Render a VIS graph for the currently selected variable
    void renderVarChart();
    // Render the settings to control the information plots
    void renderSettings();
};

#endif //STRESSLINEVIS_MULTIVARWINDOW_HPP
