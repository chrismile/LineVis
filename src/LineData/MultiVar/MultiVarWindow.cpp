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

#include <climits>

#include <Math/Math.hpp>
#include <ImGui/imgui.h>
#include <ImGui/imgui_custom.h>

#include "MultiVarWindow.hpp"

// https://github.com/ocornut/imgui/issues/2342

MultiVarWindow::MultiVarWindow()
        : showWindow(true), variableIndex(0), clearColor(sgl::Color(255, 0, 0, 1.0)),
          histogramRes(50) {

}

void MultiVarWindow::setAttributes(
        const std::vector<std::vector<float>> &_variables,
        const std::vector<std::string> &_names) {
    // Set min/max and further information here
    // Maybe also KDE for Violin Plots
    // Copy Constructor
    attributes = _variables;
    names = _names;
    variablesMinMax.clear();

    for (const std::vector<float> &attrList : attributes) {
        float minValue = std::numeric_limits<float>::max();
        float maxValue = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
        #pragma omp parallel for reduction(min: minValue) reduction(max: maxValue) shared(attrList) default(none)
#endif
        for (size_t i = 0; i < attrList.size(); i++) {
            float value = attrList.at(i);
            minValue = std::min(minValue, value);
            maxValue = std::max(maxValue, value);
        }
        variablesMinMax.emplace_back(minValue, maxValue);
    }

    // Recompute histograms
    computeHistograms();
}

bool MultiVarWindow::renderGui() {
    if (showWindow) {
        bool windowIsOpened = true;
        if (!ImGui::Begin("MultiVar Info", &windowIsOpened)) {
            ImGui::End();
            return false;
        }

        // Render the var info chart
        renderVarChart();
        // Render the settings
        renderSettings();

        ImGui::End();
        return true;
    }

    return false;
}

void MultiVarWindow::computeHistograms() {
    histograms.resize(attributes.size());

    for (auto v = 0; v < attributes.size(); ++v) {
        float histogramsMax = 0;

        const auto& var = attributes[v];
        const auto& minMax = variablesMinMax[v];
        auto& histogram = histograms[v];
        histogram = std::vector<float>(histogramRes, 0.0f);

        for (const auto &value : var) {
            int32_t index = glm::clamp(
                    static_cast<int32_t>((value - minMax.x) / (minMax.y - minMax.x)
                    * static_cast<float>(histogramRes)), 0, histogramRes - 1);
            histogram[index]++;
        }

        // Normalize values of histogram
        for (const auto &numBin : histogram) {
            histogramsMax = std::max(histogramsMax, numBin);
        }

        for (auto &numBin : histogram) {
            numBin /= histogramsMax;
        }
    }
}

void MultiVarWindow::renderVarChart() {
    int regionWidth = ImGui::GetContentRegionAvailWidth();
    int graphHeight = 150;

    const auto &histogram = histograms[variableIndex];

    ImDrawList *drawList = ImGui::GetWindowDrawList();

    ImVec2 backgroundPos = ImGui::GetCursorScreenPos();

    ImColor backgroundColor(clearColor.getFloatR(), clearColor.getFloatG(), clearColor.getFloatB());
    int border = 0;
    drawList->AddRectFilled(
            ImVec2(backgroundPos.x + border, backgroundPos.y + border),
            ImVec2(backgroundPos.x + regionWidth - border, backgroundPos.y + graphHeight - border),
            backgroundColor, ImGui::GetStyle().FrameRounding);

    ImVec2 oldPadding = ImGui::GetStyle().FramePadding;
    ImGui::GetStyle().FramePadding = ImVec2(1, 1);
    ImGui::PlotHistogram(
            "##histogram", histogram.data(), histogram.size(), 0, NULL,
            0.0f, 1.0f, ImVec2(regionWidth, graphHeight));
    ImGui::GetStyle().FramePadding = oldPadding;
}

void MultiVarWindow::renderSettings() {
    if (ImGui::ListBoxHeader("Variables", ImVec2(-1, 180))) {
        for (auto n = 0; n < names.size(); ++n) {
            if (ImGui::Selectable(names[n].c_str(), variableIndex == n)) {
                variableIndex = n;
            }
        }
        ImGui::ListBoxFooter();
    }

    if (ImGui::SliderInt("Histogram Res.", &histogramRes, 1, 255)) {
        computeHistograms();
    }
}
