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

#include <boost/algorithm/string.hpp>

#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Utils/File/Logfile.hpp>

#include "Renderers/Tubes/Tubes.hpp"

#include "LineDataMultiVar.hpp"

const char* const MULTIVAR_RENDERTYPE_DISPLAYNAMES[] = {
        "Rolls", "Twisted Rolls", "Color Bands", "Oriented Color Bands", "Checkerboard", "Fibers"
};
const char *const MULTIVAR_RADIUSTYPE_DISPLAYNAMES[] = {
        "Global", "Line"
};

void LineDataMultiVar::setClearColor(const sgl::Color& clearColor) {
    LineData::setClearColor(clearColor);
    this->clearColor = clearColor;
    multiVarWindow.setClearColor(clearColor);
}

void LineDataMultiVar::recomputeWidgetPositions() {
    if (!useMultiVarRendering) {
        for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
            colorLegendWidgets.at(i).setPositionIndex(0, 1);
        }
        return;
    }

    int numWidgetsVisible = 0;
    for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
        if (varSelected.at(i)) {
            numWidgetsVisible++;
        }
    }
    int positionCounter = 0;
    for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
        colorLegendWidgets.at(i).setPositionIndex(positionCounter, numWidgetsVisible);
        if (varSelected.at(i)) {
            positionCounter++;
        }
    }
}

bool LineDataMultiVar::renderGuiWindow(bool isRasterizer)  {
    multiVarWindow.renderGui();

    bool shallReloadGatherShader = false;
    if (useMultiVarRendering && shallRenderColorLegendWidgets) {
        for (int i = 0; i < colorLegendWidgets.size(); i++) {
            if (varSelected.at(i)) {
                colorLegendWidgets.at(i).renderGui();
            }
        }
    } else {
        shallReloadGatherShader = LineData::renderGuiWindow(isRasterizer) || shallReloadGatherShader;
    }

    return shallReloadGatherShader;
}

bool LineDataMultiVar::renderGui(bool isRasterizer) {
    bool shallReloadGatherShader = false;
    if (ImGui::Checkbox("Multivariate Rendering", &useMultiVarRendering)) {
        dirty = true;
        shallReloadGatherShader = true;
        recomputeColorLegend();
        recomputeWidgetPositions();
    }

    if (!useMultiVarRendering) {
        return LineData::renderGui(isRasterizer) || shallReloadGatherShader;
    }

    if (ImGui::CollapsingHeader("Multi-Variate Settings", NULL, ImGuiTreeNodeFlags_DefaultOpen)) {
        shallReloadGatherShader = renderGuiTechniqueSettings() || shallReloadGatherShader;
    }

    if (ImGui::CollapsingHeader("Line Rendering Settings", NULL, ImGuiTreeNodeFlags_DefaultOpen )) {
        shallReloadGatherShader = renderGuiLineRenderingSettings() || shallReloadGatherShader;
    }

    return shallReloadGatherShader;
}

bool LineDataMultiVar::renderGuiTechniqueSettings() {
    bool shallReloadGatherShader = false;

    if (ImGui::Combo(
            "Render Technique", (int *) &multiVarRenderMode, MULTIVAR_RENDERTYPE_DISPLAYNAMES,
            IM_ARRAYSIZE(MULTIVAR_RENDERTYPE_DISPLAYNAMES))) {
        shallReloadGatherShader = true;
    }

    ImGui::Separator();
    ImGui::Text("Technique settings:");
    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS) {

    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_COLOR_BANDS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD
        || multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (ImGui::Checkbox("Map Tube Diameter", &mapTubeDiameter)) {
            reRender = true;
        }

        if (mapTubeDiameter) {
            if (ImGui::Combo("Radius Mapping Mode", (int *) &multiVarRadiusMappingMode,
                             MULTIVAR_RADIUSTYPE_DISPLAYNAMES,
                             IM_ARRAYSIZE(MULTIVAR_RADIUSTYPE_DISPLAYNAMES))) {
                reRender = true;
            }
        }

    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD) {
        if (ImGui::SliderInt("Checkerboard Height", &checkerboardHeight, 1, 10)) {
            reRender = true;
        }
        if (ImGui::SliderInt("Checkerboard Width", &checkerboardWidth, 1, 20)) {
            reRender = true;
        }
        if (ImGui::SliderInt("Checkerboard Iterator", &checkerboardIterator, 1, 5)) {
            reRender = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS) {
        if (ImGui::SliderFloat("Twist Offset", &twistOffset, 0.0, 1.0, "%.2f")) {
            reRender = true;
        }

        if (ImGui::Checkbox("Constant Twist Offset", &constantTwistOffset)) {
            reRender = true;
        }
    }

    if (ImGui::SliderFloat("Separator Width", &separatorWidth, 0.0, 1.0, "%.2f")) {
        reRender = true;
    }

    if (ImGui::SliderFloat("Min Color Intensity", &minColorIntensity, 0.0, 1.0, "%.2f")) {
        reRender = true;
        recomputeColorLegend();
    }

    auto& varNames = attributeNames;
    bool itemHasChanged = false;

    std::vector<std::string> comboSelVec(0);
    if (ImGui::BeginCombo("Variables", comboValue.c_str(), ImGuiComboFlags_NoArrowButton)) {
        for (auto v = 0; v < varSelected.size(); ++v) {
            std::vector<std::string> names;
            boost::split(names, varNames[v], [](char c) { return c == '#'; });

            if (ImGui::Selectable(
                    names[0].c_str(), reinterpret_cast<bool*>(&varSelected[v]),
                    ImGuiSelectableFlags_::ImGuiSelectableFlags_DontClosePopups)) {
                itemHasChanged = true;
            }

            if (static_cast<bool>(varSelected[v])) {
                ImGui::SetItemDefaultFocus();
                comboSelVec.push_back(names[1]);
            }
        }

        numVariablesSelected = comboSelVec.size();

        comboValue = "";
        for (auto v = 0; v < comboSelVec.size(); ++v) {
            comboValue += comboSelVec[v];
            if (comboSelVec.size() > 1 && v != comboSelVec.size() - 1) {
                comboValue += ",";
            }
        }

        // Update SSBO
        if (itemHasChanged) {
            varSelectedArrayBuffer = sgl::Renderer->createGeometryBuffer(
                    varSelected.size() * sizeof(uint32_t), (void*)&varSelected.front(),
                    sgl::SHADER_STORAGE_BUFFER);
            shallReloadGatherShader = true;
            recomputeWidgetPositions();
        }

        ImGui::EndCombo();
    }

    ImGui::NewLine();

    bool colorHasChanged = false;
    for (auto v = 0; v < varSelected.size(); ++v) {
        if (varSelected[v]) {
            std::vector<std::string> names;
            boost::split(names, varNames[v], [](char c) { return c == '#'; });

            ImGui::SameLine();
            if (ImGui::ColorEdit3(
                    names[1].c_str(), reinterpret_cast<float*>(&varColors[v]),
                    ImGuiColorEditFlags_HSV | ImGuiColorEditFlags_NoInputs)) {
                colorHasChanged = true;
            }
        }
    }

    if (colorHasChanged) {
        varColorArrayBuffer = sgl::Renderer->createGeometryBuffer(
                varColors.size() * sizeof(glm::vec4), (void*)&varColors.front(),
                sgl::SHADER_STORAGE_BUFFER);
        recomputeColorLegend();
        reRender = true;
    }

    return shallReloadGatherShader;
}

bool LineDataMultiVar::renderGuiLineRenderingSettings() {
    bool shallReloadGatherShader = false;

    if (ImGui::Button("Reload Shader")) {
        shallReloadGatherShader = true;
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (ImGui::SliderInt("Num Line Segments", &numLineSegments, 3, 20)) {
            shallReloadGatherShader = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (ImGui::SliderFloat("Fiber radius", &fiberRadius, 0.0001f, 0.01f, "%.4f")) {
            reRender = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_COLOR_BANDS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD) {
        if (ImGui::SliderInt("Num Line Segments", &numInstances, 3, 20)) {
            shallReloadGatherShader = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_COLOR_BANDS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD
        || multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (ImGui::SliderFloat("Min. Radius Factor", &minRadiusFactor, 0.0f, 1.0f, "%.3f")) {
            reRender = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS) {
        if (ImGui::SliderInt("Roll Width", &rollWidth, 1, 4)) {
            reRender = true;
        }
    }

    ImGui::Separator();
    ImGui::Text("Phong Lighting Settings:");

    if (ImGui::SliderFloat("MaterialAmbient", &materialConstantAmbient, 0.0, 1.0, "%.2f")) {
        reRender = true;
    }
    if (ImGui::SliderFloat("MaterialDiffuse", &materialConstantDiffuse, 0.0, 1.0, "%.2f")) {
        reRender = true;
    }
    if (ImGui::SliderFloat("MaterialSpecular", &materialConstantSpecular, 0.0, 1.0, "%.2f")) {
        reRender = true;
    }
    if (ImGui::SliderFloat("MaterialSpecularExp", &materialConstantSpecularExp, 0.0, 100.0, "%.2f")) {
        reRender = true;
    }
    if (ImGui::Checkbox("Draw Halo", &drawHalo)) {
        reRender = true;
    }
    if (ImGui::SliderFloat("Halo Factor", &haloFactor, 0.0, 4.0, "%.1f")) {
        reRender = true;
    }

    return shallReloadGatherShader;
}
