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
#include <Utils/Events/EventManager.hpp>
#include <Utils/File/Logfile.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Renderers/LineRenderer.hpp"

#include "LineDataMultiVar.hpp"

const char* const MULTIVAR_RENDERTYPE_DISPLAYNAMES[] = {
        "Rolls", "Twisted Rolls", "Color Bands", "Oriented Color Bands", "Oriented Color Bands (Ribbon)",
        "Checkerboard", "Fibers"
};
const char *const MULTIVAR_RADIUSTYPE_DISPLAYNAMES[] = {
        "Global", "Line"
};

const char *const ORIENTED_RIBBON_MODE_DISPLAYNAMES[] = {
        "Fixed Band Width",
        "Varying Band Width",
        "Varying Band Ratio",
        "Varying Ribbon Width"
};

void LineDataMultiVar::setClearColor(const sgl::Color& clearColor) {
    LineData::setClearColor(clearColor);
    this->clearColor = clearColor;
    //multiVarWindow.setClearColor(clearColor);
    multiVarTransferFunctionWindow.setClearColor(clearColor);
}

void LineDataMultiVar::setUseLinearRGB(bool useLinearRGB) {
    multiVarTransferFunctionWindow.setUseLinearRGB(useLinearRGB);
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

bool LineDataMultiVar::renderGuiWindowSecondary()  {
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

bool LineDataMultiVar::renderGuiOverlay() {
    bool shallReloadGatherShader = false;
    if (useMultiVarRendering && shallRenderColorLegendWidgets) {
        for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
            if (varSelected.at(i)) {
                colorLegendWidgets.at(i).setAttributeMinValue(
                        multiVarTransferFunctionWindow.getSelectedRangeMin(i));
                colorLegendWidgets.at(i).setAttributeMaxValue(
                        multiVarTransferFunctionWindow.getSelectedRangeMax(i));
                colorLegendWidgets.at(i).renderGui();
            }
        }
    } else {
        shallReloadGatherShader = LineData::renderGuiOverlay() || shallReloadGatherShader;
    }
    return shallReloadGatherShader;
}

bool LineDataMultiVar::renderGuiPropertyEditorNodesRenderer(
        sgl::PropertyEditor& propertyEditor, LineRenderer* lineRenderer) {
    bool shallReloadGatherShader = LineData::renderGuiPropertyEditorNodesRenderer(propertyEditor, lineRenderer);
    if (propertyEditor.beginNode("Line Rendering Settings")) {
        shallReloadGatherShader =
                renderGuiLineRenderingSettingsPropertyEditor(propertyEditor) || shallReloadGatherShader;
        propertyEditor.endNode();
    }
    return shallReloadGatherShader;
}

bool LineDataMultiVar::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = false;
    if (rendererSupportsMultiVarRendering && propertyEditor.addCheckbox(
            "Multivariate Rendering", &useMultiVarRendering)) {
        dirty = true;
        shallReloadGatherShader = true;
        recomputeColorLegend();
        recomputeWidgetPositions();
    }

    if (!useMultiVarRendering) {
        return LineData::renderGuiPropertyEditorNodes(propertyEditor) || shallReloadGatherShader;
    }

    if (propertyEditor.beginNode("Multi-Variate Settings")) {
        shallReloadGatherShader =
                renderGuiTechniqueSettingsPropertyEditor(propertyEditor) || shallReloadGatherShader;
        propertyEditor.endNode();
    }

    return shallReloadGatherShader;
}

bool LineDataMultiVar::renderGuiTechniqueSettingsPropertyEditor(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = false;

    if (propertyEditor.addCombo(
            "Render Technique", (int *) &multiVarRenderMode, MULTIVAR_RENDERTYPE_DISPLAYNAMES,
            IM_ARRAYSIZE(MULTIVAR_RENDERTYPE_DISPLAYNAMES))) {
        shallReloadGatherShader = true;
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_COLOR_BANDS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD
        || multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (propertyEditor.addCheckbox("Map Tube Diameter", &mapTubeDiameter)) {
            reRender = true;
        }

        if (mapTubeDiameter) {
            if (propertyEditor.addCombo(
                    "Radius Mapping Mode", (int*)&multiVarRadiusMappingMode,
                    MULTIVAR_RADIUSTYPE_DISPLAYNAMES, IM_ARRAYSIZE(MULTIVAR_RADIUSTYPE_DISPLAYNAMES))) {
                reRender = true;
            }
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD) {
        if (propertyEditor.addSliderInt("Checkerboard Height", &checkerboardHeight, 1, 10)) {
            reRender = true;
        }
        if (propertyEditor.addSliderInt("Checkerboard Width", &checkerboardWidth, 1, 20)) {
            reRender = true;
        }
        if (propertyEditor.addSliderInt("Checkerboard Iterator", &checkerboardIterator, 1, 5)) {
            reRender = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS) {
        if (propertyEditor.addSliderFloat("Twist Offset", &twistOffset, 0.0, 1.0, "%.2f")) {
            reRender = true;
        }

        if (propertyEditor.addCheckbox("Constant Twist Offset", &constantTwistOffset)) {
            reRender = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS_RIBBON) {
        if (propertyEditor.addCombo(
                "Oriented Ribbon Mode", (int*)&orientedRibbonMode,
                ORIENTED_RIBBON_MODE_DISPLAYNAMES, IM_ARRAYSIZE(ORIENTED_RIBBON_MODE_DISPLAYNAMES))) {
            shallReloadGatherShader = true;
        }
        if (orientedRibbonMode == ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH) {
            if (propertyEditor.addColorEdit3(
                    "Band Background Color", reinterpret_cast<float*>(&bandBackgroundColor))) {
                reRender = true;
            }
        }
    }

    if (propertyEditor.addSliderFloat("Separator Width", &separatorWidth, 0.0, 1.0, "%.2f")) {
        reRender = true;
    }

    if (propertyEditor.addCheckbox("Use Color Intensity", &useColorIntensity)) {
        reRender = true;
        recomputeColorLegend();
    }

    auto& varNames = attributeNames;
    bool itemHasChanged = false;

    std::vector<std::string> comboSelVec(0);
    if (propertyEditor.addBeginCombo("Variables", comboValue, ImGuiComboFlags_NoArrowButton)) {
        for (size_t v = 0; v < varSelected.size(); ++v) {
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
        for (size_t v = 0; v < comboSelVec.size(); ++v) {
            comboValue += comboSelVec[v];
            if (comboSelVec.size() > 1 && v + 1 != comboSelVec.size()) {
                comboValue += ",";
            }
        }

        // Update SSBO
        if (itemHasChanged) {
            varSelectedArrayBuffer = sgl::Renderer->createGeometryBuffer(
                    varSelected.size() * sizeof(uint32_t), varSelected.data(),
                    sgl::SHADER_STORAGE_BUFFER);
            shallReloadGatherShader = true;
            recomputeWidgetPositions();
        }

        propertyEditor.addEndCombo();
    }

    return shallReloadGatherShader;
}

bool LineDataMultiVar::renderGuiLineRenderingSettingsPropertyEditor(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = false;

    if (propertyEditor.addButton("Reload Gather Shader", "Reload")) {
        shallReloadGatherShader = true;
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (propertyEditor.addSliderInt("Num Line Segments", &numLineSegments, 3, 20)) {
            shallReloadGatherShader = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (propertyEditor.addSliderFloat(
                "Fiber radius", &fiberRadius, 0.0001f, 0.01f, "%.4f")) {
            reRender = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_COLOR_BANDS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD) {
        if (propertyEditor.addSliderInt(
                "Num Line Segments", &numInstances, 3, 20)) {
            shallReloadGatherShader = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_COLOR_BANDS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_TWISTED_ROLLS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_CHECKERBOARD
            || multiVarRenderMode == MULTIVAR_RENDERMODE_FIBERS) {
        if (propertyEditor.addSliderFloat(
                "Min. Radius Factor", &minRadiusFactor, 0.0f, 1.0f, "%.3f")) {
            reRender = true;
        }
    }

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ROLLS) {
        if (propertyEditor.addSliderInt("Roll Width", &rollWidth, 1, 4)) {
            reRender = true;
        }
    }

    if (propertyEditor.beginNode("Phong Lighting Settings")) {
        if (propertyEditor.addSliderFloat(
                "MaterialAmbient", &materialConstantAmbient, 0.0, 1.0, "%.2f")) {
            reRender = true;
        }
        if (propertyEditor.addSliderFloat(
                "MaterialDiffuse", &materialConstantDiffuse, 0.0, 1.0, "%.2f")) {
            reRender = true;
        }
        if (propertyEditor.addSliderFloat(
                "MaterialSpecular", &materialConstantSpecular, 0.0, 1.0, "%.2f")) {
            reRender = true;
        }
        if (propertyEditor.addSliderFloat(
                "MaterialSpecularExp", &materialConstantSpecularExp, 0.0, 100.0, "%.2f")) {
            reRender = true;
        }
        if (propertyEditor.addCheckbox(
                "Draw Halo", &drawHalo)) {
            reRender = true;
        }
        if (propertyEditor.addSliderFloat(
                "Halo Factor", &haloFactor, 0.0, 4.0, "%.1f")) {
            reRender = true;
        }
        propertyEditor.endNode();
    }

    return shallReloadGatherShader;
}
