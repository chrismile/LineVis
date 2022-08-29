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

#include <iostream>
#include <cmath>

#ifdef __linux__
#include <sys/inotify.h>
#include <poll.h>
#endif

#include <glm/glm.hpp>

#include <Utils/AppSettings.hpp>
#include <Utils/XML.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Math/Math.hpp>
#include <Graphics/Vulkan/Buffers/Buffer.hpp>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <ImGui/imgui.h>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include "MultiVarTransferFunctionWindow.hpp"

using namespace tinyxml2;

const size_t TRANSFER_FUNCTION_TEXTURE_SIZE = 256;

GuiVarData::GuiVarData(
        MultiVarTransferFunctionWindow* window, const std::string& tfPresetFile,
        sgl::Color16* transferFunctionMap_sRGB,
        sgl::Color16* transferFunctionMap_linearRGB) {
    this->window = window;
    this->transferFunctionMap_sRGB = transferFunctionMap_sRGB;
    this->transferFunctionMap_linearRGB = transferFunctionMap_linearRGB;

    std::string tfFileName = window->saveDirectory + tfPresetFile;
    const std::string stdFileName = window->saveDirectory + "Standard.xml";
    if (tfFileName.empty() || !sgl::FileUtils::get()->exists(tfFileName)) {
        tfFileName = stdFileName;
    }
    if (sgl::FileUtils::get()->exists(tfFileName)) {
        loadTfFromFile(tfFileName);
    } else {
        colorPoints = {
                sgl::ColorPoint_sRGB(sgl::Color(59, 76, 192), 0.0f),
                sgl::ColorPoint_sRGB(sgl::Color(144, 178, 254), 0.25f),
                sgl::ColorPoint_sRGB(sgl::Color(220, 220, 220), 0.5f),
                sgl::ColorPoint_sRGB(sgl::Color(245, 156, 125), 0.75f),
                sgl::ColorPoint_sRGB(sgl::Color(180, 4, 38), 1.0f)
        };
        opacityPoints = { sgl::OpacityPoint(1.0f, 0.0f), sgl::OpacityPoint(1.0f, 1.0f) };
    }
}

bool GuiVarData::saveTfToFile(const std::string& filename) {
    FILE* file = fopen(filename.c_str(), "w");
    if (file == nullptr) {
        sgl::Logfile::get()->writeError(
                std::string() + "ERROR: MultiVarTransferFunctionWindow::saveFunctionToFile: Couldn't create file \""
                + filename + "\"!");
        return false;
    }

    XMLPrinter printer(file);
    printer.OpenElement("TransferFunction");
    printer.PushAttribute("colorspace", "sRGB"); // Currently only sRGB supported for points
    printer.PushAttribute("interpolation_colorspace", sgl::COLOR_SPACE_NAMES[interpolationColorSpace]);

    printer.OpenElement("OpacityPoints");
    // Traverse all opacity points
    for (size_t i = 0; i < opacityPoints.size(); i++) {
        printer.OpenElement("OpacityPoint");
        printer.PushAttribute("position", opacityPoints.at(i).position);
        printer.PushAttribute("opacity", opacityPoints.at(i).opacity);
        printer.CloseElement();
    }
    printer.CloseElement();

    printer.OpenElement("ColorPoints");
    printer.PushAttribute("color_data", sgl::COLOR_DATA_MODE_NAMES[int(sgl::COLOR_DATA_MODE_UNSIGNED_SHORT)]);
    // Traverse all color points
    for (size_t i = 0; i < colorPoints.size(); i++) {
        printer.OpenElement("ColorPoint");
        printer.PushAttribute("position", colorPoints.at(i).position);
        printer.PushAttribute("r", (int)colorPoints.at(i).color.getR());
        printer.PushAttribute("g", (int)colorPoints.at(i).color.getG());
        printer.PushAttribute("b", (int)colorPoints.at(i).color.getB());
        printer.CloseElement();
    }
    printer.CloseElement();

    printer.CloseElement();

    fclose(file);
    return true;
}

bool GuiVarData::loadTfFromFile(const std::string& filename) {
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != 0) {
        sgl::Logfile::get()->writeError(
                std::string() + "MultiVarTransferFunctionWindow::loadFunctionFromFile: Couldn't open file \""
                + filename + "\"!");
        return false;
    }
    XMLElement* tfNode = doc.FirstChildElement("TransferFunction");
    if (tfNode == nullptr) {
        sgl::Logfile::get()->writeError(
                "MultiVarTransferFunctionWindow::loadFunctionFromFile: No \"TransferFunction\" node found!");
        return false;
    }

    interpolationColorSpace = sgl::COLOR_SPACE_SRGB; // Standard
    const char* interpolationColorSpaceName = tfNode->Attribute("colorspace_interpolation");
    if (interpolationColorSpaceName != nullptr) {
        for (int i = 0; i < 2; i++) {
            if (strcmp(interpolationColorSpaceName, sgl::COLOR_SPACE_NAMES[interpolationColorSpace]) == 0) {
                interpolationColorSpace = (sgl::ColorSpace)i;
            }
        }
    }

    colorPoints.clear();
    opacityPoints.clear();

    // Traverse all opacity points
    auto opacityPointsNode = tfNode->FirstChildElement("OpacityPoints");
    if (opacityPointsNode != nullptr) {
        for (sgl::XMLIterator it(opacityPointsNode, sgl::XMLNameFilter("OpacityPoint")); it.isValid(); ++it) {
            XMLElement* childElement = *it;
            float position = childElement->FloatAttribute("position");
            float opacity = sgl::clamp(childElement->FloatAttribute("opacity"), 0.0f, 1.0f);
            opacityPoints.emplace_back(opacity, position);
        }
    }

    // Traverse all color points
    auto colorPointsNode = tfNode->FirstChildElement("ColorPoints");
    if (colorPointsNode != nullptr) {
        sgl::ColorDataMode colorDataMode = sgl::COLOR_DATA_MODE_UNSIGNED_BYTE;
        const char* colorDataModeName = colorPointsNode->Attribute("color_data");
        if (colorDataModeName != nullptr) {
            colorDataMode = sgl::parseColorDataModeName(colorDataModeName);
        }
        for (sgl::XMLIterator it(colorPointsNode, sgl::XMLNameFilter("ColorPoint")); it.isValid(); ++it) {
            XMLElement* childElement = *it;
            sgl::Color16 color;
            float position = childElement->FloatAttribute("position");
            if (colorDataMode == sgl::COLOR_DATA_MODE_UNSIGNED_BYTE) {
                int red = sgl::clamp(childElement->IntAttribute("r"), 0, 255);
                int green = sgl::clamp(childElement->IntAttribute("g"), 0, 255);
                int blue = sgl::clamp(childElement->IntAttribute("b"), 0, 255);
                color = sgl::Color(red, green, blue);
            } else if (colorDataMode == sgl::COLOR_DATA_MODE_UNSIGNED_SHORT) {
                int red = sgl::clamp(childElement->IntAttribute("r"), 0, 65535);
                int green = sgl::clamp(childElement->IntAttribute("g"), 0, 65535);
                int blue = sgl::clamp(childElement->IntAttribute("b"), 0, 65535);
                color = sgl::Color16(red, green, blue);
            } else if (colorDataMode == sgl::COLOR_DATA_MODE_FLOAT_NORMALIZED) {
                float red = sgl::clamp(childElement->FloatAttribute("r"), 0.0f, 1.0f);
                float green = sgl::clamp(childElement->FloatAttribute("g"), 0.0f, 1.0f);
                float blue = sgl::clamp(childElement->FloatAttribute("b"), 0.0f, 1.0f);
                color = sgl::Color16(glm::vec3(red, green, blue));
            } else if (colorDataMode == sgl::COLOR_DATA_MODE_FLOAT_255) {
                float red = sgl::clamp(childElement->FloatAttribute("r"), 0.0f, 255.0f) / 255.0f;
                float green = sgl::clamp(childElement->FloatAttribute("g"), 0.0f, 255.0f) / 255.0f;
                float blue = sgl::clamp(childElement->FloatAttribute("b"), 0.0f, 255.0f) / 255.0f;
                color = sgl::Color16(glm::vec3(red, green, blue));
            }
            colorPoints.emplace_back(color, position);
        }
    }

    selectedPointType = sgl::SELECTED_POINT_TYPE_NONE;
    rebuildTransferFunctionMap();
    return true;
}

void GuiVarData::setAttributeValues(const std::string& name, const std::vector<float>& attributes) {
    attributeName = name;
    this->attributes = attributes;

    float minAttr = std::numeric_limits<float>::max();
    float maxAttr = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
    #pragma omp parallel for reduction(min: minAttr) reduction(max: maxAttr) shared(attributes) default(none)
#endif
    for (size_t i = 0; i < attributes.size(); i++) {
        float value = attributes.at(i);
        minAttr = std::min(minAttr, value);
        maxAttr = std::max(maxAttr, value);
    }
    this->dataRange = glm::vec2(minAttr, maxAttr);
    this->selectedRange = glm::vec2(minAttr, maxAttr);
    computeHistogram();
}

void GuiVarData::computeHistogram() {
    float histogramsMax = 0;
    histogram = std::vector<float>(histogramResolution, 0.0f);

    for (const float& value : attributes) {
        int32_t index = glm::clamp(
                static_cast<int>((value - selectedRange.x) / (selectedRange.y - selectedRange.x)
                * static_cast<float>(histogramResolution)), 0, histogramResolution - 1);
        histogram.at(index)++;
    }

    // Normalize values of histogram.
    for (const float& binNumber : histogram) {
        histogramsMax = std::max(histogramsMax, binNumber);
    }

    for (float& numBin : histogram) {
        numBin /= histogramsMax;
    }
}

// For OpenGL: Has TRANSFER_FUNCTION_TEXTURE_SIZE entries.
// Get mapped color for normalized attribute by accessing entry at "attr*255".
void GuiVarData::rebuildTransferFunctionMap() {
    rebuildTransferFunctionMapLocal();
    window->rebuildTransferFunctionMap();
}

void GuiVarData::rebuildTransferFunctionMapLocal() {
    // Create linear RGB color points
    colorPoints_LinearRGB.clear();
    for (sgl::ColorPoint_sRGB& colorPoint : colorPoints) {
        glm::vec3 linearRGBColor = sgl::TransferFunctionWindow::sRGBToLinearRGB(colorPoint.color.getFloatColorRGB());
        colorPoints_LinearRGB.push_back(sgl::ColorPoint_LinearRGB(linearRGBColor, colorPoint.position));
    }

    if (interpolationColorSpace == sgl::COLOR_SPACE_LINEAR_RGB) {
        rebuildTransferFunctionMap_LinearRGB();
    } else {
        rebuildTransferFunctionMap_sRGB();
    }
}

// For OpenGL: Has 256 entries. Get mapped color for normalized attribute by accessing entry at "attr*255".
void GuiVarData::rebuildTransferFunctionMap_LinearRGB() {
    int colorPointsIdx = 0;
    int opacityPointsIdx = 0;
    for (size_t i = 0; i < TRANSFER_FUNCTION_TEXTURE_SIZE; i++) {
        glm::vec3 linearRGBColorAtIdx;
        float opacityAtIdx;
        float currentPosition = static_cast<float>(i) / float(TRANSFER_FUNCTION_TEXTURE_SIZE-1);

        // colorPoints.at(colorPointsIdx) should be to the right of/equal to currentPosition
        while (colorPoints_LinearRGB.at(colorPointsIdx).position < currentPosition) {
            colorPointsIdx++;
        }
        while (opacityPoints.at(opacityPointsIdx).position < currentPosition) {
            opacityPointsIdx++;
        }

        // Now compute the color...
        if (colorPoints_LinearRGB.at(colorPointsIdx).position == currentPosition) {
            linearRGBColorAtIdx = colorPoints_LinearRGB.at(colorPointsIdx).color;
        } else {
            glm::vec3 color0 = colorPoints_LinearRGB.at(colorPointsIdx-1).color;
            glm::vec3 color1 = colorPoints_LinearRGB.at(colorPointsIdx).color;
            float pos0 = colorPoints_LinearRGB.at(colorPointsIdx-1).position;
            float pos1 = colorPoints_LinearRGB.at(colorPointsIdx).position;
            float factor = 1.0f - (pos1 - currentPosition) / (pos1 - pos0);
            linearRGBColorAtIdx = glm::mix(color0, color1, factor);
        }

        // ... and the opacity.
        if (opacityPoints.at(opacityPointsIdx).position == currentPosition) {
            opacityAtIdx = opacityPoints.at(opacityPointsIdx).opacity;
        } else {
            float opacity0 = opacityPoints.at(opacityPointsIdx-1).opacity;
            float opacity1 = opacityPoints.at(opacityPointsIdx).opacity;
            float pos0 = opacityPoints.at(opacityPointsIdx-1).position;
            float pos1 = opacityPoints.at(opacityPointsIdx).position;
            float factor = 1.0f - (pos1 - currentPosition) / (pos1 - pos0);
            opacityAtIdx = sgl::interpolateLinear(opacity0, opacity1, factor);
        }

        transferFunctionMap_linearRGB[i] = sgl::Color16(glm::vec4(linearRGBColorAtIdx, opacityAtIdx));
        transferFunctionMap_sRGB[i] = sgl::Color16(glm::vec4(
                sgl::TransferFunctionWindow::linearRGBTosRGB(linearRGBColorAtIdx), opacityAtIdx));
    }
}

// For OpenGL: Has 256 entries. Get mapped color for normalized attribute by accessing entry at "attr*255".
void GuiVarData::rebuildTransferFunctionMap_sRGB() {
    int colorPointsIdx = 0;
    int opacityPointsIdx = 0;
    for (size_t i = 0; i < TRANSFER_FUNCTION_TEXTURE_SIZE; i++) {
        glm::vec3 sRGBColorAtIdx;
        float opacityAtIdx;
        float currentPosition = static_cast<float>(i) / float(TRANSFER_FUNCTION_TEXTURE_SIZE-1);

        // colorPoints.at(colorPointsIdx) should be to the right of/equal to currentPosition
        while (colorPoints.at(colorPointsIdx).position < currentPosition) {
            colorPointsIdx++;
        }
        while (opacityPoints.at(opacityPointsIdx).position < currentPosition) {
            opacityPointsIdx++;
        }

        // Now compute the color...
        if (colorPoints.at(colorPointsIdx).position == currentPosition) {
            sRGBColorAtIdx = colorPoints.at(colorPointsIdx).color.getFloatColorRGB();
        } else {
            glm::vec3 color0 = colorPoints.at(colorPointsIdx-1).color.getFloatColorRGB();
            glm::vec3 color1 = colorPoints.at(colorPointsIdx).color.getFloatColorRGB();
            float pos0 = colorPoints.at(colorPointsIdx-1).position;
            float pos1 = colorPoints.at(colorPointsIdx).position;
            float factor = 1.0f - (pos1 - currentPosition) / (pos1 - pos0);
            sRGBColorAtIdx = glm::mix(color0, color1, factor);
        }

        // ... and the opacity.
        if (opacityPoints.at(opacityPointsIdx).position == currentPosition) {
            opacityAtIdx = opacityPoints.at(opacityPointsIdx).opacity;
        } else {
            float opacity0 = opacityPoints.at(opacityPointsIdx-1).opacity;
            float opacity1 = opacityPoints.at(opacityPointsIdx).opacity;
            float pos0 = opacityPoints.at(opacityPointsIdx-1).position;
            float pos1 = opacityPoints.at(opacityPointsIdx).position;
            float factor = 1.0f - (pos1 - currentPosition) / (pos1 - pos0);
            opacityAtIdx = sgl::interpolateLinear(opacity0, opacity1, factor);
        }

        transferFunctionMap_linearRGB[i] = sgl::Color16(glm::vec4(
                sgl::TransferFunctionWindow::sRGBToLinearRGB(sRGBColorAtIdx), opacityAtIdx));
        transferFunctionMap_sRGB[i] = sgl::Color16(glm::vec4(sRGBColorAtIdx, opacityAtIdx));
    }
}

bool GuiVarData::renderGui() {
    renderOpacityGraph();
    renderColorBar();

    if (selectedPointType == sgl::SELECTED_POINT_TYPE_OPACITY) {
        if (ImGui::DragFloat("Opacity", &opacitySelection, 0.001f, 0.0f, 1.0f)) {
            opacityPoints.at(currentSelectionIndex).opacity = opacitySelection;
            rebuildTransferFunctionMap();
            reRender = true;
        }
    } else if (selectedPointType == sgl::SELECTED_POINT_TYPE_COLOR) {
        if (ImGui::ColorEdit3("Color", (float*)&colorSelection)) {
            colorPoints.at(currentSelectionIndex).color = sgl::color16FromFloat(
                    colorSelection.x, colorSelection.y, colorSelection.z, colorSelection.w);
            rebuildTransferFunctionMap();
            reRender = true;
        }
    }

    if (ImGui::Combo(
            "Color Space", (int*)&interpolationColorSpace,
            sgl::COLOR_SPACE_NAMES, IM_ARRAYSIZE(sgl::COLOR_SPACE_NAMES))) {
        rebuildTransferFunctionMap();
        reRender = true;
    }

    if (ImGui::SliderFloat2("Range", &selectedRange.x, dataRange.x, dataRange.y)) {
        computeHistogram();
        window->rebuildRangeSsbo();
        reRender = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        selectedRange = dataRange;
        computeHistogram();
        window->rebuildRangeSsbo();
        reRender = true;
    }

    if (ImGui::SliderInt("Histogram Res.", &histogramResolution, 1, 256)) {
        computeHistogram();
    }

    renderFileDialog();

    if (reRender) {
        reRender = false;
        return true;
    }
    return false;
}

void GuiVarData::renderFileDialog() {
    // Load file data
    if (ImGui::ListBox("##availablefiles", &selectedFileIndex, [this](void* data, int idx, const char** out_text) -> bool {
        *out_text = window->availableFiles.at(idx).c_str();
        return true;
    }, nullptr, int(window->availableFiles.size()), 4)) {
        saveFileString = window->availableFiles.at(selectedFileIndex);
    } ImVec2 cursorPosEnd = ImGui::GetCursorPos(); ImGui::SameLine();

    ImVec2 cursorPos = ImGui::GetCursorPos();
    ImGui::Text("Available files"); ImGui::SameLine(); ImGui::SetCursorPos(cursorPos + ImVec2(0.0f, 42.0f));
    if (ImGui::Button("Load file") && selectedFileIndex >= 0) {
        loadTfFromFile(window->saveDirectory + window->availableFiles.at(selectedFileIndex));
        reRender = true;
    } ImGui::SetCursorPos(cursorPosEnd);

    // Save file data
    ImGui::InputText("##savefilelabel", &saveFileString); ImGui::SameLine();
    if (ImGui::Button("Save file")) {
        saveTfToFile(window->saveDirectory + saveFileString);
        window->updateAvailableFiles();
    }
}

void GuiVarData::renderOpacityGraph() {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();
    float regionWidth = ImGui::GetContentRegionAvail().x;
    float graphHeight = 300 * scaleFactor / 1.875f;
    float border = 2*scaleFactor;
    float areaWidth = regionWidth - 2.0f*border;
    float areaHeight = graphHeight - 2.0f*border;
    opacityGraphBox.min = glm::vec2(ImGui::GetCursorScreenPos().x + border, ImGui::GetCursorScreenPos().y + border);
    opacityGraphBox.max = opacityGraphBox.min + glm::vec2(areaWidth, areaHeight);

    sgl::Color& clearColor = window->clearColor;
    ImColor backgroundColor(clearColor.getFloatR(), clearColor.getFloatG(), clearColor.getFloatB());
    ImColor borderColor(
            1.0f - clearColor.getFloatR(), 1.0f - clearColor.getFloatG(), 1.0f - clearColor.getFloatB());

    // First render the graph box
    ImVec2 startPos = ImGui::GetCursorScreenPos();
    ImVec2 cursorPosHistogram = ImGui::GetCursorPos();
    drawList->AddRectFilled(
            ImVec2(startPos.x, startPos.y),
            ImVec2(startPos.x + regionWidth, startPos.y + graphHeight),
            borderColor, ImGui::GetStyle().FrameRounding);
    drawList->AddRectFilled(
            ImVec2(startPos.x + border, startPos.y + border),
            ImVec2(startPos.x + regionWidth - border, startPos.y + graphHeight - border),
            backgroundColor, ImGui::GetStyle().FrameRounding);

    if (ImGui::ClickArea("##grapharea", ImVec2(regionWidth, graphHeight + 2), mouseReleased)) {
        onOpacityGraphClick();
    }
    //ImGui::SetItemAllowOverlap();
    ImGui::SetCursorPos(cursorPosHistogram + ImVec2(border, border));

    ImVec2 oldPadding = ImGui::GetStyle().FramePadding;
    ImGui::GetStyle().FramePadding = ImVec2(1, 1);
    ImGui::PlotHistogram(
            "##histogram", histogram.data(), int(histogram.size()), 0,
            nullptr, 0.0f, 1.0f,
            ImVec2(regionWidth - border * 2, graphHeight - border * 2));
    ImGui::GetStyle().FramePadding = oldPadding;

    // Then render the graph itself
    for (int i = 0; i < (int)opacityPoints.size()-1; i++) {
        float positionX0 = opacityPoints.at(i).position * areaWidth + border;
        float positionX1 = opacityPoints.at(i+1).position * areaWidth + border;
        float positionY0 = (1.0f - opacityPoints.at(i).opacity) * areaHeight + border;
        float positionY1 = (1.0f - opacityPoints.at(i+1).opacity) * areaHeight + border;
        drawList->AddLine(
                ImVec2(startPos.x + positionX0, startPos.y + positionY0),
                ImVec2(startPos.x + positionX1, startPos.y + positionY1),
                borderColor, 1.5f * scaleFactor);
    }

    // Finally, render the points
    for (int i = 0; i < (int)opacityPoints.size(); i++) {
        ImVec2 centerPt = ImVec2(
                startPos.x + border + opacityPoints.at(i).position * areaWidth,
                startPos.y + border + (1.0f - opacityPoints.at(i).opacity) * areaHeight);
        float radius = 4*scaleFactor;
        if (selectedPointType == sgl::SELECTED_POINT_TYPE_OPACITY && i == currentSelectionIndex) {
            radius = 6*scaleFactor;
        }
        drawList->AddCircleFilled(centerPt, radius, backgroundColor, 24);
        drawList->AddCircle(centerPt, radius, borderColor, 24, 1.5f);
    }
}

void GuiVarData::renderColorBar() {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();
    float regionWidth = ImGui::GetContentRegionAvail().x;
    float barHeight = 30 * scaleFactor / 1.875f;
    colorBarBox.min = glm::vec2(ImGui::GetCursorScreenPos().x + 1, ImGui::GetCursorScreenPos().y + 1);
    colorBarBox.max = colorBarBox.min + glm::vec2(regionWidth - 2, barHeight - 2);

    // Draw bar
    ImVec2 startPos = ImGui::GetCursorScreenPos();
    ImVec2 pos = ImVec2(startPos.x + 1, startPos.y + 1);
    for (size_t i = 0; i < TRANSFER_FUNCTION_TEXTURE_SIZE; i++) {
        sgl::Color16 color = transferFunctionMap_sRGB[i];
        ImU32 colorImgui = ImColor(color.getFloatR(), color.getFloatG(), color.getFloatB());
        drawList->AddLine(ImVec2(pos.x, pos.y), ImVec2(pos.x, pos.y + barHeight), colorImgui, 2.0f * regionWidth / 255.0f);
        pos.x += regionWidth / 255.0f;
    }

    // Draw points
    pos = ImVec2(startPos.x + 2, startPos.y + 2);
    for (int i = 0; i < (int)colorPoints.size(); i++) {
        sgl::Color16 color = colorPoints.at(i).color;
        ImU32 colorImgui = ImColor(color.getFloatR(), color.getFloatG(), color.getFloatB());
        ImU32 colorInvertedImgui = ImColor(1.0f - color.getFloatR(), 1.0f - color.getFloatG(), 1.0f - color.getFloatB());
        ImVec2 centerPt = ImVec2(pos.x + colorPoints.at(i).position * regionWidth, pos.y + barHeight/2);
        float radius = 4*scaleFactor;
        if (selectedPointType == sgl::SELECTED_POINT_TYPE_COLOR && i == currentSelectionIndex) {
            radius = 6*scaleFactor;
        }
        drawList->AddCircleFilled(centerPt, radius, colorImgui, 24);
        drawList->AddCircle(centerPt, radius, colorInvertedImgui, 24);
    }

    if (ImGui::ClickArea("##bararea", ImVec2(regionWidth + 2, barHeight), mouseReleased)) {
        onColorBarClick();
    }
}

void GuiVarData::onOpacityGraphClick() {
    glm::vec2 mousePosWidget = glm::vec2(ImGui::GetMousePos().x, ImGui::GetMousePos().y) - opacityGraphBox.min;

    glm::vec2 normalizedPosition = mousePosWidget / opacityGraphBox.getDimensions();
    normalizedPosition.y = 1.0f - normalizedPosition.y;
    normalizedPosition = glm::clamp(normalizedPosition, glm::vec2(0), glm::vec2(1));
    dragging = false;

    if (selectNearestOpacityPoint(currentSelectionIndex, mousePosWidget)) {
        // A) Point near to normalized position
        if (ImGui::GetIO().MouseClicked[0]) {
            // A.1 Left clicked? Select/drag-and-drop
            opacitySelection = opacityPoints.at(currentSelectionIndex).opacity;
            selectedPointType = sgl::SELECTED_POINT_TYPE_OPACITY;
            dragging = true;
        } else if (ImGui::GetIO().MouseClicked[1] && currentSelectionIndex != 0
                   && currentSelectionIndex != int(opacityPoints.size())-1) {
            // A.2 Middle clicked? Delete point
            opacityPoints.erase(opacityPoints.begin() + currentSelectionIndex);
            selectedPointType = sgl::SELECTED_POINT_TYPE_NONE;
            reRender = true;
        }
    } else {
        // B) If no point near and left clicked: Create new point at position
        if (ImGui::GetIO().MouseClicked[0]) {
            // Compute insert position for new point
            int insertPosition = 0;
            for (insertPosition = 0; insertPosition < (int)opacityPoints.size(); insertPosition++) {
                if (normalizedPosition.x < opacityPoints.at(insertPosition).position
                    || insertPosition == int(opacityPoints.size())-1) {
                    break;
                }
            }

            // Add new opacity point
            glm::vec2 newPosition = normalizedPosition;
            float newOpacity = newPosition.y;
            opacityPoints.insert(opacityPoints.begin() + insertPosition, sgl::OpacityPoint(newOpacity, newPosition.x));
            currentSelectionIndex = insertPosition;
            opacitySelection = opacityPoints.at(currentSelectionIndex).opacity;
            selectedPointType = sgl::SELECTED_POINT_TYPE_OPACITY;
            dragging = true;
            reRender = true;
        }
    }

    rebuildTransferFunctionMap();
}

void GuiVarData::onColorBarClick() {
    glm::vec2 mousePosWidget = glm::vec2(ImGui::GetMousePos().x, ImGui::GetMousePos().y) - colorBarBox.min;
    float normalizedPosition = mousePosWidget.x / colorBarBox.getWidth();
    dragging = false;

    if (selectNearestColorPoint(currentSelectionIndex, mousePosWidget)) {
        // A) Point near to normalized position
        if (ImGui::GetIO().MouseClicked[0]) {
            // A.1 Left clicked? Select/drag-and-drop
            sgl::Color16& color16 = colorPoints.at(currentSelectionIndex).color;
            colorSelection = ImColor(color16.getFloatR(), color16.getFloatG(), color16.getFloatB());
            selectedPointType = sgl::SELECTED_POINT_TYPE_COLOR;
            if (currentSelectionIndex != 0 && currentSelectionIndex != int(colorPoints.size())-1) {
                dragging = true;
            }
        } else if (ImGui::GetIO().MouseClicked[1] && currentSelectionIndex != 0
                   && currentSelectionIndex != int(colorPoints.size())-1) {
            // A.2 Middle clicked? Delete point
            colorPoints.erase(colorPoints.begin() + currentSelectionIndex);
            colorPoints_LinearRGB.erase(colorPoints_LinearRGB.begin() + currentSelectionIndex);
            selectedPointType = sgl::SELECTED_POINT_TYPE_NONE;
            reRender = true;
        }
    } else {
        // B) If no point near and left clicked: Create new point at position
        if (ImGui::GetIO().MouseClicked[0]) {
            // Compute insert position for new point
            int insertPosition = 0;
            for (insertPosition = 0; insertPosition < (int)colorPoints.size(); insertPosition++) {
                if (normalizedPosition < colorPoints.at(insertPosition).position
                    || insertPosition == int(colorPoints.size())-1) {
                    break;
                }
            }

            // Add new color point
            float newPosition = normalizedPosition;
            if (interpolationColorSpace == sgl::COLOR_SPACE_LINEAR_RGB) {
                // Linear RGB interplation
                glm::vec3 newColor_linearRGB = glm::mix(
                        colorPoints_LinearRGB.at(insertPosition-1).color,
                        colorPoints_LinearRGB.at(insertPosition).color,
                        1.0 - (colorPoints_LinearRGB.at(insertPosition).position - newPosition)
                              / (colorPoints_LinearRGB.at(insertPosition).position
                                 - colorPoints_LinearRGB.at(insertPosition-1).position));
                sgl::Color16 newColorsRGB(sgl::TransferFunctionWindow::linearRGBTosRGB(newColor_linearRGB));
                colorPoints_LinearRGB.insert(
                        colorPoints_LinearRGB.begin() + insertPosition,
                        sgl::ColorPoint_LinearRGB(newColor_linearRGB, newPosition));
                colorPoints.insert(
                        colorPoints.begin() + insertPosition,
                        sgl::ColorPoint_sRGB(newColorsRGB, newPosition));
            } else {
                // sRGB interpolation
                sgl::Color16 newColor = sgl::color16Lerp(
                        colorPoints.at(insertPosition-1).color,
                        colorPoints.at(insertPosition).color,
                        1.0f - (colorPoints.at(insertPosition).position - newPosition)
                              / (colorPoints.at(insertPosition).position - colorPoints.at(insertPosition-1).position));
                colorPoints.insert(
                        colorPoints.begin() + insertPosition,
                        sgl::ColorPoint_sRGB(newColor, newPosition));
                // colorPoints_LinearRGB computed in @ref rebuildTransferFunctionMap
            }
            currentSelectionIndex = insertPosition;
            sgl::Color16& color16 = colorPoints.at(currentSelectionIndex).color;
            colorSelection = ImColor(color16.getFloatR(), color16.getFloatG(), color16.getFloatB());
            selectedPointType = sgl::SELECTED_POINT_TYPE_COLOR;
            reRender = true;
        }
    }

    rebuildTransferFunctionMap();
}

void GuiVarData::dragPoint() {
    if (mouseReleased) {
        dragging = false;
    }

    glm::vec2 mousePosWidget = glm::vec2(ImGui::GetMousePos().x, ImGui::GetMousePos().y) - opacityGraphBox.min;
    if (!dragging || mousePosWidget == oldMousePosWidget) {
        oldMousePosWidget = mousePosWidget;
        return;
    }
    oldMousePosWidget = mousePosWidget;

    if (selectedPointType == sgl::SELECTED_POINT_TYPE_OPACITY) {
        glm::vec2 normalizedPosition = mousePosWidget / opacityGraphBox.getDimensions();
        normalizedPosition.y = 1.0f - normalizedPosition.y;
        normalizedPosition = glm::clamp(normalizedPosition, 0.0f, 1.0f);
        if (currentSelectionIndex == 0) {
            normalizedPosition.x = 0.0f;
        }
        if (currentSelectionIndex == int(opacityPoints.size())-1) {
            normalizedPosition.x = 1.0f;
        }
        // Clip to neighbors!
        if (currentSelectionIndex != 0
            && normalizedPosition.x < opacityPoints.at(currentSelectionIndex-1).position) {
            normalizedPosition.x = opacityPoints.at(currentSelectionIndex-1).position;
        }
        if (currentSelectionIndex != int(opacityPoints.size())-1
            && normalizedPosition.x > opacityPoints.at(currentSelectionIndex+1).position) {
            normalizedPosition.x = opacityPoints.at(currentSelectionIndex+1).position;
        }
        opacityPoints.at(currentSelectionIndex).position = normalizedPosition.x;
        opacityPoints.at(currentSelectionIndex).opacity = normalizedPosition.y;
        opacitySelection = opacityPoints.at(currentSelectionIndex).opacity;
    }

    if (selectedPointType == sgl::SELECTED_POINT_TYPE_COLOR) {
        float normalizedPosition = mousePosWidget.x / colorBarBox.getWidth();
        normalizedPosition = glm::clamp(normalizedPosition, 0.0f, 1.0f);
        // Clip to neighbors!
        if (currentSelectionIndex != 0
            && normalizedPosition < colorPoints.at(currentSelectionIndex-1).position) {
            normalizedPosition = colorPoints.at(currentSelectionIndex-1).position;
        }
        if (currentSelectionIndex != int(colorPoints.size())-1
            && normalizedPosition > colorPoints.at(currentSelectionIndex+1).position) {
            normalizedPosition = colorPoints.at(currentSelectionIndex+1).position;
        }
        colorPoints.at(currentSelectionIndex).position = normalizedPosition;
    }

    rebuildTransferFunctionMap();
    reRender = true;
}

bool GuiVarData::selectNearestOpacityPoint(int& currentSelectionIndex, const glm::vec2& mousePosWidget) {
    float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();

    for (int i = 0; i < (int)opacityPoints.size(); i++) {
        glm::vec2 centerPt = glm::vec2(
                opacityPoints.at(i).position * opacityGraphBox.getWidth(),
                (1.0f - opacityPoints.at(i).opacity) * opacityGraphBox.getHeight());
        if (glm::length(centerPt - mousePosWidget) < scaleFactor * 10.0f) {
            currentSelectionIndex = i;
            return true;
        }
    }
    return false;
}

bool GuiVarData::selectNearestColorPoint(int& currentSelectionIndex, const glm::vec2& mousePosWidget) {
    float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();

    for (int i = 0; i < (int)colorPoints.size(); i++) {
        ImVec2 centerPt = ImVec2(
                colorPoints.at(i).position * colorBarBox.getWidth(),
                colorBarBox.getHeight()/2);
        if (glm::abs(centerPt.x - mousePosWidget.x) < scaleFactor * 10.0f) {
            currentSelectionIndex = i;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

MultiVarTransferFunctionWindow::MultiVarTransferFunctionWindow(
        const std::string& saveDirectoryPrefix,
        const std::vector<std::string>& tfPresetFiles) {
    parentDirectory = sgl::AppSettings::get()->getDataDirectory();
    saveDirectory = sgl::AppSettings::get()->getDataDirectory() + "TransferFunctions/";

    if (!saveDirectoryPrefix.empty()) {
        directoryName = saveDirectoryPrefix;
        parentDirectory = saveDirectory;
        saveDirectory = saveDirectory + saveDirectoryPrefix + "/";
        sgl::FileUtils::get()->ensureDirectoryExists(saveDirectory);
    }
    this->tfPresetFiles = tfPresetFiles;

    directoryContentWatch.setPath(saveDirectory, true);
    directoryContentWatch.initialize();

    tfMapImageSettingsVulkan.imageType = VK_IMAGE_TYPE_1D;
    tfMapImageSettingsVulkan.format = VK_FORMAT_R16G16B16A16_UNORM;

    updateAvailableFiles();
}

MultiVarTransferFunctionWindow::~MultiVarTransferFunctionWindow() {
}

void MultiVarTransferFunctionWindow::setAttributesValues(
        const std::vector<std::string>& names,
        const std::vector<std::vector<float>>& allAttributes) {
    assert(names.size() == allAttributes.size());
    varNames = names;
    transferFunctionMap_sRGB.resize(TRANSFER_FUNCTION_TEXTURE_SIZE * names.size());
    transferFunctionMap_linearRGB.resize(TRANSFER_FUNCTION_TEXTURE_SIZE * names.size());
    selectedVarIndex = 0;

    if (guiVarData.size() != names.size()){
        guiVarData.clear();
        guiVarData.reserve(names.size());

        tfMapImageSettingsVulkan.width = TRANSFER_FUNCTION_TEXTURE_SIZE;
        tfMapImageSettingsVulkan.arrayLayers = uint32_t(names.size());
        tfMapTextureVulkan = std::make_shared<sgl::vk::Texture>(
                sgl::AppSettings::get()->getPrimaryDevice(), tfMapImageSettingsVulkan,
                VK_IMAGE_VIEW_TYPE_1D_ARRAY);
        minMaxSsboVulkan = std::make_shared<sgl::vk::Buffer>(
                sgl::AppSettings::get()->getPrimaryDevice(), names.size() * sizeof(glm::vec2),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);

        minMaxData.clear();
        minMaxData.resize(names.size() * 2, 0);

        for (size_t varIdx = 0; varIdx < names.size(); varIdx++) {
            guiVarData.push_back(GuiVarData(
                    this, tfPresetFiles.size() > 0 ? tfPresetFiles.at(varIdx % tfPresetFiles.size()) : "",
                    &transferFunctionMap_sRGB.at(TRANSFER_FUNCTION_TEXTURE_SIZE * varIdx),
                    &transferFunctionMap_linearRGB.at(TRANSFER_FUNCTION_TEXTURE_SIZE * varIdx)
            ));
        }
    }

    for (size_t varIdx = 0; varIdx < names.size(); varIdx++) {
        GuiVarData& varData = guiVarData.at(varIdx);
        varData.setAttributeValues(names.at(varIdx), allAttributes.at(varIdx));
    }
    if (!guiVarData.empty()) {
        currVarData = &guiVarData.at(selectedVarIndex);
    }

    updateAvailableFiles();
    rebuildTransferFunctionMapComplete();
    rebuildRangeSsbo();
}


bool MultiVarTransferFunctionWindow::loadFromTfNameList(const std::vector<std::string>& tfNames) {
    if (tfNames.size() != guiVarData.size()) {
        sgl::Logfile::get()->writeError(
                "MultiVarTransferFunctionWindow::loadFromTfNameList: tfNames.size() != guiVarData.size()");
        return false;
    }

    for (size_t varIdx = 0; varIdx < tfNames.size(); varIdx++) {
        GuiVarData& varData = guiVarData.at(varIdx);
        varData.loadTfFromFile(saveDirectory + tfNames.at(varIdx));
    }
    return true;
}

void MultiVarTransferFunctionWindow::updateAvailableFiles() {
    sgl::FileUtils::get()->ensureDirectoryExists(saveDirectory);
    std::vector<std::string> availableFilesAll = sgl::FileUtils::get()->getFilesInDirectoryVector(saveDirectory);
    availableFiles.clear();
    availableFiles.reserve(availableFilesAll.size());

    for (const std::string& filename : availableFilesAll) {
        if (sgl::FileUtils::get()->hasExtension(filename.c_str(), ".xml")) {
            availableFiles.push_back(filename);
        }
    }
    sgl::FileUtils::get()->sortPathStrings(availableFiles);

    // Update currently selected filename
    for (size_t i = 0; i < availableFiles.size(); i++) {
        availableFiles.at(i) = sgl::FileUtils::get()->getPureFilename(availableFiles.at(i));
        for (GuiVarData& varData : guiVarData) {
            if (availableFiles.at(i) == varData.getSaveFileString()) {
                varData.selectedFileIndex = (int)i;
            }
        }
    }
}

void MultiVarTransferFunctionWindow::setClearColor(const sgl::Color& clearColor) {
    this->clearColor = clearColor;
}

sgl::vk::TexturePtr& MultiVarTransferFunctionWindow::getTransferFunctionMapTextureVulkan() {
    return tfMapTextureVulkan;
}

bool MultiVarTransferFunctionWindow::getTransferFunctionMapRebuilt() {
    if (transferFunctionMapRebuilt) {
        // Reset the flag
        transferFunctionMapRebuilt = false;
        return true;
    }
    return false;
}

std::vector<sgl::Color16> MultiVarTransferFunctionWindow::getTransferFunctionMap_sRGB(int varIdx) {
    return std::vector<sgl::Color16>(
            transferFunctionMap_sRGB.cbegin() + TRANSFER_FUNCTION_TEXTURE_SIZE * varIdx,
            transferFunctionMap_sRGB.cbegin() + TRANSFER_FUNCTION_TEXTURE_SIZE * (varIdx + 1));
}

void MultiVarTransferFunctionWindow::update(float dt) {
    directoryContentWatch.update([this] { this->updateAvailableFiles(); });
    if (currVarData) {
        currVarData->dragPoint();
    }
}

void MultiVarTransferFunctionWindow::setUseLinearRGB(bool useLinearRGB) {
    this->useLinearRGB = useLinearRGB;
    rebuildTransferFunctionMapComplete();
}

void MultiVarTransferFunctionWindow::rebuildTransferFunctionMapComplete() {
    for (GuiVarData& varData : guiVarData) {
        varData.rebuildTransferFunctionMapLocal();
    }

    rebuildTransferFunctionMap();
}

void MultiVarTransferFunctionWindow::rebuildRangeSsbo() {
    if (!minMaxSsboVulkan) {
        return;
    }

    for (size_t varIdx = 0; varIdx < guiVarData.size(); varIdx++) {
        GuiVarData& varData = guiVarData.at(varIdx);
        const glm::vec2& range = varData.getSelectedRange();
        minMaxData[varIdx * 2] = range.x;
        minMaxData[varIdx * 2 + 1] = range.y;
    }
    minMaxSsboVulkan->uploadData(minMaxData.size() * sizeof(float), minMaxData.data());
}

void MultiVarTransferFunctionWindow::rebuildTransferFunctionMap() {
    transferFunctionMapRebuilt = true;

    if (!tfMapTextureVulkan) {
        return;
    }

    if (useLinearRGB) {
        tfMapTextureVulkan->getImage()->uploadData(
                TRANSFER_FUNCTION_TEXTURE_SIZE * uint32_t(varNames.size()) * 8,
                transferFunctionMap_linearRGB.data());
    } else {
        tfMapTextureVulkan->getImage()->uploadData(
                TRANSFER_FUNCTION_TEXTURE_SIZE * uint32_t(varNames.size()) * 8,
                transferFunctionMap_sRGB.data());
    }
}

bool MultiVarTransferFunctionWindow::renderGui() {
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(2, 1278, 634, 818);
    if (showWindow) {
        if (ImGui::Begin("Multi-Var Transfer Function", &showWindow)) {
            if (ImGui::BeginCombo("Variable", varNames.at(selectedVarIndex).c_str())) {
                for (size_t i = 0; i < varNames.size(); ++i) {
                    if (ImGui::Selectable(varNames.at(i).c_str(), selectedVarIndex == i)) {
                        selectedVarIndex = i;
                        if (!guiVarData.empty()) {
                            currVarData = &guiVarData.at(selectedVarIndex);
                        }
                    }
                }
                ImGui::EndCombo();
            }

            if (currVarData) {
                reRender = currVarData->renderGui() || reRender;
            }
        }
        ImGui::End();
    }

    if (reRender) {
        reRender = false;
        return true;
    }
    return false;
}
