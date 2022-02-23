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

#include <cmath>
#include <glm/glm.hpp>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <ImGui/imgui.h>
#include <ImGui/imgui_internal.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include <Math/Math.hpp>
#include <Utils/AppSettings.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "StressLineHierarchyMappingWidget.hpp"

const char* const stressDirectionNames[] = { "Major", "Medium", "Minor" };

StressLineHierarchyMappingWidget::StressLineHierarchyMappingWidget() {
    for (int psIdx = 0; psIdx < 3; psIdx++) {
        hierarchyMappingPoints[psIdx] = {
                HierarchyMappingPoint(0.0f, 0.0f),
                HierarchyMappingPoint(1.0f, 1.0f)
        };
    }

    sgl::vk::ImageSettings imageSettings;
    imageSettings.imageType = VK_IMAGE_TYPE_1D;
    imageSettings.format = VK_FORMAT_R32_SFLOAT;
    imageSettings.width = STANDARD_MAP_RESOLUTION;
    imageSettings.arrayLayers = 3;
    hierarchyMappingTexture = std::make_shared<sgl::vk::Texture>(
            sgl::AppSettings::get()->getPrimaryDevice(), imageSettings, VK_IMAGE_VIEW_TYPE_1D_ARRAY);

    rebuildHierarchyMappingTexture();
}

void StressLineHierarchyMappingWidget::setClearColor(const sgl::Color& clearColor) {
    this->clearColor = clearColor;
}

bool StressLineHierarchyMappingWidget::renderGui() {
    if (ImGui::Begin("Stress Line Hierarchy Mapping", &showWindow)) {
        for (int psIdx = 0; psIdx < 3; psIdx++) {
            std::string text = std::string() + stressDirectionNames[psIdx] + " direction:";
            ImGui::TextUnformatted(text.c_str());
            renderGraphArea(psIdx);
        }
        ImGui::End();
    }

    if (reRender) {
        reRender = false;
        return true;
    }

    return reRender;
}

void StressLineHierarchyMappingWidget::setLineHierarchyLevelValues(
        int psIdx, const std::vector<float>& lineHierarchyLevelValues) {
    histogram[psIdx].clear();
    histogram[psIdx].resize(histogramResolution);
    for (float attr : lineHierarchyLevelValues) {
        int index = glm::clamp(
                static_cast<int>(attr * (histogramResolution)), 0, histogramResolution - 1);
        histogram[psIdx].at(index) += 1;
    }

    float maxNum = 1.0f;
    for (float num : histogram[psIdx]) {
        maxNum = std::max(num, maxNum);
    }

    for (float& num : histogram[psIdx]) {
        num /= maxNum;
    }
}


sgl::vk::TexturePtr& StressLineHierarchyMappingWidget::getHierarchyMappingTexture() {
    return hierarchyMappingTexture;
}

bool StressLineHierarchyMappingWidget::getHierarchyMappingRebuilt() {
    if (hierarchyMappingRebuilt) {
        // Reset the flag
        hierarchyMappingRebuilt = false;
        return true;
    }
    return false;
}

void StressLineHierarchyMappingWidget::rebuildHierarchyMappingTexture() {
    std::vector<float> hierarchyMappingData;
    hierarchyMappingData.reserve(3 * STANDARD_MAP_RESOLUTION);
    for (int psIdx = 0; psIdx < 3; psIdx++) {
        int hierarchyMappingPointsIdx = 0;
        for (size_t i = 0; i < STANDARD_MAP_RESOLUTION; i++) {
            float hierarchyMappingAtIdx;
            float currentPosition = static_cast<float>(i) / float(STANDARD_MAP_RESOLUTION - 1);

            while (hierarchyMappingPoints[psIdx].at(hierarchyMappingPointsIdx).hierarchyLevel < currentPosition) {
                hierarchyMappingPointsIdx++;
            }

            if (hierarchyMappingPoints[psIdx].at(hierarchyMappingPointsIdx).hierarchyLevel == currentPosition) {
                hierarchyMappingAtIdx = hierarchyMappingPoints[psIdx].at(hierarchyMappingPointsIdx).mappedValue;
            } else {
                float hierarchyMapping0 = hierarchyMappingPoints[psIdx].at(hierarchyMappingPointsIdx - 1).mappedValue;
                float hierarchyMapping1 = hierarchyMappingPoints[psIdx].at(hierarchyMappingPointsIdx).mappedValue;
                float pos0 = hierarchyMappingPoints[psIdx].at(hierarchyMappingPointsIdx - 1).hierarchyLevel;
                float pos1 = hierarchyMappingPoints[psIdx].at(hierarchyMappingPointsIdx).hierarchyLevel;
                float factor = 1.0f - (pos1 - currentPosition) / (pos1 - pos0);
                hierarchyMappingAtIdx = sgl::interpolateLinear(hierarchyMapping0, hierarchyMapping1, factor);
            }

            hierarchyMappingData.push_back(hierarchyMappingAtIdx);
        }
    }

    hierarchyMappingTexture->getImage()->uploadData(
            STANDARD_MAP_RESOLUTION * 3 * sizeof(float), hierarchyMappingData.data());
}



// ------------------------------------------ Graph drag & drop logic ------------------------------------------

void StressLineHierarchyMappingWidget::update(float dt) {
    for (int psIdx = 0; psIdx < 3; psIdx++) {
        dragPoint(psIdx);
    }
}

void StressLineHierarchyMappingWidget::renderGraphArea(int psIdx) {
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();
    float regionWidth = ImGui::GetContentRegionAvail().x;
    float graphHeight = 200;
    float border = 2 * scaleFactor;
    float areaWidth = regionWidth - 2.0f * border;
    float areaHeight = graphHeight - 2.0f * border;
    graphBoxes[psIdx].min = glm::vec2(ImGui::GetCursorScreenPos().x + border, ImGui::GetCursorScreenPos().y + border);
    graphBoxes[psIdx].max = graphBoxes[psIdx].min + glm::vec2(areaWidth, areaHeight);

    ImColor backgroundColor(clearColor.getFloatR(), clearColor.getFloatG(), clearColor.getFloatB());
    ImColor borderColor(1.0f - clearColor.getFloatR(), 1.0f - clearColor.getFloatG(), 1.0f - clearColor.getFloatB());

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

    std::string areaId = std::string() + "##hmapgrapharea" + std::to_string(psIdx);
    if (ImGui::ClickArea(areaId.c_str(), ImVec2(regionWidth, graphHeight + 2), mouseReleased)) {
        onGraphClick(psIdx);
    }
    //ImGui::SetItemAllowOverlap();
    ImGui::SetCursorPos(cursorPosHistogram);

    std::string histogramId = std::string() + "##hmaphistogram" + std::to_string(psIdx);
    ImVec2 oldPadding = ImGui::GetStyle().FramePadding;
    ImGui::GetStyle().FramePadding = ImVec2(1, 1);
    ImGui::PlotHistogram(
            histogramId.c_str(), histogram[psIdx].data(), int(histogram[psIdx].size()),
            0, nullptr, 0.0f, 1.0f, ImVec2(regionWidth, graphHeight));
    ImGui::GetStyle().FramePadding = oldPadding;

    // Then render the graph itself
    for (int i = 0; i < (int)hierarchyMappingPoints[psIdx].size()-1; i++) {
        float positionX0 = hierarchyMappingPoints[psIdx].at(i).hierarchyLevel * areaWidth + border;
        float positionX1 = hierarchyMappingPoints[psIdx].at(i+1).hierarchyLevel * areaWidth + border;
        float positionY0 = (1.0f - hierarchyMappingPoints[psIdx].at(i).mappedValue) * areaHeight + border;
        float positionY1 = (1.0f - hierarchyMappingPoints[psIdx].at(i+1).mappedValue) * areaHeight + border;
        drawList->AddLine(
                ImVec2(startPos.x + positionX0, startPos.y + positionY0),
                ImVec2(startPos.x + positionX1, startPos.y + positionY1),
                borderColor, 1.5f * scaleFactor);
    }

    // Finally, render the points
    for (int i = 0; i < (int)hierarchyMappingPoints[psIdx].size(); i++) {
        ImVec2 centerPt = ImVec2(
                startPos.x + border + hierarchyMappingPoints[psIdx].at(i).hierarchyLevel * areaWidth,
                startPos.y + border + (1.0f - hierarchyMappingPoints[psIdx].at(i).mappedValue) * areaHeight);
        float radius = 4*scaleFactor;
        if (selectedPointGraphIndex == psIdx && i == currentSelectionIndex) {
            radius = 6*scaleFactor;
        }
        drawList->AddCircleFilled(centerPt, radius, backgroundColor, 24);
        drawList->AddCircle(centerPt, radius, borderColor, 24, 1.5f);
    }
}

void StressLineHierarchyMappingWidget::onGraphClick(int psIdx) {
    glm::vec2 mousePosWidget = glm::vec2(ImGui::GetMousePos().x, ImGui::GetMousePos().y) - graphBoxes[psIdx].min;

    glm::vec2 normalizedPosition = mousePosWidget / graphBoxes[psIdx].getDimensions();
    normalizedPosition.y = 1.0f - normalizedPosition.y;
    normalizedPosition = glm::clamp(normalizedPosition, glm::vec2(0), glm::vec2(1));
    dragging = false;

    if (selectNearestHierarchyMappingPoint(psIdx, currentSelectionIndex, mousePosWidget)) {
        // A) Point near to normalized position
        if (ImGui::GetIO().MouseClicked[0]) {
            // A.1 Left clicked? Select/drag-and-drop
            selectedPointGraphIndex = psIdx;
            dragging = true;
        } else if (ImGui::GetIO().MouseClicked[1] && currentSelectionIndex != 0
                   && currentSelectionIndex != int(hierarchyMappingPoints[psIdx].size())-1) {
            // A.2 Middle clicked? Delete point
            hierarchyMappingPoints[psIdx].erase(hierarchyMappingPoints[psIdx].begin() + currentSelectionIndex);
            selectedPointGraphIndex = -1;
            reRender = true;
        }
    } else {
        // B) If no point near and left clicked: Create new point at position
        if (ImGui::GetIO().MouseClicked[0]) {
            // Compute insert position for new point
            int insertPosition = 0;
            for (insertPosition = 0; insertPosition < (int)hierarchyMappingPoints[psIdx].size(); insertPosition++) {
                if (normalizedPosition.x < hierarchyMappingPoints[psIdx].at(insertPosition).hierarchyLevel
                    || insertPosition == int(hierarchyMappingPoints[psIdx].size())-1) {
                    break;
                }
            }

            // Add new opacity point
            glm::vec2 newPosition = normalizedPosition;
            float newOpacity = newPosition.y;
            hierarchyMappingPoints[psIdx].insert(
                    hierarchyMappingPoints[psIdx].begin() + insertPosition,
                    HierarchyMappingPoint(newOpacity, newPosition.x));
            currentSelectionIndex = insertPosition;
            selectedPointGraphIndex = psIdx;
            dragging = true;
            reRender = true;
        }
    }

    rebuildHierarchyMappingTexture();
}

void StressLineHierarchyMappingWidget::dragPoint(int psIdx) {
    if (mouseReleased) {
        dragging = false;
    }

    glm::vec2 mousePosWidget = glm::vec2(ImGui::GetMousePos().x, ImGui::GetMousePos().y) - graphBoxes[psIdx].min;
    if (!dragging || mousePosWidget == oldMousePosWidget) {
        oldMousePosWidget = mousePosWidget;
        return;
    }
    oldMousePosWidget = mousePosWidget;

    if (selectedPointGraphIndex == psIdx) {
        glm::vec2 normalizedPosition = mousePosWidget / graphBoxes[psIdx].getDimensions();
        normalizedPosition.y = 1.0f - normalizedPosition.y;
        normalizedPosition = glm::clamp(normalizedPosition, 0.0f, 1.0f);
        if (currentSelectionIndex == 0) {
            normalizedPosition.x = 0.0f;
        }
        if (currentSelectionIndex == int(hierarchyMappingPoints[psIdx].size())-1) {
            normalizedPosition.x = 1.0f;
        }
        // Clip to neighbors!
        if (currentSelectionIndex != 0
            && normalizedPosition.x < hierarchyMappingPoints[psIdx].at(currentSelectionIndex-1).hierarchyLevel) {
            normalizedPosition.x = hierarchyMappingPoints[psIdx].at(currentSelectionIndex-1).hierarchyLevel;
        }
        if (currentSelectionIndex != int(hierarchyMappingPoints[psIdx].size())-1
            && normalizedPosition.x > hierarchyMappingPoints[psIdx].at(currentSelectionIndex+1).hierarchyLevel) {
            normalizedPosition.x = hierarchyMappingPoints[psIdx].at(currentSelectionIndex+1).hierarchyLevel;
        }
        hierarchyMappingPoints[psIdx].at(currentSelectionIndex).hierarchyLevel = normalizedPosition.x;
        hierarchyMappingPoints[psIdx].at(currentSelectionIndex).mappedValue = normalizedPosition.y;
    }


    rebuildHierarchyMappingTexture();
    reRender = true;
}

bool StressLineHierarchyMappingWidget::selectNearestHierarchyMappingPoint(
        int psIdx, int& currentSelectionIndex, const glm::vec2& mousePosWidget) {
    float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();

    for (int i = 0; i < (int)hierarchyMappingPoints[psIdx].size(); i++) {
        glm::vec2 centerPt = glm::vec2(
                hierarchyMappingPoints[psIdx].at(i).hierarchyLevel * graphBoxes[psIdx].getWidth(),
                (1.0f - hierarchyMappingPoints[psIdx].at(i).mappedValue) * graphBoxes[psIdx].getHeight());
        if (glm::length(centerPt - mousePosWidget) < scaleFactor * 10.0f) {
            currentSelectionIndex = i;
            return true;
        }
    }
    return false;
}
