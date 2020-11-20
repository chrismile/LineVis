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

#include <ImGui/ImGuiWrapper.hpp>

#include "ImGuiVerticalText.hpp"
#include "ColorLegendWidget.hpp"

ColorLegendWidget::ColorLegendWidget() {
    transferFunctionColorMap.reserve(256);
    for (int i = 0; i < 256; i++) {
        float pct = float(i) / float(255);
        transferFunctionColorMap.push_back(sgl::Color(0, pct*255, (1.0f - pct) * 255));
    }
}

void ColorLegendWidget::setClearColor(const sgl::Color& clearColor) {
    this->clearColor = clearColor;
    this->textColor = sgl::Color(255 - clearColor.getR(), 255 - clearColor.getG(), 255 - clearColor.getB());
}

void ColorLegendWidget::renderGui() {
    const int regionHeight = 256 - 2;
    const int barWidth = 30;
    const float textRegionWidth = 90;
    const int totalWidth = barWidth + textRegionWidth;
    const int numTicks = 5;
    const int tickWidth = 10;

    std::string windowId = std::string() + "##" + attributeDisplayName;
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 windowSize = ImVec2(totalWidth + 3, regionHeight + 30);
    float windowOffset = (windowSize.x + 8) * (numPositionsTotal - positionIndex - 1);
    ImVec2 windowPos = ImVec2(
            viewport->Pos.x + viewport->Size.x - totalWidth - 12 - windowOffset,
            viewport->Pos.y + viewport->Size.y - regionHeight - 40);
    ImGui::SetNextWindowPos(windowPos);
    ImGui::SetNextWindowSize(windowSize);
    glm::vec3 clearColorFlt(clearColor.getFloatR(), clearColor.getFloatG(), clearColor.getFloatB());
    glm::vec3 textColorFlt(textColor.getFloatR(), textColor.getFloatG(), textColor.getFloatB());
    glm::vec3 bgColor = glm::mix(clearColorFlt, textColorFlt, 0.1);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(bgColor.r, bgColor.g, bgColor.b, 0.7f));
    if (ImGui::Begin(
            windowId.c_str(), &showWindow,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize/*|ImGuiWindowFlags_NoMove*/
            |ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoSavedSettings/*|ImGuiWindowFlags_NoInputs*/
            |ImGuiWindowFlags_NoFocusOnAppearing)) {
        ImGui::SetWindowFontScale(0.75f);
        ImDrawList* drawList = ImGui::GetWindowDrawList();
        float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();

        // Draw bar
        ImVec2 startPos = ImGui::GetCursorScreenPos();
        ImVec2 pos = ImVec2(startPos.x + 1, startPos.y + 1);
        const float lineHeightFactor = 1.0f / (transferFunctionColorMap.size() - 1);
        const size_t numColorMapEntries = transferFunctionColorMap.size();
        for (size_t i = 0; i < transferFunctionColorMap.size(); i++) {
            sgl::Color color = transferFunctionColorMap[numColorMapEntries - i - 1];
            ImU32 colorImgui = ImColor(color.getR(), color.getG(), color.getB());
            drawList->AddLine(
                    ImVec2(pos.x, pos.y), ImVec2(pos.x + barWidth, pos.y), colorImgui,
                    2.0f * regionHeight * lineHeightFactor);
            pos.y += regionHeight * lineHeightFactor;
        }

        ImVec2 textSize = ImGui::CalcVerticalTextSize(attributeDisplayName.c_str());
        ImVec2 textPos = ImVec2(
                startPos.x + barWidth + 40,
                startPos.y + regionHeight / 2.0f - textSize.y / 2.0f + 1);
        ImGui::AddTextVertical(
                drawList, textPos, textColor.getColorRGBA(), attributeDisplayName.c_str(),
                nullptr, false);

        ImU32 textColorImgui = textColor.getColorRGBA();

        float textHeight = ImGui::CalcTextSize(attributeDisplayName.c_str()).y;
        std::string minText = sgl::toString(attributeMinValue, 4, false);
        std::string maxText = sgl::toString(attributeMaxValue, 4, false);
        drawList->AddText(
                ImVec2(startPos.x + barWidth + 10, startPos.y + regionHeight - textHeight/2.0f + 1),
                textColorImgui, minText.c_str());
        drawList->AddText(
                ImVec2(startPos.x + barWidth + 10, startPos.y - textHeight/2.0f + 1),
                textColorImgui, maxText.c_str());

        for (int tick = 0; tick < numTicks; tick++) {
            float xpos = startPos.x + barWidth;
            float ypos = startPos.y + float(tick) / float(numTicks - 1) * regionHeight + 1;
            drawList->AddLine(
                    ImVec2(xpos - tickWidth / 2.0f, ypos),
                    ImVec2(xpos + tickWidth / 2.0f, ypos),
                    textColorImgui, 2
            );
        }
        ImGui::End();
    }
    ImGui::PopStyleColor();
}
