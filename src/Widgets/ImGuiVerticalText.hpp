/*
 * ORIGINAL CODE FROM: https://gist.github.com/Flix01/3e176c64540f51b99a4ec82cfaf759a5
 *
 * Changes published under the following license:
 *
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

#ifndef STRESSLINEVIS_IMGUIVERTICALTEXT_HPP
#define STRESSLINEVIS_IMGUIVERTICALTEXT_HPP

#include <ImGui/imgui.h>

// ImDrawList methods to display vertical text
/*
// TEST: inside a window:
        ImGuiWindow* window = ImGui::GetCurrentWindow();
        ImDrawList* dl = window->DrawList;
        const ImVec2 scroll(ImGui::GetScrollX(),ImGui::GetScrollY());
        ImVec2 pos = ImGui::GetCursorScreenPos();
        pos.x = window->Pos.x + ImGui::GetStyle().WindowPadding.x;    // Optional (to ensure we're at the far left of the window)
        pos.x-=scroll.x;pos.y-=scroll.y;        // Not sure this is necessary
        const ImU32 col = ImGui::ColorConvertFloat4ToU32(ImVec4(1,1,1,1));
        static const char testString[] = "Test Text";   // It should work with '\n' inside too
        // Test 1. clockwise text
        ImGui::AddTextVertical(dl,pos,col,testString,NULL);    // pos is at the top-left corner of the vertical text (text is written from top to bottom)
        // Test 2. counter clockwise text
        const ImVec2 textSize = ImGui::CalcVerticalTextSize(testString);    // Too bad I need to call this every time (but I'm not sure I can loop UTF8 chars from text_end to text...). Moreover it seems to return a bigger y AFAICS.
        pos.x= window->Pos.x + window->Size.x - ImGui::GetStyle().WindowPadding.x - scroll.x - textSize.x;  // to ensure we're at the far right of the window MINUS the textSize.x)
        pos.y+= textSize.y;
        ImGui::AddTextVertical(dl,pos,col,testString,NULL,true);    // counter-clockwise. pos is at the bottom-left corner of the vertical text (text is written from bottom to top).
*/

namespace ImGui {

// It seems to return a bigger y...
ImVec2 CalcVerticalTextSize(
        const char* text, const char* text_end = NULL, bool hide_text_after_double_hash = false,
        float wrap_width = -1.0f);

void RenderTextVertical(
        const ImFont* font,ImDrawList* draw_list, float size, ImVec2 pos, ImU32 col, const ImVec4& clip_rect,
        const char* text_begin, const char* text_end=NULL, float wrap_width=0.0f, bool cpu_fine_clip=false,
        bool rotateCCW=false);

void AddTextVertical(
        ImDrawList* drawList,const ImFont* font, float font_size, const ImVec2& pos, ImU32 col, const char* text_begin,
        const char* text_end=NULL, float wrap_width=0.0f, const ImVec4* cpu_fine_clip_rect=NULL,bool rotateCCW = false);

void AddTextVertical(
        ImDrawList* drawList,const ImVec2& pos, ImU32 col, const char* text_begin, const char* text_end,
        bool rotateCCW = false);

}

#endif //STRESSLINEVIS_IMGUIVERTICALTEXT_HPP
