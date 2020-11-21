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

#include "ImGuiVerticalText.hpp"

#include <ImGui/imgui_internal.h>

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

static inline bool ImCharIsSpace(int c) {
    return c == ' ' || c == '\t' || c == 0x3000;
}

// It seems to return a bigger y...
ImVec2 CalcVerticalTextSize(
        const char *text, const char *text_end, bool hide_text_after_double_hash, float wrap_width) {
    const ImVec2 rv = ImGui::CalcTextSize(text, text_end, hide_text_after_double_hash, wrap_width);
    return ImVec2(rv.y, rv.x);
}

void RenderTextVertical(
        const ImFont *font, ImDrawList *draw_list, float size, ImVec2 pos, ImU32 col, const ImVec4 &clip_rect,
        const char *text_begin, const char *text_end, float wrap_width, bool cpu_fine_clip, bool rotateCCW) {
    if (!text_end) text_end = text_begin + strlen(text_begin);

    const float scale = size / font->FontSize;
    // Align to be pixel perfect
    pos.x = (float) (int) pos.x;// + (rotateCCW ? (font->FontSize-font->DisplayOffset.y) : 0);  // Not sure it's correct
    pos.y = (float) (int) pos.y + font->DisplayOffset.x;
    float x = pos.x;
    float y = pos.y;
    if (x > clip_rect.z)
        return;

    const float line_height = font->FontSize * scale;
    const bool word_wrap_enabled = (wrap_width > 0.0f);
    const char *word_wrap_eol = NULL;
    const float y_dir = rotateCCW ? -1.f : 1.f;

    // Skip non-visible lines
    const char *s = text_begin;
    if (!word_wrap_enabled && y + line_height < clip_rect.y)
        while (s < text_end && *s != '\n')  // Fast-forward to next line
            s++;

    // Reserve vertices for remaining worse case (over-reserving is useful and easily amortized)
    const int vtx_count_max = (int) (text_end - s) * 4;
    const int idx_count_max = (int) (text_end - s) * 6;
    const int idx_expected_size = draw_list->IdxBuffer.Size + idx_count_max;
    draw_list->PrimReserve(idx_count_max, vtx_count_max);

    ImDrawVert *vtx_write = draw_list->_VtxWritePtr;
    ImDrawIdx *idx_write = draw_list->_IdxWritePtr;
    unsigned int vtx_current_idx = draw_list->_VtxCurrentIdx;
    float x1 = 0.f, x2 = 0.f, y1 = 0.f, y2 = 0.f;

    while (s < text_end) {
        if (word_wrap_enabled) {
            // Calculate how far we can render. Requires two passes on the string data but keeps the code simple and not intrusive for what's essentially an uncommon feature.
            if (!word_wrap_eol) {
                word_wrap_eol = font->CalcWordWrapPositionA(scale, s, text_end, wrap_width - (y - pos.y));
                if (word_wrap_eol ==
                    s) // Wrap_width is too small to fit anything. Force displaying 1 character to minimize the height discontinuity.
                    word_wrap_eol++;    // +1 may not be a character start point in UTF-8 but it's ok because we use s >= word_wrap_eol below
            }

            if (s >= word_wrap_eol) {
                y = pos.y;
                x += line_height;
                word_wrap_eol = NULL;

                // Wrapping skips upcoming blanks
                while (s < text_end) {
                    const char c = *s;
                    if (ImCharIsSpace(c)) { s++; }
                    else if (c == '\n') {
                        s++;
                        break;
                    }
                    else { break; }
                }
                continue;
            }
        }

        // Decode and advance source
        unsigned int c = (unsigned int) *s;
        if (c < 0x80) {
            s += 1;
        } else {
            s += ImTextCharFromUtf8(&c, s, text_end);
            if (c == 0)
                break;
        }

        if (c < 32) {
            if (c == '\n') {
                y = pos.y;
                x += line_height;

                if (x > clip_rect.z)
                    break;
                if (!word_wrap_enabled && x + line_height < clip_rect.x)
                    while (s < text_end && *s != '\n')  // Fast-forward to next line
                        s++;
                continue;
            }
            if (c == '\r')
                continue;
        }

        float char_width = 0.0f;
        if (const ImFontGlyph *glyph = font->FindGlyph((unsigned short) c)) {
            char_width = glyph->AdvanceX * scale;

            // Arbitrarily assume that both space and tabs are empty glyphs as an optimization
            if (c != ' ' && c != '\t') {
                // We don't do a second finer clipping test on the Y axis as we've already skipped anything before clip_rect.y and exit once we pass clip_rect.w
                if (!rotateCCW) {
                    x1 = x + (font->FontSize - glyph->Y1) * scale;
                    x2 = x + (font->FontSize - glyph->Y0) * scale;
                    y1 = y + glyph->X0 * scale;
                    y2 = y + glyph->X1 * scale;
                } else {
                    x1 = x + glyph->Y0 * scale;
                    x2 = x + glyph->Y1 * scale;
                    y1 = y - glyph->X0 * scale;
                    y2 = y - glyph->X1 * scale;
                }
                if (y1 <= clip_rect.w && y2 >= clip_rect.y) {
                    // Render a character
                    float u1 = glyph->U0;
                    float v1 = glyph->V0;
                    float u2 = glyph->U1;
                    float v2 = glyph->V1;

                    // CPU side clipping used to fit text in their frame when the frame is too small. Only does clipping for axis aligned quads.
                    if (cpu_fine_clip) {
                        if (x1 < clip_rect.x) {
                            u1 = u1 + (1.0f - (x2 - clip_rect.x) / (x2 - x1)) * (u2 - u1);
                            x1 = clip_rect.x;
                        }
                        if (y1 < clip_rect.y) {
                            v1 = v1 + (1.0f - (y2 - clip_rect.y) / (y2 - y1)) * (v2 - v1);
                            y1 = clip_rect.y;
                        }
                        if (x2 > clip_rect.z) {
                            u2 = u1 + ((clip_rect.z - x1) / (x2 - x1)) * (u2 - u1);
                            x2 = clip_rect.z;
                        }
                        if (y2 > clip_rect.w) {
                            v2 = v1 + ((clip_rect.w - y1) / (y2 - y1)) * (v2 - v1);
                            y2 = clip_rect.w;
                        }
                        if (x1 >= x2) {
                            y += char_width * y_dir;
                            continue;
                        }
                    }

                    // We are NOT calling PrimRectUV() here because non-inlined causes too much overhead in a debug build.
                    // Inlined here:
                    {
                        idx_write[0] = (ImDrawIdx) (vtx_current_idx);
                        idx_write[1] = (ImDrawIdx) (vtx_current_idx + 1);
                        idx_write[2] = (ImDrawIdx) (vtx_current_idx + 2);
                        idx_write[3] = (ImDrawIdx) (vtx_current_idx);
                        idx_write[4] = (ImDrawIdx) (vtx_current_idx + 2);
                        idx_write[5] = (ImDrawIdx) (vtx_current_idx + 3);
                        vtx_write[0].col = vtx_write[1].col = vtx_write[2].col = vtx_write[3].col = col;
                        vtx_write[0].pos.x = x1;
                        vtx_write[0].pos.y = y1;
                        vtx_write[1].pos.x = x2;
                        vtx_write[1].pos.y = y1;
                        vtx_write[2].pos.x = x2;
                        vtx_write[2].pos.y = y2;
                        vtx_write[3].pos.x = x1;
                        vtx_write[3].pos.y = y2;

                        if (rotateCCW) {
                            vtx_write[0].uv.x = u1;
                            vtx_write[0].uv.y = v1;
                            vtx_write[1].uv.x = u1;
                            vtx_write[1].uv.y = v2;
                            vtx_write[2].uv.x = u2;
                            vtx_write[2].uv.y = v2;
                            vtx_write[3].uv.x = u2;
                            vtx_write[3].uv.y = v1;
                        } else {
                            vtx_write[0].uv.x = u1;
                            vtx_write[0].uv.y = v2;
                            vtx_write[1].uv.x = u1;
                            vtx_write[1].uv.y = v1;
                            vtx_write[2].uv.x = u2;
                            vtx_write[2].uv.y = v1;
                            vtx_write[3].uv.x = u2;
                            vtx_write[3].uv.y = v2;
                        }

                        vtx_write += 4;
                        vtx_current_idx += 4;
                        idx_write += 6;
                    }
                }
            }
        }

        y += char_width * y_dir;
    }

    // Give back unused vertices
    draw_list->VtxBuffer.resize((int) (vtx_write - draw_list->VtxBuffer.Data));
    draw_list->IdxBuffer.resize((int) (idx_write - draw_list->IdxBuffer.Data));
    draw_list->CmdBuffer[draw_list->CmdBuffer.Size - 1].ElemCount -= (idx_expected_size - draw_list->IdxBuffer.Size);
    draw_list->_VtxWritePtr = vtx_write;
    draw_list->_IdxWritePtr = idx_write;
    draw_list->_VtxCurrentIdx = (unsigned int) draw_list->VtxBuffer.Size;
}

void AddTextVertical(
        ImDrawList *drawList, const ImFont *font, float font_size, const ImVec2 &pos, ImU32 col, const char *text_begin,
        const char *text_end, float wrap_width, const ImVec4 *cpu_fine_clip_rect, bool rotateCCW) {
    if ((col >> 24) == 0)
        return;

    if (text_end == NULL)
        text_end = text_begin + strlen(text_begin);
    if (text_begin == text_end)
        return;

    // Note: This is one of the few instance of breaking the encapsulation of ImDrawList, as we pull this from ImGui state, but it is just SO useful.
    // Might just move Font/FontSize to ImDrawList?
    if (font == NULL)
        font = GImGui->Font;
    if (font_size == 0.0f)
        font_size = GImGui->FontSize;

    IM_ASSERT(drawList && font->ContainerAtlas->TexID ==
                          drawList->_TextureIdStack.back());  // Use high-level ImGui::PushFont() or low-level ImDrawList::PushTextureId() to change font.

    ImVec4 clip_rect = drawList->_ClipRectStack.back();
    if (cpu_fine_clip_rect) {
        clip_rect.x = ImMax(clip_rect.x, cpu_fine_clip_rect->x);
        clip_rect.y = ImMax(clip_rect.y, cpu_fine_clip_rect->y);
        clip_rect.z = ImMin(clip_rect.z, cpu_fine_clip_rect->z);
        clip_rect.w = ImMin(clip_rect.w, cpu_fine_clip_rect->w);
    }
    RenderTextVertical(
            font, drawList, font_size, pos, col, clip_rect, text_begin, text_end, wrap_width,
            cpu_fine_clip_rect != NULL, rotateCCW);
}

void AddTextVertical(
        ImDrawList *drawList, const ImVec2 &pos, ImU32 col, const char *text_begin, const char *text_end,
        bool rotateCCW) {
    AddTextVertical(drawList, GImGui->Font, GImGui->FontSize, pos, col, text_begin, text_end, 0.0f, NULL, rotateCCW);
}

}
