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

#include <Utils/Convert.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <ImGui/ImGuiWrapper.hpp>

#include "TilingMode.hpp"

static int modeIndex = 2;
static int tileWidth = 2;
static int tileHeight = 8;

bool selectTilingModeUI() {
    const char* indexingModeNames[] = { "1x1", "2x2", "2x8", "8x2", "4x4", "8x8", "8x8 Morton Code" };
    if (ImGui::Combo(
            "Tiling Mode", (int*)&modeIndex, indexingModeNames, IM_ARRAYSIZE(indexingModeNames))) {
        // Reset mode
        sgl::ShaderManager->invalidateShaderCache();
        sgl::ShaderManager->removePreprocessorDefine("ADDRESSING_TILED_2x2");
        sgl::ShaderManager->removePreprocessorDefine("ADDRESSING_TILED_2x8");
        sgl::ShaderManager->removePreprocessorDefine("ADDRESSING_TILED_NxM");
        sgl::ShaderManager->removePreprocessorDefine("ADRESSING_MORTON_CODE_8x8");

        // Select new mode
        if (modeIndex == 0) {
            // No tiling
            tileWidth = 1;
            tileHeight = 1;
        } else if (modeIndex == 1) {
            tileWidth = 2;
            tileHeight = 2;
            sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_2x2", "");
        } else if (modeIndex == 2) {
            tileWidth = 2;
            tileHeight = 8;
            sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_2x8", "");
        } else if (modeIndex == 3) {
            tileWidth = 8;
            tileHeight = 2;
            sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_NxM", "");
        } else if (modeIndex == 4) {
            tileWidth = 4;
            tileHeight = 4;
            sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_NxM", "");
        } else if (modeIndex == 5) {
            tileWidth = 8;
            tileHeight = 8;
            sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_NxM", "");
        } else if (modeIndex == 6) {
            tileWidth = 8;
            tileHeight = 8;
            sgl::ShaderManager->addPreprocessorDefine("ADRESSING_MORTON_CODE_8x8", "");
        }

        sgl::ShaderManager->addPreprocessorDefine("TILE_N", sgl::toString(tileWidth));
        sgl::ShaderManager->addPreprocessorDefine("TILE_M", sgl::toString(tileHeight));

        return true;
    }
    return false;
}

void setNewTilingMode(int newTileWidth, int newTileHeight, bool useMortonCode /* = false */) {
    tileWidth = newTileWidth;
    tileHeight = newTileHeight;

    // Reset mode
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->removePreprocessorDefine("ADDRESSING_TILED_2x2");
    sgl::ShaderManager->removePreprocessorDefine("ADDRESSING_TILED_2x8");
    sgl::ShaderManager->removePreprocessorDefine("ADDRESSING_TILED_NxM");
    sgl::ShaderManager->removePreprocessorDefine("ADRESSING_MORTON_CODE_8x8");

    // Select new mode
    if (tileWidth == 1 && tileHeight == 1) {
        // No tiling
        modeIndex = 0;
    } else if (tileWidth == 2 && tileHeight == 2) {
        modeIndex = 1;
        sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_2x2", "");
    } else if (tileWidth == 2 && tileHeight == 8) {
        modeIndex = 2;
        sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_2x8", "");
    } else if (tileWidth == 8 && tileHeight == 2) {
        modeIndex = 3;
        sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_NxM", "");
    } else if (tileWidth == 4 && tileHeight == 4) {
        modeIndex = 4;
        sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_NxM", "");
    } else if (tileWidth == 8 && tileHeight == 8 && !useMortonCode) {
        modeIndex = 5;
        sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_NxM", "");
    } else if (tileWidth == 8 && tileHeight == 8 && useMortonCode) {
        modeIndex = 6;
        sgl::ShaderManager->addPreprocessorDefine("ADRESSING_MORTON_CODE_8x8", "");
    } else {
        // Invalid mode, just set to mode 5, too.
        modeIndex = 5;
        sgl::ShaderManager->addPreprocessorDefine("ADDRESSING_TILED_NxM", "");
    }

    sgl::ShaderManager->addPreprocessorDefine("TILE_N", sgl::toString(tileWidth));
    sgl::ShaderManager->addPreprocessorDefine("TILE_M", sgl::toString(tileHeight));
}

void getScreenSizeWithTiling(int& screenWidth, int& screenHeight) {
    if (screenWidth % tileWidth != 0) {
        screenWidth = (screenWidth / tileWidth + 1) * tileWidth;
    }
    if (screenHeight % tileHeight != 0) {
        screenHeight = (screenHeight / tileHeight + 1) * tileHeight;
    }
}
