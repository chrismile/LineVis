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

#ifndef STRESSLINEVIS_MULTIVARTRANSFERFUNCTIONWINDOW_HPP
#define STRESSLINEVIS_MULTIVARTRANSFERFUNCTIONWINDOW_HPP

#include <ImGui/Widgets/TransferFunctionWindow.hpp>

class MultiVarTransferFunctionWindow;

// Data for one variable.
class GuiVarData {
    friend class MultiVarTransferFunctionWindow;
public:
    /**
     *
     * @param window The @see MultiVarTransferFunctionWindow parent instance.
     * @param tfPresetFile The preset transfer function file.
     * @param transferFunctionMap_sRGB The memory for storing the sRGB transfer function map.
     * @param transferFunctionMap_linearRGB The memory for storing the linear RGB transfer function map.
     */
    GuiVarData(
            MultiVarTransferFunctionWindow* window, const std::string& tfPresetFile,
            sgl::Color* transferFunctionMap_sRGB,
            sgl::Color* transferFunctionMap_linearRGB);
    bool saveTfToFile(const std::string& filename);
    bool loadTfFromFile(const std::string& filename);
    void setAttributeValues(const std::string& name, const std::vector<float>& attributes);
    bool renderGui();
    inline const std::string& getSaveFileString() { return saveFileString; }

private:
    void computeHistogram();

    void renderFileDialog();
    void renderOpacityGraph();
    void renderColorBar();

    // Drag-and-drop functions
    void onOpacityGraphClick();
    void onColorBarClick();
    void dragPoint();
    bool selectNearestOpacityPoint(int& currentSelectionIndex, const glm::vec2& mousePosWidget);
    bool selectNearestColorPoint(int& currentSelectionIndex, const glm::vec2& mousePosWidget);

    MultiVarTransferFunctionWindow* window;

    std::string attributeName;
    int histogramResolution = 256;
    std::vector<float> histogram;
    float minAttr = 0.0f, maxAttr = 1.0f;
    std::vector<float> attributes;

    // Drag-and-drop data
    sgl::SelectedPointType selectedPointType = sgl::SELECTED_POINT_TYPE_NONE;
    bool dragging = false;
    bool mouseReleased = false;
    int currentSelectionIndex = 0;
    sgl::AABB2 opacityGraphBox, colorBarBox;
    glm::vec2 oldMousePosWidget;
    float opacitySelection = 1.0f;
    ImVec4 colorSelection = ImColor(255, 255, 255, 255);
    bool reRender = false;

    std::string saveFileString = "Standard.xml";
    int selectedFileIndex = -1;

    void rebuildTransferFunctionMap();
    void rebuildTransferFunctionMapLocal();
    void rebuildTransferFunctionMap_sRGB();
    void rebuildTransferFunctionMap_LinearRGB();

    // Pointer (with offset) to data stored by @see MultiVarTransferFunctionWindow.
    sgl::Color* transferFunctionMap_sRGB = nullptr;
    sgl::Color* transferFunctionMap_linearRGB = nullptr;

    sgl::ColorSpace interpolationColorSpace = sgl::COLOR_SPACE_LINEAR_RGB;
    std::vector<sgl::OpacityPoint> opacityPoints;
    std::vector<sgl::ColorPoint_sRGB> colorPoints;
    std::vector<sgl::ColorPoint_LinearRGB> colorPoints_LinearRGB;
};

class MultiVarTransferFunctionWindow {
    friend class GuiVarData;
public:
    /**
     *
     * @param saveDirectoryPrefix A prefix directory attached to the file names (e.g., "stress", "multivar").
     * @param tfPresetFiles A list of preset transfer function files. If more variables are given than preset files,
     * the files are repeated.
     */
    MultiVarTransferFunctionWindow(
            const std::string& saveDirectoryPrefix,
            const std::vector<std::string>& tfPresetFiles = {});

    // Multi-var functions.
    void setAttributesValues(
            const std::vector<std::string>& names,
            const std::vector<std::vector<float>>& allAttributes);

    bool saveCurrentVarTfToFile(const std::string& filename);
    bool loadTfFromFile(int varIdx, const std::string& filename);

    bool renderGui();
    void update(float dt);
    void setClearColor(const sgl::Color& clearColor);
    void setUseLinearRGB(bool useLinearRGB);

    // 1D array texture, with one 1D color (RGBA) texture slice per variable.
    sgl::TexturePtr& getTransferFunctionMapTexture();
    bool getTransferFunctionMapRebuilt();

private:
    void updateAvailableFiles();
    void rebuildTransferFunctionMap();
    void rebuildTransferFunctionMapComplete();

    std::vector<std::string> varNames;
    std::vector<GuiVarData> guiVarData;
    size_t selectedVarIndex = 0;
    GuiVarData* currVarData = nullptr;

    // GUI
    bool showWindow = true;
    bool reRender = false;
    sgl::Color clearColor;

    std::string saveDirectory = "Data/TransferFunctions/";
    std::vector<std::string> tfPresetFiles;
    std::vector<std::string> availableFiles;
    sgl::TexturePtr tfMapTexture;
    sgl::TextureSettings tfMapTextureSettings;

    bool useLinearRGB = true;
    bool transferFunctionMapRebuilt = true;
    std::vector<sgl::Color> transferFunctionMap_sRGB;
    std::vector<sgl::Color> transferFunctionMap_linearRGB;
};

#endif //STRESSLINEVIS_MULTIVARTRANSFERFUNCTIONWINDOW_HPP
