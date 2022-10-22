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

#include <Utils/File/PathWatch.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#ifdef SUPPORT_VULKAN
#include <Graphics/Vulkan/Image/Image.hpp>
#endif

class MultiVarTransferFunctionWindow;

// Data for one variable.
class GuiVarData {
    friend class MultiVarTransferFunctionWindow;
public:
    /**
     * @param window The @see MultiVarTransferFunctionWindow parent instance.
     * @param tfPresetFile The preset transfer function file.
     * @param transferFunctionMap_sRGB The memory for storing the sRGB transfer function map.
     * @param transferFunctionMap_linearRGB The memory for storing the linear RGB transfer function map.
     */
    GuiVarData(
            MultiVarTransferFunctionWindow* window, const std::string& tfPresetFile,
            sgl::Color16* transferFunctionMap_sRGB,
            sgl::Color16* transferFunctionMap_linearRGB);

    bool saveTfToFile(const std::string& filename);
    bool loadTfFromFile(const std::string& filename);
    void setAttributeValues(const std::string& name, const std::vector<float>& attributes);

    bool renderGui();
    inline const std::string& getSaveFileString() { return saveFileString; }

    // Get data range.
    inline float getDataRangeMin() const { return dataRange.x; }
    inline float getDataRangeMax() const { return dataRange.y; }
    inline const glm::vec2& getDataRange() const { return dataRange; }
    inline float getSelectedRangeMin() const { return selectedRange.x; }
    inline float getSelectedRangeMax() const { return selectedRange.y; }
    inline const glm::vec2& getSelectedRange() const { return selectedRange; }

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
    int histogramResolution = 64;
    std::vector<float> histogram;
    glm::vec2 dataRange = glm::vec2(0.0f);
    glm::vec2 selectedRange = glm::vec2(0.0f);
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
    sgl::Color16* transferFunctionMap_sRGB = nullptr;
    sgl::Color16* transferFunctionMap_linearRGB = nullptr;

    sgl::ColorSpace interpolationColorSpace = sgl::COLOR_SPACE_LINEAR_RGB;
    std::vector<sgl::OpacityPoint> opacityPoints;
    std::vector<sgl::ColorPoint_sRGB> colorPoints;
    std::vector<sgl::ColorPoint_LinearRGB> colorPoints_LinearRGB;
};

class MultiVarTransferFunctionWindow {
    friend class GuiVarData;
public:
    /**
     * @param saveDirectoryPrefix A prefix directory attached to the file names (e.g., "stress", "multivar").
     * @param tfPresetFiles A list of preset transfer function files. If more variables are given than preset files,
     * the files are repeated.
     */
    explicit MultiVarTransferFunctionWindow(
            const std::string& saveDirectoryPrefix,
            const std::vector<std::string>& tfPresetFiles = {});
    MultiVarTransferFunctionWindow();
    ~MultiVarTransferFunctionWindow();

    // Multi-var functions.
    void setAttributesValues(
            const std::vector<std::string>& names,
            const std::vector<std::vector<float>>& allAttributes);

    bool saveCurrentVarTfToFile(const std::string& filename);
    bool loadTfFromFile(int varIdx, const std::string& filename);
    bool loadFromTfNameList(const std::vector<std::string>& tfNames);

    bool renderGui();
    void update(float dt);
    void setClearColor(const sgl::Color& clearColor);
    void setUseLinearRGB(bool useLinearRGB);

    // 1D array texture, with one 1D color (RGBA) texture slice per variable.
    sgl::vk::TexturePtr& getTransferFunctionMapTextureVulkan();
    bool getTransferFunctionMapRebuilt();
    std::vector<sgl::Color16> getTransferFunctionMap_sRGB(int varIdx);

    // Get data range.
    inline float getDataRangeMin(int varIdx) const { return guiVarData.at(varIdx).dataRange.x; }
    inline float getDataRangeMax(int varIdx) const { return guiVarData.at(varIdx).dataRange.y; }
    inline const glm::vec2& getDataRange(int varIdx) const { return guiVarData.at(varIdx).dataRange; }
    inline float getSelectedRangeMin(int varIdx) const { return guiVarData.at(varIdx).selectedRange.x; }
    inline float getSelectedRangeMax(int varIdx) const { return guiVarData.at(varIdx).selectedRange.y; }
    inline const glm::vec2& getSelectedRange(int varIdx) const { return guiVarData.at(varIdx).selectedRange; }
    inline void setSelectedRange(int varIdx, const glm::vec2& range) {
        guiVarData.at(varIdx).selectedRange = range;
        guiVarData.at(varIdx).computeHistogram();
        guiVarData.at(varIdx).window->rebuildRangeSsbo();
    }

    /// Returns the data range uniform buffer object.
    inline sgl::vk::BufferPtr& getMinMaxSsboVulkan() { return minMaxSsboVulkan; }

private:
    void updateAvailableFiles();
    void rebuildTransferFunctionMap();
    void rebuildTransferFunctionMapComplete();
    void rebuildRangeSsbo();

    std::vector<std::string> varNames;
    std::vector<GuiVarData> guiVarData;
    size_t selectedVarIndex = 0;
    GuiVarData* currVarData = nullptr;

    // Data range shader storage buffer object.
    sgl::vk::BufferPtr minMaxSsboVulkan;
    std::vector<float> minMaxData;

    // GUI
    bool showWindow = true;
    bool reRender = false;
    sgl::Color clearColor;

    // Transfer function directory watch.
    sgl::PathWatch directoryContentWatch;

    std::string directoryName = "TransferFunctions";
    std::string parentDirectory;
    std::string saveDirectory;
    std::vector<std::string> tfPresetFiles;
    std::vector<std::string> availableFiles;
    sgl::vk::TexturePtr tfMapTextureVulkan;
    sgl::vk::ImageSettings tfMapImageSettingsVulkan;

    bool useLinearRGB = true;
    bool transferFunctionMapRebuilt = true;
    std::vector<sgl::Color16> transferFunctionMap_sRGB;
    std::vector<sgl::Color16> transferFunctionMap_linearRGB;
};

#endif //STRESSLINEVIS_MULTIVARTRANSFERFUNCTIONWINDOW_HPP
