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

#ifndef STRESSLINEVIS_STRESSLINEHIERARCHYMAPPINGWIDGET_HPP
#define STRESSLINEVIS_STRESSLINEHIERARCHYMAPPINGWIDGET_HPP

#include <string>
#include <vector>

#include <Graphics/Color.hpp>
#include <Graphics/Texture/Texture.hpp>

struct HierarchyMappingPoint {
    HierarchyMappingPoint(float mappedValue, float hierarchyLevel)
            : mappedValue(mappedValue), hierarchyLevel(hierarchyLevel) {}
    float mappedValue;
    float hierarchyLevel;
};

class StressLineHierarchyMappingWidget {
public:
    StressLineHierarchyMappingWidget();

    bool renderGui();
    void update(float dt);
    void setClearColor(const sgl::Color& clearColor);

    void setLineHierarchyLevelValues(int psIdx, const std::vector<float>& lineHierarchyLevelValues);

    const static int STANDARD_MAP_RESOLUTION = 256;

    /**
     * @return A texture array containing three 1D textures with STANDARD_MAP_RESOLUTION entries each.
     * Each pixel is a value between 0.0 and 1.0 representing what importance a line hierarchy level is mapped to.
     */
    sgl::TexturePtr& getHierarchyMappingTexture();
    bool getHierarchyMappingRebuilt();

private:
    void renderGraphArea(int psIdx);
    void onGraphClick(int psIdx);
    void dragPoint(int psIdx);
    bool selectNearestHierarchyMappingPoint(int psIdx, int& currentSelectionIndex, const glm::vec2& mousePosWidget);

    // Histogram data.
    std::vector<float> lineHierarchyLevelValues;
    std::vector<float> histogram[3];
    int histogramResolution = 32;

    // Texture map & data.
    void rebuildHierarchyMappingTexture();
    sgl::TexturePtr hierarchyMappingTexture;
    sgl::TextureSettings hierarchyMappingTextureSettings;
    std::vector<HierarchyMappingPoint> hierarchyMappingPoints[3];

    // Drag-and-drop data
    int selectedPointGraphIndex = -1;
    bool dragging = false;
    bool mouseReleased = false;
    int currentSelectionIndex = 0;
    sgl::AABB2 graphBoxes[3];
    glm::vec2 oldMousePosWidget;

    // Internal UI data.
    bool showWindow = true;
    bool reRender = false;
    sgl::Color clearColor = sgl::Color(255, 255, 255);
    bool hierarchyMappingRebuilt = true;
};


#endif //STRESSLINEVIS_STRESSLINEHIERARCHYMAPPINGWIDGET_HPP
