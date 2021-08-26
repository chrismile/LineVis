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

#ifndef RENDERERS_HEXAHEDRALMESHRENDERER_HPP
#define RENDERERS_HEXAHEDRALMESHRENDERER_HPP

#include <Renderers/AmbientOcclusion/AmbientOcclusionBaker.hpp>
#include "LineData/LineData.hpp"
#include "SceneData.hpp"

namespace sgl {
class TransferFunctionWindow;

namespace vk {
class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;
}
}
struct InternalState;
class SettingsMap;
class LineDataStress;

/**
 * Super-class for line renderers.
 */
class LineRenderer {
    friend class LineData;
    friend class LineDataStress;

public:
    LineRenderer(
            const std::string& windowName, SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : windowName(windowName), sceneData(sceneData), transferFunctionWindow(transferFunctionWindow) {}
    virtual void initialize();
    virtual ~LineRenderer();

    /// Returns if the visualization mapping needs to be re-generated.
    inline bool isDirty() const { return dirty; }
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { bool tmp = reRender; reRender = false; return tmp; }
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsInternalReRender() { bool tmp = internalReRender; internalReRender = false; return tmp; }
    /// Returns whether the triangle representation is used by the renderer.
    virtual bool getIsTriangleRepresentationUsed() const;

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    virtual void setLineData(LineDataPtr& lineData, bool isNewData)=0;

    /// Sets the ambient occlusion baker that can be used for computing the ambient occlusion of the line data.
    virtual void setAmbientOcclusionBaker(AmbientOcclusionBakerPtr& aoBaker);

    /// Renders the object to the scene framebuffer.
    virtual void render()=0;
    /// Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGuiWindow();
    /// Updates the internal logic (called once per frame).
    virtual void update(float dt);

    /// Called when the resolution of the application window has changed.
    virtual void onResolutionChanged() {}

    /// Called when the transfer function was changed.
    virtual void onTransferFunctionMapRebuilt() {}

    /// Called when the camera has moved.
    virtual void onHasMoved() {}
    // If the re-rendering was triggered from an outside source, frame accumulation cannot be used.
    virtual void notifyReRenderTriggeredExternally() { internalReRender = false; }

    // For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState) { }
    virtual bool setNewSettings(const SettingsMap& settings);
    void reloadGatherShaderExternal();

    /// Sets the global line width.
    static void setLineWidth(float width) { LineRenderer::lineWidth = width; }
    static void setBandWidth(float width) { LineRenderer::bandWidth = width; }
    static inline float getLineWidth() { return LineRenderer::lineWidth; }
    static inline float getBandWidth() { return LineRenderer::bandWidth; }

protected:
    // Reload the gather shader.
    virtual void reloadGatherShader(bool canCopyShaderAttributes = true);
    void updateNewLineData(LineDataPtr& lineData, bool isNewData);
    // GUI rendering code to be implemented by sub-classes.
    virtual void renderGui();
    void renderLineWidthSlider();
    bool showRendererWindow = true;
    // Rendering helpers for sub-classes.
    void renderHull();

    // Metadata about renderer.
    bool isRasterizer = true;
    bool isVulkanRenderer = false;
    std::string windowName;

    // For rendering the simulation mesh hull.
    sgl::ShaderProgramPtr gatherShaderHull;
    sgl::ShaderAttributesPtr shaderAttributesHull;

    SceneData& sceneData;
    LineDataPtr lineData;
    sgl::TransferFunctionWindow& transferFunctionWindow;
    bool dirty = true;
    bool reRender = true;
    bool internalReRender = true; ///< For use in renderers with frame data accumulation.

    // Whether to use depth cues (optionally selected in the UI).
    void updateDepthCueMode();
    void updateDepthCueGeometryData();
    void computeDepthRange();
    void setUniformData_Pass(sgl::ShaderProgramPtr shaderProgram);
    bool useDepthCues = true;
    float depthCueStrength = 0.8f;
    bool computeDepthCuesOnGpu = true;
    float minDepth = 0.0f;
    float maxDepth = 1.0f;
    std::vector<std::vector<glm::vec3>> filteredLines;
#ifdef USE_VULKAN_INTEROP
    sgl::vk::BufferPtr depthMinMaxBuffersVk[2];
#endif
    sgl::GeometryBufferPtr filteredLinesVerticesBuffer;
    sgl::GeometryBufferPtr depthMinMaxBuffers[2];
    size_t outputDepthMinMaxBufferIndex = 0;
    sgl::ShaderProgramPtr computeDepthValuesShaderProgram;
    sgl::ShaderProgramPtr minMaxReduceDepthShaderProgram;

    // Whether to use baked ambient occlusion buffers.
    void updateAmbientOcclusionMode();
    AmbientOcclusionBakerPtr ambientOcclusionBaker;
    bool useAmbientOcclusion = true;
    float ambientOcclusionStrength = 1.0f;
    bool ambientOcclusionBuffersDirty = false;

    // Minimum and maximum values in the UI.
    static constexpr float MIN_LINE_WIDTH = 0.001f;
    static constexpr float MAX_LINE_WIDTH = 0.020f;
    static constexpr float MIN_BAND_WIDTH = 0.001f;
    static constexpr float MAX_BAND_WIDTH = 0.020f;
    static float lineWidth;
    static float bandWidth;
};

#endif // RENDERERS_HEXAHEDRALMESHRENDERER_HPP