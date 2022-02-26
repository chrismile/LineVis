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

#include <Utils/Events/EventManager.hpp>
#include <Renderers/AmbientOcclusion/AmbientOcclusionBaker.hpp>
#include <utility>
#include "LineData/LineData.hpp"
#include "RenderingModes.hpp"
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
class HullRasterPass;

class ComputeDepthValuesPass;
class MinMaxDepthReductionPass;

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;

/**
 * Super-class for line renderers.
 */
class LineRenderer {
    friend class LineData;
    friend class LineDataStress;

public:
    LineRenderer(
            std::string windowName, SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : windowName(std::move(windowName)), sceneData(sceneData), renderer(*sceneData->renderer),
        transferFunctionWindow(transferFunctionWindow) {}
    virtual void initialize();
    virtual ~LineRenderer();
    virtual RenderingMode getRenderingMode()=0;
    virtual bool getIsTransparencyUsed() { return true; }

    /// Returns if the visualization mapping needs to be re-generated.
    [[nodiscard]] inline bool isDirty() const { return dirty; }
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    [[nodiscard]] virtual bool needsReRender();
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    [[nodiscard]] virtual bool needsInternalReRender() { bool tmp = internalReRender; internalReRender = false; return tmp; }
    /// Returns whether the triangle representation is used by the renderer.
    [[nodiscard]] virtual bool getIsTriangleRepresentationUsed() const;
    /// Returns whether live visualization mapping updates can be used or whether the data set is too large.
    [[nodiscard]] virtual bool getCanUseLiveUpdate(LineDataAccessType accessType) const;
    [[nodiscard]] inline bool getIsRasterizer() const { return isRasterizer; }
    [[nodiscard]] inline bool getIsVulkanRenderer() const { return isVulkanRenderer; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    virtual void setLineData(LineDataPtr& lineData, bool isNewData)=0;

    /// Sets the shader preprocessor defines used by the renderer.
    virtual void getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines);
    virtual void setGraphicsPipelineInfo(
            sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages);
    virtual void setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData);
    virtual void updateVulkanUniformBuffers();

    /// Prepares the depth cues buffer and ambient occlusion buffer/texture for the passed pipeline stage.
    virtual void renderBase(VkPipelineStageFlags pipelineStageFlags);
    virtual void renderBase() { renderBase(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT); }
    /// Renders the object to the scene framebuffer.
    virtual void render()=0;
    /// Renders the entries in the property editor.
    virtual void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);
    /// Renders GUI overlays. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGuiOverlay();
    /// For rendering secondary ImGui windows (e.g., for transfer function widgets).
    virtual void renderGuiWindowSecondary() {}
    /// Updates the internal logic (called once per frame).
    virtual void update(float dt);

    /// Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    /// Called when the background clear color was changed.
    virtual void onClearColorChanged() {}

    /// Called when the transfer function was changed.
    virtual void onTransferFunctionMapRebuilt() {}

    /// Called when the camera has moved.
    virtual void onHasMoved();
    /// If the re-rendering was triggered from an outside source, frame accumulation cannot be used.
    virtual void notifyReRenderTriggeredExternally() { internalReRender = false; }
    /**
     * Sets whether linear RGB or sRGB should be used for rendering. Most renderers won't need to do anything special,
     * as the transfer function data is automatically updated to use the correct format.
     */
    virtual void setUseLinearRGB(bool useLinearRGB) {}
    virtual void setFileDialogInstance(ImGuiFileDialog* fileDialogInstance) {}

    /**
     * Called when whether the simulation mesh hull should be rendered might have changed.
     * Only used in the Vulkan ray tracer so far.
     */
    virtual void setRenderSimulationMeshHull(bool shallRenderSimulationMeshHull) {}

    // For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState) { }
    virtual bool setNewSettings(const SettingsMap& settings);
    void reloadGatherShaderExternal();

    /// Sets the global line width.
    static void setLineWidth(float width) { LineRenderer::lineWidth = width; }
    static void setBandWidth(float width) { LineRenderer::bandWidth = width; }
    static inline float getLineWidth() { return LineRenderer::lineWidth; }
    static inline float getBandWidth() { return LineRenderer::bandWidth; }
    static inline float getMinBandThickness() { return LineData::getMinBandThickness(); }

    inline const std::string& getWindowName() { return windowName; }
    inline SceneData* getSceneData() { return sceneData; }

    // Tiling mode.
    static void setNewTilingMode(int newTileWidth, int newTileHeight, bool useMortonCode = false);

    /// For visualizing the seeding order in an animation (called by MainApp).
    virtual void setVisualizeSeedingProcess(bool visualizeSeedingProcess) {}

    bool getIsComputationRunning();

protected:
    // Reload the gather shader.
    virtual void reloadGatherShader(bool canCopyShaderAttributes = true);
    void updateNewLineData(LineDataPtr& lineData, bool isNewData);
    bool showRendererWindow = true;
    // Rendering helpers for sub-classes.
    void renderHull();

    bool isInitialized = false;
    sgl::ListenerToken onTransferFunctionMapRebuiltListenerToken{};

    // Metadata about renderer.
    bool isRasterizer = true;
    bool isVulkanRenderer = false;
    std::string windowName;

    // For rendering the simulation mesh hull.
    sgl::ShaderProgramPtr gatherShaderHull;
    sgl::ShaderAttributesPtr shaderAttributesHull;
    std::shared_ptr<HullRasterPass> hullRasterPass;

    SceneData* sceneData;
    sgl::vk::Renderer* renderer = nullptr;
    LineDataPtr lineData;
    sgl::TransferFunctionWindow& transferFunctionWindow;
    bool dirty = true;
    bool reRender = true;
    bool internalReRender = true; ///< For use in renderers with frame data accumulation.
    bool ambientOcclusionReRender = false;

    // Whether to use depth cues (optionally selected in the UI).
    void updateDepthCueMode();
    void updateDepthCueGeometryData();
    void computeDepthRange();
    void setUniformData_Pass(sgl::ShaderProgramPtr shaderProgram);
    bool useDepthCues = true;
    float depthCueStrength = 0.8f;
    float minDepth = 0.0f;
    float maxDepth = 1.0f;
    std::vector<std::vector<glm::vec3>> filteredLines;
    sgl::vk::BufferPtr filteredLinesVerticesBuffer;
    sgl::vk::BufferPtr depthMinMaxBuffers[2];
    size_t outputDepthMinMaxBufferIndex = 0;
    std::shared_ptr<ComputeDepthValuesPass> computeDepthValuesPass;
    std::shared_ptr<MinMaxDepthReductionPass> minMaxDepthReductionPass[2];

    // Tiling mode.
    static int tilingModeIndex;
    static int tileWidth;
    static int tileHeight;
    static bool tilingUseMortonCode;
    /**
     * Uses ImGui to render a tiling mode selection window.
     * @return True if a new tiling mode was selected (shaders need to be reloaded in this case).
     */
    static bool selectTilingModeUI(sgl::PropertyEditor& propertyEditor);
    /// Returns screen width and screen height padded for tile size
    static void getScreenSizeWithTiling(int& screenWidth, int& screenHeight);

    // Whether to use baked ambient occlusion buffers/images.
    void showRayQueriesUnsupportedWarning();
    void setAmbientOcclusionBaker();
    void updateAmbientOcclusionMode();
    AmbientOcclusionBakerType ambientOcclusionBakerType = AmbientOcclusionBakerType::VULKAN_RTAO;
    AmbientOcclusionBakerPtr ambientOcclusionBaker;
    bool useAmbientOcclusion = false;
    float ambientOcclusionStrength = 0.0f;
    float ambientOcclusionGamma = 1.0f;
    bool ambientOcclusionBuffersDirty = false;
    bool ambientOcclusionTexturesDirty = false;

    // Minimum and maximum values in the UI.
    static constexpr float MIN_LINE_WIDTH = 0.001f;
    static constexpr float MAX_LINE_WIDTH = 0.020f;
    static constexpr float MIN_BAND_WIDTH = 0.001f;
    static constexpr float MAX_BAND_WIDTH = 0.020f;
    static float lineWidth;
    static float bandWidth;
    static float minBandThickness;
};

#endif // RENDERERS_HEXAHEDRALMESHRENDERER_HPP