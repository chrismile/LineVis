/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#ifndef LINEVIS_DEFERREDRENDERER_HPP
#define LINEVIS_DEFERREDRENDERER_HPP

#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include "Renderers/LineRenderer.hpp"
#include "Renderers/ResolvePass.hpp"

class DeferredResolvePass;
class DownsampleBlitPass;
class VisibilityBufferDrawIndexedPass;
class VisibilityBufferDrawIndexedIndirectPass;

const char* const deferredRenderingModeNames[4] = {
        "Draw Indexed",
        "Draw Indirect",
        "LBVH Draw Indirect",
        "Task/Mesh Shader",
};
enum class DeferredRenderingMode {
    DRAW_INDEXED,
    DRAW_INDIRECT,
    TASK_MESH_SHADER,
    LBVH_DRAW_INDIRECT
};

const char* const drawIndexedGeometryModeNames[2] = {
        "Precomputed Triangles",
        "Programmable Pulling",
};
enum class DrawIndexedGeometryMode {
    TRIANGLES, PROGRAMMABLE_PULLING
};

const char* const drawIndirectReductionModeNames[2] = {
        "Atomic Counter",
        "Prefix Sum Scan",
};
enum class DrawIndirectReductionMode {
    ATOMIC_COUNTER, PREFIX_SUM_SCAN
};

class DeferredRenderer : public LineRenderer {
public:
    DeferredRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    void initialize() override;
    ~DeferredRenderer() override = default;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_DEFERRED_SHADING; }
    bool getIsTransparencyUsed() override { return false; }
    [[nodiscard]] bool getUsesTriangleMeshInternally() const override;

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Sets the shader preprocessor defines used by the renderer.
    void getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) override;
    void setGraphicsPipelineInfo(
            sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) override;
    void setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) override;
    void setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) override;

    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    /// Called when the background clear color was changed.
    void onClearColorChanged() override;

    // Renders the object to the scene framebuffer.
    void render() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState) override;
    bool setNewSettings(const SettingsMap& settings) override;

    /// Returns the integer resolution scaling factor used internally by the renderer.
    [[nodiscard]] int getResolutionIntegerScalingFactor() const override { return 1 << supersamplingMode; }

protected:
    void reloadShaders();
    void reloadGatherShader() override;
    void reloadResolveShader();
    void updateGeometryMode();
    void setUniformData();

    // Render passes.
    std::shared_ptr<VisibilityBufferDrawIndexedPass> visibilityBufferDrawIndexedPass;
    std::shared_ptr<DeferredResolvePass> deferredResolvePass;

    enum class FramebufferMode {
        VISIBILITY_BUFFER_DRAW_INDEXED_PASS, DEFERRED_RESOLVE_PASS, HULL_RASTER_PASS
    };
    FramebufferMode framebufferMode = FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_PASS;

    struct NodeCullingUniformData {
        glm::mat4 modelViewProjectionMatrix;
        glm::ivec2 viewportSize;
        uint32_t numMeshlets; // only for linear meshlet list.
        uint32_t rootNodeIdx; // only for meshlet node tree.
    };
    NodeCullingUniformData nodeCullingUniformData{};
    sgl::vk::BufferPtr nodeCullingUniformDataBuffer;

    // Vulkan render data.
    sgl::vk::ImageViewPtr primitiveIndexImage;
    sgl::vk::TexturePtr primitiveIndexTexture;
    std::vector<sgl::vk::ImageViewPtr> depthMipLevelImageViews;
    sgl::vk::ImageViewPtr depthRenderTargetImage;
    sgl::vk::TexturePtr depthBufferTexture;
    std::vector<sgl::vk::TexturePtr> depthMipLevelTextures;
    std::vector<sgl::vk::BlitRenderPassPtr> depthMipBlitRenderPasses;
    sgl::vk::ImageViewPtr colorRenderTargetImage;
    sgl::vk::TexturePtr colorRenderTargetTexture;
    std::shared_ptr<DownsampleBlitPass> downsampleBlitPass;

    bool supportsTaskMeshShaders = false;
    bool supportsDrawIndirectCount = false;
    uint32_t meshWorkgroupSize = 32;

    // GUI data.
    DeferredRenderingMode deferredRenderingMode = DeferredRenderingMode::DRAW_INDEXED;

    // Draw indexed sub-modes.
    DrawIndexedGeometryMode drawIndexedGeometryMode = DrawIndexedGeometryMode::TRIANGLES;

    // Draw indirect sub-modes.
    DrawIndirectReductionMode drawIndirectReductionMode = DrawIndirectReductionMode::ATOMIC_COUNTER;

    // Supersampling modes.
    const char* supersamplingModeNames[2] = {
            "1x",
            "2x",
    };
    int supersamplingMode = 0;
    uint32_t renderWidth = 0, renderHeight = 0;
    uint32_t finalWidth = 0, finalHeight = 0;
};

/**
 * Called after all geometry has been rasterized to the visibility and depth buffer.
 */
class DeferredResolvePass : public ResolvePass {
public:
    DeferredResolvePass(LineRenderer* lineRenderer);
    void setDrawIndexedGeometryMode(DrawIndexedGeometryMode geometryModeNew);

protected:
    void loadShader() override;

private:
    DrawIndexedGeometryMode geometryMode = DrawIndexedGeometryMode::TRIANGLES;
};

/**
 * Used for anti-aliased downsampling of an image rendered at a higher resolution (with integer scaling).
 */
class DownsampleBlitPass : public sgl::vk::BlitRenderPass {
public:
    explicit DownsampleBlitPass(sgl::vk::Renderer* renderer);
    inline void setScalingFactor(int factor) { scalingFactor = factor; }

protected:
    void _render() override;

private:
    int32_t scalingFactor = 1;
};

/**
 * Rasterizes ALL geometry to the visibility and depth buffer in one pass.
 * Currently, only TaskMeshShaderGeometryMode::TRIANGLES is supported, i.e., no programmable pulling.
 */
class VisibilityBufferDrawIndexedPass : public LineRasterPass {
public:
    explicit VisibilityBufferDrawIndexedPass(LineRenderer* lineRenderer);
    void setDrawIndexedGeometryMode(DrawIndexedGeometryMode geometryModeNew);

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;

private:
    DrawIndexedGeometryMode geometryMode = DrawIndexedGeometryMode::TRIANGLES;
};

/**
 * Rasterizes the geometry to the visibility and depth buffer using indirect rendering.
 * Currently, only TaskMeshShaderGeometryMode::TRIANGLES is supported, i.e., no programmable pulling.
 */
class VisibilityBufferDrawIndexedIndirectPass : public LineRasterPass {
public:
    explicit VisibilityBufferDrawIndexedIndirectPass(LineRenderer* lineRenderer);
    void setReductionMode(DrawIndirectReductionMode drawIndirectReductionModeNew);

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;

private:
    DrawIndirectReductionMode drawIndirectReductionMode = DrawIndirectReductionMode::ATOMIC_COUNTER;
};

#endif //LINEVIS_DEFERREDRENDERER_HPP
