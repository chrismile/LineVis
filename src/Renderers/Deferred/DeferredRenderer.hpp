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
#include "DeferredModes.hpp"

class DeferredResolvePass;
class DownsampleBlitPass;
class VisibilityBufferDrawIndexedPass;
class VisibilityBufferDrawIndexedIndirectPass;
class MeshletDrawCountNoReductionPass;
class MeshletDrawCountAtomicPass;
class MeshletVisibilityPass;
class VisibilityBufferPrefixSumScanPass;
class MeshletDrawCountPass;
class MeshletTaskMeshPass;

class DeferredRenderer : public LineRenderer {
public:
    DeferredRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    void initialize() override;
    ~DeferredRenderer() override = default;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_DEFERRED_SHADING; }
    bool getIsTransparencyUsed() override { return false; }
    [[nodiscard]] bool getIsTriangleRepresentationUsed() const override;
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
    void updateRenderingMode();
    void updateGeometryMode();
    void updateDrawIndirectReductionMode();
    void onResolutionChangedDeferredRenderingMode();
    void setUniformData();

    // Render passes.
    void renderDataEmpty();
    // DeferredRenderingMode::DRAW_INDEXED
    void renderDrawIndexed();
    std::shared_ptr<VisibilityBufferDrawIndexedPass> visibilityBufferDrawIndexedPass;
    // DeferredRenderingMode::DRAW_INDIRECT
    void renderDrawIndexedIndirectOrTaskMesh(int passIndex);
    void renderComputeHZB(int passIndex);
    std::shared_ptr<VisibilityBufferDrawIndexedIndirectPass> visibilityBufferDrawIndexedIndirectPasses[2];
    std::shared_ptr<MeshletDrawCountNoReductionPass> meshletDrawCountNoReductionPasses[2];
    std::shared_ptr<MeshletDrawCountAtomicPass> meshletDrawCountAtomicPasses[2];
    std::shared_ptr<MeshletVisibilityPass> meshletVisibilityPasses[2];
    std::shared_ptr<VisibilityBufferPrefixSumScanPass> visibilityBufferPrefixSumScanPass;
    std::shared_ptr<MeshletDrawCountPass> meshletDrawCountPass;
    // DeferredRenderingMode::TASK_MESH_SHADER
    std::shared_ptr<MeshletTaskMeshPass> meshletTaskMeshPasses[2];
    // DeferredRenderingMode::HLBVH_DRAW_INDIRECT
    void renderHLBVH();
    std::shared_ptr<VisibilityBufferDrawIndexedPass> visibilityBufferHLBVHDrawIndirectPass;
    // Resolve/further passes.
    std::shared_ptr<DeferredResolvePass> deferredResolvePass;
    std::shared_ptr<DownsampleBlitPass> downsampleBlitPass;
    size_t frameNumber = 0; ///< The frame number is reset when the visualization mapping changes.

    enum class FramebufferMode {
        // DeferredRenderingMode::DRAW_INDEXED
        VISIBILITY_BUFFER_DRAW_INDEXED_PASS,
        // DeferredRenderingMode::DRAW_INDEXED
        VISIBILITY_BUFFER_DRAW_INDEXED_INDIRECT_PASS,
        // DeferredRenderingMode::TASK_MESH_SHADER
        VISIBILITY_BUFFER_TASK_MESH_SHADER_PASS,
        // Resolve/further passes.
        DEFERRED_RESOLVE_PASS, HULL_RASTER_PASS
    };
    int framebufferModeIndex = 0;
    FramebufferMode framebufferMode = FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_PASS;

    struct VisibilityCullingUniformData {
        glm::mat4 modelViewProjectionMatrix;
        glm::ivec2 viewportSize;
        uint32_t numMeshlets; // only for linear meshlet list.
        uint32_t rootNodeIdx; // only for meshlet node tree.
    };
    VisibilityCullingUniformData visibilityCullingUniformData{};
    sgl::vk::BufferPtr visibilityCullingUniformDataBuffer;
    glm::mat4 lastFrameViewMatrix{};
    glm::mat4 lastFrameProjectionMatrix{};

    // Vulkan render data.
    sgl::vk::ImageViewPtr primitiveIndexImage;
    sgl::vk::TexturePtr primitiveIndexTexture;
    sgl::vk::ImageViewPtr colorRenderTargetImage;
    sgl::vk::TexturePtr colorRenderTargetTexture;

    // Hierarchical z-buffer (Hi-Z buffer, HZB).
    std::vector<sgl::vk::ImageViewPtr> depthMipLevelImageViews;
    sgl::vk::ImageViewPtr depthRenderTargetImage;
    sgl::vk::TexturePtr depthBufferTexture;
    std::vector<sgl::vk::TexturePtr> depthMipLevelTextures;
    std::vector<sgl::vk::BlitRenderPassPtr> depthMipBlitRenderPasses;

    // Ping-pong HZB for meshlet modes (i.e., everything but pure draw indexed).
    std::vector<sgl::vk::ImageViewPtr> depthMipLevelImageViewsPingPong[2];
    sgl::vk::ImageViewPtr depthRenderTargetImagePingPong[2];
    sgl::vk::TexturePtr depthBufferTexturePingPong[2];
    std::vector<sgl::vk::TexturePtr> depthMipLevelTexturesPingPong[2];
    std::vector<sgl::vk::BlitRenderPassPtr> depthMipBlitRenderPassesPingPong[2];

    bool supportsTaskMeshShaders = false;
    bool supportsDrawIndirect = false;
    bool supportsDrawIndirectCount = false;
    uint32_t drawIndirectMaxNumPrimitivesPerMeshlet = 128;
    uint32_t taskMeshShaderMaxNumPrimitivesPerMeshlet = 126;
    uint32_t taskMeshShaderMaxNumVerticesPerMeshlet = 64;
    uint32_t taskMeshShaderMaxNumPrimitivesSupported = 512;
    uint32_t taskMeshShaderMaxNumVerticesSupported = 256;

    // Visible meshlets in pass 1/2.
    bool showVisibleMeshletStatistics = true;
    uint32_t visibleMeshletCounters[2] = { 0, 0 };
    std::vector<sgl::vk::BufferPtr> visibleMeshletsStagingBuffers;
    std::vector<bool> frameHasNewStagingDataList;

    // Current rendering mode.
    DeferredRenderingMode deferredRenderingMode = DeferredRenderingMode::DRAW_INDEXED;

    // Draw indexed sub-modes.
    DrawIndexedGeometryMode drawIndexedGeometryMode = DrawIndexedGeometryMode::TRIANGLES;

    // Draw indirect sub-modes.
    DrawIndirectReductionMode drawIndirectReductionMode = DrawIndirectReductionMode::ATOMIC_COUNTER;

    // BVH draw indirect sub-modes.
    BvhBuildAlgorithm bvhBuildAlgorithm = BvhBuildAlgorithm::SWEEP_SAH_CPU;
    BvhBuildGeometryMode bvhBuildGeometryMode = BvhBuildGeometryMode::MESHLETS;
    BvhBuildPrimitiveCenterMode bvhBuildPrimitiveCenterMode = BvhBuildPrimitiveCenterMode::PRIMITIVE_CENTROID;

    // Task/mesh shader sub-modes.
    bool useMeshShaderWritePackedPrimitiveIndicesIfAvailable = true;

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
    explicit DeferredResolvePass(LineRenderer* lineRenderer);
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

#endif //LINEVIS_DEFERREDRENDERER_HPP
