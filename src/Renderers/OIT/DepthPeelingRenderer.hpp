/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#ifndef LINEVIS_DEPTHPEELINGRENDERER_HPP
#define LINEVIS_DEPTHPEELINGRENDERER_HPP

#include "Renderers/ResolvePass.hpp"
#include "Renderers/LineRenderer.hpp"

class DepthComplexityRasterPass;

/**
 * Renders all lines with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique depth peeling is used.
 * Depth peeling: C. Everitt, "Interactive order-independent transparency", NVIDIA Corporation, vol. 2, 10 2001.
 */
class DepthPeelingRenderer : public LineRenderer {
public:
    DepthPeelingRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    void initialize() override;
    ~DepthPeelingRenderer() override;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_DEPTH_PEELING; }

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
    void updateVulkanUniformBuffers() override;
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

private:
    void gather();
    void resolve();
    void setUniformData();
    void computeDepthComplexity();
    void reloadGatherShader() override;
    uint64_t maxDepthComplexity = 1;
    int currIdx = 0;

    // Render passes.
    std::shared_ptr<DepthComplexityRasterPass> depthComplexityRasterPass;
    std::shared_ptr<LineRasterPass> depthPeelingRasterPasses[2];
    sgl::vk::BlitRenderPassPtr depthPeelingBlitPasses[2];
    sgl::vk::BlitRenderPassPtr blitRenderPass;

    sgl::vk::BufferPtr fragmentCounterBuffer;
    std::vector<sgl::vk::BufferPtr> stagingBuffers;

    // Render data of depth peeling
    sgl::vk::TexturePtr colorRenderTextures[2];
    sgl::vk::TexturePtr depthRenderTextures[2];
    sgl::vk::TexturePtr colorAccumulatorTexture;

    sgl::vk::Renderer* depthComplexityRenderer = nullptr;
    sgl::vk::FencePtr fence;
    VkCommandPool commandPool = VK_NULL_HANDLE;
    VkCommandBuffer depthComplexityCommandBuffer = VK_NULL_HANDLE;

    // Uniform data buffer shared by all shaders.
    struct UniformData {
        int viewportW;
        int padding0;

        // The number of fragments necessary to reach the maximal color opacity.
        uint32_t numFragmentsMaxColor;
        uint32_t padding1;

        // The color to shade the framents with.
        glm::vec4 color;
    };
    UniformData uniformData = {};
    sgl::vk::BufferPtr uniformDataBuffer;
};

class DepthComplexityRasterPass : public LineRasterPass {
public:
    explicit DepthComplexityRasterPass(LineRenderer* lineRenderer);

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
};

#endif //LINEVIS_DEPTHPEELINGRENDERER_HPP
