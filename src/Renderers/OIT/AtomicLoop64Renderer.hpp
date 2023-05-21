/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2023, Christoph Neuhauser
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

#ifndef LINEVIS_AtomicLoop64Renderer_HPP
#define LINEVIS_AtomicLoop64Renderer_HPP

#include "Renderers/ResolvePass.hpp"
#include "Renderers/LineRenderer.hpp"

/**
 * Uses the "Atomic Loop 64-bit" idea from:
 * "Order Independent Transparency In OpenGL 4.x", Christoph Kubisch (2014).
 * https://on-demand.gputechconf.com/gtc/2014/presentations/S4385-order-independent-transparency-opengl.pdf
 *
 * There is also an implementation available by NVIDIA, see:
 * https://github.com/nvpro-samples/vk_order_independent_transparency/tree/master
 */
class AtomicLoop64Renderer : public LineRenderer {
public:
    AtomicLoop64Renderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    AtomicLoop64Renderer(
            const std::string& windowName, SceneData* sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    void initialize() override;
    ~AtomicLoop64Renderer() override = default;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_ATOMIC_LOOP_64; }

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

protected:
    void updateLayerMode();
    virtual void reallocateFragmentBuffer();
    void setUniformData();
    void clear();
    void gather();
    void resolve();
    void reloadShaders();
    void reloadGatherShader() override;
    void reloadResolveShader();

    // Render passes.
    std::shared_ptr<ResolvePass> resolveRasterPass;

    // Stored fragment data.
    sgl::vk::BufferPtr fragmentBuffer;

    // Uniform data buffer shared by all shaders.
    struct UniformData {
        // Size of the viewport in x direction (in pixels).
        int viewportW{};
    };
    UniformData uniformData = {};
    sgl::vk::BufferPtr uniformDataBuffer;

    // Window data.
    int windowWidth = 0, windowHeight = 0;
    int paddedWindowWidth = 0, paddedWindowHeight = 0;
    bool clearBitSet = true;
    size_t maxStorageBufferSize = 0;

    // Algorithm data.
    int numLayers = 8;
};


#endif //LINEVIS_AtomicLoop64Renderer_HPP
