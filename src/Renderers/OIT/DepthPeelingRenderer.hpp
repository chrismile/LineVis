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

#include "Renderers/LineRenderer.hpp"

/**
 * Renders all lines with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique depth peeling is used.
 * Depth peeling: C. Everitt, "Interactive order-independent transparency", NVIDIA Corporation, vol. 2, 10 2001.
 */
class DepthPeelingRenderer : public LineRenderer {
public:
    DepthPeelingRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    ~DepthPeelingRenderer() override = default;
    RenderingMode getRenderingMode() override { return RENDERING_MODE_DEPTH_PEELING; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    // Renders the object to the scene framebuffer.
    void render() override;
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    void renderGui() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState) override;

private:
    void gather();
    void resolve();
    void setUniformData();
    void computeDepthComplexity();
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;
    uint64_t maxDepthComplexity = 1;

    // Render data of depth peeling
    sgl::FramebufferObjectPtr depthPeelingFBOs[2];
    sgl::TexturePtr colorRenderTextures[2];
    sgl::TexturePtr depthRenderTextures[2];
    sgl::FramebufferObjectPtr accumulatorFBO;
    sgl::TexturePtr colorAccumulatorTexture;

    // Render data.
    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderAttributesPtr shaderAttributes;

    // For computing the depth complexity
    sgl::ShaderProgramPtr depthComplexityGatherShader;
    sgl::ShaderAttributesPtr depthComplexityShaderAttributes;
    sgl::GeometryBufferPtr fragmentCounterBuffer;
};

#endif //LINEVIS_DEPTHPEELINGRENDERER_HPP
