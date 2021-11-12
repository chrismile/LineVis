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

#ifndef LINEVIS_WBOITRENDERER_HPP
#define LINEVIS_WBOITRENDERER_HPP

#include "Renderers/LineRenderer.hpp"

/**
 * Renders all lines with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique Weighted Blended Order-Independent Transparency (WBOIT)
 * is used. For more details see: Morgan McGuire and Louis Bavoil. 2013. Weighted Blended Order-Independent
 * Transparency. Journal of Computer Graphics Techniques (JCGT), vol. 2, no. 2, 122-141, 2013.
 *
 * For more details regarding the implementation see:
 * http://casual-effects.blogspot.com/2015/03/implemented-weighted-blended-order.html
 */
class WBOITRenderer : public LineRenderer {
public:
    WBOITRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    ~WBOITRenderer() override {}
    RenderingMode getRenderingMode() override { return RENDERING_MODE_WBOIT; }

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

private:
    void setUniformData();
    void reloadShaders();
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;
    void reloadResolveShader();

    // Shaders.
    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderProgramPtr resolveShader;

    // Render data.
    sgl::ShaderAttributesPtr shaderAttributes;
    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates).
    sgl::ShaderAttributesPtr blitRenderData;

    // Render data of depth peeling
    sgl::FramebufferObjectPtr gatherPassFBO;
    sgl::TexturePtr accumulationRenderTexture;
    sgl::TexturePtr revealageRenderTexture;
};

#endif //LINEVIS_WBOITRENDERER_HPP
