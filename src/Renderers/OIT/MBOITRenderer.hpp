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

#ifndef LINEVIS_MBOITRENDERER_HPP
#define LINEVIS_MBOITRENDERER_HPP

#include "Renderers/LineRenderer.hpp"
#include "SyncMode.hpp"
#include "MBOITUtils.hpp"

/**
 * Renders all lines with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique moment-based order-independent transparency (MBOIT) is
 * used.
 *
 * C. Munstermann, S. Krumpen, R. Klein, and C. Peters, "Moment-based order-independent transparency," Proceedings of
 * the ACM on Computer Graphics and Interactive Techniques, vol. 1, no. 1, pp. 7:1–7:20, May 2018.
 *
 * This implementation only supports the variant of MBOIT using fragment shader interlock/ROVs (i.e.,
 * GL_ARB_fragment_shader_interlock).
 */
class MBOITRenderer : public LineRenderer {
public:
    MBOITRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    virtual ~MBOITRenderer() {}

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    virtual void setLineData(LineDataPtr& lineData, bool isNewMesh);

    /// Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState);

private:
    void updateSyncMode();
    void updateMomentMode();
    void setUniformData();
    void computeDepthRange();
    void gather();
    void resolve();
    void reloadShaders();
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;
    void reloadResolveShader();

    sgl::ShaderProgramPtr mboitPass1Shader;
    sgl::ShaderAttributesPtr shaderAttributesPass1;
    sgl::ShaderProgramPtr mboitPass2Shader;
    sgl::ShaderAttributesPtr shaderAttributesPass2;
    sgl::ShaderProgramPtr blendShader;

    MomentOITUniformData momentUniformData;
    sgl::GeometryBufferPtr momentOITUniformBuffer;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr blitRenderData;
    //sgl::ShaderAttributesPtr clearRenderData;

    sgl::TexturePtr b0;
    sgl::TexturePtr b;
    sgl::TexturePtr bExtra;
    sgl::TextureSettings textureSettingsB0;
    sgl::TextureSettings textureSettingsB;
    sgl::TextureSettings textureSettingsBExtra;

    sgl::FramebufferObjectPtr blendFBO;
    sgl::TexturePtr blendRenderTexture;

    sgl::GeometryBufferPtr spinlockViewportBuffer; ///!< if (syncMode == SYNC_SPINLOCK)
    SyncMode syncMode; ///!< Initialized depending on system capabilities.
    bool useOrderedFragmentShaderInterlock = true;

    // Internal mode
    static bool useStencilBuffer; ///< Use stencil buffer to mask unused pixels.
    static bool usePowerMoments;
    static int numMoments;
    static MBOITPixelFormat pixelFormat;
    static bool USE_R_RG_RGBA_FOR_MBOIT6;
    static float overestimationBeta;
};

#endif //LINEVIS_MBOITRENDERER_HPP
