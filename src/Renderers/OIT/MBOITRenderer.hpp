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

#include "SyncMode.hpp"
#include "MBOITUtils.hpp"
#include "Renderers/LineRenderer.hpp"
#include "Renderers/ResolvePass.hpp"

class MBOITPass1;
class MBOITPass2;

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
    MBOITRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    void initialize() override;
    ~MBOITRenderer() override = default;
    RenderingMode getRenderingMode() override { return RENDERING_MODE_MBOIT; }

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
    void updateSyncMode();
    void updateMomentMode();
    void computeDepthRange();
    void setUniformData();
    void gather();
    void resolve();
    void reloadShaders();
    void reloadGatherShader() override;
    void reloadResolveShader();

    // Render passes.
    std::shared_ptr<MBOITPass1> mboitPass1;
    std::shared_ptr<MBOITPass2> mboitPass2;
    std::shared_ptr<ResolvePass> resolveRasterPass;

    struct UniformData {
        // Size of viewport in x direction (in pixels).
        int viewportW;

        // Range of logarithmic depth.
        float logDepthMin;
        float logDepthMax;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformDataBuffer;

    MomentOITUniformData momentUniformData{};
    sgl::vk::BufferPtr momentOITUniformBuffer;

    // Window data.
    int windowWidth = 0, windowHeight = 0;
    int paddedWindowWidth = 0, paddedWindowHeight = 0;

    sgl::vk::ImageViewPtr b0;
    sgl::vk::ImageViewPtr b;
    sgl::vk::ImageViewPtr bExtra;
    sgl::vk::ImageSettings imageSettingsB0;
    sgl::vk::ImageSettings imageSettingsB;
    sgl::vk::ImageSettings imageSettingsBExtra;
    std::vector<sgl::vk::ImagePtr> momentImageArray;

    sgl::vk::TexturePtr blendRenderTexture; ///< Accumulator.

    sgl::vk::BufferPtr spinlockViewportBuffer; ///< if (syncMode == SYNC_SPINLOCK)
    SyncMode syncMode; ///< Initialized depending on system capabilities.
    bool useOrderedFragmentShaderInterlock = true;

    // Internal mode
    static bool usePowerMoments;
    static int numMoments;
    static MBOITPixelFormat pixelFormat;
    static bool USE_R_RG_RGBA_FOR_MBOIT6;
    static float overestimationBeta;
};

class MBOITPass1 : public LineRasterPass {
public:
    explicit MBOITPass1(LineRenderer* lineRenderer);

protected:
    void loadShader() override;
};

class MBOITPass2 : public LineRasterPass {
public:
    explicit MBOITPass2(LineRenderer* lineRenderer);

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
};

#endif //LINEVIS_MBOITRENDERER_HPP
