/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Felix Brendel
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

#ifndef LINEVIS_SVGF_HPP
#define LINEVIS_SVGF_HPP

#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>

#include "Denoiser.hpp"

class SVGF_Reproj_Pass;
class SVGF_Filter_Moments_Pass;
class SVGF_ATrous_Pass;

struct SVGF_Texture_Pack {
    struct {
        sgl::vk::TexturePtr noisy_texture;
        sgl::vk::TexturePtr temp_accum_texture;
        sgl::vk::TexturePtr accum_moments_texture;

        sgl::vk::TexturePtr motion_texture;
        sgl::vk::TexturePtr normal_texture;
        sgl::vk::TexturePtr depth_texture;
        sgl::vk::TexturePtr depth_fwidth_texture;
        sgl::vk::TexturePtr depth_nabla_texture;
        // meshid?
    } current_frame;


    struct {
        // vom shader beschrieben
        sgl::vk::TexturePtr moments_history_texture;
        sgl::vk::TexturePtr color_history_texture;

        // in c++ beschrieben
        sgl::vk::TexturePtr normal_texture;
        sgl::vk::TexturePtr depth_texture;
        // meshid?
    } previous_frame;

    sgl::vk::TexturePtr pingPongRenderTextures[2];

    sgl::vk::ImageViewPtr denoised_image;
};

class SVGFDenoiser : public Denoiser {
    SVGF_Texture_Pack textures;
    sgl::vk::Renderer* renderer;

    std::shared_ptr<SVGF_Reproj_Pass>         svgf_reproj_pass;
    std::shared_ptr<SVGF_Filter_Moments_Pass> svgf_filter_moments_pass;
    std::shared_ptr<SVGF_ATrous_Pass>         svgf_atrous_pass;

public:

    bool getWantsAccumulatedInput() const override  { return false; }
    bool getWantsGlobalFrameNumber() const override  { return true; }

    explicit SVGFDenoiser(sgl::vk::Renderer* renderer);
    DenoiserType getDenoiserType() const override { return DenoiserType::SVGF; }
    [[nodiscard]] bool getIsEnabled() const override;
    void setOutputImage(sgl::vk::ImageViewPtr& outputImage) override;
    void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) override;
    [[nodiscard]] bool getUseFeatureMap(FeatureMapType featureMapType) const override;
    void setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) override;
    void resetFrameNumber() override;
    void setTemporalDenoisingEnabled(bool enabled) override {} // always temporal denoising, thus unused.
    void denoise() override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

};

class SVGF_Reproj_Pass : public sgl::vk::ComputePass {
    friend class SVGFDenoiser;
    SVGF_Texture_Pack* textures;

    const int computeBlockSize = 16;
    sgl::vk::ComputeDataPtr compute_data;

    struct {
        float allowed_z_dist      = 0.002f;
        float allowed_normal_dist = 0.02f;
    } pc;

    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

public:
    explicit SVGF_Reproj_Pass(sgl::vk::Renderer* renderer, SVGF_Texture_Pack* textures);

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

protected:
    void loadShader() override;

};

class SVGF_Filter_Moments_Pass : public sgl::vk::ComputePass {
    friend class SVGFDenoiser;
    SVGF_Texture_Pack* textures;

    const int computeBlockSize = 16;
    sgl::vk::ComputeDataPtr compute_data;

    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

public:
    explicit SVGF_Filter_Moments_Pass(sgl::vk::Renderer* renderer, SVGF_Texture_Pack* textures);

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

protected:
    void loadShader() override;

};

class SVGF_ATrous_Pass : public sgl::vk::ComputePass {
    friend class SVGFDenoiser;

    SVGF_Texture_Pack* textures;

    int maxNumIterations = 5;

    const int computeBlockSize = 16;
    sgl::vk::ComputeDataPtr computeDataPingPong[3];
    sgl::vk::ComputeDataPtr computeDataPingPongFinal[3];


    struct {
        int i;
        float z_multiplier = 100.0f;
        float fwidth_h = 1.0f;
        float nabla_max = 0.2f;
        bool only_do_temp_accum = false;
    } pc;


    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

public:
    explicit SVGF_ATrous_Pass(sgl::vk::Renderer* renderer, SVGF_Texture_Pack* textures);

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

protected:
    void loadShader() override;

};

#endif //LINEVIS_SVGF_HPP
