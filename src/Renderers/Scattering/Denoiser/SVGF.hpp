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

class SVGFBlitPass;


class SVGFDenoiser : public Denoiser {
public:
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

private:
    std::shared_ptr<SVGFBlitPass> svgfBlitPass;
};

class SVGFBlitPass : public sgl::vk::ComputePass {
    friend class SVGFDenoiser;
public:
    explicit SVGFBlitPass(sgl::vk::Renderer* renderer);
    void recreateSwapchain(uint32_t width, uint32_t height) override;

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage);

    inline void set_color_texture(const sgl::vk::TexturePtr& texture) {
        current_frame.color_texture = texture;
        setDataDirty();
    }
    inline void set_motion_texture(const sgl::vk::TexturePtr& texture) {
        current_frame.motion_texture = texture;

        setDataDirty();
    }
    inline void set_normal_texture(const sgl::vk::TexturePtr& texture) {
        current_frame.normal_texture  = texture;
        setDataDirty();
    }
    inline void set_depth_texture(const sgl::vk::TexturePtr& texture) {
        current_frame.depth_texture  = texture;
        setDataDirty();
    }

    [[nodiscard]] inline int getMaxNumIterations() const { return maxNumIterations; }

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

    void set_previous_frame_data() {
        // NOTE(Felix): color_texture and moments_history get updated by the
        // shader
        previous_frame.depth_texture = current_frame.depth_texture;
        previous_frame.normal_texture = current_frame.normal_texture;
    }

protected:
    void loadShader() override;


private:
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    struct {
        sgl::vk::TexturePtr color_texture;
        sgl::vk::TexturePtr motion_texture;
        sgl::vk::TexturePtr normal_texture;
        sgl::vk::TexturePtr depth_texture;
        // meshid?
    } current_frame;

    float z_multiplier = 100;

    struct {
        // vom shader beschrieben
        sgl::vk::TexturePtr moments_history_texture[2];
        uint32_t ping_pong_idx = 0;

        sgl::vk::TexturePtr color_history_texture;

        // in c++ beschrieben
        sgl::vk::TexturePtr normal_texture;
        sgl::vk::TexturePtr depth_texture;
        // meshid?
    } previous_frame;

    sgl::vk::ImageViewPtr output_image;

    bool useColorWeights = true;
    bool usePositionWeights = true;
    bool useNormalWeights = true;
    float phiColor = 5.0f;
    float phiPosition = 0.1f;
    float phiNormal = 0.1f;
    // Use custom scales for different types of image generation processes.
    float phiColorScale = 1.0f;
    float phiPositionScale = 1.0f;
    float phiNormalScale = 1.0f;

    int maxNumIterations = 5;
    sgl::vk::TexturePtr pingPongRenderTextures[2];

    bool useSharedMemory = true;
    const int computeBlockSize = 16;
    sgl::vk::ComputeDataPtr computeDataPingPong[3];
    sgl::vk::ComputeDataPtr computeDataPingPongFinal[3];

    struct UniformData {
        float phiColor;
        float phiPosition;
        float phiNormal;
        float paddingFloat;
        uint32_t useColor;
        uint32_t usePosition;
        uint32_t useNormal;
        uint32_t paddingUint;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformBuffer;

    float kernel[25]{};
    sgl::vk::BufferPtr kernelBuffer;

    glm::vec2 offset[25]{};
    sgl::vk::BufferPtr offsetBuffer;
};

#endif //LINEVIS_SVGF_HPP
