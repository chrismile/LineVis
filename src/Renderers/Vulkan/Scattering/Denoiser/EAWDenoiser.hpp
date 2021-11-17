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

#ifndef LINEVIS_EAWDENOISER_HPP
#define LINEVIS_EAWDENOISER_HPP

#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>

#include "Denoiser.hpp"

class EAWBlitPass;

/**
 * An image denoiser based on the following paper.
 *
 * H. Dammertz, D. Sewtz, J. Hanika, and H. P. A. Lensch. Edge-avoiding À-trous wavelet transform for fast global
 * illumination filtering. In Proceedings of the Conference on High Performance Graphics, HPG '10, page 67-75,
 * Goslar, DEU, 2010. Eurographics Association.
 */
class EAWDenoiser : public Denoiser {
public:
    explicit EAWDenoiser(sgl::vk::Renderer* renderer);
    [[nodiscard]] const char* getDenoiserName() const override { return "EAW Denoiser"; }
    [[nodiscard]] bool getIsEnabled() const override;
    void setOutputImage(sgl::vk::ImageViewPtr& outputImage) override;
    void setFeatureMap(const std::string& featureMapName, const sgl::vk::TexturePtr& featureTexture) override;
    void denoise() override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    std::shared_ptr<EAWBlitPass> eawBlitPass;
};

class EAWBlitPass : public sgl::vk::BlitRenderPass {
public:
    explicit EAWBlitPass(sgl::vk::Renderer* renderer);
    void recreateSwapchain(uint32_t width, uint32_t height) override;

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage) override;
    inline void setColorTexture(const sgl::vk::TexturePtr& texture) { colorTexture = texture; setDataDirty(); }
    inline void setPositionTexture(const sgl::vk::TexturePtr& texture) { positionTexture = texture; setDataDirty(); }
    inline void setNormalTexture(const sgl::vk::TexturePtr& texture) { normalTexture = texture; setDataDirty(); }
    inline int getMaxNumIterations() const { return maxNumIterations; }

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

private:
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

    bool useColorWeights = true;
    bool usePositionWeights = true;
    bool useNormalWeights = false;
    float phiColor = 0.1f;
    float phiPosition = 0.1f;
    float phiNormal = 0.1f;

    sgl::vk::TexturePtr colorTexture;
    sgl::vk::TexturePtr positionTexture;
    sgl::vk::TexturePtr normalTexture;

    int maxNumIterations = 0;
    sgl::vk::TexturePtr pingPongRenderTextures[2];
    sgl::vk::FramebufferPtr framebuffersPingPong[3];
    sgl::vk::RasterDataPtr rasterDataPingPong[4];

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

#endif //LINEVIS_EAWDENOISER_HPP
