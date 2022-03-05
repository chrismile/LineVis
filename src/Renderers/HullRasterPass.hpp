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

#ifndef LINEVIS_HULLRASTERPASS_HPP
#define LINEVIS_HULLRASTERPASS_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include "SceneData.hpp"

class LineRenderer;
class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class HullRasterPass : public sgl::vk::RasterPass {
public:
    explicit HullRasterPass(LineRenderer* lineRenderer);

    // Public interface.
    void setLineData(LineDataPtr& lineData, bool isNewData);
    [[nodiscard]] inline bool getIsDataEmpty() const { return rasterData && rasterData->getNumVertices() == 0; }
    void setCustomRenderTarget(const sgl::vk::ImageViewPtr& colorImage, const sgl::vk::ImageViewPtr& depthImage);
    void setAttachmentLoadOp(VkAttachmentLoadOp loadOp);
    /**
     * This function must only be called when framebuffer compatibility can be ensured.
     * Otherwise, please use @see recreateSwapchain.
     * One use case for this function is changing the clear color.
     */
    void updateFramebuffer();
    void recreateSwapchain(uint32_t width, uint32_t height) override;

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

    LineRenderer* lineRenderer = nullptr;
    SceneData* sceneData;
    sgl::CameraPtr* camera;
    LineDataPtr lineData;
    VkAttachmentLoadOp attachmentLoadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    sgl::vk::ImageViewPtr colorRenderTargetImage;
    sgl::vk::ImageViewPtr depthRenderTargetImage;
};

#endif //LINEVIS_HULLRASTERPASS_HPP
