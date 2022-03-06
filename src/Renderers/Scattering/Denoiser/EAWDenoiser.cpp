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

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include "EAWDenoiser.hpp"

EAWDenoiser::EAWDenoiser(sgl::vk::Renderer* renderer) {
    eawBlitPass = std::make_shared<EAWBlitPass>(renderer);
}

bool EAWDenoiser::getIsEnabled() const {
    return eawBlitPass->getMaxNumIterations() > 0;
}

void EAWDenoiser::setOutputImage(sgl::vk::ImageViewPtr& outputImage) {
    eawBlitPass->setOutputImage(outputImage);
}

void EAWDenoiser::setFeatureMap(const std::string& featureMapName, const sgl::vk::TexturePtr& featureTexture) {
    if (featureMapName == "color") {
        eawBlitPass->setColorTexture(featureTexture);
    } else if (featureMapName == "position") {
        eawBlitPass->setPositionTexture(featureTexture);
    } else if (featureMapName == "normal") {
        eawBlitPass->setNormalTexture(featureTexture);
    } else {
        sgl::Logfile::get()->writeInfo(
                "Warning in EAWDenoiser::setFeatureMap: Unknown feature map '" + featureMapName + "'.");
    }
}

void EAWDenoiser::denoise() {
    eawBlitPass->render();
}

void EAWDenoiser::recreateSwapchain(uint32_t width, uint32_t height) {
    eawBlitPass->recreateSwapchain(width, height);
}

bool EAWDenoiser::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    return eawBlitPass->renderGuiPropertyEditorNodes(propertyEditor);
}



EAWBlitPass::EAWBlitPass(sgl::vk::Renderer* renderer)
        : BlitRenderPass(renderer, {"EAWDenoise.Vertex", "EAWDenoise.Fragment"}) {
    uniformBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Compute the Gaussian filter kernel weights.
    for (int iy = 0; iy < 5; iy++) {
        for (int ix = 0; ix < 5; ix++) {
            int x = ix - 2;
            int y = iy - 2;
            kernel[ix + iy * 5] = std::exp(-float(x * x + y * y) / 2.0f);
            offset[ix + iy * 5] = glm::vec2(x, y);
        }
    }
    kernelBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(float) * 25, kernel,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    offsetBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(glm::vec2) * 25, offset,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

// Public interface.
void EAWBlitPass::setOutputImage(sgl::vk::ImageViewPtr& colorImage) {
    BlitRenderPass::setOutputImage(colorImage);
}

void EAWBlitPass::recreateSwapchain(uint32_t width, uint32_t height) {
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    for (int i = 0; i < 2; i++) {
        pingPongRenderTextures[i] = std::make_shared<sgl::vk::Texture>(device, imageSettings);
    }

    for (int i = 0; i < 3; i++) {
        sgl::vk::AttachmentState attachmentState;
        attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        framebuffersPingPong[i] = std::make_shared<sgl::vk::Framebuffer>(device, width, height);
        if (i < 2) {
            framebuffersPingPong[i]->setColorAttachment(
                    pingPongRenderTextures[(i + 1) % 2]->getImageView(), 0, attachmentState);
        } else {
            framebuffersPingPong[i]->setColorAttachment(
                    outputImageViews.front(), 0, attachmentState);
        }
    }
    framebuffer = framebuffersPingPong[0];
    framebufferDirty = true;
    dataDirty = true;
}

void EAWBlitPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    for (int i = 0; i < 4; i++) {
        rasterDataPingPong[i] = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
        rasterDataPingPong[i]->setIndexBuffer(indexBuffer);
        rasterDataPingPong[i]->setVertexBuffer(vertexBuffer, 0);
        rasterDataPingPong[i]->setStaticBuffer(uniformBuffer, "UniformBuffer");
        rasterDataPingPong[i]->setStaticBuffer(kernelBuffer, "KernelBuffer");
        rasterDataPingPong[i]->setStaticBuffer(offsetBuffer, "OffsetBuffer");
        if (i == 0) {
            rasterDataPingPong[i]->setStaticTexture(colorTexture, "colorTexture");
        } else if (i == 1 || i == 2) {
            rasterDataPingPong[i]->setStaticTexture(pingPongRenderTextures[i % 2], "colorTexture");
        } else {
            rasterDataPingPong[i]->setStaticTexture(
                    pingPongRenderTextures[(maxNumIterations + 1) % 2], "colorTexture");
        }
        rasterDataPingPong[i]->setStaticTexture(positionTexture, "positionTexture");
        rasterDataPingPong[i]->setStaticTexture(normalTexture, "normalTexture");
    }
}

void EAWBlitPass::_render() {
    if (maxNumIterations < 1) {
        renderer->transitionImageLayout(
                colorTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(
                outputImageViews.front()->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        colorTexture->getImage()->blit(
                outputImageViews.front()->getImage(), renderer->getVkCommandBuffer());
        return;
    }

    uniformData.useColor = useColorWeights;
    uniformData.usePosition = usePositionWeights;
    uniformData.useNormal = useNormalWeights;
    uniformData.phiColor = phiColor;
    uniformData.phiPosition = phiPosition;
    uniformData.phiNormal = phiNormal;
    uniformBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

    renderer->transitionImageLayout(colorTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    if (usePositionWeights && positionTexture) {
        renderer->transitionImageLayout(
                positionTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
    if (useNormalWeights && normalTexture) {
        renderer->transitionImageLayout(
                normalTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    float stepWidth = 1.0f;
    for (int i = 0; i < maxNumIterations; i++) {
        int rasterDataIdx;
        int framebufferIdx;
        if (i == 0) {
            rasterDataIdx = 0;
        } else if (i == maxNumIterations - 1) {
            rasterDataIdx = 3;
        } else {
            rasterDataIdx = 2 - (i % 2);
        }
        if (i < maxNumIterations - 1) {
            framebufferIdx = i % 2;
        } else {
            framebufferIdx = 2;
        }
        renderer->pushConstants(
                rasterDataPingPong[rasterDataIdx]->getGraphicsPipeline(),
                VK_SHADER_STAGE_FRAGMENT_BIT, 0, stepWidth);
        renderer->transitionImageLayout(
                rasterDataPingPong[rasterDataIdx]->getImageView("colorTexture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        renderer->transitionImageLayout(
                rasterDataPingPong[rasterDataIdx]->getImageView("positionTexture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        renderer->transitionImageLayout(
                rasterDataPingPong[rasterDataIdx]->getImageView("normalTexture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        renderer->render(rasterDataPingPong[rasterDataIdx], framebuffersPingPong[framebufferIdx]);
        stepWidth *= 2.0f;
    }
}

bool EAWBlitPass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool reRender = false;

    if (propertyEditor.addSliderInt("#Iterations", &maxNumIterations, 0, 5)) {
        reRender = true;
        setDataDirty();
    }
    if (propertyEditor.addCheckbox("Color Weights", &useColorWeights)) {
        reRender = true;
    }
    if (propertyEditor.addCheckbox("Position Weights", &usePositionWeights)) {
        reRender = true;
    }
    if (propertyEditor.addCheckbox("Normal Weights", &useNormalWeights)) {
        reRender = true;
    }
    if (propertyEditor.addSliderFloat("Phi Color", &phiColor, 0.0f, 10.0f)) {
        reRender = true;
    }
    if (propertyEditor.addSliderFloat("Phi Position", &phiPosition, 0.0f, 10.0f)) {
        reRender = true;
    }
    if (propertyEditor.addSliderFloat("Phi Normal", &phiNormal, 0.0f, 10.0f)) {
        reRender = true;
    }

    return reRender;
}
