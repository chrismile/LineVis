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

#include <Math/Math.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
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

void EAWDenoiser::setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) {
    if (featureMapType == FeatureMapType::COLOR) {
        eawBlitPass->setColorTexture(featureTexture);
    } else if (featureMapType == FeatureMapType::POSITION) {
        eawBlitPass->setPositionTexture(featureTexture);
    } else if (featureMapType == FeatureMapType::NORMAL) {
        eawBlitPass->setNormalTexture(featureTexture);
    } else if (featureMapType == FeatureMapType::ALBEDO || featureMapType == FeatureMapType::DEPTH
            || featureMapType == FeatureMapType::FLOW) {
        // Ignore.
    } else {
        sgl::Logfile::get()->writeWarning("Warning in EAWDenoiser::setFeatureMap: Unknown feature map.");
    }
}

bool EAWDenoiser::getUseFeatureMap(FeatureMapType featureMapType) const {
    if (featureMapType == FeatureMapType::COLOR || featureMapType == FeatureMapType::NORMAL
            || featureMapType == FeatureMapType::POSITION) {
        return true;
    } else {
        return false;
    }
}

void EAWDenoiser::setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) {
    if (featureMapType == FeatureMapType::COLOR) {
        eawBlitPass->settings.useColorWeights = useFeature;
    } else if (featureMapType == FeatureMapType::NORMAL) {
        eawBlitPass->settings.useNormalWeights = useFeature;
    } else if (featureMapType == FeatureMapType::POSITION) {
        eawBlitPass->settings.usePositionWeights = useFeature;
    }
}

void EAWDenoiser::setNumIterations(int its) {
    eawBlitPass->settings.maxNumIterations = its;
    eawBlitPass->setDataDirty();
}

void EAWDenoiser::setPhiColor(float phi) {
    eawBlitPass->settings.phiColor = phi;
}

void EAWDenoiser::setPhiPosition(float phi) {
    eawBlitPass->settings.phiPosition = phi;
}

void EAWDenoiser::setPhiNormal(float phi) {
    eawBlitPass->settings.phiNormal = phi;
}

void EAWDenoiser::setWeightScaleColor(float scale) {
    eawBlitPass->settings.phiColorScale = scale;
}

void EAWDenoiser::setWeightScalePosition(float scale) {
    eawBlitPass->settings.phiPositionScale = scale;
}

void EAWDenoiser::setWeightScaleNormal(float scale) {
    eawBlitPass->settings.phiNormalScale = scale;
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



EAWBlitPass::EAWBlitPass(sgl::vk::Renderer* renderer, int maxNumIterations)
        : BlitRenderPass(renderer, {"EAWDenoise.Vertex", "EAWDenoise.Fragment"}) {
    settings.maxNumIterations = maxNumIterations;

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
    imageSettings.usage =
            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
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

void EAWBlitPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();

    std::map<std::string, std::string> preprocessorDefines;
    if (positionTexture) {
        preprocessorDefines.insert(std::make_pair("USE_POSITION_TEXTURE", ""));
    }
    if (normalTexture) {
        preprocessorDefines.insert(std::make_pair("USE_NORMAL_TEXTURE", ""));
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(shaderIds, preprocessorDefines);

    preprocessorDefines.insert(std::make_pair("BLOCK_SIZE", std::to_string(computeBlockSize)));
    shaderStagesCompute = sgl::vk::ShaderManager->getShaderStages(
            { "EAWDenoise.Compute" }, preprocessorDefines);
}

void EAWBlitPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    for (int i = 0; i < 3; i++) {
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
        }
        if (positionTexture) {
            rasterDataPingPong[i]->setStaticTexture(positionTexture, "positionTexture");
        }
        if (normalTexture) {
            rasterDataPingPong[i]->setStaticTexture(normalTexture, "normalTexture");
        }
    }

    sgl::vk::ComputePipelineInfo computePipelineInfo(shaderStagesCompute);
    sgl::vk::ComputePipelinePtr computePipeline(new sgl::vk::ComputePipeline(device, computePipelineInfo));
    for (int i = 0; i < 3; i++) {
        computeDataPingPong[i] = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
        computeDataPingPong[i]->setStaticBuffer(uniformBuffer, "UniformBuffer");
        computeDataPingPong[i]->setStaticBuffer(kernelBuffer, "KernelBuffer");
        computeDataPingPong[i]->setStaticBuffer(offsetBuffer, "OffsetBuffer");
        computeDataPingPongFinal[i] = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
        computeDataPingPongFinal[i]->setStaticBuffer(uniformBuffer, "UniformBuffer");
        computeDataPingPongFinal[i]->setStaticBuffer(kernelBuffer, "KernelBuffer");
        computeDataPingPongFinal[i]->setStaticBuffer(offsetBuffer, "OffsetBuffer");

        if (i == 0) {
            computeDataPingPong[i]->setStaticTexture(
                    colorTexture, "colorTexture");
            computeDataPingPongFinal[i]->setStaticTexture(
                    colorTexture, "colorTexture");
            computeDataPingPong[i]->setStaticImageView(
                    pingPongRenderTextures[(i + 1) % 2]->getImageView(), "outputImage");
            computeDataPingPongFinal[i]->setStaticImageView(
                    outputImageViews.front(), "outputImage");
        } else {
            computeDataPingPong[i]->setStaticTexture(
                    pingPongRenderTextures[i % 2], "colorTexture");
            computeDataPingPongFinal[i]->setStaticTexture(
                    pingPongRenderTextures[i % 2], "colorTexture");
            computeDataPingPong[i]->setStaticImageView(
                    pingPongRenderTextures[(i + 1) % 2]->getImageView(), "outputImage");
            computeDataPingPongFinal[i]->setStaticImageView(
                    outputImageViews.front(), "outputImage");
        }
        if (positionTexture) {
            computeDataPingPong[i]->setStaticTexture(positionTexture, "positionTexture");
            computeDataPingPongFinal[i]->setStaticTexture(positionTexture, "positionTexture");
        }
        if (normalTexture) {
            computeDataPingPong[i]->setStaticTexture(normalTexture, "normalTexture");
            computeDataPingPongFinal[i]->setStaticTexture(normalTexture, "normalTexture");
        }
    }
}

void EAWBlitPass::_render() {
    if (settings.maxNumIterations < 1) {
        renderer->transitionImageLayout(
                colorTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(
                outputImageViews.front()->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        colorTexture->getImage()->blit(
                outputImageViews.front()->getImage(), renderer->getVkCommandBuffer());
        return;
    }

    uniformData.useColor    = settings.useColorWeights;
    uniformData.usePosition = settings.usePositionWeights;
    uniformData.useNormal   = settings.useNormalWeights;
    uniformData.phiColor    = settings.phiColor    * settings.phiColorScale;
    uniformData.phiPosition = settings.phiPosition * settings.phiPositionScale;
    uniformData.phiNormal   = settings.phiNormal   * settings.phiNormalScale;
    uniformBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

    renderer->transitionImageLayout(colorTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    if (settings.usePositionWeights && positionTexture) {
        renderer->transitionImageLayout(
                positionTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
    if (settings.useNormalWeights && normalTexture) {
        renderer->transitionImageLayout(
                normalTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    // if (useSharedMemory) {
        _renderCompute();
    // } else {
        // _renderRaster();
    // }
}

void EAWBlitPass::_renderRaster() {
    int32_t stepWidth = 1;
    for (int i = 0; i < settings.maxNumIterations; i++) {
        int rasterDataIdx;
        int framebufferIdx;
        if (i == 0) {
            rasterDataIdx = 0;
        } else {
            rasterDataIdx = 2 - (i % 2);
        }
        if (i < settings.maxNumIterations - 1) {
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
        if (positionTexture) {
            renderer->transitionImageLayout(
                    rasterDataPingPong[rasterDataIdx]->getImageView("positionTexture")->getImage(),
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        if (normalTexture) {
            renderer->transitionImageLayout(
                    rasterDataPingPong[rasterDataIdx]->getImageView("normalTexture")->getImage(),
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        renderer->render(rasterDataPingPong[rasterDataIdx], framebuffersPingPong[framebufferIdx]);
        stepWidth *= 2;
    }
}

void EAWBlitPass::_renderCompute() {
    auto width = int(colorTexture->getImage()->getImageSettings().width);
    auto height = int(colorTexture->getImage()->getImageSettings().height);

    int32_t stepWidth = 1;
    for (int i = 0; i < settings.maxNumIterations; i++) {
        int computeDataIdx;
        sgl::vk::ComputeDataPtr computeData;
        if (i == 0) {
            computeDataIdx = 0;
        } else {
            computeDataIdx = 2 - (i % 2);
        }
        if (i == settings.maxNumIterations - 1) {
            computeData = computeDataPingPongFinal[computeDataIdx];
        } else {
            computeData = computeDataPingPong[computeDataIdx];
        }

        renderer->pushConstants(
                std::static_pointer_cast<sgl::vk::Pipeline>(computeData->getComputePipeline()),
                VK_SHADER_STAGE_COMPUTE_BIT, 0, stepWidth);
        renderer->transitionImageLayout(
                computeData->getImageView("colorTexture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        if (positionTexture) {
            renderer->transitionImageLayout(
                    computeData->getImageView("positionTexture")->getImage(),
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        if (normalTexture) {
            renderer->transitionImageLayout(
                    computeData->getImageView("normalTexture")->getImage(),
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        renderer->insertImageMemoryBarrier(
                computeData->getImageView("outputImage")->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT);

        int numWorkgroupsX = sgl::iceil(width, computeBlockSize);
        int numWorkgroupsY = sgl::iceil(height, computeBlockSize);
        renderer->dispatch(computeData, numWorkgroupsX, numWorkgroupsY, 1);
        stepWidth *= 2;
    }
}

bool EAWBlitPass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool reRender = false;

    if (propertyEditor.addSliderInt("#Iterations", &settings.maxNumIterations, 0, 5)) {
        reRender = true;
        setDataDirty();
    }
    if (propertyEditor.addCheckbox("Color Weights", &settings.useColorWeights)) {
        reRender = true;
    }
    if (propertyEditor.addCheckbox("Position Weights", &settings.usePositionWeights)) {
        reRender = true;
    }
    if (propertyEditor.addCheckbox("Normal Weights", &settings.useNormalWeights)) {
        reRender = true;
    }
    if (propertyEditor.addSliderFloat("Phi Color", &settings.phiColor, 0.0f, 1.0f)) {
        reRender = true;
    }
    if (propertyEditor.addSliderFloat("Phi Position", &settings.phiPosition, 0.0f, 1.0f, "%.9f")) {
        reRender = true;
    }
    if (propertyEditor.addSliderFloat("Phi Normal", &settings.phiNormal, 0.0f, 1.0f)) {
        reRender = true;
    }
    if (propertyEditor.addCheckbox("Use Shared Memory", &useSharedMemory)) {
        reRender = true;
    }

    return reRender;
}
