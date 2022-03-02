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

#include <GL/glew.h>

#include <Utils/File/Logfile.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "../HullRasterPass.hpp"
#include "WBOITRenderer.hpp"

// Use stencil buffer to mask unused pixels
const bool useStencilBuffer = true;

WBOITRenderer::WBOITRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer(
                "Weighted Blended Order Independent Transparency", sceneData, transferFunctionWindow) {
    resolveRasterPass = std::make_shared<WBOITResolvePass>(this);
    //glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
    resolveRasterPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);

    onClearColorChanged();
}

void WBOITRenderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();
    resolveRasterPass->setShaderDirty();
}

void WBOITRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void WBOITRenderer::getVulkanShaderPreprocessorDefines(std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "WBOITGather.glsl"));
}

void WBOITRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setBlendModeCustom(
            VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE, VK_BLEND_OP_ADD,
            VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE, VK_BLEND_OP_ADD,
            0);
    pipelineInfo.setBlendModeCustom(
            VK_BLEND_FACTOR_ZERO, VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR, VK_BLEND_OP_ADD,
            VK_BLEND_FACTOR_ZERO, VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR, VK_BLEND_OP_ADD,
            1);
    pipelineInfo.setDepthWriteEnabled(false);
}

void WBOITRenderer::setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = loadOp;
    attachmentState.initialLayout =
            loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
            VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    framebuffer->setColorAttachment(
            accumulationRenderTexture->getImageView(), 0, attachmentState,
            { 0.0f, 0.0f, 0.0f, 0.0f });
    framebuffer->setColorAttachment(
            revealageRenderTexture->getImageView(), 1, attachmentState,
            { 1.0f, 0.0f, 0.0f, 0.0f });

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = loadOp;
    depthAttachmentState.initialLayout =
            loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
            VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            (*sceneData->sceneDepthTexture)->getImageView(), depthAttachmentState, 1.0f);
}

void WBOITRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    sgl::vk::Device* device = renderer->getDevice();
    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageSettings.format =  VK_FORMAT_R32G32B32A32_SFLOAT;
    accumulationRenderTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, sgl::vk::ImageSamplerSettings(), VK_IMAGE_ASPECT_COLOR_BIT);

    imageSettings.format =  VK_FORMAT_R32_SFLOAT; // GL_R16F?
    revealageRenderTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, sgl::vk::ImageSamplerSettings(), VK_IMAGE_ASPECT_COLOR_BIT);

    lineRasterPass->recreateSwapchain(width, height);
    if (hullRasterPass) {
        hullRasterPass->recreateSwapchain(width, height);
    }

    resolveRasterPass->setInputTextures(
            accumulationRenderTexture, revealageRenderTexture);
    resolveRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    resolveRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->recreateSwapchain(width, height);
}

void WBOITRenderer::onClearColorChanged() {
    resolveRasterPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void WBOITRenderer::render() {
    // Gather passes.
    LineRenderer::renderBase();

    if (lineRasterPass->getIsDataEmpty()) {
        renderer->transitionImageLayout(
                accumulationRenderTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        renderer->transitionImageLayout(
                revealageRenderTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        accumulationRenderTexture->getImageView()->clearColor(
                { 0.0f, 0.0f, 0.0f, 0.0f }, renderer->getVkCommandBuffer());
        revealageRenderTexture->getImageView()->clearColor(
                { 1.0f, 0.0f, 0.0f, 0.0f }, renderer->getVkCommandBuffer());
    }

    lineRasterPass->render();
    renderHull();

    // Resolve pass.
    renderer->transitionImageLayout(
            accumulationRenderTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(
            revealageRenderTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->render();
}

void WBOITRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);
}


WBOITResolvePass::WBOITResolvePass(LineRenderer* lineRenderer)
        : sgl::vk::BlitRenderPass(
                *lineRenderer->getSceneData()->renderer,
                { "WBOITResolve.Vertex", "WBOITResolve.Fragment" }) {
    this->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
}

void WBOITResolvePass::setInputTextures(
        const sgl::vk::TexturePtr& accumulationTexture, const sgl::vk::TexturePtr& revealageTexture) {
    this->accumulationRenderTexture = accumulationTexture;
    this->revealageRenderTexture = revealageTexture;
    if (rasterData) {
        rasterData->setStaticTexture(accumulationTexture, "accumulationTexture");
        rasterData->setStaticTexture(revealageTexture, "revealageTexture");
    }
}

void WBOITResolvePass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(indexBuffer);
    rasterData->setVertexBuffer(vertexBuffer, 0);
    rasterData->setStaticTexture(accumulationRenderTexture, "accumulationTexture");
    rasterData->setStaticTexture(revealageRenderTexture, "revealageTexture");
}
