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

#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include "LineRenderer.hpp"
#include "HullRasterPass.hpp"

HullRasterPass::HullRasterPass(LineRenderer* lineRenderer)
        : RasterPass(*lineRenderer->getSceneData()->renderer), lineRenderer(lineRenderer),
          sceneData(lineRenderer->getSceneData()), camera(&sceneData->camera) {
}

void HullRasterPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    colorRenderTargetImage = {};
    depthRenderTargetImage = {};
    dataDirty = true;
    shaderDirty = true;
}

void HullRasterPass::setCustomRenderTarget(
        const sgl::vk::ImageViewPtr& colorImage, const sgl::vk::ImageViewPtr& depthImage) {
    colorRenderTargetImage = colorImage;
    depthRenderTargetImage = depthImage;
}

void HullRasterPass::setAttachmentLoadOp(VkAttachmentLoadOp loadOp) {
    this->attachmentLoadOp = loadOp;
}

void HullRasterPass::updateFramebuffer() {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, framebuffer->getWidth(), framebuffer->getHeight());
    lineRenderer->setFramebufferAttachments(framebuffer, attachmentLoadOp);

    if (!rasterData) {
        framebufferDirty = true;
        dataDirty = true;
    } else {
        rasterData->getGraphicsPipeline()->setCompatibleFramebuffer(framebuffer);
    }
}

void HullRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);
    lineRenderer->setFramebufferAttachments(framebuffer, attachmentLoadOp);

    framebufferDirty = true;
    dataDirty = true;
}

void HullRasterPass::clearFragmentBuffer() {
    if (rasterData) {
        rasterData->setStaticBufferOptional({}, "FragmentBuffer");
        rasterData->setStaticBufferArrayOptional({}, "FragmentBuffer");
    }
}

void HullRasterPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "MeshHull.Vertex", "MeshHull.Fragment" }, preprocessorDefines);
}

void HullRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);

    uint32_t vertexPositionBinding = shaderStages->getInputVariableLocation("vertexPosition");
    pipelineInfo.setVertexBufferBinding(vertexPositionBinding, sizeof(glm::vec3));
    pipelineInfo.setInputAttributeDescription(
            vertexPositionBinding, 0, "vertexPosition");

    uint32_t vertexNormalBinding = shaderStages->getInputVariableLocation("vertexNormal");
    pipelineInfo.setVertexBufferBinding(vertexNormalBinding, sizeof(glm::vec3));
    pipelineInfo.setInputAttributeDescription(
            vertexNormalBinding, 0, "vertexNormal");

    pipelineInfo.setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);
    pipelineInfo.setDepthWriteEnabled(false);
    pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
}

void HullRasterPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    lineRenderer->setRenderDataBindings(rasterData);

    SimulationMeshOutlineRenderData renderData = lineData->getSimulationMeshOutlineRenderData();
    rasterData->setIndexBuffer(renderData.indexBuffer);
    rasterData->setVertexBuffer(renderData.vertexPositionBuffer, "vertexPosition");
    rasterData->setVertexBuffer(renderData.vertexNormalBuffer, "vertexNormal");
}

void HullRasterPass::_render() {
    /*
     * Assumes "lineData->updateVulkanUniformBuffers(lineRenderer, renderer);" was called by LineRasterPass.
     */

    RasterPass::_render();
}
