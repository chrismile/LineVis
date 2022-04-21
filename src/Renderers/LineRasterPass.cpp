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

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "LineRenderer.hpp"
#include "LineRasterPass.hpp"

LineRasterPass::LineRasterPass(LineRenderer* lineRenderer)
        : RasterPass(*lineRenderer->getSceneData()->renderer), lineRenderer(lineRenderer),
        sceneData(lineRenderer->getSceneData()), camera(&sceneData->camera) {
}

void LineRasterPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    dataDirty = true;
}

void LineRasterPass::setAttachmentLoadOp(VkAttachmentLoadOp loadOp) {
    this->attachmentLoadOp = loadOp;
}

void LineRasterPass::updateFramebuffer() {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, framebuffer->getWidth(), framebuffer->getHeight());
    lineRenderer->setFramebufferAttachments(framebuffer, attachmentLoadOp);

    if (!rasterData) {
        framebufferDirty = true;
        dataDirty = true;
    } else {
        rasterData->getGraphicsPipeline()->setCompatibleFramebuffer(framebuffer);
    }
}

void LineRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);
    lineRenderer->setFramebufferAttachments(framebuffer, attachmentLoadOp);

    framebufferDirty = true;
    dataDirty = true;
}

void LineRasterPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            lineData->getShaderModuleNames(), preprocessorDefines);
}

void LineRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    lineData->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);

    if ((lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_TUBE_TRIANGLE_MESH && lineData->getUseCappedTubes())
            || (lineRenderer->getIsTransparencyUsed() && lineData->getLinePrimitiveMode() != LineData::LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER)) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }
}

void LineRasterPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setRasterDataBindings(rasterData);
    lineRenderer->setRenderDataBindings(rasterData);
}

void LineRasterPass::_render() {
    if (updateUniformData) {
        lineData->updateVulkanUniformBuffers(lineRenderer, renderer);
        lineRenderer->updateVulkanUniformBuffers();

        renderer->insertMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT);
    }

    if ((*sceneData->performanceMeasurer)) {
        auto renderDataSize = rasterData->getRenderDataSize();
        (*sceneData->performanceMeasurer)->setCurrentDataSetBufferSizeBytes(
                renderDataSize.vertexBufferSize
                + renderDataSize.indexBufferSize
                + renderDataSize.storageBufferSize);
    }

    RasterPass::_render();
}
