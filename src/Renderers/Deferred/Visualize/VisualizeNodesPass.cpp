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
#include "Renderers/LineRenderer.hpp"
#include "VisualizeNodesPass.hpp"

VisualizeNodesPass::VisualizeNodesPass(LineRenderer* lineRenderer)
        : sgl::vk::RasterPass(*lineRenderer->getSceneData()->renderer), lineRenderer(lineRenderer) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    uint32_t indexData[] = {
            0, 1, 2, 1, 3, 2, // front
            4, 6, 5, 5, 6, 7, // back
            0, 2, 4, 4, 2, 6, // left
            1, 5, 3, 5, 7, 3, // right
            0, 4, 1, 1, 4, 5, // bottom
            2, 3, 6, 3, 7, 6, // top
    };
    indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(uint32_t) * 36, indexData,
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    float vertexData[] = {
            0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
    };
    vertexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(float) * 24, vertexData,
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    VkDrawIndexedIndirectCommand command{};
    command.indexCount = 36u;
    drawIndexedIndirectBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(VkDrawIndexedIndirectCommand), &command,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    std::vector<sgl::OpacityPoint> opacityPoints = { sgl::OpacityPoint(1.0f, 0.0f), sgl::OpacityPoint(1.0f, 1.0f) };
    std::vector<sgl::ColorPoint_sRGB> colorPoints = {
            //sgl::ColorPoint_sRGB(sgl::Color(252, 229, 30), 0.0f),
            //sgl::ColorPoint_sRGB(sgl::Color(81, 195, 78), 0.25f),
            //sgl::ColorPoint_sRGB(sgl::Color(31, 129, 121), 0.5f),
            //sgl::ColorPoint_sRGB(sgl::Color(45, 62, 120), 0.75f),
            //sgl::ColorPoint_sRGB(sgl::Color(52, 0, 66), 1.0f)
            sgl::ColorPoint_sRGB(sgl::Color(52, 0, 66), 0.0f),
            sgl::ColorPoint_sRGB(sgl::Color(45, 62, 120), 0.25f),
            sgl::ColorPoint_sRGB(sgl::Color(31, 129, 121), 0.5f),
            sgl::ColorPoint_sRGB(sgl::Color(81, 195, 78), 0.75f),
            sgl::ColorPoint_sRGB(sgl::Color(252, 229, 30), 1.0f)
    };
    auto colorMap = sgl::TransferFunctionWindow::createColorMapFromPoints(opacityPoints, colorPoints);
    sgl::vk::ImageSettings tfMapImageSettingsVulkan;
    tfMapImageSettingsVulkan.imageType = VK_IMAGE_TYPE_1D;
    tfMapImageSettingsVulkan.format = VK_FORMAT_R16G16B16A16_UNORM;
    tfMapImageSettingsVulkan.width = uint32_t(colorMap.size());
    hierarchyLevelColorMap = std::make_shared<sgl::vk::Texture>(device, tfMapImageSettingsVulkan);
    hierarchyLevelColorMap->getImage()->uploadData(colorMap.size() * 8, colorMap.data());
}

void VisualizeNodesPass::setNodeAabbBuffer(const sgl::vk::BufferPtr& _nodeAabbBuffer) {
    nodeAabbBuffer = _nodeAabbBuffer;
    if (rasterData && nodeAabbBuffer) {
        rasterData->setStaticBuffer(nodeAabbBuffer, "NodeAabbBuffer");
    }
}

void VisualizeNodesPass::setNodeAabbCountBuffer(const sgl::vk::BufferPtr& _nodeAabbCountBuffer) {
    nodeAabbCountBuffer = _nodeAabbCountBuffer;
}

sgl::vk::BufferPtr VisualizeNodesPass::getNodeAabbBuffer() {
    return nodeAabbBuffer;
}

sgl::vk::BufferPtr VisualizeNodesPass::getNodeAabbCountBuffer() {
    return nodeAabbCountBuffer;
}

void VisualizeNodesPass::setLineWidth(float _lineWidth) {
    lineWidth = _lineWidth;
}

void VisualizeNodesPass::setUseScreenSpaceLineWidth(bool _useScreenSpaceLineWidth) {
    useScreenSpaceLineWidth = _useScreenSpaceLineWidth;
    setShaderDirty();
}

void VisualizeNodesPass::setViewportScaleFactor(int factor) {
    viewportScaleFactor = factor;
}

void VisualizeNodesPass::updateFramebuffer() {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, framebuffer->getWidth(), framebuffer->getHeight());
    lineRenderer->setFramebufferAttachments(framebuffer, VK_ATTACHMENT_LOAD_OP_LOAD);

    if (!rasterData) {
        framebufferDirty = true;
        dataDirty = true;
    } else {
        rasterData->getGraphicsPipeline()->setCompatibleFramebuffer(framebuffer);
    }
}

void VisualizeNodesPass::recreateSwapchain(uint32_t width, uint32_t height) {
    viewportWidth = width;
    viewportHeight = height;

    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);
    lineRenderer->setFramebufferAttachments(framebuffer, VK_ATTACHMENT_LOAD_OP_LOAD);

    framebufferDirty = true;
    dataDirty = true;
}

void VisualizeNodesPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    if (useScreenSpaceLineWidth) {
        preprocessorDefines.insert(std::make_pair("SCREEN_SPACE_LINE_THICKNESS", ""));
    }
    std::vector<std::string> shaderModuleNames = { "DrawNodeAabb.Vertex", "DrawNodeAabb.Fragment" };
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            shaderModuleNames, preprocessorDefines);
}

void VisualizeNodesPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexPosition", sizeof(glm::vec3));
    pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setDepthTestEnabled(true);
    pipelineInfo.setDepthWriteEnabled(true);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);
}

void VisualizeNodesPass::createRasterData(
        sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(indexBuffer);
    rasterData->setVertexBuffer(vertexBuffer, "vertexPosition");
    rasterData->setStaticBuffer(nodeAabbBuffer, "NodeAabbBuffer");
    rasterData->setStaticBuffer(uniformDataBuffer, "UniformDataBuffer");
    rasterData->setStaticTexture(hierarchyLevelColorMap, "hierarchyLevelColorMap");
    rasterData->setIndirectDrawBuffer(drawIndexedIndirectBuffer, sizeof(VkDrawIndexedIndirectCommand));
    rasterData->setIndirectDrawCount(1);
}

void VisualizeNodesPass::_render() {
    uniformData.cameraPosition = lineRenderer->getSceneData()->camera->getPosition();
    uniformData.fieldOfViewY = lineRenderer->getSceneData()->camera->getFOVy();
    float scaleFactor = sgl::ImGuiWrapper::get()->getScaleFactor();
    uniformData.viewportSize = glm::vec2(viewportWidth, viewportHeight);
    uniformData.viewportSizeVirtual = uniformData.viewportSize / float(viewportScaleFactor) / scaleFactor;
    uniformData.lineWidthBase = lineWidth;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            uniformDataBuffer);

    // Copy number of AABBs to instanceCount entry of VkDrawIndexedIndirectCommand.
    nodeAabbCountBuffer->copyDataTo(
            drawIndexedIndirectBuffer, 0, sizeof(uint32_t),
            sizeof(uint32_t), renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_INDIRECT_COMMAND_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
            drawIndexedIndirectBuffer);
    sgl::vk::RasterPass::_render();
}
