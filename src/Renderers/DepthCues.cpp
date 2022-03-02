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

#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include "DepthCues.hpp"

ComputeDepthValuesPass::ComputeDepthValuesPass(SceneData* sceneData)
        : ComputePass(*sceneData->renderer), sceneData(sceneData) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void ComputeDepthValuesPass::setLineVerticesBuffer(const sgl::vk::BufferPtr& buffer) {
    lineVerticesBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(lineVerticesBuffer, "VertexPositionBuffer");
    }
}

void ComputeDepthValuesPass::setDepthMinMaxOutBuffer(const sgl::vk::BufferPtr& buffer) {
    depthMinMaxOutBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(depthMinMaxOutBuffer, "DepthMinMaxOutBuffer");
    }
}

void ComputeDepthValuesPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"ComputeDepthValues.Compute"});
}

void ComputeDepthValuesPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(uniformDataBuffer, "UniformDataBuffer");
    computeData->setStaticBuffer(lineVerticesBuffer, "VertexPositionBuffer");
    computeData->setStaticBuffer(depthMinMaxOutBuffer, "DepthMinMaxOutBuffer");
}

void ComputeDepthValuesPass::_render() {
    auto numVertices = uint32_t(lineVerticesBuffer->getSizeInBytes() / sizeof(glm::vec4));
    uint32_t numBlocks = sgl::iceil(int(numVertices), BLOCK_SIZE_DEPTH_CUES);

    uniformData.nearDist = sceneData->camera->getNearClipDistance();
    uniformData.farDist = sceneData->camera->getFarClipDistance();
    uniformData.numVertices = numVertices;
    uniformData.cameraViewMatrix = sceneData->camera->getViewMatrix();
    uniformData.cameraProjectionMatrix = sceneData->camera->getProjectionMatrix();
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    renderer->dispatch(computeData, uint32_t(numBlocks), 1, 1);
}



MinMaxDepthReductionPass::MinMaxDepthReductionPass(SceneData* sceneData)
        : ComputePass(*sceneData->renderer), sceneData(sceneData) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void MinMaxDepthReductionPass::setDepthMinMaxBuffers(
        const sgl::vk::BufferPtr &bufferIn, const sgl::vk::BufferPtr &bufferOut) {
    depthMinMaxInBuffer = bufferIn;
    depthMinMaxOutBuffer = bufferOut;
    if (computeData) {
        computeData->setStaticBuffer(depthMinMaxInBuffer, "MinMaxInBuffer");
        computeData->setStaticBuffer(depthMinMaxOutBuffer, "MinMaxOutBuffer");
    }
}

void MinMaxDepthReductionPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"MinMaxReduce.Compute"});
}

void MinMaxDepthReductionPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(uniformDataBuffer, "UniformDataBuffer");
    computeData->setStaticBuffer(depthMinMaxInBuffer, "MinMaxInBuffer");
    computeData->setStaticBuffer(depthMinMaxOutBuffer, "MinMaxOutBuffer");
}

void MinMaxDepthReductionPass::_render() {
    uint32_t numBlocks = sgl::iceil(int(inputSize), BLOCK_SIZE_DEPTH_CUES * 2);

    uniformData.nearDist = sceneData->camera->getNearClipDistance();
    uniformData.farDist = sceneData->camera->getFarClipDistance();
    uniformData.cameraViewMatrix = sceneData->camera->getViewMatrix();
    uniformData.cameraProjectionMatrix = sceneData->camera->getProjectionMatrix();
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

    renderer->pushConstants(
            std::static_pointer_cast<sgl::vk::Pipeline>(computeData->getComputePipeline()),
            VK_SHADER_STAGE_COMPUTE_BIT, 0, inputSize);

    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

    renderer->dispatch(computeData, uint32_t(numBlocks), 1, 1);
}
