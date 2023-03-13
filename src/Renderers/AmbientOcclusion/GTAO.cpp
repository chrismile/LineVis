/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser, Felix Brendel
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

#include <random>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include "LineData/LineData.hpp"
#include "LineData/TriangleMesh/TriangleMeshData.hpp"
#include "GTAO.hpp"

GTAO::GTAO(SceneData* sceneData, sgl::vk::Renderer* renderer)
        : AmbientOcclusionBaker(renderer), sceneData(sceneData) {
    gbufferPass = std::make_shared<GTAO_GBufferPass>(sceneData, rendererMain);
    gtaoPass = std::make_shared<GTAOPass>(sceneData, rendererMain);
    std::vector<std::string> blurPassShaderIds = { "GaussianBlur.Vertex", "GaussianBlur.Fragment" };
    for (int i = 0; i < 2; i++) {
        blurPasses[i] = std::make_shared<sgl::vk::BlitRenderPass>(rendererMain, blurPassShaderIds);
        blurPasses[i]->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
        blurPasses[i]->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    }
    onResolutionChanged();
}

void GTAO::startAmbientOcclusionBaking(LineDataPtr& lineData, bool isNewData) {
    if (lineData) {
        this->lineData = lineData;
        if (isNewData) {
            reRender = true;
        }
        gbufferPass->setLineData(lineData, isNewData);
    }

    isDataReady = false;
    hasComputationFinished = false;
}

void GTAO::updateIterative(VkPipelineStageFlags pipelineStageFlags) {
    gbufferPass->render();

    gtaoPass->buildIfNecessary();



    gtao_pc.clipInfo = glm::vec4{
        sceneData->camera->getNearClipDistance(),
        sceneData->camera->getFarClipDistance(),
        0.5f * (*sceneData->viewportHeight / (2.0f * tanf(sceneData->camera->getFOVy() * 0.5f))),
        0
    };


    gtao_pc.projInfo =  {};
    gtao_pc.eyePos   =  sceneData->camera->getPosition();

    rendererMain->pushConstants(
            gtaoPass->getGraphicsPipeline(), VK_SHADER_STAGE_FRAGMENT_BIT,
            0, gtao_pc);
    gtaoPass->render();

    // int32_t blurDir0 = 0;
    // blurPasses[0]->buildIfNecessary();
    // rendererMain->pushConstants(
    //         blurPasses[0]->getGraphicsPipeline(), VK_SHADER_STAGE_FRAGMENT_BIT,
    //         0, blurDir0);
    // blurPasses[0]->render();

    // int32_t blurDir1 = 1;
    // blurPasses[1]->buildIfNecessary();
    // rendererMain->pushConstants(
    //         blurPasses[1]->getGraphicsPipeline(), VK_SHADER_STAGE_FRAGMENT_BIT,
    //         0, blurDir1);
    // blurPasses[1]->render();

    //rendererMain->insertImageMemoryBarrier(
    //        aoTexture->getImage(),
    //        aoTexture->getImage()->getVkImageLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
    //        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, pipelineStageFlags,
    //        VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);

    isDataReady = true;
    hasComputationFinished = true;
}

sgl::vk::TexturePtr GTAO::getAmbientOcclusionFrameTexture() {
    return aoTexture;
}

bool GTAO::getHasTextureResolutionChanged() {
    bool tmp = hasTextureResolutionChanged;
    hasTextureResolutionChanged = false;
    return tmp;
}

bool GTAO::needsReRender() {
    bool tmp = reRender;
    reRender = false;
    return tmp;
}

void GTAO::onHasMoved() {
    reRender = true;
}

void GTAO::onResolutionChanged() {
    sgl::vk::Device* device = rendererMain->getDevice();
    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;

    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    positionTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    normalTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);

    // TODO
    //imageSettings.format = VK_FORMAT_R32_SFLOAT;
    aoTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    blurPassTmpTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    hasTextureResolutionChanged = true;
    onHasMoved();

    gbufferPass->setOutputImages(positionTexture->getImageView(), normalTexture->getImageView());
    gbufferPass->recreateSwapchain(width, height);
    gtaoPass->setInputTextures(positionTexture, normalTexture);
    gtaoPass->setOutputImage(aoTexture->getImageView());
    gtaoPass->recreateSwapchain(width, height);
    blurPasses[0]->setInputTexture(aoTexture);
    blurPasses[0]->setOutputImage(blurPassTmpTexture->getImageView());
    blurPasses[0]->recreateSwapchain(width, height);
    blurPasses[1]->setInputTexture(blurPassTmpTexture);
    blurPasses[1]->setOutputImage(aoTexture->getImageView());
    blurPasses[1]->recreateSwapchain(width, height);
}

bool GTAO::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool optionChanged = false;

    if (propertyEditor.beginNode("GTAO")) {
        GTAOPass::SettingsData &settingsData = gtaoPass->settingsData;

        if (propertyEditor.addSliderFloat("Radius", &gtao_pc.radius, 0.0f, 0.1f) |
            (propertyEditor.addSliderIntEdit(
                "Num directions", (int*)&gtao_pc.num_directions, 1, 100) == ImGui::EditMode::INPUT_FINISHED)
            | (propertyEditor.addSliderIntEdit(
                "Num steps", (int*)&gtao_pc.num_steps, 1, 100) == ImGui::EditMode::INPUT_FINISHED)
           )
        {
            optionChanged = true;
        }

        propertyEditor.endNode();
    }

    if (optionChanged) {
        gtaoPass->numSamples = std::max(gtaoPass->numSamples, 1);
        rendererMain->getDevice()->waitIdle();
        gtaoPass->createKernelBuffer();
        optionChanged = true;
        reRender = true;
    }

    return optionChanged;
}



GTAO_GBufferPass::GTAO_GBufferPass(SceneData* sceneData, sgl::vk::Renderer* renderer)
        : sgl::vk::RasterPass(renderer), sceneData(sceneData) {
}

void GTAO_GBufferPass::setOutputImages(sgl::vk::ImageViewPtr& _positionImage, sgl::vk::ImageViewPtr& _normalImage) {
    positionImage = _positionImage;
    normalImage = _normalImage;
}

void GTAO_GBufferPass::setLineData(LineDataPtr& data, bool isNewData) {
    if (this->lineData && lineData->getType() != data->getType()) {
        setShaderDirty();
    }
    if (isNewData) {
        setDataDirty();
    }
    lineData = data;
}

void GTAO_GBufferPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);

    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachmentState.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    framebuffer->setColorAttachment(positionImage, 0, attachmentState, glm::vec4(0.0f));
    framebuffer->setColorAttachment(normalImage, 1, attachmentState, glm::vec4(0.0f));

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAttachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            (*sceneData->sceneDepthTexture)->getImageView(), depthAttachmentState, 1.0f);


    framebufferDirty = true;
    dataDirty = true;
}

void GTAO_GBufferPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"GTAO_GBufferPass.Vertex", "GTAO_GBufferPass.Fragment"});
}

void GTAO_GBufferPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setVertexBufferBinding(0, sizeof(TubeTriangleVertexData));
    pipelineInfo.setInputAttributeDescription(0, 0, "vertexPosition");
    pipelineInfo.setInputAttributeDescription(0, 4 * sizeof(float), "vertexNormal");

    bool useCulling;
    if (lineData->getType() == DATA_SET_TYPE_TRIANGLE_MESH) {
        useCulling = static_cast<TriangleMeshData*>(lineData.get())->getUseBackfaceCulling();
    } else {
        useCulling =
                lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_TUBE_TRIANGLE_MESH
                && lineData->getUseCappedTubes();
    }
    if (useCulling) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }
}

void GTAO_GBufferPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    TubeTriangleRenderData renderData = lineData->getLinePassTubeTriangleMeshRenderData(
            true, false);
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(renderData.indexBuffer);
    rasterData->setVertexBuffer(renderData.vertexBuffer, 0);
}



GTAOPass::GTAOPass(SceneData* sceneData, sgl::vk::Renderer* renderer)
    : sceneData(sceneData),
      sgl::vk::BlitRenderPass(renderer)
{
    this->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    initialize();
}

void GTAOPass::setInputTextures(sgl::vk::TexturePtr& _positionTexture, sgl::vk::TexturePtr& _normalTexture) {
    positionTexture = _positionTexture;
    normalTexture = _normalTexture;
}

void GTAOPass::setOutputImage(sgl::vk::ImageViewPtr& _aoImage) {
    aoImage = _aoImage;
}

void GTAOPass::initialize() {
    settingsDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(SettingsData), &settingsData,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);

    const size_t rotationVectorKernelLength = 4;
    const size_t rotationVectorKernelSize = rotationVectorKernelLength*rotationVectorKernelLength;
    std::vector<glm::vec4> rotationVectors = generateRotationVectors(int(rotationVectorKernelSize));

    createKernelBuffer();

    sgl::vk::ImageSettings rvecImageSettings{};
    rvecImageSettings.width = uint32_t(rotationVectorKernelLength);
    rvecImageSettings.height = uint32_t(rotationVectorKernelLength);
    rvecImageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    rvecImageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    sgl::vk::ImageSamplerSettings rvecSamplerSettings{};
    rvecSamplerSettings.minFilter = VK_FILTER_NEAREST;
    rvecSamplerSettings.magFilter = VK_FILTER_NEAREST;
    rvecSamplerSettings.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    rvecSamplerSettings.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    rvecSamplerSettings.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    rotationVectorTexture = std::make_shared<sgl::vk::Texture>(
            device, rvecImageSettings, VK_IMAGE_VIEW_TYPE_2D, rvecSamplerSettings);
    rotationVectorTexture->getImage()->uploadData(
            sizeof(glm::vec4) * rotationVectorKernelLength * rotationVectorKernelLength,
            rotationVectors.data());
}

void GTAOPass::createKernelBuffer() {
    sampleKernel = generateSSAOKernel(numSamples);
    sampleKernelBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(glm::vec4) * sampleKernel.size(), sampleKernel.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
    setShaderDirty();
}

std::vector<glm::vec4> GTAOPass::generateSSAOKernel(int numSamples) {
    std::vector<glm::vec4> kernel;
    kernel.reserve(numSamples);

    std::uniform_real_distribution<float> uniformDist(0.0f, 1.0f);
    std::default_random_engine rng;

    for (int i = 0; i < numSamples; i++) {
        // Generate uniformly distributed points in the hemisphere.
        glm::vec4 samplePosition(
                uniformDist(rng) * 2.0f - 1.0f,
                uniformDist(rng) * 2.0f - 1.0f,
                uniformDist(rng), 0.0f);
        if (glm::length(samplePosition) > 1.0f) {
            // Sample position is outside the unit hemisphere. Take a new sample.
            i--;
            continue;
        }

        // Scale the samples so that a higher number of samples is closer to the origin.
        float scale = 0.1f + static_cast<float>(i) / float(numSamples) * 0.9f;
        kernel.emplace_back(scale * samplePosition);
    }

    return kernel;
}

std::vector<glm::vec4> GTAOPass::generateRotationVectors(int numVectors) {
    std::vector<glm::vec4> rotationVectors;
    rotationVectors.reserve(numVectors);

    std::uniform_real_distribution<float> uniformDist(0.0f, 1.0f);
    std::default_random_engine rng;

    for (int i = 0; i < numVectors; i++) {
        // Represents rotation around z-axis in tangent space.
        glm::vec4 rotationVector(
                uniformDist(rng) * 2.0f - 1.0f,
                uniformDist(rng) * 2.0f - 1.0f,
                0.0f, 0.0f);
        rotationVectors.push_back(rotationVector);
    }

    return rotationVectors;
}

void GTAOPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);

    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachmentState.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    framebuffer->setColorAttachment(aoImage, 0, attachmentState, glm::vec4(0.0f));

    // auto imageSettings = (*sceneData->sceneDepthTexture)->getImage()->getImageSettings();
    // imageSettings.usage |= VK_IMAGE_USAGE_SAMPLED_BIT;

    // depthImage = std::make_shared<sgl::vk::Texture>(std::make_shared<sgl::vk::ImageView>(
                                                        // std::make_shared<sgl::vk::Image>(
                                                            // device, imageSettings),
                                                        // VK_IMAGE_ASPECT_DEPTH_BIT));

    framebufferDirty = true;
    dataDirty = true;
}

void GTAOPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    preprocessorDefines.insert(std::make_pair("KERNEL_SIZE", std::to_string(numSamples)));
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "GenerateGTAOTexture.Vertex", "GenerateGTAOTexture.Fragment" },
            preprocessorDefines);
}

void GTAOPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(indexBuffer);
    rasterData->setVertexBuffer(vertexBuffer, 0);
    // rasterData->setStaticBuffer(sampleKernelBuffer, "SamplesBuffer");
    // rasterData->setStaticBuffer(settingsDataBuffer, "SettingsBuffer");
    rasterData->setStaticTexture(positionTexture, "positionTexture");
    rasterData->setStaticTexture(*(sceneData->sceneDepthTexture),      "depthTexture");
    rasterData->setStaticTexture(normalTexture,   "normalTexture");
    // rasterData->setStaticTexture(rotationVectorTexture, "rotationVectorTexture");
}

void GTAOPass::_render() {
    settingsDataBuffer->updateData(
            sizeof(SettingsData), &settingsData, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            settingsDataBuffer);

    sgl::vk::BlitRenderPass::_render();
}
