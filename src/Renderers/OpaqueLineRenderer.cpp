/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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
#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/LineDataStress.hpp"
#include "Helpers/Sphere.hpp"
#include "OpaqueLineRenderer.hpp"

OpaqueLineRenderer::OpaqueLineRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opaque Line Renderer", sceneData, transferFunctionWindow) {
    // Get all available multisampling modes.
    maximumNumberOfSamples = (*sceneData->renderer)->getDevice()->getMaxUsableSampleCount();
    if (maximumNumberOfSamples <= 1) {
        useMultisampling = false;
    }
    supportsSampleShadingRate = renderer->getDevice()->getPhysicalDeviceFeatures().sampleRateShading;
    numSampleModes = sgl::intlog2(maximumNumberOfSamples) + 1;
    sampleModeSelection = std::min(sgl::intlog2(numSamples), numSampleModes - 1);
    for (int i = 1; i <= maximumNumberOfSamples; i *= 2) {
        sampleModeNames.push_back(std::to_string(i));
    }

    lineRasterPass = std::make_shared<MultisampledLineRasterPass>(this);
    sphereRasterPass = std::make_shared<SphereRasterPass>(sceneData);
}

void OpaqueLineRenderer::setVisualizeSeedingProcess(bool visualizeSeeding) {
    if (this->visualizeSeedingProcess != visualizeSeeding) {
        this->visualizeSeedingProcess = visualizeSeeding;
        if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
            reloadGatherShader();
        }
    }
}

void OpaqueLineRenderer::renderSphere(const glm::vec3& position, float radius, const sgl::Color& color) {
    sphereRasterPass->setSpherePosition(position);
    sphereRasterPass->setSphereRadius(radius);
    sphereRasterPass->setSphereColor(color.getFloatColorRGBA());
    sphereRasterPass->render();
}

void OpaqueLineRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    LineRenderer::reloadGatherShader();
    lineRasterPass->setShaderDirty();
    if (degeneratePointsRasterPass) {
        degeneratePointsRasterPass->setShaderDirty();
    }
    if (sphereRasterPass) {
        sphereRasterPass->setShaderDirty();
    }
}

void OpaqueLineRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);
    lineRasterPass->setLineData(lineData, isNewData);

    hasDegeneratePoints =
            lineData->getType() == DATA_SET_TYPE_STRESS_LINES &&
            static_cast<LineDataStress*>(lineData.get())->getHasDegeneratePoints();
    if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES && hasDegeneratePoints) {
        degeneratePointsRasterPass = std::make_shared<BilldboardSpheresRasterPass>(sceneData);
        degeneratePointsRasterPass->setLineData(lineData, isNewData);
    }

    dirty = false;
    reRender = true;
}

void OpaqueLineRenderer::getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("DIRECT_BLIT_GATHER", ""));
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "GatherDummy.glsl"));
    if (visualizeSeedingProcess) {
        preprocessorDefines.insert(std::make_pair("VISUALIZE_SEEDING_PROCESS", ""));
    }
}

void OpaqueLineRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    sgl::vk::Device* device = renderer->getDevice();
    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    if (numSamples > 1) {
        sgl::vk::ImageSettings imageSettings = (*sceneData->sceneTexture)->getImage()->getImageSettings();
        imageSettings.numSamples = VkSampleCountFlagBits(numSamples);
        imageSettings.usage =
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        colorRenderTargetImage = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);
        //colorRenderTargetImage->getImage()->transitionImageLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

        imageSettings = (*sceneData->sceneDepthTexture)->getImage()->getImageSettings();
        imageSettings.numSamples = VkSampleCountFlagBits(numSamples);
        imageSettings.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        depthRenderTargetImage = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_DEPTH_BIT);
        //depthRenderTargetImage->getImage()->transitionImageLayout(VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_STENCIL_READ_ONLY_OPTIMAL);
    } else {
        colorRenderTargetImage = (*sceneData->sceneTexture)->getImageView();
        depthRenderTargetImage = (*sceneData->sceneDepthTexture)->getImageView();
    }

    lineRasterPass->setRenderTarget(colorRenderTargetImage, depthRenderTargetImage);
    lineRasterPass->recreateSwapchain(width, height);
    if (degeneratePointsRasterPass) {
        degeneratePointsRasterPass->setRenderTarget(colorRenderTargetImage, depthRenderTargetImage);
        degeneratePointsRasterPass->recreateSwapchain(width, height);
    }
    if (sphereRasterPass) {
        sphereRasterPass->setRenderTarget(colorRenderTargetImage, depthRenderTargetImage);
        sphereRasterPass->recreateSwapchain(width, height);
    }
    sphereRasterPass->setRenderTarget(colorRenderTargetImage, depthRenderTargetImage);
}

void OpaqueLineRenderer::onClearColorChanged() {
    lineRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void OpaqueLineRenderer::render() {
    LineRenderer::renderBase();

    lineRasterPass->render();

    if (showDegeneratePoints && hasDegeneratePoints) {
        degeneratePointsRasterPass->setPointWidth(pointWidth);
        degeneratePointsRasterPass->setPointColor(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        degeneratePointsRasterPass->render();
    }

    if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
        LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
        if (lineDataStress->getShallRenderSeedingProcess()) {
            int currentSeedIdx = lineDataStress->getCurrentSeedIdx();
            if (currentSeedIdx >= 0) {
                renderSphere(
                        lineDataStress->getCurrentSeedPosition(), 0.006f,
                        sgl::Color(255, 40, 0));
            }
        }
    }

    //renderHull(); // TODO

    if (useMultisampling) {
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(
                (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        renderer->resolveImage(
                colorRenderTargetImage, (*sceneData->sceneTexture)->getImageView());
    }

    //lineData->setUniformGatherShaderDataOpenGL(gatherShader);
    //setUniformData_Pass(gatherShader);
}

void OpaqueLineRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (degeneratePointsRasterPass && hasDegeneratePoints
            && propertyEditor.addCheckbox("Show Degenerate Points", &showDegeneratePoints)) {
        reRender = true;
    }
    if (degeneratePointsRasterPass && showDegeneratePoints && hasDegeneratePoints) {
        if (propertyEditor.addSliderFloat(
                "Point Width", &pointWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH)) {
            degeneratePointsRasterPass->setPointWidth(pointWidth);
            reRender = true;
        }
    }
    if (maximumNumberOfSamples > 1) {
        if (propertyEditor.addCombo(
                "Samples", &sampleModeSelection, sampleModeNames.data(), numSampleModes)) {
            numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
            useMultisampling = numSamples > 1;
            onResolutionChanged();
            reRender = true;
        }
        if (useMultisampling && supportsSampleShadingRate) {
            if (propertyEditor.addCheckbox("Use Sample Shading", &useSamplingShading)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                onResolutionChanged();
                reRender = true;
            }
            if (propertyEditor.addSliderFloatEdit(
                    "Min. Sample Shading", &minSampleShading,
                    0.0f, 1.0f) == ImGui::EditMode::INPUT_FINISHED) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                onResolutionChanged();
                reRender = true;
            }
        }
    }
}



MultisampledLineRasterPass::MultisampledLineRasterPass(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
}

void MultisampledLineRasterPass::setRenderTarget(
        const sgl::vk::ImageViewPtr& colorImage, const sgl::vk::ImageViewPtr& depthImage) {
    colorRenderTargetImage = colorImage;
    depthRenderTargetImage = depthImage;
    renderTargetChanged = true;
    setDataDirty();
}

void MultisampledLineRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);

    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    framebuffer->setColorAttachment(
            colorRenderTargetImage, 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            depthRenderTargetImage, depthAttachmentState, 1.0f);

    if (renderTargetChanged || !rasterData) {
        framebufferDirty = true;
        dataDirty = true;
    } else {
        rasterData->getGraphicsPipeline()->setCompatibleFramebuffer(framebuffer);
    }
    renderTargetChanged = false;
}

void MultisampledLineRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    LineRasterPass::setGraphicsPipelineInfo(pipelineInfo);
    pipelineInfo.setMinSampleShading(useSamplingShading, minSampleShading);
}



BilldboardSpheresRasterPass::BilldboardSpheresRasterPass(SceneData* sceneData)
        : RasterPass(*sceneData->renderer), sceneData(sceneData), camera(sceneData->camera) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void BilldboardSpheresRasterPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    PointRenderData pointRenderData = static_cast<LineDataStress*>(lineData.get())->getDegeneratePointsRenderData();
    vertexBuffer = pointRenderData.vertexPositionBuffer;
    setDataDirty();
}

void BilldboardSpheresRasterPass::setRenderTarget(
        const sgl::vk::ImageViewPtr& colorImage, const sgl::vk::ImageViewPtr& depthImage) {
    colorRenderTargetImage = colorImage;
    depthRenderTargetImage = depthImage;
    setDataDirty();
}

void BilldboardSpheresRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);

    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    framebuffer->setColorAttachment(
            colorRenderTargetImage, 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    depthAttachmentState.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            depthRenderTargetImage, depthAttachmentState, 1.0f);

    framebufferDirty = true;
    dataDirty = true;
}

void BilldboardSpheresRasterPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            {"Point.Vertex", "Point.Geometry", "Point.Fragment"});
}

void BilldboardSpheresRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::POINT_LIST);
    pipelineInfo.setVertexBufferBinding(0, sizeof(glm::vec3));
    pipelineInfo.setInputAttributeDescription(0, 0, "vertexPosition");
    pipelineInfo.setMinSampleShading(useSamplingShading, minSampleShading);
}

void BilldboardSpheresRasterPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setVertexBuffer(vertexBuffer, "vertexPosition");
    rasterData->setStaticBuffer(uniformDataBuffer, "UniformDataBuffer");
}

void BilldboardSpheresRasterPass::_render() {
    glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
    glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;

    glm::vec3 cameraPosition = (*camera)->getPosition();
    uniformData.cameraPosition = cameraPosition;
    uniformData.foregroundColor = foregroundColor;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT);

    RasterPass::_render();
}



SphereRasterPass::SphereRasterPass(SceneData* sceneData)
        : RasterPass(*sceneData->renderer), sceneData(sceneData), camera(sceneData->camera) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    std::vector<glm::vec3> sphereVertexPositions;
    std::vector<glm::vec3> sphereVertexNormals;
    std::vector<uint32_t> sphereIndices;
    getSphereSurfaceRenderData(
            glm::vec3(0,0,0), 1.0f, 32, 32,
            sphereVertexPositions, sphereVertexNormals, sphereIndices);
}

void SphereRasterPass::setRenderTarget(
        const sgl::vk::ImageViewPtr& colorImage, const sgl::vk::ImageViewPtr& depthImage) {
    colorRenderTargetImage = colorImage;
    depthRenderTargetImage = depthImage;
    setDataDirty();
}

void SphereRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);

    sgl::vk::AttachmentState attachmentState;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    framebuffer->setColorAttachment(
            colorRenderTargetImage, 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    depthAttachmentState.initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            depthRenderTargetImage, depthAttachmentState, 1.0f);

    framebufferDirty = true;
    dataDirty = true;
}

void SphereRasterPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"Sphere.Vertex", "Sphere.Fragment"});
}

void SphereRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setVertexBufferBinding(0, sizeof(glm::vec3));
    pipelineInfo.setInputAttributeDescription(0, 0, "vertexPosition");
    pipelineInfo.setInputAttributeDescription(1, 0, "vertexNormal");
    pipelineInfo.setMinSampleShading(useSamplingShading, minSampleShading);
}

void SphereRasterPass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(indexBuffer);
    rasterData->setVertexBuffer(vertexPositionBuffer, "vertexPosition");
    rasterData->setVertexBuffer(vertexNormalBuffer, "vertexNormal");
    rasterData->setStaticBuffer(uniformDataBuffer, "UniformDataBuffer");
}

void SphereRasterPass::_render() {
    glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
    glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;

    glm::vec3 cameraPosition = (*camera)->getPosition();
    uniformData.cameraPosition = cameraPosition;
    uniformData.backgroundColor = backgroundColor;
    uniformData.foregroundColor = foregroundColor;
    uniformDataBuffer->updateData(sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT);

    RasterPass::_render();
}
