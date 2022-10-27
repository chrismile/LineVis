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
#include "HullRasterPass.hpp"
#include "OpaqueLineRenderer.hpp"

OpaqueLineRenderer::OpaqueLineRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opaque Renderer", sceneData, transferFunctionWindow) {
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
    supportsGeometryShaders = (*sceneData->renderer)->getDevice()->getPhysicalDeviceFeatures().geometryShader;

    sphereRasterPass = std::make_shared<SphereRasterPass>(this);
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

void OpaqueLineRenderer::reloadGatherShader() {
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

    hasDegeneratePoints =
            lineData->getType() == DATA_SET_TYPE_STRESS_LINES &&
            static_cast<LineDataStress*>(lineData.get())->getHasDegeneratePoints();
    if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES && hasDegeneratePoints && supportsGeometryShaders) {
        degeneratePointsRasterPass = std::make_shared<BilldboardSpheresRasterPass>(this);
        degeneratePointsRasterPass->setLineData(lineData, isNewData);
    }

    /*if (hullRasterPass) {
        hullRasterPass->setAttachmentLoadOp(
                lineRasterPass->getIsDataEmpty() ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD);
    }*/

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

void OpaqueLineRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setMinSampleShading(useSamplingShading, minSampleShading);
}

void OpaqueLineRenderer::setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = loadOp;
    attachmentState.initialLayout =
            loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
            VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    framebuffer->setColorAttachment(
            colorRenderTargetImage, 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = loadOp;
    depthAttachmentState.initialLayout =
            loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
            VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            depthRenderTargetImage, depthAttachmentState, 1.0f);
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
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        colorRenderTargetImage = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);

        imageSettings = (*sceneData->sceneDepthTexture)->getImage()->getImageSettings();
        imageSettings.numSamples = VkSampleCountFlagBits(numSamples);
        imageSettings.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        depthRenderTargetImage = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_DEPTH_BIT);
    } else {
        colorRenderTargetImage = (*sceneData->sceneTexture)->getImageView();
        depthRenderTargetImage = (*sceneData->sceneDepthTexture)->getImageView();
    }

    lineRasterPass->recreateSwapchain(width, height);
    if (hullRasterPass) {
        hullRasterPass->recreateSwapchain(width, height);
    }
    if (degeneratePointsRasterPass) {
        degeneratePointsRasterPass->recreateSwapchain(width, height);
    }
    if (sphereRasterPass) {
        sphereRasterPass->recreateSwapchain(width, height);
    }
}

void OpaqueLineRenderer::onClearColorChanged() {
    if (lineRasterPass && !lineRasterPass->getIsDataEmpty()) {
        lineRasterPass->updateFramebuffer();
    } else if (hullRasterPass && !hullRasterPass->getIsDataEmpty()) {
        hullRasterPass->updateFramebuffer();
    }
}

void OpaqueLineRenderer::render() {
    LineRenderer::renderBase();

    lineRasterPass->buildIfNecessary();
    if (lineRasterPass->getIsDataEmpty()) {
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        colorRenderTargetImage->clearColor(
                sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

        renderer->transitionImageLayout(
                depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        depthRenderTargetImage->clearDepthStencil(1.0f, 0, renderer->getVkCommandBuffer());
        renderer->transitionImageLayout(
                depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
    } else {
        lineRasterPass->render();
    }

    if (showDegeneratePoints && hasDegeneratePoints && supportsGeometryShaders) {
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

    renderHull();

    if (useMultisampling) {
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(
                (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        renderer->resolveImage(
                colorRenderTargetImage, (*sceneData->sceneTexture)->getImageView());
    }
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

void OpaqueLineRenderer::setNewState(const InternalState& newState) {
    if (maximumNumberOfSamples > 1) {
        if (newState.rendererSettings.getValueOpt("numSamples", numSamples)) {
            sampleModeSelection = std::min(sgl::intlog2(numSamples), numSampleModes - 1);
            useMultisampling = numSamples > 1;
            if ((*sceneData->sceneTexture)) {
                onResolutionChanged();
            }
            reRender = true;
        }
        if (useMultisampling && supportsSampleShadingRate) {
            if (newState.rendererSettings.getValueOpt("useSamplingShading", useSamplingShading)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                if ((*sceneData->sceneTexture)) {
                    onResolutionChanged();
                }
                reRender = true;
            }
            if (newState.rendererSettings.getValueOpt("minSampleShading", minSampleShading)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                if ((*sceneData->sceneTexture)) {
                    onResolutionChanged();
                }
                reRender = true;
            }
        }
    }
}

bool OpaqueLineRenderer::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = LineRenderer::setNewSettings(settings);

    if (maximumNumberOfSamples > 1) {
        if (settings.getValueOpt("num_samples", numSamples)) {
            sampleModeSelection = std::min(sgl::intlog2(numSamples), numSampleModes - 1);
            useMultisampling = numSamples > 1;
            if ((*sceneData->sceneTexture)) {
                onResolutionChanged();
            }
            reRender = true;
        }
        if (useMultisampling && supportsSampleShadingRate) {
            if (settings.getValueOpt("use_sampling_shading", useSamplingShading)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                if ((*sceneData->sceneTexture)) {
                    onResolutionChanged();
                }
                reRender = true;
            }
            if (settings.getValueOpt("min_sample_shading", minSampleShading)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                if ((*sceneData->sceneTexture)) {
                    onResolutionChanged();
                }
                reRender = true;
            }
        }
    }

    return shallReloadGatherShader;
}



BilldboardSpheresRasterPass::BilldboardSpheresRasterPass(LineRenderer* lineRenderer)
        : RasterPass(*lineRenderer->getSceneData()->renderer),
        lineRenderer(lineRenderer), sceneData(lineRenderer->getSceneData()), camera(&sceneData->camera) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void BilldboardSpheresRasterPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    PointRenderData pointRenderData = static_cast<LineDataStress*>(lineData.get())->getDegeneratePointsRenderData();
    vertexBuffer = pointRenderData.vertexPositionBuffer;
    setDataDirty();
}

void BilldboardSpheresRasterPass::setAttachmentLoadOp(VkAttachmentLoadOp loadOp) {
    this->attachmentLoadOp = loadOp;
    recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void BilldboardSpheresRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);
    lineRenderer->setFramebufferAttachments(framebuffer, attachmentLoadOp);

    framebufferDirty = true;
    dataDirty = true;
}

void BilldboardSpheresRasterPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            {"Point.Vertex", "Point.Geometry", "Point.Fragment"});
}

void BilldboardSpheresRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::POINT_LIST);
    pipelineInfo.setVertexBufferBinding(0, sizeof(glm::vec3));
    pipelineInfo.setInputAttributeDescription(0, 0, "vertexPosition");
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



SphereRasterPass::SphereRasterPass(LineRenderer* lineRenderer)
        : RasterPass(*lineRenderer->getSceneData()->renderer),
          lineRenderer(lineRenderer), sceneData(lineRenderer->getSceneData()), camera(&sceneData->camera) {
    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData), VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    std::vector<glm::vec3> sphereVertexPositions;
    std::vector<glm::vec3> sphereVertexNormals;
    std::vector<uint32_t> sphereIndices;
    getSphereSurfaceRenderData(
            glm::vec3(0,0,0), 1.0f, 32, 32,
            sphereVertexPositions, sphereVertexNormals, sphereIndices);

    vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(glm::vec3) * sphereVertexPositions.size(), sphereVertexPositions.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    vertexNormalBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(glm::vec3) * sphereVertexNormals.size(), sphereVertexNormals.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(uint32_t) * sphereIndices.size(), sphereIndices.data(),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void SphereRasterPass::setAttachmentLoadOp(VkAttachmentLoadOp loadOp) {
    this->attachmentLoadOp = loadOp;
    recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void SphereRasterPass::recreateSwapchain(uint32_t width, uint32_t height) {
    framebuffer = std::make_shared<sgl::vk::Framebuffer>(device, width, height);
    lineRenderer->setFramebufferAttachments(framebuffer, attachmentLoadOp);

    framebufferDirty = true;
    dataDirty = true;
}

void SphereRasterPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"Sphere.Vertex", "Sphere.Fragment"});
}

void SphereRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    pipelineInfo.setVertexBufferBinding(0, sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexPosition", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexNormal", sizeof(glm::vec3));
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
