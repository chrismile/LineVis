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

#include "DeferredRenderer.hpp"

#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include <utility>

#include "LineData/LineDataStress.hpp"
#include "../HullRasterPass.hpp"
#include "DeferredRenderer.hpp"

DeferredRenderer::DeferredRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opaque Line Renderer", sceneData, transferFunctionWindow) {
    const auto& meshShaderFeatures =
            (*sceneData->renderer)->getDevice()->getPhysicalDeviceMeshShaderFeaturesNV();
    const auto& meshShaderProperties =
            (*sceneData->renderer)->getDevice()->getPhysicalDeviceMeshShaderPropertiesNV();
    supportsTaskMeshShaders = meshShaderFeatures.taskShader && meshShaderFeatures.meshShader;
    supportsDrawIndirectCount = sgl::AppSettings::get()->getPrimaryDevice()->isDeviceExtensionSupported(
            VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME);
    meshWorkgroupSize = meshShaderProperties.maxMeshWorkGroupSize[0];
}

void DeferredRenderer::initialize() {
    LineRenderer::initialize();

    nodeCullingUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(NodeCullingUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    visibilityBufferDrawIndexedPass = std::make_shared<VisibilityBufferDrawIndexedPass>(this);

    deferredResolvePass = std::shared_ptr<ResolvePass>(new DeferredResolvePass(
            this, {"DeferredShading.Vertex", "DeferredShading.Fragment"}));
    deferredResolvePass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    downsampleBlitPass = std::make_shared<DownsampleBlitPass>(renderer);

    onClearColorChanged();
}

void DeferredRenderer::reloadShaders() {
    reloadGatherShader();
    reloadResolveShader();
}

void DeferredRenderer::reloadResolveShader() {
    deferredResolvePass->setShaderDirty();
}

void DeferredRenderer::reloadGatherShader() {
    LineRenderer::reloadGatherShader();
    if (visibilityBufferDrawIndexedPass) {
        visibilityBufferDrawIndexedPass->setShaderDirty();
    }
}

void DeferredRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    if (visibilityBufferDrawIndexedPass) {
        visibilityBufferDrawIndexedPass->setLineData(lineData, isNewData);
    }

    reloadResolveShader();
    deferredResolvePass->setDataDirty();

    if (hullRasterPass) {
        hullRasterPass->setAttachmentLoadOp(
                visibilityBufferDrawIndexedPass->getIsDataEmpty()
                ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD);
    }

    dirty = false;
    reRender = true;
}

void DeferredRenderer::getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("DIRECT_BLIT_GATHER", ""));
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "GatherDummy.glsl"));
    if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        preprocessorDefines.insert(std::make_pair("WORKGROUP_SIZE", std::to_string(meshWorkgroupSize)));
    }
}

void DeferredRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
}

void DeferredRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);

    if (renderData->getShaderStages()->getHasModuleId("DeferredShading.Fragment")) {
        lineData->setVulkanRenderDataDescriptors(renderData);

        TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderData(
                true, false);
        if (tubeRenderData.indexBuffer) {
            renderData->setStaticBuffer(tubeRenderData.indexBuffer, "TriangleIndexBuffer");
            renderData->setStaticBuffer(tubeRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
            renderData->setStaticBuffer(tubeRenderData.linePointDataBuffer, "LinePointDataBuffer");
            if (tubeRenderData.multiVarAttributeDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.multiVarAttributeDataBuffer, "AttributeDataArrayBuffer");
            }
            if (tubeRenderData.stressLinePointDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.stressLinePointDataBuffer,
                        "StressLinePointDataBuffer");
            }
            if (tubeRenderData.stressLinePointPrincipalStressDataBuffer) {
                renderData->setStaticBufferOptional(
                        tubeRenderData.stressLinePointPrincipalStressDataBuffer,
                        "StressLinePointPrincipalStressDataBuffer");
            }
        }
        renderData->setStaticTexture(primitiveIndexTexture, "primitiveIndexBuffer");
        renderData->setStaticTexture(depthBufferTexture, "depthBuffer");
    }
}

void DeferredRenderer::setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    if (framebufferMode == FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_PASS) {
        // Visibility pass.
        sgl::vk::AttachmentState primitiveIndexAttachmentState;
        primitiveIndexAttachmentState.loadOp = loadOp;
        primitiveIndexAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        framebuffer->setColorAttachmentUint(
                primitiveIndexImage, 0, primitiveIndexAttachmentState,
                glm::uvec4(std::numeric_limits<uint32_t>::max()));

        sgl::vk::AttachmentState depthAttachmentState;
        depthAttachmentState.loadOp = loadOp;
        depthAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        framebuffer->setDepthStencilAttachment(
                depthRenderTargetImage, depthAttachmentState, 1.0f);
    } else if (framebufferMode == FramebufferMode::DEFERRED_RESOLVE_PASS) {
        // Deferred pass.
        sgl::vk::AttachmentState colorAttachmentState;
        colorAttachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE; // TODO: VK_ATTACHMENT_LOAD_OP_CLEAR?
        colorAttachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        framebuffer->setColorAttachment(
                colorRenderTargetImage, 0, colorAttachmentState,
                sceneData->clearColor->getFloatColorRGBA());
    } else if (framebufferMode == FramebufferMode::HULL_RASTER_PASS) {
        // Hull pass.
        sgl::vk::AttachmentState colorAttachmentState;
        colorAttachmentState.loadOp = loadOp;
        colorAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        framebuffer->setColorAttachment(
                colorRenderTargetImage, 0, colorAttachmentState,
                sceneData->clearColor->getFloatColorRGBA());

        sgl::vk::AttachmentState depthAttachmentState;
        depthAttachmentState.loadOp = loadOp;
        depthAttachmentState.initialLayout =
                loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
                VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        framebuffer->setDepthStencilAttachment(
                depthRenderTargetImage, depthAttachmentState, 1.0f);
    } else {
        sgl::Logfile::get()->throwError(
                "Error in DeferredRenderer::setFramebufferAttachments: Invalid framebuffer mode.");
    }
}

void DeferredRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    sgl::vk::Device* device = renderer->getDevice();
    auto scalingFactor = uint32_t(getResolutionIntegerScalingFactor());
    renderWidth = *sceneData->viewportWidth * scalingFactor;
    renderHeight = *sceneData->viewportHeight * scalingFactor;
    finalWidth = *sceneData->viewportWidth;
    finalHeight = *sceneData->viewportHeight;

    sgl::vk::ImageSamplerSettings samplerSettings;
    samplerSettings.minFilter = VK_FILTER_NEAREST;
    samplerSettings.magFilter = VK_FILTER_NEAREST;
    samplerSettings.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;

    sgl::vk::ImageSettings imageSettings = (*sceneData->sceneTexture)->getImage()->getImageSettings();
    imageSettings.width = renderWidth;
    imageSettings.height = renderHeight;
    imageSettings.format = VK_FORMAT_R32_UINT;
    imageSettings.usage =
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    primitiveIndexImage = std::make_shared<sgl::vk::ImageView>(
            std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);
    primitiveIndexTexture = std::make_shared<sgl::vk::Texture>(primitiveIndexImage, samplerSettings);

    // Create scene depth texture.
    imageSettings.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageSettings.format = device->getSupportedDepthFormat();
    // https://registry.khronos.org/OpenGL/extensions/ARB/ARB_texture_non_power_of_two.txt
    imageSettings.mipLevels = 1 + uint32_t(std::floor(std::log2(std::max(int(renderWidth), int(renderHeight)))));
    depthBufferTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, samplerSettings, VK_IMAGE_ASPECT_DEPTH_BIT);

    depthMipLevelImageViews.clear();
    depthMipLevelImageViews.resize(imageSettings.mipLevels);
    depthMipLevelTextures.clear();
    depthMipLevelTextures.resize(imageSettings.mipLevels);
    for (uint32_t level = 0; level < imageSettings.mipLevels; level++) {
        depthMipLevelImageViews.at(level) = std::make_shared<sgl::vk::ImageView>(
                depthBufferTexture->getImage(), VK_IMAGE_VIEW_TYPE_2D,
                level, 1, 0, 1,
                VK_IMAGE_ASPECT_DEPTH_BIT);
        sgl::vk::ImageSamplerSettings samplerSettingsMipLevel = samplerSettings;
        depthMipLevelTextures.at(level) = std::make_shared<sgl::vk::Texture>(
                depthMipLevelImageViews.at(level), samplerSettingsMipLevel);
    }
    depthRenderTargetImage = depthMipLevelTextures.at(0)->getImageView();
    depthMipBlitRenderPasses.clear();
    depthMipBlitRenderPasses.resize(imageSettings.mipLevels - 1);
    auto mipWidth = renderWidth;
    auto mipHeight = renderHeight;
    for (uint32_t level = 0; level < imageSettings.mipLevels - 1; level++) {
        if (mipWidth > 1) mipWidth /= 2;
        if (mipHeight > 1) mipHeight /= 2;
        sgl::vk::BlitRenderPassPtr depthMipBlitRenderPass = std::make_shared<sgl::vk::BlitRenderPass>(
                renderer, std::vector<std::string>{ "GenerateHZB.Vertex", "GenerateHZB.Fragment" });
        depthMipBlitRenderPass->setColorWriteEnabled(false);
        depthMipBlitRenderPass->setDepthWriteEnabled(true);
        depthMipBlitRenderPass->setDepthTestEnabled(true);
        depthMipBlitRenderPass->setDepthCompareOp(VK_COMPARE_OP_ALWAYS);
        depthMipBlitRenderPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
        depthMipBlitRenderPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_STORE);
        depthMipBlitRenderPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
        depthMipBlitRenderPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        depthMipBlitRenderPass->setInputTexture(depthMipLevelTextures.at(level));
        depthMipBlitRenderPass->setOutputImage(depthMipLevelImageViews.at(level + 1));
        depthMipBlitRenderPass->recreateSwapchain(mipWidth, mipHeight);
        depthMipBlitRenderPasses.at(level) = depthMipBlitRenderPass;
    }

    if (hullRasterPass) {
        framebufferMode = FramebufferMode::HULL_RASTER_PASS;
        hullRasterPass->recreateSwapchain(renderWidth, renderHeight);
    }
    if (visibilityBufferDrawIndexedPass) {
        framebufferMode = FramebufferMode::VISIBILITY_BUFFER_DRAW_INDEXED_PASS;
        visibilityBufferDrawIndexedPass->recreateSwapchain(renderWidth, renderHeight);
    }

    framebufferMode = FramebufferMode::DEFERRED_RESOLVE_PASS;
    if (supersamplingMode == 0) {
        // No supersampling, thus we can directly draw the result to the scene data color image.
        colorRenderTargetImage = (*sceneData->sceneTexture)->getImageView();
        colorRenderTargetTexture = {};
    } else {
        // Use intermediate high-resolution texture, which is then downsampled in the next step.
        imageSettings.format = (*sceneData->sceneTexture)->getImage()->getImageSettings().format;
        imageSettings.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        imageSettings.mipLevels = 1;
        samplerSettings.minFilter = VK_FILTER_LINEAR;
        samplerSettings.magFilter = VK_FILTER_LINEAR;
        samplerSettings.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        colorRenderTargetImage = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);
        colorRenderTargetTexture = std::make_shared<sgl::vk::Texture>(
                colorRenderTargetImage, samplerSettings);

        downsampleBlitPass->setScalingFactor(int(scalingFactor));
        downsampleBlitPass->setInputTexture(colorRenderTargetTexture);
        downsampleBlitPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
        downsampleBlitPass->recreateSwapchain(finalWidth, finalHeight);
    }

    deferredResolvePass->setOutputImage(colorRenderTargetImage);
    deferredResolvePass->recreateSwapchain(renderWidth, renderHeight);
}

void DeferredRenderer::onClearColorChanged() {
    deferredResolvePass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
    if (hullRasterPass && !hullRasterPass->getIsDataEmpty()) {
        framebufferMode = FramebufferMode::HULL_RASTER_PASS;
        hullRasterPass->updateFramebuffer();
    }
}

void DeferredRenderer::setUniformData() {
    nodeCullingUniformData.modelViewProjectionMatrix =
            sceneData->camera->getProjectionMatrix() * sceneData->camera->getViewMatrix();
    nodeCullingUniformData.viewportSize = glm::ivec2(renderWidth, renderHeight);
    nodeCullingUniformData.numMeshlets = 0; // TODO
    nodeCullingUniformData.rootNodeIdx = 0; // TODO
    nodeCullingUniformDataBuffer->updateData(
            sizeof(NodeCullingUniformData), &nodeCullingUniformData,
            renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void DeferredRenderer::render() {
    LineRenderer::renderBase();
    setUniformData();

    // TODO
    //if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDEXED)
    visibilityBufferDrawIndexedPass->buildIfNecessary();
    if (visibilityBufferDrawIndexedPass->getIsDataEmpty()) {
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        colorRenderTargetImage->clearColor(
                sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

        renderer->transitionImageLayout(
                depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        depthRenderTargetImage->clearDepthStencil(
                1.0f, 0, renderer->getVkCommandBuffer());
    } else {
        visibilityBufferDrawIndexedPass->render();
        renderer->transitionImageLayout(
                primitiveIndexImage->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        renderer->transitionImageLayout(
                depthRenderTargetImage, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        for (uint32_t level = 0; level < uint32_t(depthMipBlitRenderPasses.size()); level++) {
            depthMipBlitRenderPasses.at(level)->render();
        }

        deferredResolvePass->render();
    }

    //renderHull();
    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        // Can't use transitionImageLayout, as subresource transition by HZB build leaves wrong internal layout set.
        //renderer->transitionImageLayout(
        //        depthRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
        renderer->insertImageMemoryBarrier(
                depthRenderTargetImage->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT);
        hullRasterPass->render();
    }

    if (supersamplingMode != 0) {
        renderer->transitionImageLayout(
                colorRenderTargetImage->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        downsampleBlitPass->render();
    }
}

void DeferredRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    int numRenderingModes = IM_ARRAYSIZE(deferredRenderingModeNames);
    if (!supportsDrawIndirectCount) {
        numRenderingModes -= 3;
    } else if (!supportsTaskMeshShaders) {
        numRenderingModes -= 1;
    }
    if (propertyEditor.addCombo(
            "Deferred Mode", (int*)&deferredRenderingMode,
            deferredRenderingModeNames, numRenderingModes)) {
        reloadGatherShader();
        onResolutionChanged();
        reRender = true;
    }

    if (propertyEditor.addCombo(
            "Supersampling", &supersamplingMode,
            supersamplingModeNames, IM_ARRAYSIZE(supersamplingModeNames))) {
        onResolutionChanged();
        reRender = true;
    }

    if (deferredRenderingMode == DeferredRenderingMode::DRAW_INDIRECT) {
        if (propertyEditor.addCombo(
                "Reduction Mode", (int*)&drawIndirectReductionMode,
                drawIndirectReductionModeNames, IM_ARRAYSIZE(drawIndirectReductionModeNames))) {
            reloadGatherShader();
            reRender = true;
        }
        if (propertyEditor.addCombo(
                "Geometry Mode", (int*)&drawIndirectGeometryMode,
                drawIndirectGeometryModeNames, IM_ARRAYSIZE(drawIndirectGeometryModeNames))) {
            reloadGatherShader();
            reRender = true;
        }
    } else if (deferredRenderingMode == DeferredRenderingMode::TASK_MESH_SHADER) {
        if (propertyEditor.addCombo(
                "Geometry Mode", (int*)&taskMeshShaderGeometryMode,
                taskMeshShaderGeometryModeNames, IM_ARRAYSIZE(taskMeshShaderGeometryModeNames))) {
            reloadGatherShader();
            reRender = true;
        }
    }
}

void DeferredRenderer::setNewState(const InternalState& newState) {
    std::string deferredRenderingModeString;
    if (newState.rendererSettings.getValueOpt("deferredRenderingMode", deferredRenderingModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(deferredRenderingModeNames); i++) {
            if (deferredRenderingModeString == deferredRenderingModeNames[i]) {
                deferredRenderingMode = DeferredRenderingMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndirectReductionModeString;
    if (newState.rendererSettings.getValueOpt("drawIndirectReductionMode", drawIndirectReductionModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndirectReductionModeNames); i++) {
            if (drawIndirectReductionModeString == drawIndirectReductionModeNames[i]) {
                drawIndirectReductionMode = DrawIndirectReductionMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndirectGeometryModeString;
    if (newState.rendererSettings.getValueOpt("drawIndirectGeometryMode", drawIndirectGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndirectGeometryModeNames); i++) {
            if (drawIndirectGeometryModeString == drawIndirectGeometryModeNames[i]) {
                drawIndirectGeometryMode = DrawIndirectGeometryMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string taskMeshShaderGeometryModeString;
    if (newState.rendererSettings.getValueOpt("taskMeshShaderGeometryMode", taskMeshShaderGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(taskMeshShaderGeometryModeNames); i++) {
            if (taskMeshShaderGeometryModeString == taskMeshShaderGeometryModeNames[i]) {
                taskMeshShaderGeometryMode = TaskMeshShaderGeometryMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }
}

bool DeferredRenderer::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = LineRenderer::setNewSettings(settings);

    std::string deferredRenderingModeString;
    if (settings.getValueOpt("deferred_rendering_mode", deferredRenderingModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(deferredRenderingModeNames); i++) {
            if (deferredRenderingModeString == deferredRenderingModeNames[i]) {
                deferredRenderingMode = DeferredRenderingMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndirectReductionModeString;
    if (settings.getValueOpt("draw_indirect_reduction_mode", drawIndirectReductionModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndirectReductionModeNames); i++) {
            if (drawIndirectReductionModeString == drawIndirectReductionModeNames[i]) {
                drawIndirectReductionMode = DrawIndirectReductionMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string drawIndirectGeometryModeString;
    if (settings.getValueOpt("draw_indirect_geometry_mode", drawIndirectGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(drawIndirectGeometryModeNames); i++) {
            if (drawIndirectGeometryModeString == drawIndirectGeometryModeNames[i]) {
                drawIndirectGeometryMode = DrawIndirectGeometryMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    std::string taskMeshShaderGeometryModeString;
    if (settings.getValueOpt("task_mesh_shader_geometry_mode", taskMeshShaderGeometryModeString)) {
        for (int i = 0; i < IM_ARRAYSIZE(taskMeshShaderGeometryModeNames); i++) {
            if (taskMeshShaderGeometryModeString == taskMeshShaderGeometryModeNames[i]) {
                taskMeshShaderGeometryMode = TaskMeshShaderGeometryMode(i);
                reloadGatherShader();
                reRender = true;
                break;
            }
        }
    }

    return shallReloadGatherShader;
}


DeferredResolvePass::DeferredResolvePass(LineRenderer* lineRenderer, std::vector<std::string> customShaderIds)
        : ResolvePass(lineRenderer, std::move(customShaderIds)) {}

void DeferredResolvePass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineRenderer->getLineData()->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    // Resolve passes don't need fragment shader interlock.
    auto it = preprocessorDefines.find("__extensions");
    if (it != preprocessorDefines.end()) {
        if (it->second == "GL_ARB_fragment_shader_interlock") {
            preprocessorDefines.erase(it);
        }
    }
    preprocessorDefines.insert(std::make_pair("RESOLVE_PASS", ""));

    shaderStages = sgl::vk::ShaderManager->getShaderStages(shaderIds, preprocessorDefines);
}



VisibilityBufferDrawIndexedPass::VisibilityBufferDrawIndexedPass(LineRenderer* lineRenderer)
        : LineRasterPass(lineRenderer) {
}

void VisibilityBufferDrawIndexedPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    std::vector<std::string> shaderModuleNames = {
            "VisibilityBufferDrawIndexed.Vertex",
            "VisibilityBufferDrawIndexed.Fragment"
    };
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            shaderModuleNames, preprocessorDefines);
}

void VisibilityBufferDrawIndexedPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);
    pipelineInfo.setVertexBufferBindingByLocationIndex(
            "vertexPosition", sizeof(TubeTriangleVertexData));

    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    if (lineData->getUseCappedTubes()) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }

    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setDepthTestEnabled(true);
    pipelineInfo.setDepthWriteEnabled(true);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::OVERWRITE);
}

void VisibilityBufferDrawIndexedPass::createRasterData(
        sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    //lineRenderer->setRenderDataBindings(rasterData);

    TubeTriangleRenderData tubeRenderData = lineData->getLinePassTubeTriangleMeshRenderData(
            true, false);

    if (!tubeRenderData.indexBuffer) {
        return;
    }

    rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
    rasterData->setVertexBuffer(tubeRenderData.vertexBuffer, "vertexPosition");
}



DownsampleBlitPass::DownsampleBlitPass(sgl::vk::Renderer* renderer) : sgl::vk::BlitRenderPass(
        renderer, { "DownsampleBlit.Vertex", "DownsampleBlit.Fragment" }) {
}

void DownsampleBlitPass::_render() {
    renderer->pushConstants(
            rasterData->getGraphicsPipeline(), VK_SHADER_STAGE_FRAGMENT_BIT,
            0, scalingFactor);
    BlitRenderPass::_render();
}
