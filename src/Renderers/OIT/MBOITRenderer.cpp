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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Scene/Camera.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "MBOITRenderer.hpp"

// Internal mode
bool MBOITRenderer::usePowerMoments = true;
int MBOITRenderer::numMoments = 4;
MBOITPixelFormat MBOITRenderer::pixelFormat = MBOIT_PIXEL_FORMAT_FLOAT_32;
bool MBOITRenderer::USE_R_RG_RGBA_FOR_MBOIT6 = true;
float MBOITRenderer::overestimationBeta = 0.1f;

MBOITRenderer::MBOITRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Moment-Based Order Independent Transparency",
                       sceneData, transferFunctionWindow) {
}

void MBOITRenderer::initialize() {
    LineRenderer::initialize();

    syncMode = getSupportedSyncMode(renderer->getDevice());

    uniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    // Create moment OIT uniform data buffer.
    momentUniformData.moment_bias = 5*1e-7f;
    momentUniformData.overestimation = overestimationBeta;
    computeWrappingZoneParameters(momentUniformData.wrapping_zone_parameters);
    momentOITUniformBuffer =  std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(MomentOITUniformData), &momentUniformData,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    resolveRasterPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"MBOITBlend.Vertex", "MBOITBlend.Fragment"}));
    resolveRasterPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);

    mboitPass1 = std::make_shared<MBOITPass1>(this);
    mboitPass1->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    mboitPass2 = std::make_shared<MBOITPass2>(this);
    mboitPass2->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
    mboitPass2->setUpdateUniformData(false);

    // Disable render passes of parent class.
    lineRasterPass = {};
    hullRasterPass = {};

    onClearColorChanged();
}

void MBOITRenderer::updateSyncMode() {
    checkSyncModeSupported(sceneData, renderer->getDevice(), syncMode);

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    spinlockViewportBuffer = {};
    if (syncMode == SYNC_SPINLOCK) {
        spinlockViewportBuffer = std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), sizeof(uint32_t) * size_t(paddedWidth) * size_t(paddedHeight),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

        // Set all values in the buffer to zero.
        spinlockViewportBuffer->fill(0, renderer->getVkCommandBuffer());
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                spinlockViewportBuffer);
    }
}

void MBOITRenderer::reloadShaders() {
    if (lineData) {
        reloadGatherShader();
    }
    reloadResolveShader();
}

void MBOITRenderer::reloadGatherShader() {
    mboitPass1->setShaderDirty();
    mboitPass2->setShaderDirty();
}

void MBOITRenderer::reloadResolveShader() {
    resolveRasterPass->setShaderDirty();
}

void MBOITRenderer::updateMomentMode() {
    // Reload the shaders.
    reloadShaders();

    // Create moment images.
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);

    VkFormat format1 = VK_FORMAT_R32_SFLOAT;
    VkFormat format2 =
            pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32 ? VK_FORMAT_R32G32_SFLOAT : VK_FORMAT_R16G16_UNORM;
    VkFormat format4 =
            pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32 ? VK_FORMAT_R32G32B32A32_SFLOAT : VK_FORMAT_R16G16B16A16_UNORM;

    int depthB0 = 1;
    int depthB = 1;
    int depthBExtra = 0;
    VkFormat formatB0 = format1;
    VkFormat formatB = format4;
    VkFormat formatBExtra = format4;

    if (numMoments == 6) {
        if (USE_R_RG_RGBA_FOR_MBOIT6) {
            depthBExtra = 1;
            formatB = format2;
            formatBExtra = format4;
        } else {
            depthB = 3;
            formatB = format2;
        }
    } else if (numMoments == 8) {
        depthB = 2;
    }

    momentImageArray.clear();

    imageSettingsB0 = {};
    imageSettingsB0.format = formatB0;
    imageSettingsB0.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageSettingsB0.width = width;
    imageSettingsB0.height = height;
    imageSettingsB0.arrayLayers = depthB0;
    b0 = std::make_shared<sgl::vk::ImageView>(
            std::make_shared<sgl::vk::Image>(renderer->getDevice(), imageSettingsB0),
            VK_IMAGE_VIEW_TYPE_2D_ARRAY);
    renderer->transitionImageLayout(b0->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    b0->clearColor({ 0.0f, 0.0f, 0.0f, 0.0f }, renderer->getVkCommandBuffer());
    momentImageArray.push_back(b0->getImage());

    imageSettingsB = imageSettingsB0;
    imageSettingsB.format = formatB;
    imageSettingsB.arrayLayers = depthB;
    b = std::make_shared<sgl::vk::ImageView>(
            std::make_shared<sgl::vk::Image>(renderer->getDevice(), imageSettingsB),
            VK_IMAGE_VIEW_TYPE_2D_ARRAY);
    renderer->transitionImageLayout(b->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    b->clearColor({ 0.0f, 0.0f, 0.0f, 0.0f }, renderer->getVkCommandBuffer());
    momentImageArray.push_back(b->getImage());

    if (numMoments == 6 && USE_R_RG_RGBA_FOR_MBOIT6) {
        imageSettingsBExtra = imageSettingsB0;
        imageSettingsBExtra.format = formatBExtra;
        imageSettingsB.arrayLayers = depthBExtra;
        bExtra = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(renderer->getDevice(), imageSettingsBExtra),
                VK_IMAGE_VIEW_TYPE_2D_ARRAY);
        renderer->transitionImageLayout(bExtra->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        bExtra->clearColor({ 0.0f, 0.0f, 0.0f, 0.0f }, renderer->getVkCommandBuffer());
        momentImageArray.push_back(bExtra->getImage());
    }

    renderer->insertImageMemoryBarriers(
            momentImageArray, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            VK_ACCESS_TRANSFER_WRITE_BIT,
            VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);

    size_t baseSizeBytes = pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32 ? 4 : 2;
    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(
                baseSizeBytes * numMoments * width * height + 4 * width * height);
    }

    // Set algorithm-dependent bias.
    if (usePowerMoments) {
        if (numMoments == 4 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 6*1e-4f;    // 6*1e-5
        } else if (numMoments == 4 && pixelFormat ==  MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 5*1e-7f;    // 5*1e-7
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 6*1e-3f;    // 6*1e-4
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 5*1e-6f;    // 5*1e-6
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 2.5f*1e-2f; // 2.5*1e-3
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 5*1e-5f;    // 5*1e-5
        }
    } else {
        if (numMoments == 4 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 4*1e-3f;    // 4*1e-4
        } else if (numMoments == 4 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 4*1e-7f;    // 4*1e-7
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 6.5f*1e-3f; // 6.5*1e-4
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 8*1e-6f;    // 8*1e-7
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 8.5f*1e-3f; // 8.5*1e-4
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 1.5f*1e-5f; // 1.5*1e-6;
        }
    }

    momentOITUniformBuffer->updateData(
            sizeof(MomentOITUniformData), &momentUniformData,
            renderer->getVkCommandBuffer());
}

void MBOITRenderer::setNewState(const InternalState& newState) {
    numMoments = newState.rendererSettings.getIntValue("numMoments");
    pixelFormat =
            newState.rendererSettings.getValue("pixelFormat") == "Float"
            ? MBOIT_PIXEL_FORMAT_FLOAT_32 : MBOIT_PIXEL_FORMAT_UNORM_16;
    if (numMoments == 6) {
        USE_R_RG_RGBA_FOR_MBOIT6 = newState.rendererSettings.getBoolValue("USE_R_RG_RGBA_FOR_MBOIT6");
    }
    usePowerMoments = newState.rendererSettings.getBoolValue("usePowerMoments");

    if (newState.rendererSettings.getValueOpt("overestimationBeta", overestimationBeta)) {
        momentUniformData.overestimation = overestimationBeta;
        // subData already called in updateMomentMode
        //momentOITUniformBuffer->subData(0, sizeof(MomentOITUniformData), &momentUniformData);
    } else {
        overestimationBeta = 0.1f;
        momentUniformData.overestimation = overestimationBeta;
    }

    bool recompileGatherShader = false;
    if (newState.rendererSettings.getValueOpt(
            "useOrderedFragmentShaderInterlock", useOrderedFragmentShaderInterlock)) {
        recompileGatherShader = true;
    }
    int syncModeInt = int(syncMode);
    if (newState.rendererSettings.getValueOpt("syncMode", syncModeInt)) {
        syncMode = SyncMode(syncModeInt);
        updateSyncMode();
        recompileGatherShader = true;
    }
    if (recompileGatherShader && lineData) {
        reloadGatherShader();
    }

    updateMomentMode();
}

void MBOITRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    mboitPass1->setLineData(lineData, isNewData);
    mboitPass2->setLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void MBOITRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"MLABGather.glsl\""));
    preprocessorDefines.insert(std::make_pair("ROV", "1")); // Always use ROV mode for now.
    preprocessorDefines.insert(std::make_pair("NUM_MOMENTS", sgl::toString(numMoments)));
    preprocessorDefines.insert(std::make_pair(
            "SINGLE_PRECISION", sgl::toString((int)(pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32))));
    preprocessorDefines.insert(std::make_pair("TRIGONOMETRIC", sgl::toString((int)(!usePowerMoments))));
    preprocessorDefines.insert(std::make_pair(
            "USE_R_RG_RGBA_FOR_MBOIT6", sgl::toString((int)USE_R_RG_RGBA_FOR_MBOIT6)));
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK) {
        preprocessorDefines.insert(std::make_pair("__extensions", "GL_ARB_fragment_shader_interlock"));
        preprocessorDefines.insert(std::make_pair("USE_SYNC_FRAGMENT_SHADER_INTERLOCK", ""));
        if (!useOrderedFragmentShaderInterlock) {
            preprocessorDefines.insert(std::make_pair("INTERLOCK_UNORDERED", ""));
        }
    } else if (syncMode == SYNC_SPINLOCK) {
        preprocessorDefines.insert(std::make_pair("USE_SYNC_SPINLOCK", ""));
        // Do not discard while keeping the spinlock locked.
        preprocessorDefines.insert(std::make_pair("GATHER_NO_DISCARD", ""));
    }
    preprocessorDefines.insert(std::make_pair("USE_SCREEN_SPACE_POSITION", ""));
}

void MBOITRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setColorWriteEnabled(false);
    pipelineInfo.setDepthWriteEnabled(false);
}

void MBOITRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    renderData->setStaticImageView(b0, 0);
    renderData->setStaticImageView(b, 1);
    if (numMoments == 6 && USE_R_RG_RGBA_FOR_MBOIT6) {
        renderData->setStaticImageView(bExtra, 2);
    }
    renderData->setStaticTextureOptional(blendRenderTexture, "transparentSurfaceAccumulator");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
    renderData->setStaticBufferOptional(momentOITUniformBuffer, "MomentOITUniformData");
    if (syncMode == SYNC_SPINLOCK) {
        renderData->setStaticBufferOptional(spinlockViewportBuffer, "SpinlockViewportBuffer");
    }
}

void MBOITRenderer::updateVulkanUniformBuffers() {
}

void MBOITRenderer::setFramebufferAttachments(
        sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::AttachmentState attachmentState;
    if (loadOp == VK_ATTACHMENT_LOAD_OP_DONT_CARE) {
        attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    } else {
        attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    }
    framebuffer->setColorAttachment(
            blendRenderTexture->getImageView(), 0, attachmentState,
            { 0.0f, 0.0f, 0.0f, 0.0f });
}

void MBOITRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    windowWidth = width;
    windowHeight = height;
    paddedWindowWidth = windowWidth, paddedWindowHeight = windowHeight;
    getScreenSizeWithTiling(paddedWindowWidth, paddedWindowHeight);

    updateSyncMode();
    updateMomentMode();

    sgl::vk::ImageSettings imageSettings;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    imageSettings.width = width;
    imageSettings.height = height;
    blendRenderTexture = std::make_shared<sgl::vk::Texture>(renderer->getDevice(), imageSettings);

    mboitPass1->recreateSwapchain(width, height);
    mboitPass2->recreateSwapchain(width, height);

    resolveRasterPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    resolveRasterPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void MBOITRenderer::onClearColorChanged() {
    resolveRasterPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void MBOITRenderer::render() {
    LineRenderer::renderBase();

    computeDepthRange();
    setUniformData();
    gather();
    resolve();
}

void MBOITRenderer::computeDepthRange() {
    const sgl::AABB3& boundingBox = lineData->getModelBoundingBox();
    sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed(sceneData->camera->getViewMatrix());

    // Add offset of 0.1 for e.g. point data sets where additonal vertices may be added in the shader for quads.
    float minViewZ = screenSpaceBoundingBox.getMaximum().z + 0.1f;
    float maxViewZ = screenSpaceBoundingBox.getMinimum().z - 0.1f;
    minViewZ = std::max(-minViewZ, sceneData->camera->getNearClipDistance());
    maxViewZ = std::min(-maxViewZ, sceneData->camera->getFarClipDistance());
    minViewZ = std::min(minViewZ, sceneData->camera->getFarClipDistance());
    maxViewZ = std::max(maxViewZ, sceneData->camera->getNearClipDistance());
    float logmin = log(minViewZ);
    float logmax = log(maxViewZ);
    uniformData.logDepthMin = logmin;
    uniformData.logDepthMax = logmax;
}

void MBOITRenderer::setUniformData() {
    uniformData.viewportW = paddedWindowWidth;
    uniformDataBuffer->updateData(
            sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void MBOITRenderer::gather() {
    //renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    //renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    //renderer->setModelMatrix(sgl::matrixIdentity());

    mboitPass1->render();
    if (syncMode == SYNC_SPINLOCK) {
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
    }
    renderer->insertImageMemoryBarriers(
            momentImageArray, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);

    mboitPass2->render();

    renderer->insertImageMemoryBarriers(
            momentImageArray, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT);
}

void MBOITRenderer::resolve() {
    renderer->transitionImageLayout(
            blendRenderTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolveRasterPass->render();
}

void MBOITRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    // USE_R_RG_RGBA_FOR_MBOIT6
    const char *const momentModes[] = {
            "Power Moments: 4", "Power Moments: 6 (Layered)",
            "Power Moments: 6 (R_RG_RGBA)", "Power Moments: 8",
            "Trigonometric Moments: 2", "Trigonometric Moments: 3 (Layered)",
            "Trigonometric Moments: 3 (R_RG_RGBA)", "Trigonometric Moments: 4"};
    const int momentModesNumMoments[] = {4, 6, 6, 8, 4, 6, 6, 8};
    static int momentModeIndex = -1;
    if (true) { // momentModeIndex == -1
        // Initialize
        momentModeIndex = usePowerMoments ? 0 : 4;
        momentModeIndex += numMoments/2 - 2;
        momentModeIndex += (USE_R_RG_RGBA_FOR_MBOIT6 && numMoments == 6) ? 1 : 0;
        momentModeIndex += (numMoments == 8) ? 1 : 0;
    }

    if (propertyEditor.addCombo(
            "Moment Mode", &momentModeIndex, momentModes, IM_ARRAYSIZE(momentModes))) {
        usePowerMoments = (momentModeIndex / 4) == 0;
        numMoments = momentModesNumMoments[momentModeIndex]; // Count complex moments * 2
        USE_R_RG_RGBA_FOR_MBOIT6 = (momentModeIndex == 2) || (momentModeIndex == 6);
        updateMomentMode();
        reRender = true;
    }

    const char *const pixelFormatModes[] = {"Float 32-bit", "UNORM Integer 16-bit"};
    if (propertyEditor.addCombo(
            "Pixel Format", (int*)&pixelFormat, pixelFormatModes,
            IM_ARRAYSIZE(pixelFormatModes))) {
        updateMomentMode();
        reRender = true;
    }

    if (propertyEditor.addSliderFloat(
            "Overestimation", &overestimationBeta, 0.0f, 1.0f, "%.2f")) {
        momentUniformData.overestimation = overestimationBeta;
        momentOITUniformBuffer->updateData(
                sizeof(MomentOITUniformData), &momentUniformData,
                renderer->getVkCommandBuffer());
        reRender = true;
    }

    const char *syncModeNames[] = { "No Sync (Unsafe)", "Fragment Shader Interlock", "Spinlock" };
    if (propertyEditor.addCombo(
            "Sync Mode", (int*)&syncMode, syncModeNames, IM_ARRAYSIZE(syncModeNames))) {
        updateSyncMode();
        reloadGatherShader();
        reRender = true;
    }
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK && propertyEditor.addCheckbox(
            "Ordered Sync", &useOrderedFragmentShaderInterlock)) {
        reloadGatherShader();
        reRender = true;
    }
}



MBOITPass1::MBOITPass1(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
}

void MBOITPass1::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines["OIT_GATHER_HEADER"] = "\"MBOITPass1.glsl\"";
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            lineData->getShaderModuleNames(), preprocessorDefines);
}



MBOITPass2::MBOITPass2(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
}

void MBOITPass2::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines["OIT_GATHER_HEADER"] = "\"MBOITPass2.glsl\"";
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            lineData->getShaderModuleNames(), preprocessorDefines);
}

void MBOITPass2::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    LineRasterPass::setGraphicsPipelineInfo(pipelineInfo);
    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::ONE);
}
