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

#include <Utils/AppSettings.hpp>
#include <Utils/Defer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/CommandBuffer.hpp>
#include <Graphics/Vulkan/Render/RayTracingPipeline.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include <memory>
#include <utility>
#include <ImGui/imgui_custom.h>
#include "Renderers/LineRenderer.hpp"
#include "LineData/LineData.hpp"

#include "Renderers/Scattering/Denoiser/EAWDenoiser.hpp"
#include "Renderers/Scattering/Denoiser/SpatialHashingDenoiser.hpp"
#ifdef SUPPORT_OPTIX
#include "Renderers/Scattering/Denoiser/OptixVptDenoiser.hpp"
#endif

#include "VulkanRayTracedAmbientOcclusion.hpp"

VulkanRayTracedAmbientOcclusion::VulkanRayTracedAmbientOcclusion(SceneData* sceneData, sgl::vk::Renderer* renderer)
        : AmbientOcclusionBaker(renderer), sceneData(sceneData) {
    rtaoRenderPass = std::make_shared<VulkanRayTracedAmbientOcclusionPass>(
            sceneData, rendererMain, [this]() { this->onHasMoved(); });
    VulkanRayTracedAmbientOcclusion::onResolutionChanged();
}

VulkanRayTracedAmbientOcclusion::~VulkanRayTracedAmbientOcclusion() = default;

bool VulkanRayTracedAmbientOcclusion::needsReRender() {
    return rtaoRenderPass->needsReRender();
}

void VulkanRayTracedAmbientOcclusion::onHasMoved() {
    accumulatedFramesCounter = 0;
    rtaoRenderPass->onHasMoved();
}

void VulkanRayTracedAmbientOcclusion::onResolutionChanged() {
    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;
    hasTextureResolutionChanged = true;
    rtaoRenderPass->recreateSwapchain(width, height);
    onHasMoved();
}

void VulkanRayTracedAmbientOcclusion::startAmbientOcclusionBaking(LineDataPtr& lineData, bool isNewData) {
    if (lineData) {
        this->lineData = lineData;
        rtaoRenderPass->setLineData(lineData, isNewData);
    }

    accumulatedFramesCounter = 0;
    rtaoRenderPass->onHasMoved();
    isDataReady = false;
    hasComputationFinished = false;
}

void VulkanRayTracedAmbientOcclusion::updateIterative(VkPipelineStageFlags pipelineStageFlags) {
    sgl::vk::TexturePtr aoTextureVk = rtaoRenderPass->getAmbientOcclusionTextureVk();

    rtaoRenderPass->setFrameNumber(accumulatedFramesCounter);
    rtaoRenderPass->render();
    rendererMain->insertImageMemoryBarrier(
            aoTextureVk->getImage(),
            aoTextureVk->getImage()->getVkImageLayout(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, pipelineStageFlags,
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);

    accumulatedFramesCounter++;
    isDataReady = true;
    hasComputationFinished = accumulatedFramesCounter >= maxNumAccumulatedFrames;
}

sgl::vk::TexturePtr VulkanRayTracedAmbientOcclusion::getAmbientOcclusionFrameTexture()  {
    return rtaoRenderPass->getAmbientOcclusionTextureVk();
}

bool VulkanRayTracedAmbientOcclusion::getHasTextureResolutionChanged() {
    bool tmp = hasTextureResolutionChanged;
    hasTextureResolutionChanged = false;
    return tmp;
}

bool VulkanRayTracedAmbientOcclusion::setNewSettings(const SettingsMap& settings) {
    bool optionChanged = false;

    if (settings.getValueOpt("ambient_occlusion_iterations", maxNumAccumulatedFrames)) {
        optionChanged = true;
    }
    if (settings.getValueOpt(
            "ambient_occlusion_samples_per_frame", rtaoRenderPass->numAmbientOcclusionSamplesPerFrame)) {
        optionChanged = true;
    }
    if (settings.getValueOpt("ambient_occlusion_radius", rtaoRenderPass->ambientOcclusionRadius)) {
        optionChanged = true;
    }
    if (settings.getValueOpt("ambient_occlusion_distance_based", rtaoRenderPass->useDistance)) {
        optionChanged = true;
    }
    if (settings.getValueOpt("use_jittered_primary_rays", rtaoRenderPass->useJitteredPrimaryRays)) {
        optionChanged = true;
    }

    if (rtaoRenderPass->setNewSettings(settings)) {
        optionChanged = true;
    }
    if (optionChanged) {
        accumulatedFramesCounter = 0;
        rtaoRenderPass->onHasMoved();
    }

    return optionChanged;
}

bool VulkanRayTracedAmbientOcclusion::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool optionChanged = false;

    if (propertyEditor.beginNode("RTAO")) {
        if (propertyEditor.addSliderIntEdit(
                "#Iterations", &maxNumAccumulatedFrames,
                1, 4096) == ImGui::EditMode::INPUT_FINISHED) {
            optionChanged = true;
        }
        if (propertyEditor.addSliderIntEdit(
                "#Samples/Frame",
                reinterpret_cast<int*>(&rtaoRenderPass->numAmbientOcclusionSamplesPerFrame),
                1, 4096) == ImGui::EditMode::INPUT_FINISHED) {
            optionChanged = true;
        }
        if (propertyEditor.addSliderFloatEdit(
                "AO Radius", &rtaoRenderPass->ambientOcclusionRadius,
                0.01f, 0.2f) == ImGui::EditMode::INPUT_FINISHED) {
            optionChanged = true;
        }
        if (propertyEditor.addCheckbox("Use Distance-based AO", &rtaoRenderPass->useDistance)) {
            optionChanged = true;
        }

        if (propertyEditor.addCheckbox("Jittered Primary Rays", &rtaoRenderPass->useJitteredPrimaryRays)) {
            rtaoRenderPass->setShaderDirty();
            optionChanged = true;
        }

        if (rtaoRenderPass->renderGuiPropertyEditorNodes(propertyEditor)) {
            optionChanged = true;
        }

        propertyEditor.endNode();
    }

    if (optionChanged) {
        accumulatedFramesCounter = 0;
        rtaoRenderPass->onHasMoved();
    }

    return optionChanged;
}

void VulkanRayTracedAmbientOcclusion::setFileDialogInstance(ImGuiFileDialog* _fileDialogInstance) {
    rtaoRenderPass->setFileDialogInstance(_fileDialogInstance);
}



VulkanRayTracedAmbientOcclusionPass::VulkanRayTracedAmbientOcclusionPass(
        SceneData* sceneData, sgl::vk::Renderer* renderer, std::function<void()> onHasMovedCallback)
        : ComputePass(renderer), sceneData(sceneData), onHasMovedParent(std::move(onHasMovedCallback)) {
    uniformBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    blitResultRenderPass = std::make_shared<sgl::vk::BlitRenderPass>(renderer);

    createDenoiser();
}

void VulkanRayTracedAmbientOcclusionPass::createDenoiser() {
    denoiser = createDenoiserObject(denoiserType, renderer, &sceneData->camera, DenoisingMode::AMBIENT_OCCLUSION);
    if (denoiser) {
        denoiser->setFileDialogInstance(fileDialogInstance);
    }

    globalFrameNumber = 0;
    if (accumulationTexture) {
        checkRecreateFeatureMaps();
        setDenoiserFeatureMaps();
        if (denoiser) {
            denoiser->recreateSwapchain(lastViewportWidth, lastViewportHeight);
        }
    }
}

void VulkanRayTracedAmbientOcclusionPass::setDenoiserFeatureMaps() {
    if (denoiser) {
        denoiser->setFeatureMap(FeatureMapType::COLOR, accumulationTexture);
        if (denoiser->getUseFeatureMap(FeatureMapType::NORMAL)) {
            denoiser->setFeatureMap(FeatureMapType::NORMAL, normalMapTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::NORMAL_WORLD)) {
            denoiser->setFeatureMap(FeatureMapType::NORMAL_WORLD, normalMapWorldTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::DEPTH)) {
            denoiser->setFeatureMap(FeatureMapType::DEPTH, depthMapTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::POSITION)) {
            denoiser->setFeatureMap(FeatureMapType::POSITION, positionMapTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::POSITION_WORLD)) {
            denoiser->setFeatureMap(FeatureMapType::POSITION_WORLD, positionMapWorldTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::ALBEDO)) {
            denoiser->setFeatureMap(FeatureMapType::ALBEDO, albedoTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::FLOW)) {
            denoiser->setFeatureMap(FeatureMapType::FLOW, flowMapTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::DEPTH_NABLA)) {
            denoiser->setFeatureMap(FeatureMapType::DEPTH_NABLA, depthNablaTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::DEPTH_FWIDTH)) {
            denoiser->setFeatureMap(FeatureMapType::DEPTH_FWIDTH, depthFwidthTexture);
        }
        denoiser->setOutputImage(denoisedTexture->getImageView());
    }
}

void VulkanRayTracedAmbientOcclusionPass::recreateSwapchain(uint32_t width, uint32_t height) {
    lastViewportWidth = width;
    lastViewportHeight = height;

    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;

    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    accumulationTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    imageSettings.usage =
            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT
            | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    denoisedTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);

    imageSettings.usage =
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
            | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    resultTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);

    recreateFeatureMaps();

    blitResultRenderPass->setInputTexture(accumulationTexture);
    blitResultRenderPass->setOutputImage(resultTexture->getImageView());
    blitResultRenderPass->recreateSwapchain(width, height);

    if (useDenoiser && denoiser) {
        denoiser->recreateSwapchain(width, height);
    }

    setDataDirty();
}

void VulkanRayTracedAmbientOcclusionPass::recreateFeatureMaps() {
    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = lastViewportWidth;
    imageSettings.height = lastViewportHeight;

    normalMapTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL)) {
        imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        normalMapTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    normalMapWorldTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL_WORLD)) {
        imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        normalMapWorldTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    depthMapTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH)) {
        imageSettings.format = VK_FORMAT_R32_SFLOAT;
        imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        depthMapTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    positionMapTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION)) {
        imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        positionMapTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    positionMapWorldTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION_WORLD)) {
        imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        positionMapWorldTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    albedoTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::ALBEDO)) {
        imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        imageSettings.usage =
                VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        albedoTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
        VkCommandBuffer commandBuffer = device->beginSingleTimeCommands();
        albedoTexture->getImage()->transitionImageLayout(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, commandBuffer);
        albedoTexture->getImageView()->clearColor(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), commandBuffer);
        device->endSingleTimeCommands(commandBuffer);
    }

    flowMapTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::FLOW)) {
        imageSettings.format = VK_FORMAT_R32G32_SFLOAT;
        imageSettings.usage =
                VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        flowMapTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    depthNablaTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_NABLA)) {
        imageSettings.format = VK_FORMAT_R32G32_SFLOAT;
        imageSettings.usage =
                VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        depthNablaTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    depthFwidthTexture = {};
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_FWIDTH)) {
        imageSettings.format = VK_FORMAT_R32_SFLOAT;
        imageSettings.usage =
                VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        depthFwidthTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    }

    setDenoiserFeatureMaps();

    // NOTE(Felix): initialization code
    static bool first_iteration = true;
    defer { first_iteration = false; };
    if (first_iteration) {
        denoiserType = DenoiserType::SVGF;
        changedDenoiserSettings = true;
        createDenoiser();
    }
}

void VulkanRayTracedAmbientOcclusionPass::checkRecreateFeatureMaps() {
    bool useNormalMapRenderer = normalMapTexture.get() != nullptr;
    bool useNormalMapWorldRenderer = normalMapWorldTexture.get() != nullptr;
    bool useDepthRenderer = depthMapTexture.get() != nullptr;
    bool usePositionRenderer = positionMapTexture.get() != nullptr;
    bool usePositionWorldRenderer = positionMapWorldTexture.get() != nullptr;
    bool useAlbedoRenderer = albedoTexture.get() != nullptr;
    bool useFlowRenderer = flowMapTexture.get() != nullptr;
    bool useDepthNablaRenderer = depthNablaTexture.get() != nullptr;
    bool useDepthFwidthRenderer = depthFwidthTexture.get() != nullptr;

    bool shallRecreateFeatureMaps = false;
    if (denoiser) {
        if (useNormalMapRenderer != denoiser->getUseFeatureMap(FeatureMapType::NORMAL)
                || useNormalMapWorldRenderer != denoiser->getUseFeatureMap(FeatureMapType::NORMAL_WORLD)
                || useDepthRenderer != denoiser->getUseFeatureMap(FeatureMapType::DEPTH)
                || usePositionRenderer != denoiser->getUseFeatureMap(FeatureMapType::POSITION)
                || usePositionWorldRenderer != denoiser->getUseFeatureMap(FeatureMapType::POSITION_WORLD)
                || useAlbedoRenderer != denoiser->getUseFeatureMap(FeatureMapType::ALBEDO)
                || useFlowRenderer != denoiser->getUseFeatureMap(FeatureMapType::FLOW)
                || useDepthNablaRenderer != denoiser->getUseFeatureMap(FeatureMapType::DEPTH_NABLA)
                || useDepthFwidthRenderer != denoiser->getUseFeatureMap(FeatureMapType::DEPTH_FWIDTH)) {
            shallRecreateFeatureMaps = true;
        }
    } else {
        if (useNormalMapRenderer || useNormalMapWorldRenderer || useDepthRenderer
                || usePositionRenderer || usePositionWorldRenderer || useAlbedoRenderer
                || useFlowRenderer || useDepthNablaRenderer || useDepthFwidthRenderer) {
            shallRecreateFeatureMaps = true;
        }
    }

    // Check if inputs should be accumulated.
    if (denoiser) {
        if (accumulateInputs != denoiser->getWantsAccumulatedInput()) {
            accumulateInputs = denoiser->getWantsAccumulatedInput();
            shallRecreateFeatureMaps = true;
        }
        useGlobalFrameNumber = denoiser->getWantsGlobalFrameNumber();
    } else {
        if (!accumulateInputs) {
            accumulateInputs = true;
            shallRecreateFeatureMaps = true;
        }
        useGlobalFrameNumber = false;
    }

    if (shallRecreateFeatureMaps) {
        setShaderDirty();
        device->waitIdle();
        recreateFeatureMaps();
        onHasMovedParent();
        changedDenoiserSettings = false;
    }
}

void VulkanRayTracedAmbientOcclusionPass::setLineData(LineDataPtr& data, bool isNewData) {
    if (this->lineData && lineData->getType() != data->getType()) {
        setShaderDirty();
    }
    lineData = data;
    topLevelAS = lineData->getRayTracingTubeTriangleTopLevelAS();
    tubeTriangleRenderData = lineData->getLinePassTubeTriangleMeshRenderData(false, true);

    bool useSplitBlasesNew = tubeTriangleRenderData.instanceTriangleIndexOffsetBuffer.get() != nullptr;
    if (useSplitBlases != useSplitBlasesNew) {
        useSplitBlases = useSplitBlasesNew;
        setShaderDirty();
    }

    if (denoiser) {
        denoiser->resetFrameNumber();
    }
    lastFrameViewProjectionMatrix = sceneData->camera->getProjectionMatrix() * sceneData->camera->getViewMatrix();

    uniformData.frameNumber = 0;
    globalFrameNumber = 0;
    setDataDirty();
}

void VulkanRayTracedAmbientOcclusionPass::setFileDialogInstance(ImGuiFileDialog* _fileDialogInstance) {
    this->fileDialogInstance = _fileDialogInstance;
}

void VulkanRayTracedAmbientOcclusionPass::onHasMoved() {
    uniformData.frameNumber = 0;
}

void VulkanRayTracedAmbientOcclusionPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    if (lineData && lineData->getType() == DATA_SET_TYPE_TRIANGLE_MESH) {
        preprocessorDefines.insert(std::make_pair("GENERAL_TRIANGLE_MESH", ""));
    }
    if (useSplitBlases) {
        preprocessorDefines.insert(std::make_pair("USE_INSTANCE_TRIANGLE_INDEX_OFFSET", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL)) {
        preprocessorDefines.insert(std::make_pair("WRITE_NORMAL_MAP", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL_WORLD)) {
        preprocessorDefines.insert(std::make_pair("WRITE_NORMAL_WORLD_MAP", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH)) {
        preprocessorDefines.insert(std::make_pair("WRITE_DEPTH_MAP", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION)) {
        preprocessorDefines.insert(std::make_pair("WRITE_POSITION_MAP", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION_WORLD)) {
        preprocessorDefines.insert(std::make_pair("WRITE_POSITION_WORLD_MAP", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::FLOW)) {
        preprocessorDefines.insert(std::make_pair("WRITE_FLOW_MAP", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_NABLA)) {
        preprocessorDefines.insert(std::make_pair("WRITE_DEPTH_NABLA_MAP", ""));
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_FWIDTH)) {
        preprocessorDefines.insert(std::make_pair("WRITE_DEPTH_FWIDTH_MAP", ""));
    }
    if (denoiser && !denoiser->getWantsAccumulatedInput()) {
        preprocessorDefines.insert(std::make_pair("DISABLE_ACCUMULATION", ""));
    }
    if (useJitteredPrimaryRays) {
        preprocessorDefines.insert(std::make_pair("USE_JITTERED_RAYS", ""));
    }
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "VulkanRayTracedAmbientOcclusion.Compute" }, preprocessorDefines);
}

void VulkanRayTracedAmbientOcclusionPass::setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) {
}

void VulkanRayTracedAmbientOcclusionPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {

    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticImageView(accumulationTexture->getImageView(), "outputImage");
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL)) {
        computeData->setStaticImageView(normalMapTexture->getImageView(), "normalViewSpaceMap");
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL_WORLD)) {
        computeData->setStaticImageView(normalMapWorldTexture->getImageView(), "normalWorldSpaceMap");
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH)) {
        computeData->setStaticImageView(depthMapTexture->getImageView(), "depthMap");
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION)) {
        computeData->setStaticImageView(positionMapTexture->getImageView(), "positionViewSpaceMap");
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION_WORLD)) {
        computeData->setStaticImageView(positionMapWorldTexture->getImageView(), "positionWorldSpaceMap");
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::FLOW)) {
        computeData->setStaticImageView(flowMapTexture->getImageView(), "flowMap");
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_NABLA)) {
        computeData->setStaticImageView(depthNablaTexture->getImageView(), "depthNablaMap");
    }
    if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_FWIDTH)) {
        computeData->setStaticImageView(depthFwidthTexture->getImageView(), "depthFwidthMap");
    }
    computeData->setTopLevelAccelerationStructure(topLevelAS, "topLevelAS");
    computeData->setStaticBuffer(uniformBuffer, "UniformsBuffer");

    // Geometry data.
    computeData->setStaticBuffer(
            tubeTriangleRenderData.indexBuffer, "TubeIndexBuffer");
    computeData->setStaticBuffer(
            tubeTriangleRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
    if (tubeTriangleRenderData.linePointDataBuffer) {
        computeData->setStaticBuffer(
                tubeTriangleRenderData.linePointDataBuffer, "LinePointDataBuffer");
    }
    if (tubeTriangleRenderData.instanceTriangleIndexOffsetBuffer) {
        computeData->setStaticBuffer(
                tubeTriangleRenderData.instanceTriangleIndexOffsetBuffer,
                "InstanceTriangleIndexOffsetBuffer");
    }
}

void VulkanRayTracedAmbientOcclusionPass::_render() {
    if (!changedDenoiserSettings) {
        uniformData.viewMatrix = sceneData->camera->getViewMatrix();
        uniformData.inverseViewMatrix = glm::inverse(uniformData.viewMatrix);
        uniformData.inverseProjectionMatrix = glm::inverse(sceneData->camera->getProjectionMatrix());
        uniformData.inverseTransposedViewMatrix = glm::transpose(uniformData.inverseViewMatrix);
        uniformData.lastFrameViewProjectionMatrix = lastFrameViewProjectionMatrix;
        uniformData.nearDistance = sceneData->camera->getNearClipDistance();
        uniformData.farDistance = sceneData->camera->getFarClipDistance();
        uniformData.numSamplesPerFrame = numAmbientOcclusionSamplesPerFrame;
        uniformData.useDistance = useDistance;
        uniformData.ambientOcclusionRadius = ambientOcclusionRadius;
        if (useGlobalFrameNumber) {
            uniformData.globalFrameNumber = globalFrameNumber;
        } else {
            uniformData.globalFrameNumber = uniformData.frameNumber;
        }
        globalFrameNumber++;
        //float radius = LineRenderer::getLineWidth();
        //if (lineData->getUseBandRendering()) {
        //    radius = std::max(LineRenderer::getLineWidth(), LineRenderer::getBandWidth());
        //}
        //uniformData.subdivisionCorrectionFactor =
        //        radius * (1.0f - std::cos(sgl::PI / float(lineData->getTubeNumSubdivisions())));
        uniformData.subdivisionCorrectionFactor = std::cos(sgl::PI / float(lineData->getTubeNumSubdivisions()));
        uniformBuffer->updateData(
                sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                uniformBuffer);

        renderer->transitionImageLayout(accumulationTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL)) {
            renderer->transitionImageLayout(normalMapTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::NORMAL_WORLD)) {
            renderer->transitionImageLayout(normalMapWorldTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH)) {
            renderer->transitionImageLayout(depthMapTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION)) {
            renderer->transitionImageLayout(positionMapTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::POSITION_WORLD)) {
            renderer->transitionImageLayout(positionMapWorldTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::FLOW)) {
            renderer->transitionImageLayout(flowMapTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_NABLA)) {
            renderer->transitionImageLayout(depthNablaTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        if (denoiser && denoiser->getUseFeatureMap(FeatureMapType::DEPTH_FWIDTH)) {
            renderer->transitionImageLayout(depthFwidthTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        }
        auto& imageSettings = accumulationTexture->getImage()->getImageSettings();
        int width = int(imageSettings.width);
        int height = int(imageSettings.height);
        int groupCountX = sgl::iceil(width, 16);
        int groupCountY = sgl::iceil(height, 16);
        if (topLevelAS) {
            renderer->dispatch(computeData, groupCountX, groupCountY, 1);
        }
    }
    lastFrameViewProjectionMatrix = sceneData->camera->getProjectionMatrix() * sceneData->camera->getViewMatrix();
    changedDenoiserSettings = false;

    if (useDenoiser && denoiser && denoiser->getIsEnabled()) {
        denoiser->denoise();
        renderer->transitionImageLayout(
                denoisedTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(
                resultTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        denoisedTexture->getImage()->blit(
                resultTexture->getImage(), renderer->getVkCommandBuffer());
    } else {
        renderer->transitionImageLayout(
                accumulationTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        blitResultRenderPass->render();
        /*renderer->transitionImageLayout(
                 accumulationTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
         renderer->transitionImageLayout(
                 resultTextureVk->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
         accumulationTexture->getImage()->blit(
                 resultTextureVk->getImage(), renderer->getVkCommandBuffer());*/
    }

    /*renderer->transitionImageLayout(
            resultTextureVk->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);*/
    /*renderer->insertImageMemoryBarrier(
            resultTextureVk->getImage(),
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);*/
}

bool VulkanRayTracedAmbientOcclusionPass::setNewSettings(const SettingsMap& settings) {
    bool optionChanged = false;

    int numDenoisersSupported = IM_ARRAYSIZE(DENOISER_NAMES);
#ifdef SUPPORT_OPTIX
    if (!OptixVptDenoiser::isOptixEnabled()) {
        numDenoisersSupported--;
    }
#endif
    std::string denoiserName;
    if (settings.getValueOpt("ambient_occlusion_denoiser", denoiserName)) {
        // NOTE(Felix): I put this here so we can control denoiser specific
        //   settigns here
        if (denoiserName == "EAW") {
            denoiserType = DenoiserType::EAW;
            createDenoiser();
            auto eaw = (EAWDenoiser*)denoiser.get();

            reRender = true;
            changedDenoiserSettings = true;

            float phi_pos    = 0.3f;
            float phi_color  = 0.49f;
            float phi_normal = 0.1f;
            int   num_iters = 3;
            settings.getValueOpt("eaw_color_phi", phi_color);
            settings.getValueOpt("eaw_pos_phi", phi_pos);
            settings.getValueOpt("eaw_normal_phi", phi_normal);
            settings.getValueOpt("eaw_num_iters", num_iters);
            eaw->setPhiColor(phi_color);
            eaw->setPhiPosition(phi_pos);
            eaw->setPhiNormal(phi_normal);
            eaw->setNumIterations(num_iters);

        } if (denoiserName == "SH") {
            denoiserType = DenoiserType::SPATIAL_HASHING;
            createDenoiser();
            auto shd = (Spatial_Hashing_Denoiser*)denoiser.get();
            auto eaw_settings = shd->eaw_pass->get_settings();

            eaw_settings.useColorWeights = false;
            eaw_settings.phiPosition     = 0.3f;
            eaw_settings.phiNormal       = 0.1f;
            eaw_settings.maxNumIterations = 3;

            // NOTE(Felix): SH does not make use of color weights since it is
            //   meant to blur the color diff, not respect it
            eaw_settings.useColorWeights = false;
            eaw_settings.phiPositionScale = 1;
            eaw_settings.phiNormalScale   = 1;

            shd->textures.uniform_buffer.s_nd = 3.0f;
            shd->textures.uniform_buffer.s_p  = 8;

            settings.getValueOpt("eaw_pos_phi",    eaw_settings.phiPosition);
            settings.getValueOpt("eaw_normal_phi", eaw_settings.phiNormal);
            settings.getValueOpt("eaw_num_iters",  eaw_settings.maxNumIterations);
            settings.getValueOpt("sh_s_nd",    shd->textures.uniform_buffer.s_nd);
            settings.getValueOpt("sh_s_p",     shd->textures.uniform_buffer.s_p);

            reRender = true;
            changedDenoiserSettings = true;

            shd->eaw_pass->set_settings(eaw_settings);
        } else {
            for (int i = 0; i < numDenoisersSupported; i++) {
                if (denoiserName == DENOISER_NAMES[i]) {
                    if (denoiserType == DenoiserType(i)) {
                        break;
                    }
                    denoiserType = DenoiserType(i);
                    createDenoiser();
                    reRender = true;
                    changedDenoiserSettings = true;
                    break;
                }
            }
        }
    }

    if (useDenoiser && denoiser) {
        optionChanged |= denoiser->getWantsFrameNumberReset();
    }

    return optionChanged;
}

bool VulkanRayTracedAmbientOcclusionPass::renderGuiPropertyEditorNodes(sgl::PropertyEditor &propertyEditor) {
    bool optionChanged = false;

    int numDenoisersSupported = IM_ARRAYSIZE(DENOISER_NAMES);
#ifdef SUPPORT_OPTIX
    if (!OptixVptDenoiser::isOptixEnabled()) {
        numDenoisersSupported--;
    }
#endif
    if (propertyEditor.addCombo(
            "Denoiser", (int*)&denoiserType, DENOISER_NAMES, numDenoisersSupported)) {
        reRender = true;
        changedDenoiserSettings = true;
        createDenoiser();
    }

    if (useDenoiser && denoiser) {
        if (propertyEditor.beginNode(denoiser->getDenoiserName())) {
            bool denoiserReRender = denoiser->renderGuiPropertyEditorNodes(propertyEditor);
            reRender = denoiserReRender || reRender;
            changedDenoiserSettings = denoiserReRender || changedDenoiserSettings;
            if (denoiserReRender) {
                checkRecreateFeatureMaps();
            }
            propertyEditor.endNode();
        }
        optionChanged |= denoiser->getWantsFrameNumberReset();
    }

    return optionChanged;
}
