/*
* BSD 2-Clause License
 *
 * Copyright (c) 2025-2026, Christoph Neuhauser
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

#include <cstring>

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Instance.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>
#include <Graphics/Vulkan/Image/Image.hpp>

#include <xess/xess.h>
#include <xess/xess_vk.h>

#include "XeSS.hpp"

XessSupersampler::~XessSupersampler() {
    _free();
}

void XessSupersampler::_free() {
    if (context) {
        if (xessDestroyContext(context) != XESS_RESULT_SUCCESS) {
            context = {};
            sgl::Logfile::get()->throwError("xessDestroyContext failed.");
        }
        context = {};
    }
}

static void xessLoggingCallback(const char* message, xess_logging_level_t loggingLevel) {
    if (loggingLevel == XESS_LOGGING_LEVEL_ERROR) {
        sgl::Logfile::get()->writeError(message);
    } else if (loggingLevel == XESS_LOGGING_LEVEL_WARNING) {
        sgl::Logfile::get()->writeWarning(message);
    } else {
        sgl::Logfile::get()->write(message, sgl::BLACK);
    }
}

bool XessSupersampler::initialize(sgl::vk::Device* _device) {
    device = _device;
    if (xessVKCreateContext(
            device->getInstance()->getVkInstance(), device->getVkPhysicalDevice(), device->getVkDevice(),
            &context) != XESS_RESULT_SUCCESS) {
        sgl::Logfile::get()->writeError("xessVKCreateContext failed.");
        return false;
    }
    if (xessSetLoggingCallback(context, XESS_LOGGING_LEVEL_DEBUG, &xessLoggingCallback) != XESS_RESULT_SUCCESS) {
        sgl::Logfile::get()->writeError("xessSetLoggingCallback failed.");
        return false;
    }
    return true;
}

bool XessSupersampler::queryOptimalSettings(
        uint32_t displayWidth, uint32_t displayHeight,
        uint32_t& renderWidthOptimal, uint32_t& renderHeightOptimal,
        uint32_t& renderWidthMax, uint32_t& renderHeightMax,
        uint32_t& renderWidthMin, uint32_t& renderHeightMin,
        float& sharpness) {
    xess_2d_t outputResolution = { displayWidth, displayHeight };
    xess_2d_t inputResolutionOptimal{}, inputResolutionMin{}, inputResolutionMax{};
    if (xessGetOptimalInputResolution(
            context, &outputResolution, xess_quality_settings_t(qualityMode), &inputResolutionOptimal,
            &inputResolutionMin, &inputResolutionMax) != XESS_RESULT_SUCCESS) {
        sgl::Logfile::get()->throwError("xessGetOptimalInputResolution failed.");
        return false;
    }
    renderWidthOptimal = inputResolutionOptimal.x;
    renderHeightOptimal = inputResolutionOptimal.y;
    renderWidthMin = inputResolutionMin.x;
    renderHeightMin = inputResolutionMin.y;
    renderWidthMax = inputResolutionMax.x;
    renderHeightMax = inputResolutionMax.y;
    return true;
}

void XessSupersampler::setUseAntiAliasingMode() {
    setQualityMode(XessQualityMode::AA);
}

void XessSupersampler::setUseJitteredMotionVectors(bool _useJitteredMotionVectors) {
    useJitteredMotionVectors = _useJitteredMotionVectors;
}

void XessSupersampler::setJitterOffset(float _jitterOffsetX, float _jitterOffsetY) {
    jitterOffsetX = _jitterOffsetX;
    jitterOffsetY = _jitterOffsetY;
}

void XessSupersampler::resetAccum() {
    shallResetAccum = true;
}

void XessSupersampler::checkRecreateFeature(
        const sgl::vk::ImageViewPtr& colorImageIn,
        const sgl::vk::ImageViewPtr& colorImageOut,
        VkCommandBuffer commandBuffer) {
    if (isInitialized) {
        return;
    }

    xess_vk_init_params_t initParams{};
    initParams.outputResolution.x = colorImageOut->getImage()->getImageSettings().width;
    initParams.outputResolution.y = colorImageOut->getImage()->getImageSettings().height;
    initParams.qualitySetting = xess_quality_settings_t(qualityMode);
    // Unused: XESS_INIT_FLAG_RESPONSIVE_PIXEL_MASK, XESS_INIT_FLAG_USE_NDC_VELOCITY
    // TODO: XESS_INIT_FLAG_EXPOSURE_SCALE_TEXTURE seems to be broken. Even providing 1 as value results in white output.
    initParams.initFlags = 0;
    initParams.initFlags |= motionVectorLowRes ? 0 : XESS_INIT_FLAG_HIGH_RES_MV;
    initParams.initFlags |= useJitteredMotionVectors ? 0 : XESS_INIT_FLAG_JITTERED_MV;
    initParams.initFlags |= isHdr ? 0 : XESS_INIT_FLAG_LDR_INPUT_COLOR;
    initParams.initFlags |= isDepthInverted ? XESS_INIT_FLAG_INVERTED_DEPTH : 0;
    initParams.initFlags |= enableAutoExposure ? XESS_INIT_FLAG_ENABLE_AUTOEXPOSURE : 0;
    xessVKInit(context, &initParams);
    // Currently unused: xessIsOptimalDriver, xessGetJitterScale, xessGetVelocityScale,
    // xessSetExposureMultiplier, xessSetMaxResponsiveMaskValue, xessForceLegacyScaleFactors.
}

static xess_vk_image_view_info createXessVkImageView(const sgl::vk::ImageViewPtr& imageView) {
    const auto& imageSettings = imageView->getImage()->getImageSettings();
    xess_vk_image_view_info xessImageViewInfo{};
    xessImageViewInfo.imageView = imageView->getVkImageView();
    xessImageViewInfo.image = imageView->getImage()->getVkImage();
    xessImageViewInfo.subresourceRange = imageView->getVkImageSubresourceRange();
    xessImageViewInfo.format = imageSettings.format;
    xessImageViewInfo.width = imageSettings.width;
    xessImageViewInfo.height = imageSettings.height;
    return xessImageViewInfo;
}

bool XessSupersampler::apply(
        const sgl::vk::ImageViewPtr& colorImageIn,
        const sgl::vk::ImageViewPtr& colorImageOut,
        const sgl::vk::ImageViewPtr& depthImage,
        const sgl::vk::ImageViewPtr& motionVectorImage,
        const sgl::vk::ImageViewPtr& exposureImage,
        VkCommandBuffer commandBuffer) {
    bool _isHdr =
            colorImageIn->getImage()->getImageSettings().format == VK_FORMAT_R16G16B16A16_SFLOAT
            || colorImageIn->getImage()->getImageSettings().format == VK_FORMAT_R32G32B32A32_SFLOAT;
    if (isHdr != _isHdr) {
        isHdr = _isHdr;
        if (context) {
            isInitialized = false;
        }
    }

    auto renderWidth = colorImageIn->getImage()->getImageSettings().width;
    auto renderHeight = colorImageIn->getImage()->getImageSettings().height;
    auto displayWidth = colorImageOut->getImage()->getImageSettings().width;
    auto displayHeight = colorImageOut->getImage()->getImageSettings().height;
    if (cachedRenderWidth != renderWidth || cachedRenderHeight != renderHeight
            || cachedDisplayWidth != displayWidth || cachedDisplayHeight != displayHeight) {
        cachedRenderWidth = renderWidth;
        cachedRenderHeight = renderHeight;
        cachedDisplayWidth = displayWidth;
        cachedDisplayHeight = displayHeight;
        if (context) {
            isInitialized = false;
        }
    }

    bool useSingleTimeCommandBuffer = commandBuffer == VK_NULL_HANDLE;
    if (useSingleTimeCommandBuffer) {
        commandBuffer = device->beginSingleTimeCommands();
    }
    checkRecreateFeature(colorImageIn, colorImageOut, commandBuffer);

    colorImageIn->transitionImageLayoutEx(
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT,
            commandBuffer);
    depthImage->transitionImageLayoutEx(
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT,
            commandBuffer);
    motionVectorImage->transitionImageLayoutEx(
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT,
            commandBuffer);
    exposureImage->transitionImageLayoutEx(
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT,
            commandBuffer);

    xess_vk_execute_params_t executeParams{};
    executeParams.colorTexture = createXessVkImageView(colorImageIn);
    executeParams.velocityTexture = createXessVkImageView(motionVectorImage);
    executeParams.depthTexture = createXessVkImageView(depthImage);
    executeParams.exposureScaleTexture = createXessVkImageView(exposureImage);
    //executeParams.responsivePixelMaskTexture; // Unused.
    executeParams.outputTexture = createXessVkImageView(colorImageOut);
    executeParams.jitterOffsetX = jitterOffsetX;
    executeParams.jitterOffsetY = -jitterOffsetY;
    executeParams.exposureScale = 1.0f;
    executeParams.resetHistory = uint32_t(shallResetAccum);
    executeParams.inputWidth = colorImageIn->getImage()->getImageSettings().width;
    executeParams.inputHeight = colorImageIn->getImage()->getImageSettings().height;
    if (xessVKExecute(context, commandBuffer, &executeParams) != XESS_RESULT_SUCCESS) {
        sgl::Logfile::get()->throwError("xessVKExecute failed.");
        return false;
    }

    colorImageOut->getImage()->_updateLayout(VK_IMAGE_LAYOUT_GENERAL);

    if (useSingleTimeCommandBuffer) {
        device->endSingleTimeCommands(commandBuffer);
    }

    return true;
}

void XessSupersampler::setQualityMode(XessQualityMode _qualityMode) {
    qualityMode = _qualityMode;
    if (qualityMode != _qualityMode) {
        qualityMode = _qualityMode;
        isInitialized = false;
    }
}

void XessSupersampler::setUpscaleAlpha(bool _upscaleAlpha) {
    // Unimplemented by XeSS.
}


bool getInstanceXessSupportInfo(
        uint32_t& apiVersion, std::vector<const char*>& requiredInstanceExtensions) {
    uint32_t instanceExtensionCountXess{};
    const char* const* instanceExtensionsXess{};
    uint32_t vulkanApiVersionXess{};
    if (xessVKGetRequiredInstanceExtensions(
            &instanceExtensionCountXess, &instanceExtensionsXess, &vulkanApiVersionXess) != XESS_RESULT_SUCCESS) {
        return false;
    }
    apiVersion = vulkanApiVersionXess;
    for (uint32_t i = 0; i < instanceExtensionCountXess; i++) {
        // VK_KHR_get_physical_device_properties2 has been promoted to core Vulkan 1.1.
        // This app requires at least Vulkan 1.1, so this extension is not necessary.
        if (strcmp(instanceExtensionsXess[i], VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME) == 0) {
            continue;
        }
        requiredInstanceExtensions.push_back(instanceExtensionsXess[i]);
    }
    return true;
}

bool getPhysicalDeviceXessSupportInfo(
        sgl::vk::Instance* instance,
        VkPhysicalDevice physicalDevice,
        std::vector<const char*>& requiredDeviceExtensions,
        sgl::vk::DeviceFeatures& requestedDeviceFeatures) {
    uint32_t deviceExtensionCountXess{};
    const char* const* deviceExtensionsXess{};
    if (xessVKGetRequiredDeviceExtensions(
            instance->getVkInstance(), physicalDevice,
            &deviceExtensionCountXess, &deviceExtensionsXess) != XESS_RESULT_SUCCESS) {
        return false;
    }
    void* pNext = nullptr;
    if (xessVKGetRequiredDeviceFeatures(
            instance->getVkInstance(), physicalDevice, &pNext) != XESS_RESULT_SUCCESS) {
        return false;
    }
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.shaderStorageImageReadWithoutFormat = VK_TRUE;
    /*
     * VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2
     * VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MUTABLE_DESCRIPTOR_TYPE_FEATURES_EXT
     * VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES
     * VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES
     */
    if (!requestedDeviceFeatures.setOptionalFeaturesFromPNextChain(pNext, requiredDeviceExtensions)) {
        return false;
    }
    for (uint32_t i = 0; i < deviceExtensionCountXess; i++) {
        bool foundExtensionName = false;
        for (const char* requiredDeviceExtension : requiredDeviceExtensions) {
            if (strcmp(deviceExtensionsXess[i], requiredDeviceExtension) == 0) {
                foundExtensionName = true;
                break;
            }
        }
        if (!foundExtensionName) {
            requiredDeviceExtensions.push_back(deviceExtensionsXess[i]);
        }
    }
    return true;
}
