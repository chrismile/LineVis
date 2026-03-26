/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2024-2026, Christoph Neuhauser
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

#include <string>
#include <iostream>
#include <filesystem>
#include <cstdlib>
#include <cstring>

#include <Utils/StringUtils.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Instance.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>
#include <Graphics/Vulkan/Image/Image.hpp>

#include <nvsdk_ngx.h>
#include <nvsdk_ngx_helpers.h>
#include <nvsdk_ngx_vk.h>
#include <nvsdk_ngx_helpers_vk.h>

#include "DLSS.hpp"

#include <ImGui/Widgets/PropertyEditor.hpp>

#ifdef __linux__
#include <dlfcn.h>
#endif

#ifdef _WIN32
#define _WIN32_IE 0x0400
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <shlobj.h>
#include <shlwapi.h>
#include <windef.h>
#include <windows.h>
#endif

NVSDK_NGX_ProjectIdDescription getDlssProjectIdDescription() {
    NVSDK_NGX_ProjectIdDescription projectIdDescription{};
    projectIdDescription.EngineType = NVSDK_NGX_ENGINE_TYPE_CUSTOM;
    projectIdDescription.EngineVersion = "0.1.0";
    projectIdDescription.ProjectId = "494d83ef-7ae9-4bf1-9fba-85477321c7e4";
    return projectIdDescription;
}

std::wstring getApplicationDataPath() {
#if defined(__unix__)
    std::filesystem::path configDirectory = std::string() + getenv("HOME") + "/.config/LineVis/";
#else
    // Use WinAPI to query the AppData folder
    char userDataPath[MAX_PATH];
    SHGetSpecialFolderPathA(NULL, userDataPath, CSIDL_APPDATA, true);
    std::string dir = std::string() + userDataPath + "/";
    for (std::string::iterator it = dir.begin(); it != dir.end(); ++it) {
        if (*it == '\\') *it = '/';
    }
    std::filesystem::path configDirectory = dir + "LineVis/";
#endif
    return configDirectory.wstring();
}

struct FeatureCommonTemp {
    std::wstring dllSearchPath;
    const wchar_t* dllSearchPathC = nullptr;
};

bool getInstanceDlssSupportInfo(std::vector<const char*>& requiredInstanceExtensions) {
    auto applicationDataPath = getApplicationDataPath();
    NVSDK_NGX_FeatureCommonInfo featureCommonInfo{};
    NVSDK_NGX_FeatureDiscoveryInfo featureDiscoveryInfo{};
    featureDiscoveryInfo.SDKVersion = NVSDK_NGX_Version_API;
    featureDiscoveryInfo.FeatureID = NVSDK_NGX_Feature_SuperSampling;
    featureDiscoveryInfo.Identifier.IdentifierType = NVSDK_NGX_Application_Identifier_Type_Project_Id;
    featureDiscoveryInfo.ApplicationDataPath = applicationDataPath.c_str();
    featureDiscoveryInfo.Identifier.v.ProjectDesc = getDlssProjectIdDescription();
    featureDiscoveryInfo.FeatureInfo = &featureCommonInfo;

    uint32_t extensionCount = 0;
    VkExtensionProperties* extensions = nullptr;
    auto result = NVSDK_NGX_VULKAN_GetFeatureInstanceExtensionRequirements(
            &featureDiscoveryInfo, &extensionCount, &extensions);
    if (result != NVSDK_NGX_Result_Success) {
        return false;
    }
    for (uint32_t i = 0; i < extensionCount; i++) {
        // VK_KHR_get_physical_device_properties2 has been promoted to core Vulkan 1.1.
        // This app requires at least Vulkan 1.1, so this extension is not necessary.
        if (strcmp(extensions[i].extensionName, VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME) == 0) {
            continue;
        }
        requiredInstanceExtensions.push_back(extensions[i].extensionName);
    }
    return true;
}

bool getPhysicalDeviceDlssSupportInfo(
        sgl::vk::Instance* instance,
        VkPhysicalDevice physicalDevice,
        std::vector<const char*>& requiredDeviceExtensions,
        sgl::vk::DeviceFeatures& requestedDeviceFeatures) {
    auto applicationDataPath = getApplicationDataPath();
    NVSDK_NGX_FeatureCommonInfo featureCommonInfo{};
    NVSDK_NGX_FeatureDiscoveryInfo featureDiscoveryInfo{};
    featureDiscoveryInfo.SDKVersion = NVSDK_NGX_Version_API;
    featureDiscoveryInfo.FeatureID = NVSDK_NGX_Feature_SuperSampling;
    featureDiscoveryInfo.Identifier.IdentifierType = NVSDK_NGX_Application_Identifier_Type_Project_Id;
    featureDiscoveryInfo.ApplicationDataPath = applicationDataPath.c_str();
    featureDiscoveryInfo.Identifier.v.ProjectDesc = getDlssProjectIdDescription();
    featureDiscoveryInfo.FeatureInfo = &featureCommonInfo;

    NVSDK_NGX_FeatureRequirement featureRequirement{};
    NVSDK_NGX_Result result = NVSDK_NGX_VULKAN_GetFeatureRequirements(
            instance->getVkInstance(), physicalDevice, &featureDiscoveryInfo, &featureRequirement);
    if (result != NVSDK_NGX_Result_Success
            && result != NVSDK_NGX_Result_FAIL_NotImplemented) {
        return false;
    }
    if (featureRequirement.FeatureSupported != NVSDK_NGX_FeatureSupportResult_Supported) {
        return false;
    }
    // We ignore MinHWArchitecture and MinOSVersion for now, as we can't do anything about it anyway.

    uint32_t extensionCount = 0;
    VkExtensionProperties* extensions = nullptr;
    result = NVSDK_NGX_VULKAN_GetFeatureDeviceExtensionRequirements(
            instance->getVkInstance(), physicalDevice, &featureDiscoveryInfo, &extensionCount, &extensions);
    if (result != NVSDK_NGX_Result_Success) {
        return false;
    }
    for (uint32_t i = 0; i < extensionCount; i++) {
        requiredDeviceExtensions.push_back(extensions[i].extensionName);
    }

    return true;
}

DlssSupersampler::~DlssSupersampler() {
    _free();
}

void DlssSupersampler::_free() {
    if (dlssFeature) {
        if (NVSDK_NGX_VULKAN_ReleaseFeature(dlssFeature) != NVSDK_NGX_Result_Success) {
            sgl::Logfile::get()->throwError("Error in NVSDK_NGX_VULKAN_ReleaseFeature.");
        }
        dlssFeature = {};
    }
    if (params) {
        if (NVSDK_NGX_VULKAN_DestroyParameters(params) != NVSDK_NGX_Result_Success) {
            sgl::Logfile::get()->throwError("Error in NVSDK_NGX_VULKAN_DestroyParameters.");
        }
        params = {};
    }
    if (isInitialized) {
        if (NVSDK_NGX_VULKAN_Shutdown1(device->getVkDevice()) != NVSDK_NGX_Result_Success) {
            sgl::Logfile::get()->throwError("Error in NVSDK_NGX_VULKAN_Shutdown1.");
        }
        isInitialized = false;
    }
}

//NVSDK_NGX_Result  NVSDK_CONV NVSDK_NGX_VULKAN_Init_with_ProjectID(
//        const char *InProjectId, NVSDK_NGX_EngineType InEngineType, const char *InEngineVersion, const wchar_t *InApplicationDataPath,
//        VkInstance InInstance, VkPhysicalDevice InPD, VkDevice InDevice, PFN_vkGetInstanceProcAddr InGIPA = nullptr, PFN_vkGetDeviceProcAddr InGDPA = nullptr,
//        const NVSDK_NGX_FeatureCommonInfo *InFeatureInfo = nullptr, NVSDK_NGX_Version InSDKVersion = NVSDK_NGX_Version_API);

bool DlssSupersampler::initialize(sgl::vk::Device* _device) {
    device = _device;
    if (device->getDeviceDriverId() != VK_DRIVER_ID_NVIDIA_PROPRIETARY) {
        sgl::Logfile::get()->writeWarning("DLSS is only supported on NVIDIA GPUs.", true);
        return false;
    }

    auto applicationDataPath = getApplicationDataPath();
    NVSDK_NGX_FeatureCommonInfo featureCommonInfo{};
    NVSDK_NGX_Result result = NVSDK_NGX_VULKAN_Init_with_ProjectID(
            "494d83ef-7ae9-4bf1-9fba-85477321c7e4", NVSDK_NGX_ENGINE_TYPE_CUSTOM, "0.1.0",
            applicationDataPath.c_str(),
            device->getInstance()->getVkInstance(), device->getVkPhysicalDevice(), device->getVkDevice(),
            sgl::vk::Instance::getVkInstanceProcAddrFunctionPointer(),
            sgl::vk::Device::getVkDeviceProcAddrFunctionPointer(),
            &featureCommonInfo, NVSDK_NGX_Version_API);
    if (result != NVSDK_NGX_Result_Success) {
        sgl::Logfile::get()->writeError("NVSDK_NGX_VULKAN_Init_with_ProjectID failed.");
        return false;
    }
    isInitialized = true;

    result = NVSDK_NGX_VULKAN_GetCapabilityParameters(&params);
    if (result != NVSDK_NGX_Result_Success) {
        _free();
        sgl::Logfile::get()->writeError("NVSDK_NGX_VULKAN_GetCapabilityParameters failed.");
        return false;
    }

    int needsUpdatedDriver = 0;
    unsigned int minDriverVersionMajor = 0;
    unsigned int minDriverVersionMinor = 0;
    NVSDK_NGX_Result resultUpdate = NVSDK_NGX_Parameter_GetI(
            params, NVSDK_NGX_Parameter_SuperSampling_NeedsUpdatedDriver, &needsUpdatedDriver);
    NVSDK_NGX_Result resultMajor = NVSDK_NGX_Parameter_GetUI(
            params, NVSDK_NGX_Parameter_SuperSampling_MinDriverVersionMajor, &minDriverVersionMajor);
    NVSDK_NGX_Result resultMinor = NVSDK_NGX_Parameter_GetUI(
            params, NVSDK_NGX_Parameter_SuperSampling_MinDriverVersionMinor, &minDriverVersionMinor);
    if (resultUpdate != NVSDK_NGX_Result_Success || resultMajor != NVSDK_NGX_Result_Success
            || resultMinor != NVSDK_NGX_Result_Success) {
        _free();
        sgl::Logfile::get()->writeError("NVSDK_NGX_Parameter_Get* failed.");
        return false;
    }
    if (needsUpdatedDriver) {
        _free();
        sgl::Logfile::get()->writeError(
                "DLSS could not be initialized: NVIDIA driver version >= "
                + std::to_string(minDriverVersionMajor) + "." + std::to_string(minDriverVersionMinor) + " needed.");
        return false;
    }

    int isDlssAvailable = 0;
    result = NVSDK_NGX_Parameter_GetI(params, NVSDK_NGX_Parameter_SuperSampling_Available, &isDlssAvailable);
    if (result != NVSDK_NGX_Result_Success || !isDlssAvailable) {
        int featureInitResult = 0;
        NVSDK_NGX_Result resultInit = NVSDK_NGX_Parameter_GetI(
                params, NVSDK_NGX_Parameter_SuperSampling_FeatureInitResult, &featureInitResult);
        _free();
        if (resultInit != NVSDK_NGX_Result_Success) {
            sgl::Logfile::get()->writeError("DLSS could not be initialized.");
            return false;
        }
        if (featureCommonInfo.PathListInfo.Length > 0) {
            sgl::Logfile::get()->writeInfo("DLSS library search path: "
                    + sgl::wideStringArrayToStdString(featureCommonInfo.PathListInfo.Path[0]));
        }
        sgl::Logfile::get()->writeError(
                std::string() + "DLSS is not available on this system. Feature init result: "
                + sgl::wideStringArrayToStdString(GetNGXResultAsString(NVSDK_NGX_Result(featureInitResult))));
        return false;
    }
    /*result = NVSDK_NGX_Parameter_GetI(params, NVSDK_NGX_Parameter_SuperSampling_FeatureInitResult, &isDlssAvailable);
    if (result != NVSDK_NGX_Result_Success || !isDlssAvailable) {
        _free();
        delete temp;
        sgl::Logfile::get()->writeError("DLSS could not be initialized.");
        return true;
    }*/

    return true;
}

bool DlssSupersampler::queryOptimalSettings(
        uint32_t displayWidth, uint32_t displayHeight,
        uint32_t& renderWidthOptimal, uint32_t& renderHeightOptimal,
        uint32_t& renderWidthMax, uint32_t& renderHeightMax,
        uint32_t& renderWidthMin, uint32_t& renderHeightMin,
        float& sharpness) {
    NVSDK_NGX_Result result = NGX_DLSS_GET_OPTIMAL_SETTINGS(
            params, displayWidth, displayHeight, NVSDK_NGX_PerfQuality_Value(perfQuality),
            &renderWidthOptimal, &renderHeightOptimal,
            &renderWidthMax, &renderHeightMax,
            &renderWidthMin, &renderHeightMin,
            &sharpness);
    return result == NVSDK_NGX_Result_Success;
}

void DlssSupersampler::setUseAntiAliasingMode() {
    setPerfQuality(DlssPerfQuality::DLAA);
    setRenderPreset(DlssRenderPreset::K); //< F is also possible for DLAA, but it was deprecated.
}

void DlssSupersampler::setUseJitteredMotionVectors(bool _useJitteredMotionVectors) {
    useJitteredMotionVectors = _useJitteredMotionVectors;
}

void DlssSupersampler::setJitterOffset(float _jitterOffsetX, float _jitterOffsetY) {
    jitterOffsetX = _jitterOffsetX;
    jitterOffsetY = _jitterOffsetY;
}

static NVSDK_NGX_Resource_VK createNgxImageView(const sgl::vk::ImageViewPtr& imageView) {
    const auto& imageSettings = imageView->getImage()->getImageSettings();
    return NVSDK_NGX_Create_ImageView_Resource_VK(
            imageView->getVkImageView(), imageView->getImage()->getVkImage(),
            imageView->getVkImageSubresourceRange(), imageSettings.format,
            imageSettings.width, imageSettings.height, (imageSettings.usage & VK_IMAGE_USAGE_STORAGE_BIT) != 0);
}

void DlssSupersampler::resetAccum() {
    shallResetAccum = true;
}

bool DlssSupersampler::renderGui(sgl::PropertyEditor& propertyEditor) {
    bool settingsChanged = false;
    DlssPerfQuality _perfQuality = perfQuality;
    if (propertyEditor.addCombo(
            "Mode", (int*)&_perfQuality, DLSS_PERF_QUALITY_NAMES, IM_ARRAYSIZE(DLSS_PERF_QUALITY_NAMES))) {
        setPerfQuality(_perfQuality);
        settingsChanged = true;
    }
    DlssRenderPreset _renderPreset = renderPreset;
    if (propertyEditor.addCombo(
            "Preset", (int*)&_renderPreset, DLSS_RENDER_PRESET_NAMES, IM_ARRAYSIZE(DLSS_RENDER_PRESET_NAMES))) {
        setRenderPreset(_renderPreset);
        settingsChanged = true;
    }
    if (propertyEditor.addCheckbox("Reset always", &resetEveryFrame)) {
        settingsChanged = true;
    }
    return settingsChanged;
}

void DlssSupersampler::checkRecreateFeature(
        const sgl::vk::ImageViewPtr& colorImageIn,
        const sgl::vk::ImageViewPtr& colorImageOut,
        VkCommandBuffer commandBuffer) {
    if (dlssFeature) {
        return;
    }

    //NVSDK_NGX_VULKAN_GetScratchBufferSize(NVSDK_NGX_Feature InFeatureId, const NVSDK_NGX_Parameter *InParameters, size_t *OutSizeInBytes);
    unsigned int creationNodeMask = 1;
    unsigned int visibilityNodeMask = 1;

    int dlssFeatureCreateFlags = NVSDK_NGX_DLSS_Feature_Flags_None;
    dlssFeatureCreateFlags |= motionVectorLowRes ? NVSDK_NGX_DLSS_Feature_Flags_MVLowRes : 0;
    dlssFeatureCreateFlags |= useJitteredMotionVectors ? NVSDK_NGX_DLSS_Feature_Flags_MVJittered : 0;
    dlssFeatureCreateFlags |= isHdr ? NVSDK_NGX_DLSS_Feature_Flags_IsHDR : 0;
    dlssFeatureCreateFlags |= isDepthInverted ? NVSDK_NGX_DLSS_Feature_Flags_DepthInverted : 0;
    dlssFeatureCreateFlags |= enableAutoExposure ? NVSDK_NGX_DLSS_Feature_Flags_AutoExposure : 0;
    dlssFeatureCreateFlags |= upscaleAlpha ? NVSDK_NGX_DLSS_Feature_Flags_AlphaUpscaling : 0;

    auto renderWidth = colorImageIn->getImage()->getImageSettings().width;
    auto renderHeight = colorImageIn->getImage()->getImageSettings().height;
    auto displayWidth = colorImageOut->getImage()->getImageSettings().width;
    auto displayHeight = colorImageOut->getImage()->getImageSettings().height;
    NVSDK_NGX_DLSS_Create_Params dlssCreateParams{};
    dlssCreateParams.Feature.InWidth = renderWidth;
    dlssCreateParams.Feature.InHeight = renderHeight;
    dlssCreateParams.Feature.InTargetWidth = displayWidth;
    dlssCreateParams.Feature.InTargetHeight = displayHeight;
    dlssCreateParams.Feature.InPerfQualityValue = NVSDK_NGX_PerfQuality_Value(perfQuality);
    dlssCreateParams.InFeatureCreateFlags = dlssFeatureCreateFlags;

    NVSDK_NGX_Parameter_SetUI(params, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_DLAA, unsigned(renderPreset));
    NVSDK_NGX_Parameter_SetUI(params, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_Quality, unsigned(renderPreset));
    NVSDK_NGX_Parameter_SetUI(params, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_Balanced, unsigned(renderPreset));
    NVSDK_NGX_Parameter_SetUI(params, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_Performance, unsigned(renderPreset));
    NVSDK_NGX_Parameter_SetUI(params, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_UltraPerformance, unsigned(renderPreset));
    NVSDK_NGX_Parameter_SetUI(params, NVSDK_NGX_Parameter_DLSS_Hint_Render_Preset_UltraQuality, unsigned(renderPreset));

    NVSDK_NGX_Result result = NGX_VULKAN_CREATE_DLSS_EXT1(
            device->getVkDevice(), commandBuffer,
            creationNodeMask, visibilityNodeMask,
            &dlssFeature, params, &dlssCreateParams);
    if (result != NVSDK_NGX_Result_Success) {
        _free();
        sgl::Logfile::get()->throwError("NGX_VULKAN_CREATE_DLSS_EXT1 failed: "
                + sgl::wideStringArrayToStdString(GetNGXResultAsString(NVSDK_NGX_Result(result))));
    }
}

bool DlssSupersampler::apply(
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
        if (dlssFeature) {
            NVSDK_NGX_VULKAN_ReleaseFeature(dlssFeature);
            dlssFeature = {};
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
        if (dlssFeature) {
            NVSDK_NGX_VULKAN_ReleaseFeature(dlssFeature);
            dlssFeature = {};
        }
    }

    bool useSingleTimeCommandBuffer = commandBuffer == VK_NULL_HANDLE;
    if (useSingleTimeCommandBuffer) {
        commandBuffer = device->beginSingleTimeCommands();
    }
    checkRecreateFeature(colorImageIn, colorImageOut, commandBuffer);

    auto colorImageInNgx = createNgxImageView(colorImageIn);
    auto colorImageOutNgx = createNgxImageView(colorImageOut);
    auto depthImageNgx = createNgxImageView(depthImage);
    auto motionVectorImageNgx = createNgxImageView(motionVectorImage);
    auto exposureImageNgx = createNgxImageView(exposureImage);

    if (resetEveryFrame) {
        shallResetAccum = true;
    }

    float mvScaleX = 1.0f;
    float mvScaleY = 1.0f;
    NVSDK_NGX_Coordinates renderOffset = { 0, 0 };
    NVSDK_NGX_Dimensions renderSize = { renderWidth, renderHeight };
    NVSDK_NGX_VK_DLSS_Eval_Params dlssEvalParams{};
    dlssEvalParams.Feature.pInColor = &colorImageInNgx;
    dlssEvalParams.Feature.pInOutput = &colorImageOutNgx;
    dlssEvalParams.pInDepth = &depthImageNgx;
    dlssEvalParams.pInMotionVectors = &motionVectorImageNgx;
    dlssEvalParams.pInExposureTexture = &exposureImageNgx;
    dlssEvalParams.InJitterOffsetX = jitterOffsetX;
    dlssEvalParams.InJitterOffsetY = jitterOffsetY;
    dlssEvalParams.InReset = shallResetAccum;
    dlssEvalParams.InMVScaleX = mvScaleX;
    dlssEvalParams.InMVScaleY = mvScaleY;
    dlssEvalParams.InColorSubrectBase = renderOffset;
    dlssEvalParams.InDepthSubrectBase = renderOffset;
    dlssEvalParams.InTranslucencySubrectBase = renderOffset;
    dlssEvalParams.InMVSubrectBase = renderOffset;
    dlssEvalParams.InRenderSubrectDimensions = renderSize;
    NVSDK_NGX_Result result = NGX_VULKAN_EVALUATE_DLSS_EXT(commandBuffer, dlssFeature, params, &dlssEvalParams);
    if (result != NVSDK_NGX_Result_Success) {
        _free();
        sgl::Logfile::get()->throwError("NGX_VULKAN_EVALUATE_DLSS_EXT failed.");
    }
    if (useSingleTimeCommandBuffer) {
        device->endSingleTimeCommands(commandBuffer);
    }
    shallResetAccum = false;

    return true;
}

void DlssSupersampler::setPerfQuality(DlssPerfQuality _perfQuality) {
    if (perfQuality != _perfQuality) {
        perfQuality = _perfQuality;
        if (dlssFeature) {
            NVSDK_NGX_VULKAN_ReleaseFeature(dlssFeature);
            dlssFeature = {};
        }
    }
}

void DlssSupersampler::setRenderPreset(DlssRenderPreset _preset) {
    if (renderPreset != _preset) {
        renderPreset = _preset;
        if (dlssFeature) {
            NVSDK_NGX_VULKAN_ReleaseFeature(dlssFeature);
            dlssFeature = {};
        }
    }
}

void DlssSupersampler::setUpscaleAlpha(bool _upscaleAlpha) {
    if (upscaleAlpha != _upscaleAlpha) {
        upscaleAlpha = _upscaleAlpha;
        if (dlssFeature) {
            NVSDK_NGX_VULKAN_ReleaseFeature(dlssFeature);
            dlssFeature = {};
        }
    }
}
