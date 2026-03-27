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

#ifndef LINEVIS_XESS_HPP
#define LINEVIS_XESS_HPP

#include <vector>
#include <memory>

#include "Upscaler.hpp"

typedef struct _xess_context_handle_t* xess_context_handle_t;

namespace sgl { namespace vk {
class Instance;
struct DeviceFeatures;
class Device;
class ImageView;
typedef std::shared_ptr<ImageView> ImageViewPtr;
}}

enum class XessQualityMode {
    ULTRA_PERFORMANCE = 100, // x3
    PERFORMANCE = 101, // x2.3
    BALANCED = 102, // x2
    QUALITY = 103, // x1.7
    ULTRA_QUALITY = 103, // x1.5
    ULTRA_QUALITY_PLUS = 105, // x1.3
    AA = 106 // x1
};
const char* const XESS_QUALITY_MODE_NAMES[] = {
        "Ultra Performance (x3)", " Performance (x2.3)", "Balanced (x2)", "Quality (x1.7)", "Ultra Quality (x1.5)",
        "Ultra Quality Plus", "AA"
};

class XessSupersampler : public Upscaler {
public:
    ~XessSupersampler() override;
    UpscalerType getUpscalerType() override { return UpscalerType::XESS; }
    bool initialize(sgl::vk::Device* _device) override;
    void setQualityMode(XessQualityMode _qualityMode);
    void setUpscaleAlpha(bool _upscaleAlpha) override;
    bool queryOptimalSettings(
            uint32_t displayWidth, uint32_t displayHeight,
            uint32_t& renderWidthOptimal, uint32_t& renderHeightOptimal,
            uint32_t& renderWidthMax, uint32_t& renderHeightMax,
            uint32_t& renderWidthMin, uint32_t& renderHeightMin,
            float& sharpness) override;
    void setUseAntiAliasingMode() override;
    void setUseJitteredMotionVectors(bool _useJitteredMotionVectors) override;
    void setJitterOffset(float _jitterOffsetX, float _jitterOffsetY) override;
    void resetAccum() override;
    bool renderGui(sgl::PropertyEditor& propertyEditor) override;
    bool apply(
            const sgl::vk::ImageViewPtr& colorImageIn,
            const sgl::vk::ImageViewPtr& colorImageOut,
            const sgl::vk::ImageViewPtr& depthImage,
            const sgl::vk::ImageViewPtr& motionVectorImage,
            const sgl::vk::ImageViewPtr& exposureImage,
            VkCommandBuffer commandBuffer) override;

private:
    void checkRecreateFeature(
            const sgl::vk::ImageViewPtr& colorImageIn,
            const sgl::vk::ImageViewPtr& colorImageOut,
            VkCommandBuffer commandBuffer);
    void _free();
    sgl::vk::Device* device = nullptr;
    bool isInitialized = false;
    xess_context_handle_t context = nullptr;
    XessQualityMode qualityMode = XessQualityMode::BALANCED;

    bool shallResetAccum = true;
    uint32_t cachedRenderWidth = 0;
    uint32_t cachedRenderHeight = 0;
    uint32_t cachedDisplayWidth = 0;
    uint32_t cachedDisplayHeight = 0;
    // Settings of XeSS upscaler.
    bool motionVectorLowRes = true;
    bool useJitteredMotionVectors = false;
    bool isHdr = false;
    bool isDepthInverted = false;
    bool enableAutoExposure = false;
    float jitterOffsetX = 0.0f, jitterOffsetY = 0.0f;
    // Debug options.
    bool resetEveryFrame = false;
};

bool getInstanceXessSupportInfo(
        uint32_t& apiVersion, std::vector<const char*>& requiredInstanceExtensions);

bool getPhysicalDeviceXessSupportInfo(
        sgl::vk::Instance* instance,
        VkPhysicalDevice physicalDevice,
        std::vector<const char*>& requiredDeviceExtensions,
        sgl::vk::DeviceFeatures& requestedDeviceFeatures);

#endif //LINEVIS_XESS_HPP
