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

#ifndef LINEVIS_DLSS_HPP
#define LINEVIS_DLSS_HPP

#include <vector>
#include <memory>

#include "Upscaler.hpp"

struct NVSDK_NGX_Handle;
struct NVSDK_NGX_Parameter;

namespace sgl { namespace vk {
class Instance;
struct DeviceFeatures;
class Device;
class ImageView;
typedef std::shared_ptr<ImageView> ImageViewPtr;
}}

enum class DlssPerfQuality {
    MAX_PERF, BALANCED, MAX_QUALITY, ULTRA_PERFORMANCE, ULTRA_QUALITY, DLAA
};
const char* const DLSS_PERF_QUALITY_NAMES[] = {
    "Max Performance", "Balanced", "Max Quality", "Ultra Performance", "Ultra Quality", "DLAA"
};
enum class DlssRenderPreset {
    DEFAULT, // all
    A, // Perf/Balanced/Quality (removed in 310.4.0; J or K recommended by docs)
    B, // Ultra Perf            (removed in 310.4.0; J or K recommended by docs)
    C, // Perf/Balanced/Quality (removed in 310.4.0; J or K recommended by docs)
    D, // Perf/Balanced/Quality (removed in 310.4.0; J or K recommended by docs)
    E, // Perf/Balanced/Quality (removed in 310.4.0; J or K recommended by docs)
    F, // Ultra Perf/DLAA       (deprecated)
    G, // Unused
    H, // Unused, reserved
    I, // Unused, reserved
    J, // Transformer model, less ghosting, more flickering (since SDK version 310.1.0)
    K, // Transformer model, DLAA/Balanced/Quality, recommended over J (since SDK version 310.2.1)
    L, // Default for Ultra Perf; more stable & expensive than J, K (since SDK version 310.5.0)
    M, // Default for Perf; similar quality as L, but faster; best perf on RTX 40+ (since SDK version 310.5.0)
};
const char* const DLSS_RENDER_PRESET_NAMES[] = {
    "Default", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M"
};

class DlssSupersampler : public Upscaler {
public:
    ~DlssSupersampler() override;
    UpscalerType getUpscalerType() override { return UpscalerType::DLSS; }
    bool initialize(sgl::vk::Device* _device) override;
    void setPerfQuality(DlssPerfQuality _perfQuality);
    void setRenderPreset(DlssRenderPreset _preset);
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
    NVSDK_NGX_Handle* dlssFeature = nullptr;
    NVSDK_NGX_Parameter* params = nullptr;
    DlssPerfQuality perfQuality = DlssPerfQuality::MAX_PERF;
    DlssRenderPreset renderPreset = DlssRenderPreset::DEFAULT;
    bool shallResetAccum = true;
    uint32_t cachedRenderWidth = 0;
    uint32_t cachedRenderHeight = 0;
    uint32_t cachedDisplayWidth = 0;
    uint32_t cachedDisplayHeight = 0;
    // Settings of DLSS feature.
    bool motionVectorLowRes = true;
    bool useJitteredMotionVectors = false;
    bool isHdr = false;
    bool isDepthInverted = false;
    bool enableAutoExposure = false;
    bool upscaleAlpha = false;
    float jitterOffsetX = 0.0f, jitterOffsetY = 0.0f;
    // Debug options.
    bool resetEveryFrame = false;
};

bool getInstanceDlssSupportInfo(std::vector<const char*>& requiredInstanceExtensions);

bool getPhysicalDeviceDlssSupportInfo(
        sgl::vk::Instance* instance,
        VkPhysicalDevice physicalDevice,
        std::vector<const char*>& requiredDeviceExtensions,
        sgl::vk::DeviceFeatures& requestedDeviceFeatures);

#endif //LINEVIS_DLSS_HPP
