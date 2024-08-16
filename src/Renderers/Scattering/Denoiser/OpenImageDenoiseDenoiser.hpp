/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2024, Christoph Neuhauser
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

#ifndef CLOUDRENDERING_OPENIMAGEDENOISEDENOISER_HPP
#define CLOUDRENDERING_OPENIMAGEDENOISEDENOISER_HPP

#include <Graphics/Vulkan/Render/Renderer.hpp>

#ifdef SUPPORT_CUDA_INTEROP
#include <Graphics/Vulkan/Utils/InteropCuda.hpp>
#endif

#include <OpenImageDenoise/oidn.h>

#include "Denoiser.hpp"

enum class OIDNDeviceTypeCustom {
    DEFAULT = 0, GPU_UUID = 1, CPU = 2, SYCL = 3, CUDA = 4, HIP = 5, METAL = 6, CUDA_SHARED = 7
};

const char* const OIDN_DEVICE_TYPE_NAMES[] = {
        "Default", "GPU by UUID", "CPU", "SYCL", "CUDA", "HIP", "Metal", "CUDA (Shared)"
};

enum class OIDNQualityCustom {
    DEFAULT, FAST, BALANCED, HIGH
};

const char* const OIDN_QUALITY_NAMES[] = {
        "Default", "Fast", "Balanced", "High"
};

const int OIDN_QUALITY_MAP[] = {
        OIDN_QUALITY_DEFAULT, OIDN_QUALITY_FAST, OIDN_QUALITY_BALANCED, OIDN_QUALITY_HIGH
};

enum class OIDNFeatureMaps {
    NONE, ALBEDO, ALBEDO_NORMAL
};

const char* const OIDN_FEATURE_MAP_NAMES[] = {
        "None", "Albedo", "Albedo + Normal"
};

namespace sgl { namespace vk {
class BufferCustomInteropVk;
typedef std::shared_ptr<BufferCustomInteropVk> BufferCustomInteropVkPtr;
}}

class AlphaBlitPass;

class OpenImageDenoiseDenoiser : public Denoiser {
public:
    explicit OpenImageDenoiseDenoiser(sgl::vk::Renderer* renderer, bool _denoiseAlpha);
    ~OpenImageDenoiseDenoiser() override;

#ifdef SUPPORT_CUDA_INTEROP
    static bool initGlobalCuda(CUcontext _cuContext, CUdevice _cuDevice);
#endif

    [[nodiscard]] DenoiserType getDenoiserType() const override { return DenoiserType::OPEN_IMAGE_DENOISE; }
    void setOutputImage(sgl::vk::ImageViewPtr& outputImage) override;
    void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) override;
    [[nodiscard]] bool getUseFeatureMap(FeatureMapType featureMapType) const override;
    void setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) override;
    void resetFrameNumber() override {} // No temporal denoising, thus unused.
    void setTemporalDenoisingEnabled(bool enabled) override {} // No temporal denoising, thus unused.
    void denoise() override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    sgl::vk::Renderer* renderer = nullptr;
    std::shared_ptr<AlphaBlitPass> alphaBlitPass;
    sgl::vk::ImageViewPtr inputImageVulkan, albedoImageVulkan, normalImageVulkan, outputImageVulkan;
    sgl::vk::BufferPtr inputImageBufferVk, inputAlbedoBufferVk, inputNormalBufferVk, outputImageBufferVk;
    sgl::vk::BufferCustomInteropVkPtr inputImageBufferInterop, inputAlbedoBufferInterop, inputNormalBufferInterop, outputImageBufferInterop;

#ifdef SUPPORT_CUDA_INTEROP
    static CUcontext cuContext;
    static CUdevice cuDevice;
    CUstream cuStream{};
    sgl::vk::BufferCudaDriverApiExternalMemoryVkPtr inputImageBufferCu, inputAlbedoBufferCu, inputNormalBufferCu, outputImageBufferCu;
    std::vector<sgl::vk::CommandBufferPtr> postRenderCommandBuffers;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> renderFinishedSemaphores;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> denoiseFinishedSemaphores;
    uint64_t timelineValue = 0;
#endif

    // OpenImageDenoise data.
    void _createDenoiser();
    void _freeDenoiser();
    void _createFilter();
    void _freeFilter();
    void _createBuffers();
    void _freeBuffers();
    uint32_t inputWidth = 0;
    uint32_t inputHeight = 0;
    OIDNDeviceTypeCustom deviceType = OIDNDeviceTypeCustom::GPU_UUID;
    int deviceTypeIdx = int(OIDNDeviceTypeCustom::GPU_UUID);
    std::vector<OIDNDeviceTypeCustom> deviceTypes;
    std::vector<const char*> deviceTypeNames;
    OIDNQualityCustom filterQuality = OIDNQualityCustom::DEFAULT;
    bool useNormalMap = false;
    bool useAlbedo = false;
    bool denoiseAlpha = false;
    bool supportsMemoryImport = false;
    bool recreateDenoiserNextFrame = false;
    bool recreateFilterNextFrame = false;
    bool recreateBuffersNextFrame = false;
    OIDNDevice oidnDevice{};
    OIDNBuffer oidnInputColorBuffer{};
    OIDNBuffer oidnInputAlbedoBuffer{};
    OIDNBuffer oidnInputNormalBuffer{};
    OIDNBuffer oidnOutputColorBuffer{};
    OIDNFilter oidnFilter{}, oidnFilterAlpha{};
};

#endif //CLOUDRENDERING_OPENIMAGEDENOISEDENOISER_HPP
