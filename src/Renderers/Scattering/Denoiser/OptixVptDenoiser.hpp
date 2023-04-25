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

#ifndef LINEVIS_OPTIXVPTDENOISER_HPP
#define LINEVIS_OPTIXVPTDENOISER_HPP

#include <Graphics/Vulkan/Render/CommandBuffer.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>

#include "Denoiser.hpp"

#include <Graphics/Vulkan/Utils/InteropCuda.hpp>
#include <optix_types.h>

class VectorBlitPass;

/**
 * A wrapper of the NVIDIA OptiX Denoiser (https://developer.nvidia.com/optix-denoiser).
 */
class OptixVptDenoiser : public Denoiser {
public:
    // Initializes/frees CUDA and OptiX. Should be called at the start/end of the program.
    static bool initGlobal(CUcontext _cuContext, CUdevice _cuDevice);
    static void freeGlobal();
    static bool isOptixEnabled();

    explicit OptixVptDenoiser(sgl::vk::Renderer* renderer);
    ~OptixVptDenoiser() override;

    DenoiserType getDenoiserType() const override { return DenoiserType::OPTIX; }
    void setOutputImage(sgl::vk::ImageViewPtr& outputImage) override;
    void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) override;
    [[nodiscard]] bool getUseFeatureMap(FeatureMapType featureMapType) const override;
    [[nodiscard]] bool getWantsAccumulatedInput() const override {
        return true;
    };
    void setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) override;
    void setTemporalDenoisingEnabled(bool enabled); //< Call if renderer doesn't support temporal denoising.
    void resetFrameNumber() override;
    void denoise() override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;

    bool setNewSettings(const SettingsMap& settings) override;

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    sgl::vk::Renderer* renderer = nullptr;

    // Global CUDA and OptiX data.
    static CUcontext cuContext;
    static CUdevice cuDevice;
    static void* optixHandle;
    static OptixDeviceContext context;
    static bool isOptixInitialized;

    // Denoiser data.
    void createDenoiser();
    void runOptixDenoiser();
    CUstream stream{};
    OptixDenoiser denoiser{};
    // If supporting OPTIX_DENOISER_MODEL_KIND_UPSCALE2X/OPTIX_DENOISER_MODEL_KIND_TEMPORAL_UPSCALE2X in the future:
    // Check OPTIX_VERSION >= 70500.
    const char* const OPTIX_DENOISER_MODEL_KIND_NAME[3] = {
            "LDR", "HDR", "Temporal"
    };
    OptixDenoiserModelKind denoiserModelKind = OPTIX_DENOISER_MODEL_KIND_TEMPORAL;
    // int denoiserModelKindIndex = int(denoiserModelKind) - int(OPTIX_DENOISER_MODEL_KIND_LDR);
    int denoiserModelKindIndex = 2;
    int numDenoisersSupported = (int)(sizeof(OPTIX_DENOISER_MODEL_KIND_NAME) / sizeof(*OPTIX_DENOISER_MODEL_KIND_NAME));
    bool useNormalMap = false;
    bool useAlbedo = false;
    bool denoiseAlpha = false;
    bool recreateDenoiserNextFrame = false;
    bool isFirstFrame = true; ///< For resetting temporal accumulation.

    // Frame size dependent data.
    void _freeBuffers();
    uint32_t inputWidth = 0;
    uint32_t inputHeight = 0;
    CUdeviceptr denoiserState{};
    CUdeviceptr scratch{};
    CUdeviceptr imageIntensity{};
    size_t denoiserStateSizeInBytes = 0;
    size_t scratchSizeInBytes = 0;

    // Image data.
    sgl::vk::ImageViewPtr inputImageVulkan, normalImageVulkan, albedoImageVulkan, flowImageVulkan, outputImageVulkan;
    OptixImage2D inputImageOptix{}, normalImageOptix{}, albedoImageOptix{}, flowImageOptix{}, outputImageOptix{};
    sgl::vk::BufferPtr inputImageBufferVk, normalImageBufferVk, albedoImageBufferVk, flowImageBufferVk, outputImageBufferVk;
    sgl::vk::BufferCudaDriverApiExternalMemoryVkPtr inputImageBufferCu, normalImageBufferCu, albedoImageBufferCu, flowImageBufferCu, outputImageBufferCu;

    // For blitting 3D homogeneous vectors with four components into a contiguous array of 3 floating point elements.
    std::shared_ptr<VectorBlitPass> normalBlitPass;

    // Synchronization primitives.
    std::vector<sgl::vk::CommandBufferPtr> postRenderCommandBuffers;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> renderFinishedSemaphores;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> denoiseFinishedSemaphores;
    uint64_t timelineValue = 0;
};

class VectorBlitPass : public sgl::vk::ComputePass {
public:
    explicit VectorBlitPass(sgl::vk::Renderer* renderer);
    void setInputImage(const sgl::vk::ImageViewPtr& _inputImage);
    void setOutputBuffer(const sgl::vk::BufferPtr& _outputBuffer);

protected:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

private:
    const int BLOCK_SIZE = 16;
    sgl::vk::ImageViewPtr inputImage;
    sgl::vk::BufferPtr outputBuffer;
};

#endif //LINEVIS_OPTIXVPTDENOISER_HPP
