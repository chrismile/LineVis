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

const char* const OPTIX_DENOISTER_MODEL_KIND_NAME[] = {
        "LDR", "HDR"
};

/**
 * A wrapper of the NVIDIA OptiX Denoiser (https://developer.nvidia.com/optix-denoiser).
 */
class OptixVptDenoiser : public Denoiser {
public:
    // Initializes/frees CUDA and OptiX. Should be called at the start/end of the program.
    static bool initGlobal();
    static void freeGlobal();
    static bool isOptixEnabled();

    explicit OptixVptDenoiser(sgl::vk::Renderer* renderer);
    ~OptixVptDenoiser() override;

    DenoiserType getDenoiserType() override { return DenoiserType::OPTIX; }
    [[nodiscard]] const char* getDenoiserName() const override { return "OptiX Denoiser"; }
    void setOutputImage(sgl::vk::ImageViewPtr& outputImage) override;
    void setFeatureMap(const std::string& featureMapName, const sgl::vk::TexturePtr& featureTexture) override;
    void denoise() override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    sgl::vk::Renderer* renderer = nullptr;
    bool showWindow = true;

    // Global CUDA and OptiX data.
    static CUcontext cuContext;
    static CUdevice cuDevice;
    static void* optixHandle;
    static OptixDeviceContext context;

    // Denoiser data.
    void createDenoiser();
    void runOptixDenoiser();
    CUstream stream{};
    OptixDenoiser denoiser{};
    OptixDenoiserModelKind denoiserModelKind = OPTIX_DENOISER_MODEL_KIND_HDR;
    int denoiserModelKindIndex = int(denoiserModelKind) - int(OPTIX_DENOISER_MODEL_KIND_LDR);
    bool denoiseAlpha = false;

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
    sgl::vk::ImageViewPtr inputImageVulkan;
    sgl::vk::ImageViewPtr outputImageVulkan;
    sgl::vk::BufferPtr inputImageBufferVk, outputImageBufferVk;
    sgl::vk::BufferCudaDriverApiExternalMemoryVkPtr inputImageBufferCu, outputImageBufferCu;
    OptixImage2D inputImageOptix{};
    OptixImage2D outputImageOptix{};

    // Synchronization primitives.
    std::vector<sgl::vk::CommandBufferPtr> postRenderCommandBuffers;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> renderFinishedSemaphores;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> denoiseFinishedSemaphores;
    uint64_t timelineValue = 0;
};

#endif //LINEVIS_OPTIXVPTDENOISER_HPP
