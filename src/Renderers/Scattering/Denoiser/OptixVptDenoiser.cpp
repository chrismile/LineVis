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

#include <Math/Math.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Render/CommandBuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include "OptixVptDenoiser.hpp"

#include <optix.h>
#include <optix_stubs.h>
#include <optix_function_table_definition.h>

#if CUDA_VERSION >= 11020
#define USE_TIMELINE_SEMAPHORES
#elif defined(_WIN32)
#error Binary semaphore sharing is broken on Windows. Please install CUDA >= 11.2 for timeline semaphore support.
#endif

static void _checkOptixResult(OptixResult result, const std::string& text, const std::string& locationText) {
    if (result != OPTIX_SUCCESS) {
        sgl::Logfile::get()->throwError(std::string() + locationText + ": " + text + optixGetErrorString(result));
    }
}

#define checkOptixResult(result, text) _checkOptixResult(result, text, __FILE__ ":" TOSTRING(__LINE__))

static void optixLogCallback(unsigned int level, const char* tag, const char* message, void* cbdata) {
    std::string logText0 = "Optix log level: " + std::to_string(level);
    std::string logText1 = std::string() + "Optix log tag: " + tag;
    std::string logText2 = std::string() + "Optix log message: " + message;

    // 1: fatal
    // 2: error
    // 3: warning
    // 4: print
    if (level < 4) {
        sgl::Logfile::get()->writeError(logText0);
        sgl::Logfile::get()->writeError(logText1);
        sgl::Logfile::get()->writeError(logText2);
        sgl::Logfile::get()->writeError("");
    } else {
        sgl::Logfile::get()->writeInfo(logText0);
        sgl::Logfile::get()->writeInfo(logText1);
        sgl::Logfile::get()->writeInfo(logText2);
        sgl::Logfile::get()->writeInfo("");
    }
}


CUcontext OptixVptDenoiser::cuContext = {};
CUdevice OptixVptDenoiser::cuDevice = 0;
void* OptixVptDenoiser::optixHandle = nullptr;
OptixDeviceContext OptixVptDenoiser::context = {};
bool OptixVptDenoiser::isOptixInitialized = false;

bool OptixVptDenoiser::initGlobal() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    if (!device || device->getDeviceDriverId() != VK_DRIVER_ID_NVIDIA_PROPRIETARY) {
        sgl::Logfile::get()->writeInfo(
                "Using a Vulkan driver that is not the proprietary NVIDIA driver. Disabling OptiX support.");
        return false;
    }

    if (!sgl::vk::getIsCudaDeviceApiFunctionTableInitialized() && !sgl::vk::initializeCudaDeviceApiFunctionTable()) {
        return false;
    }

    CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuInit(0);
    if (cuResult == CUDA_ERROR_NO_DEVICE) {
        sgl::Logfile::get()->writeInfo("No CUDA-capable device was found. Disabling OptiX support.");
        return false;
    }
    sgl::vk::checkCUresult(cuResult, "Error in cuInit: ");

    cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuCtxCreate(&cuContext, CU_CTX_SCHED_SPIN, cuDevice);
    sgl::vk::checkCUresult(cuResult, "Error in cuCtxCreate: ");

    OptixResult result = optixInitWithHandle(&optixHandle);
    if (result == OPTIX_ERROR_LIBRARY_NOT_FOUND) {
        sgl::Logfile::get()->writeInfo("OptiX shared library was not found. Disabling OptiX support.");
        return false;
    }
    if (result == OPTIX_ERROR_UNSUPPORTED_ABI_VERSION) {
        sgl::Logfile::get()->writeInfo("Unsupported OptiX ABI version. Disabling OptiX support.");
        return false;
    }
    checkOptixResult(result, "Error in optixInitWithHandle: ");

    OptixDeviceContextOptions contextOptions{};
    result = optixDeviceContextCreate(cuContext, &contextOptions, &context);
    checkOptixResult(result, "Error in optixDeviceContextCreate: ");

    optixDeviceContextSetLogCallback(
            context, optixLogCallback, nullptr, 3);
    checkOptixResult(result, "Error in optixDeviceContextCreate: ");

    isOptixInitialized = true;

    return true;
}

void OptixVptDenoiser::freeGlobal() {
    OptixResult result = optixDeviceContextDestroy(context);
    checkOptixResult(result, "Error in optixDeviceContextDestroy: ");

    result = optixUninitWithHandle(optixHandle);
    checkOptixResult(result, "Error in optixUninitWithHandle: ");

    CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuCtxDestroy(cuContext);
    sgl::vk::checkCUresult(cuResult, "Error in cuCtxDestroy: ");

    sgl::vk::freeCudaDeviceApiFunctionTable();
}

bool OptixVptDenoiser::isOptixEnabled() {
    return isOptixInitialized && optixHandle && cuContext;
}


OptixVptDenoiser::OptixVptDenoiser(sgl::vk::Renderer* renderer) : renderer(renderer) {
    CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuStreamCreate(&stream, CU_STREAM_DEFAULT);
    sgl::vk::checkCUresult(cuResult, "Error in cuStreamCreate: ");

    normalBlitPass = std::make_shared<VectorBlitPass>(renderer);

    createDenoiser();
}

void OptixVptDenoiser::createDenoiser() {
    if (denoiser) {
        OptixResult result = optixDenoiserDestroy(denoiser);
        checkOptixResult(result, "Error in optixDenoiserDestroy: ");
        denoiser = {};
    }

    OptixDenoiserOptions options;
    options.guideAlbedo = useAlbedo && albedoImageVulkan ? 1 : 0;
    options.guideNormal = useNormalMap && normalImageVulkan ? 1 : 0;
    OptixResult result = optixDenoiserCreate(
            context, denoiserModelKind, &options, &denoiser);
    checkOptixResult(result, "Error in optixDenoiserCreate: ");
}

OptixVptDenoiser::~OptixVptDenoiser() {
    _freeBuffers();

    if (denoiser) {
        OptixResult result = optixDenoiserDestroy(denoiser);
        checkOptixResult(result, "Error in optixDenoiserDestroy: ");
        denoiser = {};
    }

    CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuStreamDestroy(stream);
    sgl::vk::checkCUresult(cuResult, "Error in cuStreamDestroy: ");
}

void OptixVptDenoiser::setOutputImage(sgl::vk::ImageViewPtr& outputImage) {
    outputImageVulkan = outputImage;
}

void OptixVptDenoiser::setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) {
    if (featureMapType == FeatureMapType::COLOR) {
        inputImageVulkan = featureTexture->getImageView();
    } else if (featureMapType == FeatureMapType::NORMAL) {
        normalImageVulkan = featureTexture->getImageView();
    } else if (featureMapType == FeatureMapType::ALBEDO) {
        albedoImageVulkan = featureTexture->getImageView();
    } else if (featureMapType == FeatureMapType::FLOW) {
        flowImageVulkan = featureTexture->getImageView();
    } else if (featureMapType == FeatureMapType::DEPTH || featureMapType == FeatureMapType::POSITION) {
        // Ignore.
    } else {
        sgl::Logfile::get()->writeWarning("Warning in OptixVptDenoiser::setFeatureMap: Unknown feature map.");
    }
}

bool OptixVptDenoiser::getUseFeatureMap(FeatureMapType featureMapType) const {
    if (featureMapType == FeatureMapType::COLOR) {
        return true;
    } else if (featureMapType == FeatureMapType::ALBEDO) {
        return useAlbedo;
    } else if (featureMapType == FeatureMapType::NORMAL) {
        return useNormalMap;
    } else if (featureMapType == FeatureMapType::FLOW) {
        return denoiserModelKind == OPTIX_DENOISER_MODEL_KIND_TEMPORAL;
    } else {
        return false;
    }
}

void OptixVptDenoiser::setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) {
    if ((featureMapType == FeatureMapType::ALBEDO && useAlbedo != useFeature)
            || (featureMapType == FeatureMapType::NORMAL && useNormalMap != useFeature)
            || (featureMapType == FeatureMapType::FLOW && getUseFeatureMap(FeatureMapType::FLOW) != useFeature)) {
        recreateDenoiserNextFrame = true;
    }
    if (featureMapType == FeatureMapType::COLOR) {
        if (!useFeature) {
            sgl::Logfile::get()->writeError(
                    "Warning in OptixVptDenoiser::setUseFeatureMap: Cannot disable use of color feature map.");
        }
    } else if (featureMapType == FeatureMapType::ALBEDO) {
        useAlbedo = useFeature;
    } else if (featureMapType == FeatureMapType::NORMAL) {
        useNormalMap = useFeature;
    } else if (featureMapType == FeatureMapType::FLOW) {
        denoiserModelKind = useFeature ? OPTIX_DENOISER_MODEL_KIND_HDR : OPTIX_DENOISER_MODEL_KIND_TEMPORAL;
        denoiserModelKindIndex = useFeature ? 0 : 2;
    }
}

void OptixVptDenoiser::setTemporalDenoisingEnabled(bool enabled) {
    numDenoisersSupported = IM_ARRAYSIZE(OPTIX_DENOISER_MODEL_KIND_NAME) - (enabled ? 0 : 1);
}

void OptixVptDenoiser::recreateSwapchain(uint32_t width, uint32_t height) {
    _freeBuffers();
    sgl::vk::Device* device = renderer->getDevice();

    CUresult cuResult;
    OptixResult result;

    inputWidth = width;
    inputHeight = height;

    OptixDenoiserSizes denoiserSizes;
    result = optixDenoiserComputeMemoryResources(
            denoiser, inputWidth, inputHeight,
            &denoiserSizes);
    checkOptixResult(result, "Error in optixDenoiserComputeMemoryResources: ");

    denoiserStateSizeInBytes = denoiserSizes.stateSizeInBytes;
    cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuMemAlloc(&denoiserState, denoiserStateSizeInBytes);
    sgl::vk::checkCUresult(cuResult, "Error in cuMemAlloc: ");

    scratchSizeInBytes = denoiserSizes.withoutOverlapScratchSizeInBytes;
    cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuMemAlloc(&scratch, scratchSizeInBytes);
    sgl::vk::checkCUresult(cuResult, "Error in cuMemAlloc: ");

    cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuMemAlloc(&imageIntensity, sizeof(float));
    sgl::vk::checkCUresult(cuResult, "Error in cuMemAlloc: ");

    result = optixDenoiserSetup(
            denoiser, stream, inputWidth, inputHeight,
            denoiserState, denoiserStateSizeInBytes, scratch, scratchSizeInBytes);
    checkOptixResult(result, "Error in optixDenoiserSetup: ");

    inputImageBufferCu = {};
    inputImageBufferVk = std::make_shared<sgl::vk::Buffer>(
            device, inputWidth * inputHeight * 4 * sizeof(float),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
            true, true);
    inputImageBufferCu = std::make_shared<sgl::vk::BufferCudaDriverApiExternalMemoryVk>(inputImageBufferVk);

    inputImageOptix = {};
    inputImageOptix.width = inputWidth;
    inputImageOptix.height = inputHeight;
    inputImageOptix.format = OPTIX_PIXEL_FORMAT_FLOAT4;
    inputImageOptix.pixelStrideInBytes = 4 * sizeof(float);
    inputImageOptix.rowStrideInBytes = inputWidth * 4 * sizeof(float);
    inputImageOptix.data = inputImageBufferCu->getCudaDevicePtr();

    if (useAlbedo && albedoImageVulkan) {
        albedoImageBufferCu = {};
        albedoImageBufferVk = std::make_shared<sgl::vk::Buffer>(
                device, inputWidth * inputHeight * 4 * sizeof(float),
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT
                | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
                true, true);
        albedoImageBufferCu = std::make_shared<sgl::vk::BufferCudaDriverApiExternalMemoryVk>(albedoImageBufferVk);

        albedoImageOptix = {};
        albedoImageOptix.width = inputWidth;
        albedoImageOptix.height = inputHeight;
        albedoImageOptix.format = OPTIX_PIXEL_FORMAT_FLOAT4;
        albedoImageOptix.pixelStrideInBytes = 4 * sizeof(float);
        albedoImageOptix.rowStrideInBytes = inputWidth * 4 * sizeof(float);
        albedoImageOptix.data = albedoImageBufferCu->getCudaDevicePtr();
    }

    if (useNormalMap && normalImageVulkan) {
        normalImageBufferCu = {};
        normalImageBufferVk = std::make_shared<sgl::vk::Buffer>(
                device, inputWidth * inputHeight * 3 * sizeof(float),
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT
                | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
                true, true);
        normalImageBufferCu = std::make_shared<sgl::vk::BufferCudaDriverApiExternalMemoryVk>(normalImageBufferVk);

        normalImageOptix = {};
        normalImageOptix.width = inputWidth;
        normalImageOptix.height = inputHeight;
        normalImageOptix.format = OPTIX_PIXEL_FORMAT_FLOAT3;
        normalImageOptix.pixelStrideInBytes = 3 * sizeof(float);
        normalImageOptix.rowStrideInBytes = inputWidth * 3 * sizeof(float);
        normalImageOptix.data = normalImageBufferCu->getCudaDevicePtr();

        normalBlitPass->setInputImage(normalImageVulkan);
        normalBlitPass->setOutputBuffer(normalImageBufferVk);
    }

    if (getUseFeatureMap(FeatureMapType::FLOW) && flowImageVulkan) {
        flowImageBufferCu = {};
        flowImageBufferVk = std::make_shared<sgl::vk::Buffer>(
                device, inputWidth * inputHeight * 2 * sizeof(float),
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT
                | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
                true, true);
        flowImageBufferCu = std::make_shared<sgl::vk::BufferCudaDriverApiExternalMemoryVk>(flowImageBufferVk);

        flowImageOptix = {};
        flowImageOptix.width = inputWidth;
        flowImageOptix.height = inputHeight;
        flowImageOptix.format = OPTIX_PIXEL_FORMAT_FLOAT2;
        flowImageOptix.pixelStrideInBytes = 2 * sizeof(float);
        flowImageOptix.rowStrideInBytes = inputWidth * 2 * sizeof(float);
        flowImageOptix.data = flowImageBufferCu->getCudaDevicePtr();
    }

    outputImageBufferCu = {};
    outputImageBufferVk = std::make_shared<sgl::vk::Buffer>(
            device, inputWidth * inputHeight * 4 * sizeof(float),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
            true, true);
    outputImageBufferCu = std::make_shared<sgl::vk::BufferCudaDriverApiExternalMemoryVk>(outputImageBufferVk);

    outputImageOptix = {};
    outputImageOptix.width = inputWidth;
    outputImageOptix.height = inputHeight;
    outputImageOptix.format = OPTIX_PIXEL_FORMAT_FLOAT4;
    outputImageOptix.pixelStrideInBytes = 4 * sizeof(float);
    outputImageOptix.rowStrideInBytes = inputWidth * 4 * sizeof(float);
    outputImageOptix.data = outputImageBufferCu->getCudaDevicePtr();

    sgl::vk::Swapchain* swapchain = sgl::AppSettings::get()->getSwapchain();
    size_t numImages = swapchain ? swapchain->getNumImages() : 1;

    postRenderCommandBuffers.clear();
    renderFinishedSemaphores.clear();
    denoiseFinishedSemaphores.clear();
    sgl::vk::CommandPoolType commandPoolType;
    commandPoolType.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    for (size_t frameIdx = 0; frameIdx < numImages; frameIdx++) {
        postRenderCommandBuffers.push_back(std::make_shared<sgl::vk::CommandBuffer>(device, commandPoolType));
#ifdef USE_TIMELINE_SEMAPHORES
        denoiseFinishedSemaphores.push_back(std::make_shared<sgl::vk::SemaphoreVkCudaDriverApiInterop>(
                device, 0, VK_SEMAPHORE_TYPE_TIMELINE,
                timelineValue));
        renderFinishedSemaphores.push_back(std::make_shared<sgl::vk::SemaphoreVkCudaDriverApiInterop>(
                device, 0, VK_SEMAPHORE_TYPE_TIMELINE,
                timelineValue));
#else
        denoiseFinishedSemaphores.push_back(std::make_shared<sgl::vk::SemaphoreVkCudaDriverApiInterop>(device));
        renderFinishedSemaphores.push_back(std::make_shared<sgl::vk::SemaphoreVkCudaDriverApiInterop>(device));
#endif
    }
}

void OptixVptDenoiser::_freeBuffers() {
    if (denoiserState) {
        CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuMemFree(denoiserState);
        sgl::vk::checkCUresult(cuResult, "Error in cuMemFree: ");
        denoiserState = {};
    }
    if (scratch) {
        CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuMemFree(scratch);
        sgl::vk::checkCUresult(cuResult, "Error in cuMemFree: ");
        scratch = {};
    }
    if (imageIntensity) {
        CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuMemFree(imageIntensity);
        sgl::vk::checkCUresult(cuResult, "Error in cuMemFree: ");
        imageIntensity = {};
    }
}

void OptixVptDenoiser::resetFrameNumber() {
    isFirstFrame = true;
}

void OptixVptDenoiser::denoise() {
    if (recreateDenoiserNextFrame) {
        createDenoiser();
        recreateSwapchain(
                inputImageVulkan->getImage()->getImageSettings().width,
                inputImageVulkan->getImage()->getImageSettings().height);
        recreateDenoiserNextFrame = false;
    }

    sgl::vk::Swapchain* swapchain = sgl::AppSettings::get()->getSwapchain();
    uint32_t frameIndex = swapchain ? swapchain->getImageIndex() : 0;
    timelineValue++;

    inputImageVulkan->getImage()->transitionImageLayout(
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, renderer->getVkCommandBuffer());
    inputImageVulkan->getImage()->copyToBuffer(
            inputImageBufferVk, renderer->getVkCommandBuffer());

    if (useAlbedo && albedoImageVulkan) {
        albedoImageVulkan->getImage()->transitionImageLayout(
                VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, renderer->getVkCommandBuffer());
        albedoImageVulkan->getImage()->copyToBuffer(
                albedoImageBufferVk, renderer->getVkCommandBuffer());
    }

    if (useNormalMap && normalImageVulkan) {
        if (normalImageVulkan->getImage()->getImageSettings().format == VK_FORMAT_R32G32B32_SFLOAT) {
            normalImageVulkan->getImage()->transitionImageLayout(
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, renderer->getVkCommandBuffer());
            normalImageVulkan->getImage()->copyToBuffer(
                    normalImageBufferVk, renderer->getVkCommandBuffer());
        } else {
            normalImageVulkan->getImage()->transitionImageLayout(
                    VK_IMAGE_LAYOUT_GENERAL, renderer->getVkCommandBuffer());
            normalBlitPass->render();
        }
    }

    if (getUseFeatureMap(FeatureMapType::FLOW) && flowImageVulkan) {
        flowImageVulkan->getImage()->transitionImageLayout(
                VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, renderer->getVkCommandBuffer());
        flowImageVulkan->getImage()->copyToBuffer(
                flowImageBufferVk, renderer->getVkCommandBuffer());
    }

    //renderer->insertBufferMemoryBarrier(
    //        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_MEMORY_READ_BIT,
    //        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
    //        inputImageBufferVk);

    sgl::vk::CommandBufferPtr commandBufferPreDenoise = renderer->getCommandBuffer();
    sgl::vk::SemaphorePtr renderFinishedSemaphore = renderFinishedSemaphores.at(frameIndex);
#ifdef USE_TIMELINE_SEMAPHORES
    renderFinishedSemaphore->setSignalSemaphoreValue(timelineValue);
#endif
    commandBufferPreDenoise->pushSignalSemaphore(renderFinishedSemaphore);
    renderer->endCommandBuffer();

    runOptixDenoiser();

    sgl::vk::CommandBufferPtr postRenderCommandBuffer = postRenderCommandBuffers.at(frameIndex);
    sgl::vk::SemaphorePtr denoiseFinishedSemaphore = denoiseFinishedSemaphores.at(frameIndex);
#ifdef USE_TIMELINE_SEMAPHORES
    denoiseFinishedSemaphore->setWaitSemaphoreValue(timelineValue);
#endif
    renderer->pushCommandBuffer(postRenderCommandBuffer);
    renderer->beginCommandBuffer();
    postRenderCommandBuffer->pushWaitSemaphore(
            denoiseFinishedSemaphore, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);

    //renderer->insertBufferMemoryBarrier(
    //        VK_ACCESS_MEMORY_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
    //        VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
    //        outputImageBufferVk);
    outputImageVulkan->getImage()->transitionImageLayout(
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, renderer->getVkCommandBuffer());
    outputImageVulkan->getImage()->copyFromBuffer(outputImageBufferVk, renderer->getVkCommandBuffer());
}

void OptixVptDenoiser::runOptixDenoiser() {
    sgl::vk::Swapchain* swapchain = sgl::AppSettings::get()->getSwapchain();
    uint32_t frameIndex = swapchain ? swapchain->getImageIndex() : 0;
    sgl::vk::SemaphoreVkCudaDriverApiInteropPtr renderFinishedSemaphore = renderFinishedSemaphores.at(frameIndex);
#ifdef USE_TIMELINE_SEMAPHORES
    renderFinishedSemaphore->waitSemaphoreCuda(stream, timelineValue);
#else
    renderFinishedSemaphore->waitSemaphoreCuda(stream);
#endif

    OptixDenoiserParams params{};
#if OPTIX_VERSION >= 70500
    params.denoiseAlpha = denoiseAlpha ? OPTIX_DENOISER_ALPHA_MODE_COPY : OPTIX_DENOISER_ALPHA_MODE_ALPHA_AS_AOV;
#else
    params.denoiseAlpha = denoiseAlpha;
#endif
    params.blendFactor = 0.0f;

    if (denoiserModelKind != OPTIX_DENOISER_MODEL_KIND_LDR) {
        // Use optixDenoiserComputeIntensity for HDR images.
        OptixResult result = optixDenoiserComputeIntensity(
                denoiser, stream, &inputImageOptix, imageIntensity,
                scratch, scratchSizeInBytes);
        checkOptixResult(result, "Error in optixDenoiserComputeIntensity: ");
        params.hdrIntensity = imageIntensity;
    }

    OptixDenoiserLayer denoiserLayer{};
    denoiserLayer.input = inputImageOptix;
    denoiserLayer.output = outputImageOptix;
    OptixDenoiserGuideLayer guideLayer{};

    if (useAlbedo && albedoImageVulkan) {
        guideLayer.albedo = albedoImageOptix;
    }

    if (useNormalMap && normalImageVulkan) {
        guideLayer.normal = normalImageOptix;
    }

    if (denoiserModelKind == OPTIX_DENOISER_MODEL_KIND_TEMPORAL) {
        if (isFirstFrame) {
            denoiserLayer.previousOutput = inputImageOptix;
            guideLayer.flow = flowImageOptix; // All flow vectors set to zero.
            sgl::vk::g_cudaDeviceApiFunctionTable.cuMemsetD8Async(
                    flowImageBufferCu->getCudaDevicePtr(), 0, sizeof(float) * 2 * inputWidth * inputHeight, stream);
        } else {
            /**
             * https://raytracing-docs.nvidia.com/optix7/guide/index.html#ai_denoiser#structure-and-use-of-image-buffers
             * "previousOutput is read in optixDenoiserInvoke before writing a new output, so previousOutput could be
             * set to output (the same buffer) for efficiency if useful in the application."
             */
            denoiserLayer.previousOutput = outputImageOptix;
            guideLayer.flow = flowImageOptix;
        }
#if OPTIX_VERSION >= 70500
        params.temporalModeUsePreviousLayers = isFirstFrame ? 0 : 1;
#endif
    }
    isFirstFrame = false;

    OptixResult result = optixDenoiserInvoke(
            denoiser, stream, &params, denoiserState, denoiserStateSizeInBytes,
            &guideLayer, &denoiserLayer, 1, 0, 0,
            scratch, scratchSizeInBytes);
    checkOptixResult(result, "Error in optixDenoiserInvoke: ");

    sgl::vk::SemaphoreVkCudaDriverApiInteropPtr denoiseFinishedSemaphore = denoiseFinishedSemaphores.at(frameIndex);
#ifdef USE_TIMELINE_SEMAPHORES
    denoiseFinishedSemaphore->signalSemaphoreCuda(stream, timelineValue);
#else
    denoiseFinishedSemaphore->signalSemaphoreCuda(stream);
#endif
}

bool OptixVptDenoiser::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool reRender = false;

    if (propertyEditor.addCombo(
            "Feature Map", (int*)&denoiserModelKindIndex,
            OPTIX_DENOISER_MODEL_KIND_NAME, numDenoisersSupported)) {
        reRender = true;
        if (denoiserModelKindIndex == 0) {
            denoiserModelKind = OPTIX_DENOISER_MODEL_KIND_LDR;
        } else if (denoiserModelKindIndex == 1) {
            denoiserModelKind = OPTIX_DENOISER_MODEL_KIND_HDR;
        } else if (denoiserModelKindIndex == 2) {
            denoiserModelKind = OPTIX_DENOISER_MODEL_KIND_TEMPORAL;
        }
        recreateDenoiserNextFrame = true;
    }

    if (propertyEditor.addCheckbox("Use Albedo", &useAlbedo)) {
        reRender = true;
        recreateDenoiserNextFrame = true;
    }

    if (propertyEditor.addCheckbox("Use Normal Map", &useNormalMap)) {
        reRender = true;
        recreateDenoiserNextFrame = true;
    }

    if (reRender) {
        resetFrameNumber();
    }

    return reRender;
}



VectorBlitPass::VectorBlitPass(sgl::vk::Renderer* renderer) : ComputePass(renderer) {
}

void VectorBlitPass::setInputImage(const sgl::vk::ImageViewPtr& _inputImage) {
    inputImage = _inputImage;
    if (computeData) {
        computeData->setStaticImageView(inputImage, "inputImage");
    }
}

void VectorBlitPass::setOutputBuffer(const sgl::vk::BufferPtr& _outputBuffer) {
    outputBuffer = _outputBuffer;
    if (computeData) {
        computeData->setStaticBuffer(outputBuffer, "OutputBuffer");
    }
}

void VectorBlitPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    preprocessorDefines.insert(std::make_pair("BLOCK_SIZE", std::to_string(BLOCK_SIZE)));
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "VectorBlit.Compute" }, preprocessorDefines);
}

void VectorBlitPass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticImageView(inputImage, "inputImage");
    computeData->setStaticBuffer(outputBuffer, "OutputBuffer");
}

void VectorBlitPass::_render() {
    auto width = int(inputImage->getImage()->getImageSettings().width);
    auto height = int(inputImage->getImage()->getImageSettings().height);
    renderer->dispatch(
            computeData,
            sgl::iceil(width, BLOCK_SIZE), sgl::iceil(height, BLOCK_SIZE), 1);
}
