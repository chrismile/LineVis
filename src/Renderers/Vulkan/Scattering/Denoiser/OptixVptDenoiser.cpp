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

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Render/CommandBuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include "OptixVptDenoiser.hpp"

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

bool OptixVptDenoiser::initGlobal() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    if (!device || device->getDeviceDriverId() != VK_DRIVER_ID_NVIDIA_PROPRIETARY) {
        sgl::Logfile::get()->writeInfo(
                "Using a Vulkan driver that is not the proprietary NVIDIA driver. Disabling OptiX support.");
        return false;
    }

    if (!sgl::vk::initializeCudaDeviceApiFunctionTable()) {
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
    return optixHandle && cuContext;
}


OptixVptDenoiser::OptixVptDenoiser(sgl::vk::Renderer* renderer) : renderer(renderer) {
    CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuStreamCreate(&stream, CU_STREAM_DEFAULT);
    sgl::vk::checkCUresult(cuResult, "Error in cuStreamCreate: ");

    createDenoiser();
}

void OptixVptDenoiser::createDenoiser() {
    if (denoiser) {
        OptixResult result = optixDenoiserDestroy(denoiser);
        checkOptixResult(result, "Error in optixDenoiserDestroy: ");
        denoiser = {};
    }

    // TODO: Support OPTIX_DENOISER_MODEL_KIND_TEMPORAL?
    OptixDenoiserOptions options;
    options.guideAlbedo = 0;
    options.guideNormal = 0;
    OptixResult result = optixDenoiserCreate(
            context, denoiserModelKind, &options, &denoiser);
    checkOptixResult(result, "Error in optixDenoiserCreate: ");
}

OptixVptDenoiser::~OptixVptDenoiser() {
    _freeBuffers();

    CUresult cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuStreamDestroy(stream);
    sgl::vk::checkCUresult(cuResult, "Error in cuStreamDestroy: ");
}

void OptixVptDenoiser::setOutputImage(sgl::vk::ImageViewPtr& outputImage) {
    outputImageVulkan = outputImage;
}

void OptixVptDenoiser::setFeatureMap(const std::string& featureMapName, const sgl::vk::TexturePtr& featureTexture) {
    if (featureMapName == "color") {
        inputImageVulkan = featureTexture->getImageView();
    } else if (featureMapName == "position") {
        // Ignore.
    } else if (featureMapName == "normal") {
        // Ignore.
    } else {
        sgl::Logfile::get()->writeInfo(
                "Warning in OptixVptDenoiser::setFeatureMap::setFeatureMap: Unknown feature map '"
                + featureMapName + "'.");
    }
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

    inputImageBufferVk = std::make_shared<sgl::vk::Buffer>(
            device, inputWidth * inputHeight * 4 * sizeof(float),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
            true, true);
    inputImageBufferCu = std::make_shared<sgl::vk::BufferCudaDriverApiExternalMemoryVk>(inputImageBufferVk);

    outputImageBufferVk = std::make_shared<sgl::vk::Buffer>(
            device, inputWidth * inputHeight * 4 * sizeof(float),
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT
            | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
            true, true);
    outputImageBufferCu = std::make_shared<sgl::vk::BufferCudaDriverApiExternalMemoryVk>(outputImageBufferVk);

    inputImageOptix = {};
    inputImageOptix.width = inputWidth;
    inputImageOptix.height = inputHeight;
    inputImageOptix.format = OPTIX_PIXEL_FORMAT_FLOAT4;
    inputImageOptix.pixelStrideInBytes = 4 * sizeof(float);
    inputImageOptix.rowStrideInBytes = inputWidth * 4 * sizeof(float);
    inputImageOptix.data = inputImageBufferCu->getCudaDevicePtr(); // TODO

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
    inputImageVulkan->getImage()->copyToBuffer(inputImageBufferVk, renderer->getVkCommandBuffer());
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
    params.denoiseAlpha = denoiseAlpha;
    params.blendFactor = 0.0f;

    if (denoiserModelKind != OPTIX_DENOISER_MODEL_KIND_LDR) {
        // Use optixDenoiserComputeIntensity for HDR images.
        OptixResult result = optixDenoiserComputeIntensity(
                denoiser, stream, &inputImageOptix, imageIntensity,
                scratch, scratchSizeInBytes);
        checkOptixResult(result, "Error in optixDenoiserComputeIntensity: ");
        params.hdrIntensity = imageIntensity;
    }

    OptixDenoiserGuideLayer guideLayer{};
    //guideLayer.albedo;
    //guideLayer.normal;
    //guideLayer.flow;

    OptixDenoiserLayer denoiserLayer{};
    denoiserLayer.input = inputImageOptix;
    denoiserLayer.output = outputImageOptix;
    //denoiserLayer.previousOutput;

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
            "Feature Map", (int*)&denoiserModelKindIndex, OPTIX_DENOISTER_MODEL_KIND_NAME,
            IM_ARRAYSIZE(OPTIX_DENOISTER_MODEL_KIND_NAME))) {
        reRender = true;
        denoiserModelKind = OptixDenoiserModelKind(denoiserModelKindIndex + int(OPTIX_DENOISER_MODEL_KIND_LDR));
        recreateDenoiserNextFrame = true;
    }

    return reRender;
}
