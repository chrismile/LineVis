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

#ifndef CLOUDRENDERING_PYTORCHDENOISER_HPP
#define CLOUDRENDERING_PYTORCHDENOISER_HPP

#include <vector>
#include <unordered_map>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Passes/Pass.hpp>

#include "Denoiser.hpp"

enum class PyTorchDevice {
    CPU,
#ifdef SUPPORT_CUDA_INTEROP
    CUDA
#endif
};
const char* const PYTORCH_DEVICE_NAMES[] = {
        "CPU", "CUDA"
};

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;

namespace sgl {
namespace vk {
class BufferCudaDriverApiExternalMemoryVk;
typedef std::shared_ptr<BufferCudaDriverApiExternalMemoryVk> BufferCudaDriverApiExternalMemoryVkPtr;
class SemaphoreVkCudaDriverApiInterop;
typedef std::shared_ptr<SemaphoreVkCudaDriverApiInterop> SemaphoreVkCudaDriverApiInteropPtr;
}
}

struct ModuleWrapper;
class FeatureCombinePass;

/**
 * Loads a PyTorch denoiser model from a TorchScript intermediate representation file.
 *
 * Information on how to save a PyTorch model to a TorchScript intermediate representation file in Python:
 * Models saved using "torch.save(model.state_dict(), 'model_name.pt')" can only be read in Python. Instead, use:
 *
 * https://pytorch.org/tutorials/beginner/Intro_to_TorchScript_tutorial.html
 * example_input, example_label = next(iter(dataloader))
 * script_module = torch.jit.trace(model, example_input)  # -> torch.jit.ScriptModule
 * script_module = torch.jit.script(model)  # Alternative
 * script_module.save('model_name.pt')
 *
 * Loading in Python: script_module = torch.jit.load('model_name.pt')
 *
 * https://pytorch.org/tutorials/advanced/cpp_export.html
 * "If you need to exclude some methods in your nn.Module because they use Python features that TorchScript doesn't
 * support yet, you could annotate those with @torch.jit.ignore."
 *
 * Examples of using custom operators: https://github.com/pytorch/pytorch/tree/master/test/custom_operator
 *
 * https://pytorch.org/docs/stable/jit.html
 * Mixing tracing and scripting: @torch.jit.script - "Traced functions can call script functions."
 *
 * How to add metadata? Add _extra_files=extra_files as an argument to torch.jit.save, e.g.:
 * extra_files = { 'model_info.json': '{ "input_feature_maps": [ "color", "normal" ] }' }
 * torch.jit.save(script_module, 'model_name.pt', _extra_files=extra_files)
 */
class PyTorchDenoiser : public Denoiser {
public:
    explicit PyTorchDenoiser(sgl::vk::Renderer* renderer);
    ~PyTorchDenoiser() override;
    DenoiserType getDenoiserType() const override { return DenoiserType::PYTORCH_DENOISER; }
    [[nodiscard]] bool getIsEnabled() const override { return wrapper != nullptr; }
    void setOutputImage(sgl::vk::ImageViewPtr& outputImage) override;
    void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) override;
    [[nodiscard]] bool getUseFeatureMap(FeatureMapType featureMapType) const override;
    void setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) override;
    void resetFrameNumber() override {} // No temporal denoising supported yet, thus unused.
    void setTemporalDenoisingEnabled(bool enabled) override {} // No temporal denoising supported yet, thus unused.
    void denoise() override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;
    void setFileDialogInstance(ImGuiFileDialog* _fileDialogInstance) override { this->fileDialogInstance = _fileDialogInstance; }

    bool loadModelFromFile(const std::string& modelPath);
    void setPyTorchDevice(PyTorchDevice pyTorchDeviceNew);

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    void computeNumChannels();
    sgl::vk::Renderer* renderer = nullptr;

    PyTorchDevice pyTorchDevice = PyTorchDevice::CPU;
    std::shared_ptr<ModuleWrapper> wrapper;
    std::string modelFilePath;
    std::vector<FeatureMapType> inputFeatureMapsUsed;
    std::unordered_map<FeatureMapType, size_t> inputFeatureMapsIndexMap;
    std::vector<sgl::vk::TexturePtr> inputFeatureMaps;
    std::vector<uint32_t> inputFeatureMapsChannelOffset;
    uint32_t numChannels = 0;
    bool isFirstContiguousWarning = true;
    bool useBatchDimension = false; ///< Does the model use 3 or 4 input dimensions?
    ImGuiFileDialog* fileDialogInstance = nullptr;
    std::string fileDialogDirectory;

    // Image data.
    sgl::vk::ImageViewPtr inputImageVulkan, outputImageVulkan;

    // Data for CPU rendering.
    std::vector<sgl::vk::BufferPtr> renderImageStagingBuffers;
    sgl::vk::BufferPtr denoisedImageStagingBuffer;
    sgl::vk::FencePtr renderFinishedFence;
    float* renderedImageData = nullptr;
    float* denoisedImageData = nullptr;

#ifdef SUPPORT_CUDA_INTEROP
    // Synchronization primitives.
    sgl::vk::BufferPtr inputImageBufferVk, outputImageBufferVk;
    sgl::vk::BufferCudaDriverApiExternalMemoryVkPtr inputImageBufferCu, outputImageBufferCu;
    std::vector<sgl::vk::CommandBufferPtr> postRenderCommandBuffers;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> renderFinishedSemaphores;
    std::vector<sgl::vk::SemaphoreVkCudaDriverApiInteropPtr> denoiseFinishedSemaphores;
    uint64_t timelineValue = 0;
#endif
    std::shared_ptr<FeatureCombinePass> featureCombinePass;
};

class FeatureCombinePass : public sgl::vk::ComputePass {
public:
    explicit FeatureCombinePass(sgl::vk::Renderer* renderer);
    void setUsedInputFeatureMaps(
            const std::vector<FeatureMapType>& inputFeatureMapsUsed,
            const std::unordered_map<FeatureMapType, size_t>& inputFeatureMapsIndexMap,
            const std::vector<uint32_t>& inputFeatureMapsChannelOffset,
            uint32_t numChannels);
    void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture);
    void setOutputBuffer(const sgl::vk::BufferPtr& _outputBuffer);

protected:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

private:
    uint32_t getFeatureMapWriteOffset(FeatureMapType featureMapType);
    bool getUseFeatureMap(FeatureMapType featureMapType);
    static std::string getFeatureMapImageName(FeatureMapType featureMapType);

    const int computeBlockSize = 16;
    struct UniformData {
        uint32_t numChannelsOut = 0;
        uint32_t colorWriteStartOffset = 0;
        uint32_t albedoWriteStartOffset = 0;
        uint32_t normalWriteStartOffset = 0;
        uint32_t depthWriteStartOffset = 0;
        uint32_t positionWriteStartOffset = 0;
        uint32_t flowWriteStartOffset = 0;
        uint32_t padding = 0;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformBuffer;

    std::vector<FeatureMapType> inputFeatureMapsUsed;
    std::unordered_map<FeatureMapType, size_t> inputFeatureMapsIndexMap;
    std::vector<sgl::vk::TexturePtr> inputFeatureMaps;
    std::vector<uint32_t> inputFeatureMapsChannelOffset;
    uint32_t numChannels = 0;
    sgl::vk::BufferPtr outputBuffer;
};


#endif //CLOUDRENDERING_PYTORCHDENOISER_HPP
