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

#ifndef LINEVIS_DENOISER_HPP
#define LINEVIS_DENOISER_HPP

#include <string>

#include <Graphics/Vulkan/Image/Image.hpp>

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;
namespace sgl {
class PropertyEditor;
}

enum class DenoiserType {
    NONE,
    EAW,
#ifdef SUPPORT_OPTIX
    OPTIX
#endif
};
const char* const DENOISER_NAMES[] = {
        "None",
        "Edge-Avoiding À-Trous Wavelet Transform",
#ifdef SUPPORT_OPTIX
        "OptiX Denoiser"
#endif
};

enum class FeatureMapType {
    COLOR, ALBEDO, NORMAL, DEPTH, POSITION, FLOW
};
const char* const FEATURE_MAP_NAMES[] = {
        "Color", "Albedo", "Normal", "Depth", "Position", "Flow",
};
const uint32_t FEATURE_MAP_NUM_CHANNELS[] = {
        4, 4, 3, 1, 3, 2
};
const uint32_t FEATURE_MAP_NUM_CHANNELS_PADDED[] = {
        4, 4, 4, 1, 4, 2
};

class Denoiser {
public:
    virtual ~Denoiser() = default;
    virtual DenoiserType getDenoiserType() = 0;
    [[nodiscard]] virtual const char* getDenoiserName() const = 0;
    [[nodiscard]] virtual bool getIsEnabled() const { return true; }
    virtual void setOutputImage(sgl::vk::ImageViewPtr& outputImage) = 0;
    virtual void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) = 0;
    [[nodiscard]] virtual bool getUseFeatureMap(FeatureMapType featureMapType) const = 0;
    virtual void setUseFeatureMap(FeatureMapType featureMapType, bool useNormals) = 0;
    virtual void setTemporalDenoisingEnabled(bool enabled) = 0;
    virtual void resetFrameNumber() = 0; // For temporal denoisers to indicate reset of temporal accumulation.
    virtual void denoise() = 0;
    virtual void recreateSwapchain(uint32_t width, uint32_t height) {}
    virtual void setFileDialogInstance(ImGuiFileDialog* _fileDialogInstance) {}

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    virtual bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) { return false; }
};

enum class DenoisingMode {
    PATH_TRACING, AMBIENT_OCCLUSION, VOLUMETRIC_PATH_TRACING
};

std::shared_ptr<Denoiser> createDenoiserObject(
        DenoiserType denoiserType, sgl::vk::Renderer* renderer, DenoisingMode mode);

#endif //LINEVIS_DENOISER_HPP
