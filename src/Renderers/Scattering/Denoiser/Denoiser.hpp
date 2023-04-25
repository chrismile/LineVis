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

#include "Utils/InternalState.hpp"

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;
namespace sgl {
class PropertyEditor;
class Camera;
typedef std::shared_ptr<Camera> CameraPtr;
}

// enum type, name, num channels, num channels padded
#define FEATURE_MAPS                                       \
    FEATURE_MAP(COLOR,          "Color",            4, 4)    \
    FEATURE_MAP(ALBEDO,         "Albedo",           4, 4)    \
    FEATURE_MAP(NORMAL,         "Normal",           3, 4)    \
    FEATURE_MAP(NORMAL_WORLD,   "Normal (World)",   3, 4)    \
    FEATURE_MAP(DEPTH,          "Depth",            1, 1)    \
    FEATURE_MAP(POSITION,       "Position",         3, 4)    \
    FEATURE_MAP(POSITION_WORLD, "Position (World)", 3, 4)    \
    FEATURE_MAP(FLOW,           "Flow",             2, 2)    \
    FEATURE_MAP(DEPTH_NABLA,    "nabla(z)",         2, 2)    \
    FEATURE_MAP(DEPTH_FWIDTH,   "fwidth(z)",        1, 1)    \

// denoiser type, name
#define STD_DENOISERS                                                   \
    DENOISER(NONE,            "None")                                   \
    DENOISER(SPATIAL_HASHING, "Spatial Hashing")                        \
    DENOISER(SVGF,            "SVGF")                                   \
    DENOISER(EAW,             "Edge-Avoiding À-Trous Wavelet Transform") \

#ifdef SUPPORT_OPTIX
#  define OPTIX_DENOISER_OPT \
    DENOISER(OPTIX, "OptiX Denoiser")
#else
#  define OPTIX_DENOISER_OPT
#endif

#ifdef SUPPORT_PYTORCH_DENOISER
#  define PYTORCH_DENOISER_OPT \
    DENOISER(PYTORCH_DENOISER, "PyTorch Denoiser Module")
#else
#  define PYTORCH_DENOISER_OPT
#endif

#define DENOISERS STD_DENOISERS OPTIX_DENOISER_OPT PYTORCH_DENOISER_OPT

enum class DenoiserType {
#define DENOISER(type, name) type,
    DENOISERS
#undef DENOISER
};
const char* const DENOISER_NAMES[] = {
#define DENOISER(type, name) name,
    DENOISERS
#undef DENOISER
};

enum class FeatureMapType {
#define FEATURE_MAP(enum_name, _1, _2, _3) enum_name,
    FEATURE_MAPS
#undef FEATURE_MAP
};

const char* const FEATURE_MAP_NAMES[] = {
#define FEATURE_MAP(_1, string_name, _2, _3) string_name,
    FEATURE_MAPS
#undef FEATURE_MAP
};

const uint32_t FEATURE_MAP_NUM_CHANNELS[] = {
#define FEATURE_MAP(_1, _2, num_channels, _3) num_channels,
    FEATURE_MAPS
#undef FEATURE_MAP
};
const uint32_t FEATURE_MAP_NUM_CHANNELS_PADDED[] = {
#define FEATURE_MAP(_1, _2, _3, num_channels_padded) num_channels_padded,
    FEATURE_MAPS
#undef FEATURE_MAP
};

class Denoiser {
public:
    virtual ~Denoiser() = default;
    virtual DenoiserType getDenoiserType() const = 0;
    [[nodiscard]] const char* getDenoiserName() const { return DENOISER_NAMES[(int)getDenoiserType()]; }
    [[nodiscard]] virtual bool getIsEnabled() const { return true; }
    virtual void setOutputImage(sgl::vk::ImageViewPtr& outputImage) = 0;
    virtual void setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) = 0;
    [[nodiscard]] virtual bool getUseFeatureMap(FeatureMapType featureMapType) const = 0;
    [[nodiscard]] virtual bool getWantsAccumulatedInput() const { return true; }
    [[nodiscard]] virtual bool getWantsGlobalFrameNumber() const { return false; }
    virtual void setUseFeatureMap(FeatureMapType featureMapType, bool useNormals) = 0;
    virtual void setTemporalDenoisingEnabled(bool enabled) = 0;
    virtual void resetFrameNumber() = 0; // For temporal denoisers to indicate reset of temporal accumulation.
    bool getWantsFrameNumberReset() { bool tmp = wantsFrameNumberReset; wantsFrameNumberReset = false; return tmp; }
    virtual void denoise() = 0;
    virtual void recreateSwapchain(uint32_t width, uint32_t height) {}
    virtual void setFileDialogInstance(ImGuiFileDialog* _fileDialogInstance) {}

    virtual bool setNewSettings(const SettingsMap& settings) { return false; }

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    virtual bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) { return false; }

    void setWantsFrameNumberReset() { wantsFrameNumberReset = true; }
protected:

private:
    bool wantsFrameNumberReset = false;
};

enum class DenoisingMode {
    PATH_TRACING, AMBIENT_OCCLUSION, VOLUMETRIC_PATH_TRACING
};

std::shared_ptr<Denoiser> createDenoiserObject(
        DenoiserType denoiserType, sgl::vk::Renderer* renderer, sgl::CameraPtr* camera, DenoisingMode mode);

#endif //LINEVIS_DENOISER_HPP
