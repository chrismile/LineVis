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

#ifndef LINEVIS_UPSCALER_HPP
#define LINEVIS_UPSCALER_HPP

#include <vector>
#include <memory>
#include <cstdint>
#include <glm/fwd.hpp>

#include <glm/vec2.hpp>
#include <glm/mat4x4.hpp>

#ifndef VK_NULL_HANDLE
typedef struct VkCommandBuffer_T* VkCommandBuffer;
typedef struct VkPhysicalDevice_T* VkPhysicalDevice;
#define VK_NULL_HANDLE nullptr
#endif

namespace sgl {
class PropertyEditor;
}

namespace sgl { namespace vk {
class Device;
class ImageView;
typedef std::shared_ptr<ImageView> ImageViewPtr;
}}

enum class UpscalerType {
    DLSS, XESS, INVALID
};

class Upscaler {
public:
    Upscaler() = default;
    virtual ~Upscaler() = default;
    virtual UpscalerType getUpscalerType() = 0;
    virtual bool initialize(sgl::vk::Device* _device) = 0;
    virtual void setUpscaleAlpha(bool _upscaleAlpha) = 0;
    virtual bool queryOptimalSettings(
            uint32_t displayWidth, uint32_t displayHeight,
            uint32_t& renderWidthOptimal, uint32_t& renderHeightOptimal,
            uint32_t& renderWidthMax, uint32_t& renderHeightMax,
            uint32_t& renderWidthMin, uint32_t& renderHeightMin) = 0;
    virtual void setUseAntiAliasingMode() = 0;
    virtual void setUseJitteredMotionVectors(bool _useJitteredMotionVectors) = 0;
    virtual void setJitterOffset(float _jitterOffsetX, float _jitterOffsetY) = 0;
    virtual void resetAccum() = 0;
    virtual bool renderGui(sgl::PropertyEditor& propertyEditor) = 0;
    virtual bool apply(
            const sgl::vk::ImageViewPtr& colorImageIn,
            const sgl::vk::ImageViewPtr& colorImageOut,
            const sgl::vk::ImageViewPtr& depthImage,
            const sgl::vk::ImageViewPtr& motionVectorImage,
            const sgl::vk::ImageViewPtr& exposureImage,
            const sgl::vk::ImageViewPtr& responsivePixelMaskImage,
            VkCommandBuffer commandBuffer) = 0;
    inline bool apply(
            const sgl::vk::ImageViewPtr& colorImageIn,
            const sgl::vk::ImageViewPtr& colorImageOut,
            const sgl::vk::ImageViewPtr& depthImage,
            const sgl::vk::ImageViewPtr& motionVectorImage,
            const sgl::vk::ImageViewPtr& exposureImage,
            const sgl::vk::ImageViewPtr& responsivePixelMaskImage) {
        return apply(
                colorImageIn, colorImageOut, depthImage, motionVectorImage, exposureImage, responsivePixelMaskImage,
                VK_NULL_HANDLE);
    }
    inline bool apply(
            const sgl::vk::ImageViewPtr& colorImageIn,
            const sgl::vk::ImageViewPtr& colorImageOut,
            const sgl::vk::ImageViewPtr& depthImage,
            const sgl::vk::ImageViewPtr& motionVectorImage,
            const sgl::vk::ImageViewPtr& exposureImage) {
        return apply(colorImageIn, colorImageOut, depthImage, motionVectorImage, exposureImage, {}, VK_NULL_HANDLE);
    }
};

Upscaler* createNewUpscaler(UpscalerType upscalerType);
void computeJitteredSamples(
        std::vector<glm::vec2>& jitteredSamples, int numSamplesBase,
        uint32_t renderWidth, uint32_t renderHeight,
        uint32_t displayWidth, uint32_t displayHeight);
void adaptProjectionMatrixJitterSample(
        glm::mat4& projectionMatrix,
        const glm::vec2& jitterSample,
        uint32_t renderWidth, uint32_t renderHeight);
glm::mat4 computeJitterSampleMatrix(
        const glm::vec2& jitterSample,
        uint32_t renderWidth, uint32_t renderHeight);

#endif //LINEVIS_UPSCALER_HPP
