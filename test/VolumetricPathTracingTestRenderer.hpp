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

#ifndef LINEVIS_VOLUMETRICPATHTRACINGTESTRENDERER_HPP
#define LINEVIS_VOLUMETRICPATHTRACINGTESTRENDERER_HPP

#include <Graphics/Vulkan/Utils/SyncObjects.hpp>
#include <Graphics/Vulkan/Image/Image.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>

class CloudData;
typedef std::shared_ptr<CloudData> CloudDataPtr;
class VolumetricPathTracingPass;

namespace sgl {
class Camera;
typedef std::shared_ptr<Camera> CameraPtr;
}

class VolumetricPathTracingTestRenderer {
public:
    explicit VolumetricPathTracingTestRenderer(sgl::vk::Renderer* renderer);
    ~VolumetricPathTracingTestRenderer();

    /// Sets the cloud data that is rendered when calling @see renderFrame.
    void setCloudData(const CloudDataPtr& cloudData);

    /// Called when the resolution of the application window has changed.
    void setRenderingResolution(uint32_t width, uint32_t height);
    inline uint32_t getFrameWidth() { return renderImageView->getImage()->getImageSettings().width; }
    inline uint32_t getFrameHeight() { return renderImageView->getImage()->getImageSettings().height; }

    /// Sets whether linear RGB or sRGB should be used for rendering.
    void setUseLinearRGB(bool useLinearRGB);

    /// Sets the volumetric path tracing mode used for rendering.
    void setVptModeFromString(const std::string& vptModeName);

    /**
     * Renders the path traced volume object to the scene framebuffer.
     * @param numFrames The number of frames to accumulate.
     * @return An floating point array of size width * height * 3 containing the frame data.
     * NOTE: The returned data is managed by this class.
     */
    float* renderFrame(int numFrames);

private:
    sgl::CameraPtr camera;
    sgl::vk::Renderer* renderer = nullptr;
    std::shared_ptr<VolumetricPathTracingPass> vptPass;

    sgl::vk::ImageViewPtr renderImageView;
    sgl::vk::ImagePtr renderImageStaging;
    sgl::vk::FencePtr renderFinishedFence;
    float* imageData = nullptr;
};

#endif //LINEVIS_VOLUMETRICPATHTRACINGTESTRENDERER_HPP
