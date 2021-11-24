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

#ifndef LINEVIS_DATAVIEW_HPP
#define LINEVIS_DATAVIEW_HPP

#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/Buffers/RBO.hpp>

namespace sgl { namespace vk {
class Renderer;
class SemaphoreVkGlInterop;
typedef std::shared_ptr<SemaphoreVkGlInterop> SemaphoreVkGlInteropPtr;
}}

class LineRenderer;

class DataView {
public:
    DataView(SceneData* parentSceneData, sgl::ShaderProgramPtr gammaCorrectionShader);
    ~DataView();
    virtual void resize(int newWidth, int newHeight);
    virtual void beginRender();
    virtual void endRender();

    /// For data views using 2D cameras. 3D cameras are handled by SciVisApp.
    void updateCameraMode();
    void moveCamera2dKeyboard(float dt);
    void moveCamera2dMouse(float dt);

    inline sgl::TexturePtr& getSceneTextureResolved() { return useLinearRGB ? resolvedSceneTexture : sceneTexture; }
    inline sgl::FramebufferObjectPtr& getSceneFramebuffer() {
        return useLinearRGB ? resolvedSceneFramebuffer : sceneFramebuffer;
    }
    inline void setClearColor(const sgl::Color& color) { clearColor = color; }
    [[nodiscard]] inline std::string getWindowName(int index) const {
        if (lineRenderer) {
            return lineRenderer->getWindowName() + " (" + std::to_string(index + 1) + ")###data_view_"
                    + std::to_string(index);
        } else {
            return "Data View (" + std::to_string(index + 1) + ")###data_view_" + std::to_string(index);
        }
    }

    SceneData* parentSceneData = nullptr;
    bool showWindow = true;
    bool useLinearRGB = false;
    sgl::Color clearColor;
    sgl::ShaderProgramPtr gammaCorrectionShader;

    RenderingMode renderingMode = RENDERING_MODE_NONE, oldRenderingMode = RENDERING_MODE_NONE;
    LineRenderer* lineRenderer = nullptr;
    bool reRender = false;

    sgl::FramebufferObjectPtr sceneFramebuffer;
    sgl::TexturePtr sceneTexture;
    sgl::FramebufferObjectPtr resolvedSceneFramebuffer;
    sgl::TexturePtr resolvedSceneTexture;
    sgl::RenderbufferType sceneDepthRBOType = sgl::RBO_DEPTH24_STENCIL8;
    sgl::RenderbufferObjectPtr sceneDepthRBO;

    /// Scene data (e.g., camera, main framebuffer, ...).
    bool syncWithParentCamera = true;
    sgl::CameraPtr camera;
    sgl::CameraPtr camera2d;
    uint32_t viewportWidth = 0;
    uint32_t viewportHeight = 0;
    SceneData sceneData;
};
typedef std::shared_ptr<DataView> DataViewPtr;

#ifdef USE_VULKAN_INTEROP
class DataViewVulkan : public DataView {
public:
    DataViewVulkan(
            SceneData* parentSceneData, sgl::ShaderProgramPtr gammaCorrectionShader, sgl::vk::Renderer* renderer);
    void resize(int newWidth, int newHeight) override;
    void beginRender() override;
    void endRender() override;

    sgl::vk::Renderer* renderer = nullptr;
    sgl::vk::TexturePtr sceneTextureVk;
    sgl::vk::TexturePtr sceneDepthTextureVk;
    sgl::SemaphoreVkGlInteropPtr renderReadySemaphore, renderFinishedSemaphore;
};
#endif

#endif //LINEVIS_DATAVIEW_HPP
