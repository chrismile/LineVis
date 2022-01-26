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

#ifndef LINEVIS_VOLUMETRICPATHTRACINGRENDERER_HPP
#define LINEVIS_VOLUMETRICPATHTRACINGRENDERER_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "Renderers/LineRenderer.hpp"

namespace sgl {
class Texture;
typedef std::shared_ptr<Texture> TexturePtr;
class SemaphoreVkGlInterop;
typedef std::shared_ptr<SemaphoreVkGlInterop> SemaphoreVkGlInteropPtr;
}

namespace sgl { namespace vk {
class Texture;
typedef std::shared_ptr<Texture> TexturePtr;
class RasterData;
typedef std::shared_ptr<RasterData> RasterDataPtr;
class Renderer;
}}

class VolumetricPathTracingPass;

/**
 * A dummy renderer for testing OpenGL-Vulkan interoperability.
 */
class VolumetricPathTracingRenderer : public LineRenderer {
public:
    VolumetricPathTracingRenderer(
            SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk);
    ~VolumetricPathTracingRenderer() override;
    RenderingMode getRenderingMode() override { return RENDERING_MODE_VOLUMETRIC_PATH_TRACER; }

    /// Returns whether the triangle representation is used by the renderer.
    [[nodiscard]] bool getIsTriangleRepresentationUsed() const override { return false; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    bool needsReRender() override;
    /// Called when the camera has moved.
    void onHasMoved() override;
    /// If the re-rendering was triggered from an outside source, frame accumulation cannot be used.
    void notifyReRenderTriggeredExternally() override;
    /// Sets whether linear RGB or sRGB should be used for rendering.
    void setUseLinearRGB(bool useLinearRGB) override;

    // Renders the object to the scene framebuffer.
    void render() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;
    /// Renders GUI overlays. The "dirty" and "reRender" flags might be set depending on the user's actions.
    void renderGuiOverlay() override;

private:
    // OpenGL-Vulkan interoperability data.
    sgl::vk::TexturePtr renderTextureVk;
    sgl::TexturePtr renderTextureGl;
    sgl::SemaphoreVkGlInteropPtr renderReadySemaphore, renderFinishedSemaphore;

    // For fixing a bug occuring on newer NVIDIA Linux drivers (> 470.xx).
    bool isFirstFrame = true;
    bool isLinuxAndNvidia495OrNewer = false;

    // Vulkan render data.
    sgl::vk::Renderer* rendererVk = nullptr;
    std::shared_ptr<VolumetricPathTracingPass> vptPass;
};

#endif //LINEVIS_VOLUMETRICPATHTRACINGRENDERER_HPP
