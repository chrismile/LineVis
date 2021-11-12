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

#ifndef LINEVIS_LINEDENSITYMAPRENDERER_HPP
#define LINEVIS_LINEDENSITYMAPRENDERER_HPP

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

class LineDensityFieldDvrPass;

/**
 * A dummy renderer for testing OpenGL-Vulkan interoperability.
 */
class LineDensityMapRenderer : public LineRenderer {
public:
    LineDensityMapRenderer(
            SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk);
    ~LineDensityMapRenderer() override;
    RenderingMode getRenderingMode() override { return RENDERING_MODE_LINE_DENSITY_MAP_RENDERER; }

    /// Returns whether the triangle representation is used by the renderer.
    bool getIsTriangleRepresentationUsed() const override { return false; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    // Renders the object to the scene framebuffer.
    void render() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    // OpenGL-Vulkan interoperability data.
    sgl::vk::TexturePtr renderTextureVk;
    sgl::TexturePtr renderTextureGl;
    sgl::SemaphoreVkGlInteropPtr renderReadySemaphore, renderFinishedSemaphore;

    // Vulkan render data.
    sgl::vk::Renderer* rendererVk = nullptr;
    std::shared_ptr<LineDensityFieldDvrPass> lineDensityFieldDvrPass;
};

/**
 * Direct volume rendering (DVR) pass.
 */
class LineDensityFieldDvrPass : public sgl::vk::ComputePass {
public:
    explicit LineDensityFieldDvrPass(sgl::vk::Renderer* renderer, sgl::CameraPtr camera);

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage);
    void setBackgroundColor(const glm::vec4& color);
    void setLineData(LineDataPtr& lineData, bool isNewData);
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

protected:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override {}
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

private:
    sgl::CameraPtr camera;
    LineDataPtr lineData;
    sgl::vk::ImageViewPtr sceneImageView;
    VulkanLineDataScatteringRenderData lineDataScatteringRenderData;

    struct CameraSettings {
        glm::mat4 viewMatrix;
        glm::mat4 projectionMatrix;
        glm::mat4 inverseViewMatrix;
        glm::mat4 inverseProjectionMatrix;
    };
    CameraSettings cameraSettings{};
    sgl::vk::BufferPtr cameraSettingsBuffer;

    struct RenderSettingsData {
        glm::vec4 backgroundColor;
        glm::vec3 minBoundingBox;
        float attenuationCoefficient = 200;
        glm::vec3 maxBoundingBox;
        float voxelSize;
    };
    RenderSettingsData renderSettingsData{};
    sgl::vk::BufferPtr renderSettingsBuffer;
};

#endif //LINEVIS_LINEDENSITYMAPRENDERER_HPP
