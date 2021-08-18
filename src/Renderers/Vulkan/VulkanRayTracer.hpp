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

#ifndef LINEVIS_VULKANRAYTRACER_HPP
#define LINEVIS_VULKANRAYTRACER_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include <Graphics/Vulkan/Render/AccelerationStructure.hpp>

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
class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;
class RasterData;
typedef std::shared_ptr<RasterData> RasterDataPtr;
class Renderer;
}}

class RayTracingRenderPass;

/**
 * A Vulkan ray tracer using the OpenGL-Vulkan interoperability support of sgl.
 * It supports both fully opaque and transparent rendering.
 */
class VulkanRayTracer : public LineRenderer {
public:
    VulkanRayTracer(
            SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk);
    ~VulkanRayTracer() override;

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    // Renders the object to the scene framebuffer.
    void render() override;
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    void renderGui() override;

private:
    // OpenGL-Vulkan interoperability data.
    sgl::vk::TexturePtr renderTextureVk;
    sgl::TexturePtr renderTextureGl;
    sgl::SemaphoreVkGlInteropPtr renderReadySemaphore, renderFinishedSemaphore;

    // OpenGL blit data (ignores model-view-projection matrix and uses normalized device coordinates).
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderProgramPtr blitShader;

    // Vulkan render data.
    sgl::vk::Renderer* rendererVk = nullptr;
    std::shared_ptr<RayTracingRenderPass> rayTracingRenderPass;
};

class RayTracingRenderPass : public sgl::vk::RayTracingPass {
public:
    explicit RayTracingRenderPass(sgl::vk::Renderer* renderer, const sgl::CameraPtr& camera);

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage);
    void setBackgroundColor(const glm::vec4& color);
    void setTubeTriangleRenderData(const VulkanTubeTriangleRenderData& triangleRenderData);

private:
    void loadShader() override;
    sgl::vk::RayTracingPipelinePtr createRayTracingPipeline() override;
    void createRayTracingData(sgl::vk::Renderer* renderer, sgl::vk::RayTracingPipelinePtr& rayTracingPipeline) override;
    void _render() override;

    sgl::CameraPtr camera;
    sgl::vk::ImageViewPtr sceneImageView;

    VulkanTubeTriangleRenderData tubeTriangleRenderData;
    sgl::vk::TopLevelAccelerationStructurePtr topLevelAS;

    // Uniform buffer object storing the camera settings.
    struct CameraSettings {
        glm::mat4 inverseViewMatrix;
        glm::mat4 inverseProjectionMatrix;
    };
    CameraSettings cameraSettings{};
    sgl::vk::BufferPtr cameraSettingsBuffer;

    // Uniform buffer object storing the line rendering settings.
    void updateLineRenderSettings();
    struct LineRenderSettings {
        glm::vec3 cameraPosition;
        uint32_t maxDepthComplexity = 1024; // TODO
        glm::vec4 backgroundColor;
    };
    LineRenderSettings lineRenderSettings{};
    sgl::vk::BufferPtr lineRenderSettingsBuffer;
};

#endif //LINEVIS_VULKANRAYTRACER_HPP
