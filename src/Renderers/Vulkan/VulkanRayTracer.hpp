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
    RenderingMode getRenderingMode() override { return RENDERING_MODE_VULKAN_RAY_TRACER; }

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
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;
    // Updates the internal logic (called once per frame).
    void update(float dt) override;
    // Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    bool needsReRender() override;
    // If the re-rendering was triggered from an outside source, frame accumulation cannot be used.
    void notifyReRenderTriggeredExternally() override;
    // Called when the camera has moved.
    void onHasMoved() override;

    /// Returns whether the triangle representation is used by the renderer.
    bool getIsTriangleRepresentationUsed() const override;

    /**
     * Called when whether the simulation mesh hull should be rendered might have changed.
     * Only used in the Vulkan ray tracer so far.
     */
    void setRenderSimulationMeshHull(bool shallRenderSimulationMeshHull) override;

protected:
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;

private:
    // OpenGL-Vulkan interoperability data.
    sgl::vk::TexturePtr renderTextureVk;
    sgl::TexturePtr renderTextureGl;
    sgl::SemaphoreVkGlInteropPtr renderReadySemaphore, renderFinishedSemaphore;

    // Vulkan render data.
    sgl::vk::Renderer* rendererVk = nullptr;
    std::shared_ptr<RayTracingRenderPass> rayTracingRenderPass;

    // Whether to trace rays against a triangle mesh or analytic tubes using line segment AABBs.
    bool useAnalyticIntersections = true;

    // How many rays should be used per frame?
    uint32_t numSamplesPerFrame = 2;
    // The maximum number of transparent fragments to blend before stopping early.
    uint32_t maxDepthComplexity = 1024;

    // Multiple frames can be accumulated to achieve a multisampling effect (additionally to using numSamplesPerFrame).
    uint32_t maxNumAccumulatedFrames = 32;
    uint32_t accumulatedFramesCounter = 0;
};

class RayTracingRenderPass : public sgl::vk::RayTracingPass {
public:
    explicit RayTracingRenderPass(sgl::vk::Renderer* renderer, sgl::CameraPtr camera);

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage);
    void setBackgroundColor(const glm::vec4& color);
    void setLineData(LineDataPtr& lineData, bool isNewData);
    void setRenderSimulationMeshHull(bool shallRenderSimulationMeshHull);
    inline void setNumSamplesPerFrame(uint32_t numSamples) {
        rayTracerSettings.numSamplesPerFrame = numSamples;
        updateUseJitteredSamples();
    }
    inline void setFrameNumber(uint32_t frameNumber) { rayTracerSettings.frameNumber = frameNumber; }
    inline void setMaxNumFrames(uint32_t numFrames) { maxNumFrames = numFrames; updateUseJitteredSamples(); }
    inline void setMaxDepthComplexity(uint32_t maxDepth) { rayTracerSettings.maxDepthComplexity = maxDepth; }
    inline void setUseDepthCues(bool depthCuesOn) { useDepthCues = depthCuesOn; }
    inline void setDepthMinMaxBuffer(const sgl::vk::BufferPtr& buffer) { depthMinMaxBuffer = buffer; }
    inline void setUseAmbientOcclusion(bool ambientOcclusionOn) { useAmbientOcclusion = ambientOcclusionOn; }
    inline void setAmbientOcclusionBaker(const AmbientOcclusionBakerPtr& baker) { ambientOcclusionBaker = baker; }
    inline void setUseAnalyticIntersections(bool analyticIntersections) { useAnalyticIntersections = analyticIntersections; }

private:
    void updateUseJitteredSamples();
    void loadShader() override;
    sgl::vk::RayTracingPipelinePtr createRayTracingPipeline() override;
    void createRayTracingData(sgl::vk::Renderer* renderer, sgl::vk::RayTracingPipelinePtr& rayTracingPipeline) override;
    void _render() override;

    sgl::CameraPtr camera;
    LineDataPtr lineData;
    sgl::vk::ImageViewPtr sceneImageView;

    bool useDepthCues = false;
    sgl::vk::BufferPtr depthMinMaxBuffer;
    bool useAmbientOcclusion = false;
    AmbientOcclusionBakerPtr ambientOcclusionBaker;

    VulkanTubeTriangleRenderData tubeTriangleRenderData;
    VulkanTubeAabbRenderData tubeAabbRenderData;
    VulkanHullTriangleRenderData hullTriangleRenderData;
    sgl::vk::TopLevelAccelerationStructurePtr topLevelAS;

    // Uniform buffer object storing the camera settings.
    struct CameraSettings {
        glm::mat4 viewMatrix;
        glm::mat4 projectionMatrix;
        glm::mat4 inverseViewMatrix;
        glm::mat4 inverseProjectionMatrix;
    };
    CameraSettings cameraSettings{};
    sgl::vk::BufferPtr cameraSettingsBuffer;

    // Uniform buffer object storing the line rendering settings.
    void updateLineRenderSettings();
    struct RayTracerSettings {
        glm::vec3 cameraPosition{};
        float paddingFlt = 0.0f;
        glm::vec4 backgroundColor{};
        glm::vec4 foregroundColor{};

        // The maximum number of transparent fragments to blend before stopping early.
        uint32_t maxDepthComplexity = 1024;
        // How many rays should be shot per frame?
        uint32_t numSamplesPerFrame = 1;

        // The number of this frame (used for accumulation of samples across frames).
        uint32_t frameNumber = 0;

        uint32_t paddingUint = 0;
    };
    RayTracerSettings rayTracerSettings{};
    sgl::vk::BufferPtr rayTracerSettingsBuffer;

    bool useAnalyticIntersections = false;
    bool useJitteredSamples = true;
    uint32_t maxNumFrames = 1;
};

#endif //LINEVIS_VULKANRAYTRACER_HPP
