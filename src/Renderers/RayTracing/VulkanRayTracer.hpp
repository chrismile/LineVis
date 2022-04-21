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
 * A Vulkan ray tracer supporting both fully opaque and transparency rendering.
 *
 * Optionally, it also supports using multi-layer alpha tracing (MLAT) for accelerated, approximated order-independent
 * transparency (OIT) rendering. For more details on MLAT, please refer to:
 * F. Brüll and T. Grosch. Multi-Layer Alpha Tracing. In J. Krüger, M. Niessner, and J. Stückler, editors, Vision,
 * Modeling, and Visualization. The Eurographics Association, 2020.
 */
class VulkanRayTracer : public LineRenderer {
public:
    VulkanRayTracer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
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

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState) override;

    /// Returns whether the triangle representation is used by the renderer.
    [[nodiscard]] bool getIsTriangleRepresentationUsed() const override;

    /**
     * Called when whether the simulation mesh hull should be rendered might have changed.
     * Only used in the Vulkan ray tracer so far.
     */
    void setRenderSimulationMeshHull(bool shallRenderSimulationMeshHull) override;

    /// For visualizing the seeding order in an animation (called by MainApp).
    void setVisualizeSeedingProcess(bool visualizeSeeding) override;

protected:
    void reloadGatherShader() override;

private:
    // Vulkan render data.
    std::shared_ptr<RayTracingRenderPass> rayTracingRenderPass;

    // Whether to trace rays against a triangle mesh or analytic tubes using line segment AABBs.
    bool useAnalyticIntersections = false;

    /// For visualizing the seeding order in an animation.
    bool visualizeSeedingProcess = false; //< Stress lines only.

    // Whether to use multi-layer alpha tracing (MLAT).
    bool useMlat = false;
    int mlatNumNodes = 8;

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
    RayTracingRenderPass(
            SceneData* sceneData, VulkanRayTracer* vulkanRayTracer, sgl::vk::Renderer* renderer,
            sgl::CameraPtr* camera);

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage);
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
    inline void setVisualizeSeedingProcess(bool visualizeSeeding) { visualizeSeedingProcess = visualizeSeeding; }
    inline void setDepthMinMaxBuffer(const sgl::vk::BufferPtr& buffer) { depthMinMaxBuffer = buffer; }
    inline void setUseAmbientOcclusion(bool ambientOcclusionOn) { useAmbientOcclusion = ambientOcclusionOn; }
    inline void setAmbientOcclusionBaker(const AmbientOcclusionBakerPtr& baker) { ambientOcclusionBaker = baker; }
    inline void setUseAnalyticIntersections(bool analyticIntersections) { useAnalyticIntersections = analyticIntersections; }
    inline void setUseMlat(bool mlat) { useMlat = mlat; }
    inline void setMlatNumNodes(int numNodes) { mlatNumNodes = numNodes; }
    inline bool getIsAccelerationStructureEmpty() { return topLevelAS.get() == nullptr; }

private:
    void updateUseJitteredSamples();
    void loadShader() override;
    sgl::vk::RayTracingPipelinePtr createRayTracingPipeline() override;
    void createRayTracingData(sgl::vk::Renderer* renderer, sgl::vk::RayTracingPipelinePtr& rayTracingPipeline) override;
    void _render() override;

    SceneData* sceneData = nullptr;
    VulkanRayTracer* vulkanRayTracer = nullptr;

    sgl::CameraPtr* camera;
    LineDataPtr lineData;

    sgl::vk::ImageViewPtr sceneImageView;

    /// For visualizing the seeding order in an animation.
    bool visualizeSeedingProcess = false; //< Stress lines only.

    bool useDepthCues = false;
    sgl::vk::BufferPtr depthMinMaxBuffer;
    bool useAmbientOcclusion = false;
    AmbientOcclusionBakerPtr ambientOcclusionBaker;

    TubeTriangleRenderData tubeTriangleRenderData;
    TubeAabbRenderData tubeAabbRenderData;
    HullTriangleRenderData hullTriangleRenderData;
    sgl::vk::TopLevelAccelerationStructurePtr topLevelAS;

    // Uniform buffer object storing the line rendering settings.
    void updateLineRenderSettings();
    struct RayTracerSettings {
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

    bool useJitteredSamples = true;
    uint32_t maxNumFrames = 1;
    bool useAnalyticIntersections = false;

    // Whether to use multi-layer alpha tracing (MLAT).
    bool useMlat = false;
    int mlatNumNodes = 8;
};

#endif //LINEVIS_VULKANRAYTRACER_HPP
