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

#ifndef LINEVIS_RAYTRACERCOMPUTE_HPP
#define LINEVIS_RAYTRACERCOMPUTE_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>

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

class RayTracingComputePass;

/**
 * A test ray tracer traversing a BVH in a compute shader.
 */
class RayTracerCompute : public LineRenderer {
public:
    RayTracerCompute(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    ~RayTracerCompute() override;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_VULKAN_RAY_TRACER; }

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
    bool setNewSettings(const SettingsMap& settings) override;

    /// Returns whether the triangle representation is used by the renderer.
    [[nodiscard]] bool getIsTriangleRepresentationUsed() const override;

protected:
    void reloadGatherShader() override;

private:
    // Vulkan render data.
    std::shared_ptr<RayTracingComputePass> rayTracingComputePass;
};

struct RayTracerComputeNode {
    glm::vec3 worldSpaceAabbMin;
    uint32_t indexCount;
    glm::vec3 worldSpaceAabbMax;
    uint32_t firstChildOrPrimitiveIndex;
};

class RayTracingComputePass : public sgl::vk::ComputePass {
public:
    RayTracingComputePass(
            SceneData* sceneData, RayTracerCompute* vulkanRayTracer, sgl::vk::Renderer* renderer,
            sgl::CameraPtr* camera);

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage);
    void setLineData(LineDataPtr& lineData, bool isNewData);
    inline bool getIsAccelerationStructureEmpty() { return bvhNodes.get() == nullptr; }

private:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    SceneData* sceneData = nullptr;
    RayTracerCompute* rayTracerCompute = nullptr;

    sgl::CameraPtr* camera;
    LineDataPtr lineData;

    const uint32_t WORKGROUP_SIZE_X = 16;
    const uint32_t WORKGROUP_SIZE_Y = 16;
    sgl::vk::ImageViewPtr sceneImageView;

    TubeTriangleRenderData tubeTriangleRenderData;
    sgl::vk::BufferPtr bvhNodes;
    uint32_t bvhTreeHeight = 0;
};

#endif //LINEVIS_RAYTRACERCOMPUTE_HPP
