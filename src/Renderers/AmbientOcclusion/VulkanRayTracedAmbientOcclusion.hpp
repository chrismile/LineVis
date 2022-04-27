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

#ifndef LINEVIS_VULKANRAYTRACEDAMBIENTOCCLUSION_HPP
#define LINEVIS_VULKANRAYTRACEDAMBIENTOCCLUSION_HPP

#include <Graphics/Scene/Camera.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include "Renderers/SceneData.hpp"
#include "Renderers/Scattering/Denoiser/Denoiser.hpp"
#include "Renderers/AmbientOcclusion/AmbientOcclusionBaker.hpp"

namespace sgl {

class TransferFunctionWindow;

namespace vk {

class Renderer;
class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;
class Semaphore;
typedef std::shared_ptr<Semaphore> SemaphorePtr;

}

}

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class VulkanRayTracedAmbientOcclusionPass;

class VulkanRayTracedAmbientOcclusion : public AmbientOcclusionBaker {
public:
    VulkanRayTracedAmbientOcclusion(SceneData* sceneData, sgl::vk::Renderer* renderer);
    ~VulkanRayTracedAmbientOcclusion() override;

    AmbientOcclusionBakerType getType() override { return AmbientOcclusionBakerType::VULKAN_RTAO; }
    bool getIsStaticPrebaker() override { return false; }
    void startAmbientOcclusionBaking(LineDataPtr& lineData, bool isNewData) override;
    void updateIterative(VkPipelineStageFlags pipelineStageFlags) override;
    void updateMultiThreaded(VkPipelineStageFlags pipelineStageFlags) override {}
    bool getIsDataReady() override { return isDataReady; }
    bool getIsComputationRunning() override { return accumulatedFramesCounter < maxNumAccumulatedFrames; }
    bool getHasComputationFinished() override { return hasComputationFinished; }
    bool getHasThreadUpdate() override { return false; }

    sgl::vk::BufferPtr getAmbientOcclusionBufferVulkan() override { return {}; }
    sgl::vk::BufferPtr getBlendingWeightsBufferVulkan() override { return {}; }
    uint32_t getNumTubeSubdivisions() override { return 0; }
    uint32_t getNumLineVertices() override { return 0; }
    uint32_t getNumParametrizationVertices() override { return 0; }

    sgl::vk::TexturePtr getAmbientOcclusionFrameTextureVulkan() override;
    bool getHasTextureResolutionChanged() override;

    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    bool needsReRender() override;
    /// Called when the camera has moved.
    void onHasMoved() override;
    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    /// Returns whether the baking process was re-run.
    bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    std::shared_ptr<VulkanRayTracedAmbientOcclusionPass> rtaoRenderPass;

    SceneData* sceneData;
    LineDataPtr lineData;
    bool isDataReady = false;
    bool hasComputationFinished = true;
    bool hasTextureResolutionChanged = false;

    // Multiple frames can be accumulated to achieve a multisampling effect (additionally to using numSamplesPerFrame).
    int maxNumAccumulatedFrames = 64;
    int accumulatedFramesCounter = 0;
};

class VulkanRayTracedAmbientOcclusionPass : public sgl::vk::ComputePass {
    friend class VulkanRayTracedAmbientOcclusion;
public:
    explicit VulkanRayTracedAmbientOcclusionPass(SceneData* sceneData, sgl::vk::Renderer* renderer);

    // Public interface.
    void recreateSwapchain(uint32_t width, uint32_t height) override;
    void setLineData(LineDataPtr& data, bool isNewData);
    inline void setFrameNumber(uint32_t frameNumber) { uniformData.frameNumber = frameNumber; }

    sgl::vk::TexturePtr getAmbientOcclusionTextureVk() { return resultTexture; }

    // Called when the camera has moved.
    void onHasMoved();
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { bool tmp = reRender; reRender = false; return tmp; }
    /// Renders the GUI. The "reRender" flag might be set depending on the user's actions.
    virtual bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

private:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    SceneData* sceneData;
    bool reRender = true;
    LineDataPtr lineData;

    // Resolution of the ambient occlusion data.
    uint32_t numAmbientOcclusionSamplesPerFrame = 4;
    float ambientOcclusionRadius = 0.1f;
    bool useDistance = true;

    uint32_t lastViewportWidth = 0, lastViewportHeight = 0;

    sgl::vk::TexturePtr accumulationTexture;
    sgl::vk::TexturePtr denoisedTexture;
    sgl::vk::TexturePtr resultTexture;

    bool changedDenoiserSettings = false;

    TubeTriangleRenderData tubeTriangleRenderData;
    sgl::vk::TopLevelAccelerationStructurePtr topLevelAS;
    bool useSplitBlases = false;

    sgl::vk::BlitRenderPassPtr blitResultRenderPass;

    void createDenoiser();
    void setDenoiserFeatureMaps();
    DenoiserType denoiserType = DenoiserType::NONE;
    bool useDenoiser = true;
    std::shared_ptr<Denoiser> denoiser;

    // Uniform buffer object storing the camera settings.
    struct UniformData {
        glm::mat4 inverseViewMatrix;
        glm::mat4 inverseProjectionMatrix;

        // The number of this frame (used for accumulation of samples across frames).
        uint32_t frameNumber{};
        // The number of samples accumulated in one rendering pass.
        uint32_t numSamplesPerFrame{};

        // Should the distance of the AO hits be used?
        uint32_t useDistance{};

        uint32_t padding0{};

        // What is the radius to take into account for ambient occlusion?
        float ambientOcclusionRadius{};

        // A factor which should be used for offsetting secondary rays.
        float subdivisionCorrectionFactor{};

        float padding1{}, padding2{};
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformBuffer;
};

#endif //LINEVIS_VULKANRAYTRACEDAMBIENTOCCLUSION_HPP
