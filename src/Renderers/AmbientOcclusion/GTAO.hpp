/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022-2023, Felix Brendel, Christoph Neuhauser
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

#ifndef LINEVIS_GTAO_HPP
#define LINEVIS_GTAO_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include "Renderers/SceneData.hpp"
#include "Renderers/LineRasterPass.hpp"
#include "AmbientOcclusionBaker.hpp"

static struct GTAO_PC {
    glm::vec4 projInfo;
    glm::vec4 clipInfo;
    glm::vec3 eyePos;
    float     _padding;

    glm::uint num_directions = 32;
    glm::uint num_steps      = 16;
    float     radius         = 0.15f;
} gtao_pc;

class GTAO_GBufferPass;
class GTAOPass;

class GTAO : public AmbientOcclusionBaker {
public:
    GTAO(SceneData* sceneData, sgl::vk::Renderer* renderer);

    AmbientOcclusionBakerType getType() override { return AmbientOcclusionBakerType::GTAO; }
    bool getIsStaticPrebaker() override { return false; }
    void startAmbientOcclusionBaking(LineDataPtr& lineData, bool isNewData) override;
    void updateIterative(VkPipelineStageFlags pipelineStageFlags) override;
    void updateMultiThreaded(VkPipelineStageFlags pipelineStageFlags) override {}
    bool getIsDataReady() override { return isDataReady; }
    bool getIsComputationRunning() override { return false; }
    bool getHasComputationFinished() override { return hasComputationFinished; }
    bool getHasThreadUpdate() override { return false; }

    sgl::vk::BufferPtr getAmbientOcclusionBuffer() override { return {}; }
    sgl::vk::BufferPtr getBlendingWeightsBuffer() override { return {}; }
    uint32_t getNumTubeSubdivisions() override { return 0; }
    uint32_t getNumLineVertices() override { return 0; }
    uint32_t getNumParametrizationVertices() override { return 0; }

    sgl::vk::TexturePtr getAmbientOcclusionFrameTexture() override;
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
    std::shared_ptr<GTAO_GBufferPass> gbufferPass;
    std::shared_ptr<GTAOPass> gtaoPass;
    std::shared_ptr<sgl::vk::BlitRenderPass> blurPasses[2];

    sgl::vk::TexturePtr positionTexture;
    sgl::vk::TexturePtr normalTexture;
    sgl::vk::TexturePtr aoTexture;
    sgl::vk::TexturePtr blurPassTmpTexture;

    SceneData* sceneData;
    LineDataPtr lineData;
    bool isDataReady = false;
    bool hasComputationFinished = true;
    bool hasTextureResolutionChanged = false;
    bool reRender = true;
};

class GTAO_GBufferPass : public sgl::vk::RasterPass {
public:
    GTAO_GBufferPass(SceneData* sceneData, sgl::vk::Renderer* renderer);

    // Public interface.
    void setOutputImages(sgl::vk::ImageViewPtr& _positionImage, sgl::vk::ImageViewPtr& _normalImage);
    void setLineData(LineDataPtr& data, bool isNewData);
    void recreateSwapchain(uint32_t width, uint32_t height) override;

private:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;

    SceneData* sceneData;
    LineDataPtr lineData;

    sgl::vk::ImageViewPtr positionImage;
    sgl::vk::ImageViewPtr normalImage;
};

class GTAOPass : public sgl::vk::BlitRenderPass {
    friend class GTAO;
public:
    explicit GTAOPass(SceneData* sceneData, sgl::vk::Renderer* renderer);
    void initialize();

    // Public interface.
    void setInputTextures(sgl::vk::TexturePtr& _positionTexture, sgl::vk::TexturePtr& _normalTexture);
    void setOutputImage(sgl::vk::ImageViewPtr& _aoImage) override;
    void recreateSwapchain(uint32_t width, uint32_t height) override;

protected:
    void loadShader() override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

private:
    void createKernelBuffer();

    /**
     * Generate kernel sample positions inside a unit hemisphere.
     * Basic idea for kernel generation from https://learnopengl.com/Advanced-Lighting/SSAO.
     * However, this code makes sure to discard samples outside of the unit hemisphere (in order to get a real uniform
     * distribution in the unit sphere instead of normalizing the values).
     */
    static std::vector<glm::vec4> generateSSAOKernel(int numSamples);

    /**
     * Generates a kernel of random rotation vectors in tangent space.
     */
    static std::vector<glm::vec4> generateRotationVectors(int numVectors);

    sgl::vk::TexturePtr positionTexture;
    sgl::vk::TexturePtr normalTexture;
    sgl::vk::TexturePtr depthImage;

    sgl::vk::ImageViewPtr aoImage;

    const size_t rotationVectorKernelLength = 4;
    int numSamples = 64;

    struct SettingsData {
        float radius = 0.05f;
        float bias = 0.001f;
    };
    SettingsData settingsData{};
    sgl::vk::BufferPtr settingsDataBuffer;

    // Internal data for the algorithm.
    std::vector<glm::vec4> sampleKernel;
    sgl::vk::BufferPtr sampleKernelBuffer;
    sgl::vk::TexturePtr rotationVectorTexture;
    SceneData* sceneData;
};

#endif //LINEVIS_GTAO_HPP
