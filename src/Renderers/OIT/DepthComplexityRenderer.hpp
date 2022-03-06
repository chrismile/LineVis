/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2022, Christoph Neuhauser
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

#ifndef STRESSLINEVIS_DEPTHCOMPLEXITYRENDERER_HPP
#define STRESSLINEVIS_DEPTHCOMPLEXITYRENDERER_HPP

#include "Renderers/ResolvePass.hpp"
#include "Renderers/LineRenderer.hpp"

class AutoPerfMeasurer;

/**
 * Renders the line data and determines the depth complexity in each pixel.
 */
class DepthComplexityRenderer : public LineRenderer {
public:
    DepthComplexityRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    void initialize() override;
    ~DepthComplexityRenderer() override = default;
    RenderingMode getRenderingMode() override { return RENDERING_MODE_DEPTH_COMPLEXITY; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Sets the shader preprocessor defines used by the renderer.
    void getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) override;
    void setGraphicsPipelineInfo(
            sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) override;
    void setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) override;
    void updateVulkanUniformBuffers() override;
    void setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) override;

    // Renders the object to the scene framebuffer.
    void render() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

    // Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    /// Called when the background clear color was changed.
    void onClearColorChanged() override;

    // Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    bool needsReRender() override;

protected:
    void reloadGatherShader() override;
    void computeStatistics(bool isReRender);
    void setUniformData();
    void clear();
    void gather();
    void resolve();

    // Render passes.
    std::shared_ptr<ResolvePass> resolveRasterPass;
    std::shared_ptr<ResolvePass> clearRasterPass;

    sgl::vk::BufferPtr fragmentCounterBuffer;
    std::vector<sgl::vk::BufferPtr> stagingBuffers;

    sgl::vk::FencePtr fence;
    sgl::vk::CommandBufferPtr commandBuffer;

    // Uniform data buffer shared by all shaders.
    struct UniformData {
        int viewportW;
        int padding0;

        // The number of fragments necessary to reach the maximal color opacity.
        uint32_t numFragmentsMaxColor;
        uint32_t padding1;

        // The color to shade the framents with.
        glm::vec4 color;
    };
    UniformData uniformData = {};
    sgl::vk::BufferPtr uniformDataBuffer;

    ImVec4 colorSelection = ImColor(0, 255, 255, 127);
    sgl::Color renderColor = sgl::Color(0, 255, 255);
    uint32_t numFragmentsMaxColor{}; // = max(16, max. depth complexity of scene)
    bool firstFrame = true;

    // User interface
    bool showWindow = true;
    uint64_t totalNumFragments = 0;
    uint64_t usedLocations = 1;
    uint64_t maxComplexity = 0;
    uint64_t bufferSize = 1;
    float intensity = 1.5f;
};

#endif //STRESSLINEVIS_DEPTHCOMPLEXITYRENDERER_HPP
