/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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

#ifndef LINEDENSITYCONTROL_OPAQUELINERENDERER_HPP
#define LINEDENSITYCONTROL_OPAQUELINERENDERER_HPP

#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include "LineRasterPass.hpp"
#include "LineRenderer.hpp"

class BilldboardSpheresRasterPass;
class SphereRasterPass;

class OpaqueLineRenderer : public LineRenderer {
public:
    OpaqueLineRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    ~OpaqueLineRenderer() override = default;
    RenderingMode getRenderingMode() override { return RENDERING_MODE_ALL_LINES_OPAQUE; }
    bool getIsTransparencyUsed() override { return false; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Sets the shader preprocessor defines used by the renderer.
    void getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) override;
    void setGraphicsPipelineInfo(
            sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) override;
    void setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) override;

    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;

    /// Called when the background clear color was changed.
    void onClearColorChanged() override;

    // Renders the object to the scene framebuffer.
    void render() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState) override;

    /// For visualizing the seeding order in an animation (called by MainApp).
    void setVisualizeSeedingProcess(bool visualizeSeeding) override;

protected:
    void renderSphere(const glm::vec3& position, float radius, const sgl::Color& color);
    void reloadGatherShader() override;

    // Vulkan render data.
    sgl::vk::ImageViewPtr colorRenderTargetImage;
    sgl::vk::ImageViewPtr depthRenderTargetImage;

    /// For rendering degenerate points (as points extruded to billboard spheres).
    std::shared_ptr<BilldboardSpheresRasterPass> degeneratePointsRasterPass;
    /// For rendering the current seed point.
    std::shared_ptr<SphereRasterPass> sphereRasterPass;

    // GUI data.
    bool useMultisampling = true;

    bool supportsGeometryShaders = false;
    bool showDegeneratePoints = false; ///< Stress lines only.
    bool hasDegeneratePoints = false; ///< Stress lines only.
    bool visualizeSeedingProcess = false; ///< Stress lines only.
    float pointWidth = STANDARD_LINE_WIDTH;

    int maximumNumberOfSamples = 1;
    int numSamples = 4;
    bool supportsSampleShadingRate = true;
    bool useSamplingShading = false;
    float minSampleShading = 1.0f;
    int numSampleModes = -1;
    int sampleModeSelection = -1;
    std::vector<std::string> sampleModeNames;
};

class BilldboardSpheresRasterPass : public sgl::vk::RasterPass {
public:
    explicit BilldboardSpheresRasterPass(LineRenderer* lineRenderer);

    // Public interface.
    void setLineData(LineDataPtr& lineData, bool isNewData);
    inline void setPointWidth(float pointWidth) { uniformData.pointWidth = pointWidth; }
    inline void setPointColor(const glm::vec4& color) { uniformData.pointColor = color; }

    void setAttachmentLoadOp(VkAttachmentLoadOp loadOp);
    void recreateSwapchain(uint32_t width, uint32_t height) override;

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

private:
    LineRenderer* lineRenderer;
    SceneData* sceneData;
    sgl::CameraPtr* camera;
    LineDataPtr lineData;
    VkAttachmentLoadOp attachmentLoadOp = VK_ATTACHMENT_LOAD_OP_LOAD;

    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexBuffer;

    struct UniformData {
        glm::vec3 cameraPosition;
        float pointWidth = STANDARD_LINE_WIDTH;
        glm::vec4 pointColor;
        glm::vec3 foregroundColor;
        float padding;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformDataBuffer;
};

class SphereRasterPass : public sgl::vk::RasterPass {
public:
    explicit SphereRasterPass(LineRenderer* lineRenderer);

    // Public interface.
    inline void setSpherePosition(const glm::vec3& position) { uniformData.spherePosition = position; }
    inline void setSphereRadius(float pointWidth) { uniformData.sphereRadius = pointWidth; }
    inline void setSphereColor(const glm::vec4& color) { uniformData.sphereColor = color; }

    void setAttachmentLoadOp(VkAttachmentLoadOp loadOp);
    void recreateSwapchain(uint32_t width, uint32_t height) override;

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

private:
    LineRenderer* lineRenderer;
    SceneData* sceneData;
    sgl::CameraPtr* camera;
    LineDataPtr lineData;
    VkAttachmentLoadOp attachmentLoadOp = VK_ATTACHMENT_LOAD_OP_LOAD;

    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;

    struct UniformData {
        glm::vec3 cameraPosition;
        float padding0;
        glm::vec3 spherePosition;
        float sphereRadius;
        glm::vec4 sphereColor;
        glm::vec3 backgroundColor;
        float padding1;
        glm::vec3 foregroundColor;
        float padding2;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformDataBuffer;
};

#endif //LINEDENSITYCONTROL_OPAQUELINERENDERER_HPP
