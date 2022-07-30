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

#ifndef LINEVIS_VOXELRAYCASTINGRENDERER_HPP
#define LINEVIS_VOXELRAYCASTINGRENDERER_HPP

#include "Renderers/ResolvePass.hpp"
#include "Renderers/LineRenderer.hpp"
#include "VoxelCurveDiscretizer.hpp"

class LineHullRasterPass;
class VoxelRayCastingPass;

/**
 * A voxel ray caster (VRC) for line rendering based on the work by:
 *
 * M. Kanzler, M. Rautenhaus, R. Westermann. A Voxel-based Rendering Pipeline for Large 3D Line Sets.
 * IEEE Transactions on Visualization and Computer Graphics 2018.
 * https://www.in.tum.de/cg/research/publications/2018/a-voxel-based-rendering-pipeline-for-large-3d-line-sets/
 */
class VoxelRayCastingRenderer : public LineRenderer {
public:
    VoxelRayCastingRenderer(
            SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    void initialize() override;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_VOXEL_RAY_CASTING; }

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

protected:
    void reloadGatherShader() override;
    void setUniformData();

private:
    sgl::vk::TexturePtr renderTexture;

    // Rendering data for a hull enclosing the mesh.
    std::shared_ptr<LineHullRasterPass> lineHullRasterPasses[2];
    sgl::vk::TexturePtr nearestLineHullHitDepthTexture;
    sgl::vk::TexturePtr furthestLineHullHitDepthTexture;

    std::shared_ptr<ResolvePass> voxelRayCastingPass;
    VoxelCurveDiscretizer voxelCurveDiscretizer;
    glm::mat4 worldToVoxelGridMatrix{}, voxelGridToWorldMatrix{};
    sgl::vk::BufferPtr voxelGridLineSegmentOffsetsBuffer;
    sgl::vk::BufferPtr voxelGridNumLineSegmentsBuffer;
    sgl::vk::BufferPtr voxelGridLineSegmentsBuffer;

    // Uniform data buffer shared by all shaders.
    struct UniformData {
        glm::vec3 cameraPositionVoxelGrid;
        float aspectRatio;
        glm::vec3 paddingUniform;
        float lineRadius; //< in voxel space.
        glm::mat4 worldSpaceToVoxelSpace;
        glm::mat4 voxelSpaceToWorldSpace;
        glm::mat4 ndcToVoxelSpace;
    };
    UniformData uniformData = {};
    sgl::vk::BufferPtr uniformDataBuffer;

    // Rendering settings.
    int gridResolution1D = 64, quantizationResolution1D = 64;
    bool gridResolutionSetManually = false;
    int maxNumLinesPerVoxel = 32;
    int maxNumHits = 8;
    bool useGpuForVoxelization = true;
    glm::ivec3 gridResolution{};
    glm::uvec3 quantizationResolution{};
    bool computeNearestFurthestHitsUsingHull = true;
};

class LineHullRasterPass : public sgl::vk::RasterPass {
public:
    explicit LineHullRasterPass(LineRenderer* lineRenderer, VoxelCurveDiscretizer* voxelCurveDiscretizer);

    // Public interface.
    void setLineData(LineDataPtr& lineData, bool isNewData);
    void setDepthCompareOp(VkCompareOp compareOp);
    void setOutputDepthImageView(const sgl::vk::ImageViewPtr& imageView);
    void recreateSwapchain(uint32_t width, uint32_t height) override;

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

    LineRenderer* lineRenderer = nullptr;
    SceneData* sceneData;
    sgl::CameraPtr* camera;
    LineDataPtr lineData;
    VoxelCurveDiscretizer* voxelCurveDiscretizer = nullptr;

    // Uniform data buffer shared by all shaders.
    struct UniformData {
        glm::mat4 voxelSpaceToWorldSpace;
    };
    UniformData uniformData = {};
    sgl::vk::BufferPtr uniformDataBuffer;

    VkCompareOp depthCompareOp = VK_COMPARE_OP_LESS;
    sgl::vk::ImageViewPtr outputDepthImageView;
};

#endif //LINEVIS_VOXELRAYCASTINGRENDERER_HPP
