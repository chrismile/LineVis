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

#ifndef LINEDENSITYCONTROL_PERPIXELLINKEDLISTLINERENDERER_HPP
#define LINEDENSITYCONTROL_PERPIXELLINKEDLISTLINERENDERER_HPP

#include <Graphics/Vulkan/Utils/Timer.hpp>

#include "Renderers/PPLL.hpp"
#include "Renderers/LineRenderer.hpp"
#include "Renderers/ResolvePass.hpp"

const int MESH_MODE_DEPTH_COMPLEXITIES_PPLL[2][2] = {
        {20, 100}, // avg and max depth complexity medium
        //{80, 256}, // avg and max depth complexity medium
        {120, 380} // avg and max depth complexity very large
};

/**
 * Renders all lines with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique per-pixel linked lists is used.
 * For more details see: Yang, J. C., Hensley, J., Grün, H. and Thibieroz, N., "Real-Time Concurrent
 * Linked List Construction on the GPU", Computer Graphics Forum, 29, 2010.
 *
 * For a comparison of different OIT algorithms see:
 * M. Kern, C. Neuhauser, T. Maack, M. Han, W. Usher and R. Westermann, "A Comparison of Rendering Techniques for 3D
 * Line Sets with Transparency," in IEEE Transactions on Visualization and Computer Graphics, 2020.
 * doi: 10.1109/TVCG.2020.2975795
 * URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9007507&isnumber=4359476
 */
class PerPixelLinkedListLineRenderer : public LineRenderer {
public:
    PerPixelLinkedListLineRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    ~PerPixelLinkedListLineRenderer() override;
    RenderingMode getRenderingMode() override { return RENDERING_MODE_PER_PIXEL_LINKED_LIST; }

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
    void updateLargeMeshMode();
    void reallocateFragmentBuffer();
    void clear();
    void gather();
    void resolve();
    void reloadGatherShader() override;
    void reloadResolveShader();

    // Sorting algorithm for PPLL.
    SortingAlgorithmMode sortingAlgorithmMode = SORTING_ALGORITHM_MODE_PRIORITY_QUEUE;

    // Render passes.
    std::shared_ptr<ResolvePass> resolveRasterPass;
    std::shared_ptr<ResolvePass> clearRasterPass;

    // Per-pixel linked list data.
    size_t fragmentBufferSize = 0;
    sgl::vk::BufferPtr fragmentBuffer;
    sgl::vk::BufferPtr startOffsetBuffer;
    sgl::vk::BufferPtr fragmentCounterBuffer;

    struct UniformData {
        // Number of fragments we can store in total.
        uint linkedListSize;
        // Size of the viewport in x direction (in pixels).
        int viewportW;
    };
    UniformData uniformData = {};
    sgl::vk::BufferPtr uniformDataBuffer;

    // Window data.
    int paddedWindowWidth = 0, paddedWindowHeight = 0;

    // Data for performance measurements.
    int frameCounter = 0;
    std::string currentStateName;
    bool timerDataIsWritten = true;
    sgl::vk::TimerPtr timer;

    // Per-pixel linked list settings.
    enum LargeMeshMode {
        MESH_SIZE_MEDIUM, MESH_SIZE_LARGE
    };
    LargeMeshMode largeMeshMode = MESH_SIZE_MEDIUM;
    int expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_PPLL[0][0];
    int expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_PPLL[0][1];
};

#endif //LINEDENSITYCONTROL_PERPIXELLINKEDLISTLINERENDERER_HPP
