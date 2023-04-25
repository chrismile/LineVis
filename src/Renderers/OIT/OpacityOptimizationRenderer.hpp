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

#ifndef STRESSLINEVIS_OPACITYOPTIMIZATIONRENDERER_HPP
#define STRESSLINEVIS_OPACITYOPTIMIZATIONRENDERER_HPP

#include <Graphics/Vulkan/Utils/Timer.hpp>

#include "Renderers/PPLL.hpp"
#include "Renderers/ResolvePass.hpp"
#include "Renderers/LineRenderer.hpp"

const int MESH_MODE_DEPTH_COMPLEXITIES_OPOPT[2][2] = {
        {20, 100}, // avg and max depth complexity medium
        //{80, 256}, // avg and max depth complexity medium
        {120, 380} // avg and max depth complexity very large
};

class PpllOpacitiesLineRasterPass;
class PpllFinalLineRasterPass;
class ConvertPerSegmentOpacitiesPass;
class SmoothPerSegmentOpacitiesPass;
class ComputePerVertexOpacitiesPass;

/**
 * Implementation of opacity optimization as described in:
 * Tobias Günther, Holger Theisel, and Markus Gross. 2017. Decoupled Opacity Optimization for Points, Lines and
 * Surfaces. Comput. Graph. Forum 36, 2 (May 2017), 153–162. DOI:https://doi.org/10.1111/cgf.13115
 *
 * For this, the order-independent transparency (OIT) technique per-pixel linked lists is used.
 * For more details see: Yang, J. C., Hensley, J., Grün, H. and Thibieroz, N., "Real-Time Concurrent
 * Linked List Construction on the GPU", Computer Graphics Forum, 29, 2010.
 */
class OpacityOptimizationRenderer : public LineRenderer {
public:
    OpacityOptimizationRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    void initialize() override;
    ~OpacityOptimizationRenderer() override;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_OPACITY_OPTIMIZATION; }

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

    /// Renders the object to the scene framebuffer.
    void render() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;
    /// Updates the internal logic (called once per frame).
    void update(float dt) override;
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    bool needsReRender() override;
    // /Called when the camera has moved.
    void onHasMoved() override;

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState) override;

protected:
    void updateLargeMeshMode();
    void reallocateFragmentBuffer();
    void setUniformData();
    void reloadGatherShader() override;
    void reloadResolveShader();

    // Rendering steps.
    void clearPpllOpacities();
    void gatherPpllOpacities();
    void resolvePpllOpacities();
    void convertPerSegmentOpacities();
    void smoothPerSegmentOpacities();
    void computePerVertexOpacities();
    void clearPpllFinal();
    void gatherPpllFinal();
    void resolvePpllFinal();

    // Events.
    sgl::ListenerToken onOpacityEstimationRecomputeListenerToken;

    // Line data.
    std::vector<std::vector<glm::vec3>> lines;

    // When the camera has moved or parameters have been changed, we need to render some more frames due to temporal
    // smoothing if continuous rendering is disabled.
    const int NUM_SMOOTHING_FRAMES = 40;
    int smoothingFramesCounter = 0;

    // Blending weight parametrization for the line segments.
    void generateBlendingWeightParametrization(bool isNewData);
    void recomputeViewDependentParametrization();
    void recomputeStaticParametrization();
    bool useViewDependentParametrization = false;
    float linesLengthSum = 0.0f; ///< Length sum of all polylineLengths.
    std::vector<float> polylineLengths; ///< Lengths of all polylines.

    // Sorting algorithm for PPLL.
    SortingAlgorithmMode sortingAlgorithmMode = SORTING_ALGORITHM_MODE_PRIORITY_QUEUE;

    // Render passes.
    std::shared_ptr<PpllOpacitiesLineRasterPass> gatherPpllOpacitiesPass;
    std::shared_ptr<ResolvePass> clearPpllOpacitiesPass;
    std::shared_ptr<ResolvePass> resolvePpllOpacitiesPass;
    std::shared_ptr<ConvertPerSegmentOpacitiesPass> convertPerSegmentOpacitiesPass;
    std::shared_ptr<SmoothPerSegmentOpacitiesPass> smoothPerSegmentOpacitiesPasses[2];
    std::shared_ptr<ComputePerVertexOpacitiesPass> computePerVertexOpacitiesPass;
    std::shared_ptr<PpllFinalLineRasterPass> gatherPpllFinalPass;
    std::shared_ptr<ResolvePass> clearPpllFinalPass;
    std::shared_ptr<ResolvePass> resolvePpllFinalPass;

    // Shaders.
    const uint32_t WORK_GROUP_SIZE_1D = 64; ///< For compute shaders.

    // Geometry data.
    // Per-vertex.
    //sgl::vk::BufferPtr vertexPositionBuffer; ///< Per-vertex data.
    //sgl::vk::BufferPtr vertexAttributeBuffer; ///< Per-vertex data.
    //sgl::vk::BufferPtr vertexTangentBuffer; ///< Per-vertex data.
    sgl::vk::BufferPtr vertexOpacityBuffer; ///< Per-vertex data.
    sgl::vk::BufferPtr lineSegmentIdBuffer; ///< Per-vertex data.
    sgl::vk::BufferPtr blendingWeightParametrizationBuffer; ///< Per-vertex data (see Günther et al. 2013).
    // Per-segment.
    sgl::vk::BufferPtr segmentVisibilityBuffer; ///< Per-segment data. Uint because of GPU atomics.
    sgl::vk::BufferPtr segmentOpacityUintBuffer; ///< Per-segment data. Uint because of GPU atomics.
    sgl::vk::BufferPtr segmentOpacityBuffers[2]; ///< Per-segment data. Ping-pong computation for smoothing.
    int segmentOpacityBufferIdx = 0; ///< 0 or 1 depending on ping-pong rendering state.
    sgl::vk::BufferPtr lineSegmentConnectivityBuffer; ///< Per-segment data. uvec2 values storing neighbor index.

    // Information about geometry data.
    uint32_t numPolylineSegments = 0;
    uint32_t numLineSegments = 0;
    uint32_t numLineVertices = 0;

    // Per-pixel linked list data.
    float opacityBufferScaleFactor = 0.5f; ///< Use half width/height of final pass for opacity pass.
    size_t fragmentBufferSizeOpacity = 0;
    sgl::vk::BufferPtr fragmentBufferOpacities;
    sgl::vk::BufferPtr startOffsetBufferOpacities;
    sgl::vk::BufferPtr fragmentCounterBufferOpacities;
    size_t fragmentBufferSizeFinal = 0;
    sgl::vk::BufferPtr fragmentBufferFinal;
    sgl::vk::BufferPtr startOffsetBufferFinal;
    sgl::vk::BufferPtr fragmentCounterBufferFinal;

    // Uniform data buffer shared by all shaders.
    struct PpllUniformData {
        // Number of fragments we can store in total.
        uint32_t linkedListSize;
        // Size of the viewport in x direction (in pixels).
        int viewportW;
    };
    PpllUniformData ppllUniformDataOpacities = {};
    sgl::vk::BufferPtr ppllUniformDataBufferOpacities;
    PpllUniformData ppllUniformDataFinal = {};
    sgl::vk::BufferPtr ppllUniformDataBufferFinal;

    // Parameters for opacity optimization.
    struct OpacityOptimizationUniformData {
        float q = 2000.0f; ///< Overall opacity, q >= 0.
        float r = 20.0f; ///< Clearing of background, r >= 0.
        int s = 15; ///< Iterations for smoothing.
        float lambda = 2.0f; ///< Emphasis of important structures, lambda > 0.
        float relaxationConstant = 0.1f; ///< Relaxation constant for spatial smoothing, relaxationConstant > 0.
        /// Temporal smoothing constant, 0 <= temporalSmoothingFactor <= 1. 1 means immediate update, 0 no update at all.
        float temporalSmoothingFactor = 0.15f;
        float opacityOptimizationUniformDataPadding0 = 0;
        float opacityOptimizationUniformDataPadding1 = 0;
    };
    OpacityOptimizationUniformData opacityOptimizationUniformData = {};
    sgl::vk::BufferPtr opacityOptimizationUniformDataBuffer;

    struct AttributeRangeUniformData {
        float minAttrValue = 0.0f;
        float maxAttrValue = 1.0f;
        float padding0{}, padding1{};
    };
    AttributeRangeUniformData attributeRangeUniformData = {};
    sgl::vk::BufferPtr attributeRangeUniformDataBuffer;

    // Viewport data.
    int viewportWidthOpacity = 0, viewportHeightOpacity = 0;
    int paddedViewportWidthOpacity = 0, paddedViewportHeightOpacity = 0;
    int viewportWidthFinal = 0, viewportHeightFinal = 0;
    int paddedViewportWidthFinal = 0, paddedViewportHeightFinal = 0;
    bool isOpacitiesStep = true;
    size_t maxStorageBufferSize = 0;

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
    int expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_OPOPT[0][0];
    int expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_OPOPT[0][1];

    // Multisampling (CSAA) mode.
    bool useMultisampling = false;
    int maximumNumberOfSamples = 1;
    int numSamples = 4;
    bool supportsSampleShadingRate = true;
    bool useSamplingShading = false;
    float minSampleShading = 1.0f;
    int numSampleModes = -1;
    int sampleModeSelection = -1;
    std::vector<std::string> sampleModeNames;

    // Multisampling (CSAA) data.
    sgl::vk::ImageViewPtr colorRenderTargetImageOpacities;
    sgl::vk::ImageViewPtr colorRenderTargetImageFinal;

    // GUI data.
    bool showRendererWindow = true;
};

struct OpacityOptimizationLineDataBuffers {
    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexAttributeBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;
    sgl::vk::BufferPtr vertexTangentBuffer;
    sgl::vk::BufferPtr vertexOffsetLeftBuffer;
    sgl::vk::BufferPtr vertexOffsetRightBuffer;
    sgl::vk::BufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::vk::BufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.
    sgl::vk::BufferPtr lineSegmentIdBuffer;
    sgl::vk::BufferPtr vertexOpacityBuffer;
};

class PpllOpacitiesLineRasterPass : public LineRasterPass {
public:
    explicit PpllOpacitiesLineRasterPass(LineRenderer* lineRenderer);
    void setOpacityOptimizationLineDataBuffers(const OpacityOptimizationLineDataBuffers& dataBuffers);

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;

    OpacityOptimizationLineDataBuffers buffers;
};

class PpllFinalLineRasterPass : public LineRasterPass {
public:
    explicit PpllFinalLineRasterPass(LineRenderer* lineRenderer);
    void setOpacityOptimizationLineDataBuffers(const OpacityOptimizationLineDataBuffers& dataBuffers);

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;

    OpacityOptimizationLineDataBuffers buffers;
};

class ConvertPerSegmentOpacitiesPass : public sgl::vk::ComputePass {
public:
    explicit ConvertPerSegmentOpacitiesPass(LineRenderer* lineRenderer);
    inline void setNumLineSegments(uint32_t num) { numLineSegments = num; }
    void setSegmentOpacityBuffer(const sgl::vk::BufferPtr& buffer);
    void setSegmentOpacityUintBuffer(const sgl::vk::BufferPtr& buffer);

protected:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    LineRenderer* lineRenderer;
    const uint32_t WORK_GROUP_SIZE_1D = 64;
    uint32_t numLineSegments = 0;
    sgl::vk::BufferPtr segmentOpacityBuffer;
    sgl::vk::BufferPtr segmentOpacityUintBuffer;
};

class SmoothPerSegmentOpacitiesPass : public sgl::vk::ComputePass {
public:
    explicit SmoothPerSegmentOpacitiesPass(LineRenderer* lineRenderer);
    inline void setNumLineSegments(uint32_t num) { numLineSegments = num; }
    void setSegmentOpacityBufferIn(const sgl::vk::BufferPtr& buffer);
    void setSegmentOpacityBufferOut(const sgl::vk::BufferPtr& buffer);
    void setSegmentVisibilityBuffer(const sgl::vk::BufferPtr& buffer);
    void setLineSegmentConnectivityBuffer(const sgl::vk::BufferPtr& buffer);

protected:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    LineRenderer* lineRenderer;
    const uint32_t WORK_GROUP_SIZE_1D = 64;
    uint32_t numLineSegments = 0;
    sgl::vk::BufferPtr segmentOpacityBufferIn;
    sgl::vk::BufferPtr segmentOpacityBufferOut;
    sgl::vk::BufferPtr segmentVisibilityBuffer;
    sgl::vk::BufferPtr lineSegmentConnectivityBuffer;
};

class ComputePerVertexOpacitiesPass : public sgl::vk::ComputePass {
public:
    explicit ComputePerVertexOpacitiesPass(LineRenderer* lineRenderer);
    inline void setNumLineVertices(uint32_t num) { numLineVertices = num; }
    void setVertexOpacityBuffer(const sgl::vk::BufferPtr& buffer);
    void setSegmentOpacityBuffer(const sgl::vk::BufferPtr& buffer);
    void setSegmentVisibilityBuffer(const sgl::vk::BufferPtr& buffer);
    void setBlendingWeightParametrizationBuffer(const sgl::vk::BufferPtr& buffer);

protected:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    LineRenderer* lineRenderer;
    const uint32_t WORK_GROUP_SIZE_1D = 64;
    uint32_t numLineVertices = 0;
    sgl::vk::BufferPtr vertexOpacityBuffer;
    sgl::vk::BufferPtr segmentOpacityBuffer;
    sgl::vk::BufferPtr segmentVisibilityBuffer;
    sgl::vk::BufferPtr blendingWeightParametrizationBuffer;
};

#endif //STRESSLINEVIS_OPACITYOPTIMIZATIONRENDERER_HPP
