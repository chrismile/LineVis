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

#include <Graphics/Shader/ShaderAttributes.hpp>
#include <Graphics/OpenGL/TimerGL.hpp>

#include "Renderers/PPLL.hpp"
#include "Renderers/LineRenderer.hpp"

const int MESH_MODE_DEPTH_COMPLEXITIES[2][2] = {
        80, 256, // avg and max depth complexity medium
        120, 380 // avg and max depth complexity very large
};

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
    OpacityOptimizationRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    virtual ~OpacityOptimizationRenderer() {}

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    virtual void setLineData(LineDataPtr& lineData, bool isNewMesh);

    /// Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();
    // Updates the internal logic (called once per frame).
    virtual void update(float dt);
    // Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender();
    // Called when the camera has moved.
    virtual void onHasMoved();

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState);

protected:
    void setSortingAlgorithmDefine();
    void updateLargeMeshMode();
    void reallocateFragmentBuffer();
    void setUniformData();
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;
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

    // Line data.
    std::vector<std::vector<glm::vec3>> lines;

    // When the camera has moved or parameters have been changed, we need to render some more frames due to temporal
    // smoothing if continuous rendering is disabled.
    const int NUM_SMOOTHING_FRAMES = 40;
    int smoothingFramesCounter = 0;

    // Blending weight parametrization for the line segments.
    void generateBlendingWeightParametrization(bool isNewMesh);
    void recomputeViewDependentParametrization();
    void recomputeStaticParametrization();
    bool useViewDependentParametrization = false;
    float linesLengthSum = 0.0f; ///< Length sum of all polylineLengths.
    std::vector<float> polylineLengths; ///< Lengths of all polylines.

    // Sorting algorithm for PPLL.
    SortingAlgorithmMode sortingAlgorithmMode = SORTING_ALGORITHM_MODE_PRIORITY_QUEUE;

    // Shaders.
    const uint32_t WORK_GROUP_SIZE_1D = 64; ///< For compute shaders.
    sgl::ShaderProgramPtr gatherPpllOpacitiesShader;
    sgl::ShaderProgramPtr clearPpllOpacitiesShader;
    sgl::ShaderProgramPtr resolvePpllOpacitiesShader;
    sgl::ShaderProgramPtr convertPerSegmentOpacitiesShader;
    sgl::ShaderProgramPtr smoothPerSegmentOpacitiesShader;
    sgl::ShaderProgramPtr computePerVertexOpacitiesShader;
    sgl::ShaderProgramPtr gatherPpllFinalShader;
    sgl::ShaderProgramPtr clearPpllFinalShader;
    sgl::ShaderProgramPtr resolvePpllFinalShader;

    // Render data.
    sgl::ShaderAttributesPtr gatherPpllOpacitiesRenderData;
    sgl::ShaderAttributesPtr clearPpllOpacitiesRenderData;
    sgl::ShaderAttributesPtr resolvePpllOpacitiesRenderData;
    sgl::ShaderAttributesPtr gatherPpllFinalRenderData;
    sgl::ShaderAttributesPtr clearPpllFinalRenderData;
    sgl::ShaderAttributesPtr resolvePpllFinalRenderData;

    // Geometry data.
    // Per-vertex.
    sgl::GeometryBufferPtr vertexPositionBuffer; ///< Per-vertex data.
    sgl::GeometryBufferPtr vertexAttributeBuffer; ///< Per-vertex data.
    sgl::GeometryBufferPtr vertexTangentBuffer; ///< Per-vertex data.
    sgl::GeometryBufferPtr vertexOpacityBuffer; ///< Per-vertex data.
    sgl::GeometryBufferPtr lineSegmentIdBuffer; ///< Per-vertex data.
    sgl::GeometryBufferPtr blendingWeightParametrizationBuffer; ///< Per-vertex data (see Günther et al. 2013).
    // Per-segment.
    sgl::GeometryBufferPtr segmentVisibilityBuffer; ///< Per-segment data. Uint because of GPU atomics.
    sgl::GeometryBufferPtr segmentOpacityUintBuffer; ///< Per-segment data. Uint because of GPU atomics.
    sgl::GeometryBufferPtr segmentOpacityBuffers[2]; ///< Per-segment data. Ping-pong computation for smoothing.
    int segmentOpacityBufferIdx = 0; ///< 0 or 1 depending on ping-pong rendering state.
    sgl::GeometryBufferPtr lineSegmentConnectivityBuffer; ///< Per-segment data. uvec2 values storing neighbor index.

    // Information about geometry data.
    uint32_t numPolylineSegments = 0;
    uint32_t numLineSegments = 0;
    uint32_t numLineVertices = 0;

    // Per-pixel linked list data.
    float opacityBufferScaleFactor = 0.5; ///< Use half width/height of final pass for opacity pass.
    size_t fragmentBufferSizeOpacity = 0;
    sgl::GeometryBufferPtr fragmentBufferOpacities;
    sgl::GeometryBufferPtr startOffsetBufferOpacities;
    sgl::GeometryBufferPtr atomicCounterBufferOpacities;
    size_t fragmentBufferSizeFinal = 0;
    sgl::GeometryBufferPtr fragmentBufferFinal;
    sgl::GeometryBufferPtr startOffsetBufferFinal;
    sgl::GeometryBufferPtr atomicCounterBufferFinal;

    // Viewport data.
    int viewportWidthOpacity = 0, viewportHeightOpacity = 0;
    int paddedViewportWidthOpacity = 0, paddedViewportHeightOpacity = 0;
    int viewportWidthFinal = 0, viewportHeightFinal = 0;
    int paddedViewportWidthFinal = 0, paddedViewportHeightFinal = 0;

    // Data for performance measurements. (TODO)
    int frameCounter = 0;
    std::string currentStateName;
    bool timerDataIsWritten = true;
    sgl::TimerGL* timer = nullptr;

    // Per-pixel linked list settings.
    enum LargeMeshMode {
        MESH_SIZE_MEDIUM, MESH_SIZE_LARGE
    };
    LargeMeshMode largeMeshMode = MESH_SIZE_MEDIUM;
    int expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[0][0];
    int expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[0][1];

    // Parameters for Opacity Optimization.
    float q = 2000.0f; ///< Overall opacity, q >= 0.
    float r = 20.0f; ///< Clearing of background, r >= 0.
    int s = 15; ///< Iterations for smoothing.
    float lambda = 2.0f; ///< Emphasis of important structures, lambda > 0.
    float relaxationConstant = 0.1f; ///< Relaxation constant for spatial smoothing, relaxationConstant > 0.
    /// Temporal smoothing constant, 0 <= temporalSmoothingFactor <= 1. 1 means immediate update, 0 no update at all.
    float temporalSmoothingFactor = 0.15f;

    // Multisampling (CSAA) mode.
    bool useMultisampling = false;
    int maximumNumberOfSamples = 1;
    int numSamples = 4;
    int numSampleModes = -1;
    int sampleModeSelection = -1;
    std::vector<std::string> sampleModeNames;

    // Multisampling (CSAA) data.
    sgl::FramebufferObjectPtr msaaSceneFBO;
    sgl::TexturePtr msaaRenderTexture;
    sgl::RenderbufferObjectPtr msaaDepthRBO;

    // GUI data.
    bool showRendererWindow = true;
};

#endif //STRESSLINEVIS_OPACITYOPTIMIZATIONRENDERER_HPP
