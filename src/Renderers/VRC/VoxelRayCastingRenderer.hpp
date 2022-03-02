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

#include "Renderers/LineRenderer.hpp"
#include "VoxelCurveDiscretizer.hpp"

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
    RenderingMode getRenderingMode() override { return RENDERING_MODE_VOXEL_RAY_CASTING; }

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

protected:
    void reloadGatherShader() override;
    void setUniformData();

private:
    sgl::TexturePtr renderTexture;

    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates)
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderProgramPtr renderShader;

    // Rendering data for a hull enclosing the mesh.
    sgl::ShaderProgramPtr lineHullShader;
    sgl::ShaderAttributesPtr lineHullRenderData;
    sgl::FramebufferObjectPtr nearestLineHullHitFbo;
    sgl::FramebufferObjectPtr furthestLineHullHitFbo;
    sgl::TexturePtr nearestLineHullHitDepthTexture;
    sgl::TexturePtr furthestLineHullHitDepthTexture;

    VoxelCurveDiscretizer voxelCurveDiscretizer;
    glm::mat4 worldToVoxelGridMatrix{}, voxelGridToWorldMatrix{};
    sgl::GeometryBufferPtr voxelGridLineSegmentOffsetsBuffer;
    sgl::GeometryBufferPtr voxelGridNumLineSegmentsBuffer;
    sgl::GeometryBufferPtr voxelGridLineSegmentsBuffer;

    // Rendering settings.
    int gridResolution1D = 64, quantizationResolution1D = 64;
    int maxNumLinesPerVoxel = 32;
    int maxNumHits = 8;
    bool useGpuForVoxelization = true;
    glm::ivec3 gridResolution{};
    glm::uvec3 quantizationResolution{};
    bool computeNearestFurthestHitsUsingHull = true;
};

#endif //LINEVIS_VOXELRAYCASTINGRENDERER_HPP
