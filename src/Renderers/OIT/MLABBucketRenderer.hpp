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

#ifndef LINEVIS_MLABBUCKETRENDERER_HPP
#define LINEVIS_MLABBUCKETRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>
#include <Graphics/OpenGL/TimerGL.hpp>

#include "SyncMode.hpp"
#include "MLABRenderer.hpp"

/**
 * Renders all lines with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique Multi-Layer Alpha Blending with Depth Buckets (MLABDB)
 * is used. This is a revised version of Multi-Layer Alpha Blending (MLAB).
 *
 * For more details on MLAB see: Marco Salvi and Karthik Vaidyanathan. 2014. Multi-layer Alpha Blending. In Proceedings
 * of the 18th Meeting of the ACM SIGGRAPH Symposium on Interactive 3D Graphics and Games (San Francisco, California)
 * (I3D ’14). ACM, New York, NY, USA, 151–158. https://doi.org/10.1145/2556700.2556705
 *
 * For more details on MLABDB and a comparison of different OIT algorithms see:
 * M. Kern, C. Neuhauser, T. Maack, M. Han, W. Usher and R. Westermann, "A Comparison of Rendering Techniques for 3D
 * Line Sets with Transparency," in IEEE Transactions on Visualization and Computer Graphics, 2020.
 * doi: 10.1109/TVCG.2020.2975795
 * URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9007507&isnumber=4359476
 */
class MLABBucketRenderer : public MLABRenderer {
public:
    MLABBucketRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    virtual void initialize();
    virtual ~MLABBucketRenderer();
    RenderingMode getRenderingMode() override { return RENDERING_MODE_MLAB_BUCKETS; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    virtual void setLineData(LineDataPtr& lineData, bool isNewData);

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState);

protected:
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;
    void reallocateFragmentBuffer() override;
    void setUniformData();
    void gather();
    void computeDepthRange();

    sgl::ShaderProgramPtr minDepthPassShader;
    sgl::ShaderAttributesPtr minDepthPassShaderAttributes;
    sgl::GeometryBufferPtr minDepthBuffer;
};

#endif //LINEVIS_MLABBUCKETRENDERER_HPP
