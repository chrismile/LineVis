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

#ifndef STRESSLINEVIS_MLABRENDERER_HPP
#define STRESSLINEVIS_MLABRENDERER_HPP

#include <Graphics/Shader/ShaderAttributes.hpp>
#include <Graphics/OpenGL/TimerGL.hpp>

#include "SyncMode.hpp"
#include "Renderers/LineRenderer.hpp"

/**
 * Renders all lines with transparency values determined by the transfer function set by the user.
 * For this, the order-independent transparency (OIT) technique Multi-Layer Alpha Blending (MLAB) is used.
 * For more details see: Marco Salvi and Karthik Vaidyanathan. 2014. Multi-layer Alpha Blending. In Proceedings of the
 * 18th Meeting of the ACM SIGGRAPH Symposium on Interactive 3D Graphics and Games (San Francisco, California)
 * (I3D ’14). ACM, New York, NY, USA, 151–158. https://doi.org/10.1145/2556700.2556705
 *
 * For a comparison of different OIT algorithms see:
 * M. Kern, C. Neuhauser, T. Maack, M. Han, W. Usher and R. Westermann, "A Comparison of Rendering Techniques for 3D
 * Line Sets with Transparency," in IEEE Transactions on Visualization and Computer Graphics, 2020.
 * doi: 10.1109/TVCG.2020.2975795
 * URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9007507&isnumber=4359476
 */
class MLABRenderer : public LineRenderer {
public:
    MLABRenderer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    MLABRenderer(
            const std::string& windowName, SceneData* sceneData, sgl::TransferFunctionWindow &transferFunctionWindow);
    void initialize() override;
    ~MLABRenderer() override {}
    RenderingMode getRenderingMode() override { return RENDERING_MODE_MLAB; }

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

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState) override;

protected:
    void updateSyncMode();
    void updateLayerMode();
    virtual void reallocateFragmentBuffer();
    void setUniformData();
    void clear();
    void gather();
    void resolve();
    void reloadShaders();
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;
    void reloadResolveShader();

    // Shaders.
    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderProgramPtr clearShader;
    sgl::ShaderProgramPtr resolveShader;

    // Render data.
    sgl::ShaderAttributesPtr shaderAttributes;
    // Blit data (ignores model-view-projection matrix and uses normalized device coordinates).
    sgl::ShaderAttributesPtr blitRenderData;
    sgl::ShaderAttributesPtr clearRenderData;

    // Stored fragment data.
    sgl::GeometryBufferPtr fragmentBuffer;
    sgl::GeometryBufferPtr spinlockViewportBuffer; ///!< if (syncMode == SYNC_SPINLOCK)

    // Window data.
    int windowWidth = 0, windowHeight = 0;
    int paddedWindowWidth = 0, paddedWindowHeight = 0;
    bool clearBitSet = true;

    // Data for performance measurements.
    int frameCounter = 0;
    std::string currentStateName;
    bool timerDataIsWritten = true;
    sgl::TimerGL* timer = nullptr;

    // MLAB settings.
    static bool useStencilBuffer;
    int numLayers = 8;
    SyncMode syncMode; ///!< Initialized depending on system capabilities.
    bool useOrderedFragmentShaderInterlock = true;
};

#endif //STRESSLINEVIS_MLABRENDERER_HPP
