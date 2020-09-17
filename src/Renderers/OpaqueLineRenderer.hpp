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

#include <Graphics/Shader/ShaderAttributes.hpp>
#include "LineRenderer.hpp"

class OpaqueLineRenderer : public LineRenderer {
public:
    OpaqueLineRenderer(SceneData& sceneData, TransferFunctionWindow& transferFunctionWindow);
    virtual ~OpaqueLineRenderer() {}

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

    /// Only for stress lines: Should we use the principal stress direction ID for rendering?
    virtual void setUsePrincipalStressDirectionIndex(bool usePrincipalStressDirectionIndex) override;

protected:
    void reloadGatherShader();

    /// Only for stress lines: Should we use the principal stress direction ID for rendering?
    bool usePrincipalStressDirectionIndex = false;

    sgl::ShaderProgramPtr shaderProgramGeometryShader;
    sgl::ShaderProgramPtr shaderProgramProgrammableFetch;
    sgl::ShaderAttributesPtr shaderAttributes;
    sgl::GeometryBufferPtr linePointDataSSBO; ///< For programmable fetch.

    sgl::TexturePtr msaaTexture;
    sgl::FramebufferObjectPtr msaaSceneFBO;
    sgl::TexturePtr msaaRenderTexture;
    sgl::RenderbufferObjectPtr msaaDepthRBO;

    // GUI data.
    bool showRendererWindow = true;
    bool useProgrammableFetch = true;
    bool useMultisampling = true;
    int maximumNumberOfSamples = 1;
    int numSamples = 4;
    int numSampleModes = -1;
    int sampleModeSelection = -1;
    std::vector<std::string> sampleModeNames;
};

#endif //LINEDENSITYCONTROL_OPAQUELINERENDERER_HPP
