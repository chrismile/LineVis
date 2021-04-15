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
    OpaqueLineRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    virtual ~OpaqueLineRenderer() {}

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    virtual void setLineData(LineDataPtr& lineData, bool isNewData);

    /// Called when the resolution of the application window has changed.
    virtual void onResolutionChanged();

    // Renders the object to the scene framebuffer.
    virtual void render();
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui();

    // For use in MainApp.cpp.
    void setVisualizeSeedingProcess(bool visualizeSeedingProcess);

protected:
    void reloadSphereRenderData();
    void renderSphere(const glm::vec3& position, float radius, const sgl::Color& color);
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;

    sgl::ShaderProgramPtr gatherShader;
    sgl::ShaderAttributesPtr shaderAttributes;

    /// For rendering degenerate points (as points extruded to billboard spheres).
    sgl::ShaderProgramPtr gatherShaderPoints;
    sgl::ShaderAttributesPtr shaderAttributesDegeneratePoints; ///< Stress lines only.
    /// For rendering the current seed point.
    sgl::ShaderProgramPtr gatherShaderSphere;
    sgl::ShaderAttributesPtr shaderAttributesSphere; ///< Stress lines only.

    sgl::FramebufferObjectPtr msaaSceneFBO;
    sgl::TexturePtr msaaRenderTexture;
    sgl::RenderbufferObjectPtr msaaDepthRBO;

    // GUI data.
    bool useMultisampling = true;

    bool showDegeneratePoints = false; ///< Stress lines only.
    bool hasDegeneratePoints = false; ///< Stress lines only.
    bool visualizeSeedingProcess = false; ///< Stress lines only.
    float pointWidth = STANDARD_LINE_WIDTH;

    int maximumNumberOfSamples = 1;
    int numSamples = 4;
    int numSampleModes = -1;
    int sampleModeSelection = -1;
    std::vector<std::string> sampleModeNames;
};

#endif //LINEDENSITYCONTROL_OPAQUELINERENDERER_HPP
