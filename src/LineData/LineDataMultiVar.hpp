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

#ifndef STRESSLINEVIS_LINEDATAMULTIVAR_HPP
#define STRESSLINEVIS_LINEDATAMULTIVAR_HPP

#include "LineDataFlow.hpp"
#include "MultiVar/BezierTrajectory.hpp"
#include "MultiVar/MultiVarWindow.hpp"

struct TubeRenderDataMultiVar {
    // IBO
    sgl::GeometryBufferPtr indexBuffer;
    // VBOs
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexMultiVariableBuffer;
    sgl::GeometryBufferPtr vertexVariableDescBuffer;
    // SSBOs
    sgl::GeometryBufferPtr variableArrayBuffer;
    sgl::GeometryBufferPtr lineDescArrayBuffer;
    sgl::GeometryBufferPtr varDescArrayBuffer;
    sgl::GeometryBufferPtr lineVarDescArrayBuffer;
    sgl::GeometryBufferPtr varSelectedArrayBuffer;
    sgl::GeometryBufferPtr varColorArrayBuffer;
};

class LineDataMultiVar : public LineDataFlow {
public:
    LineDataMultiVar(sgl::TransferFunctionWindow &transferFunctionWindow);
    ~LineDataMultiVar();
    virtual void setTrajectoryData(const Trajectories& trajectories) override;

    // --- Retrieve data for rendering. Preferred way. ---
    virtual sgl::ShaderProgramPtr reloadGatherShader() override;
    virtual sgl::ShaderAttributesPtr getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader) override;
    virtual void setUniformGatherShaderData(sgl::ShaderProgramPtr& gatherShader) override;
    virtual void setUniformGatherShaderData_AllPasses() override;
    virtual void setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader) override;

    /**
     * For selecting rendering technique (e.g., screen-oriented bands, tubes) and other line data settings.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGui(bool isRasterizer) override;
    /**
     * For rendering a separate ImGui window.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGuiWindow(bool isRasterizer) override;
    /// Certain GUI widgets might need the clear color.
    virtual void setClearColor(const sgl::Color& clearColor) override;

private:
    virtual void recomputeHistogram() override;
    virtual void recomputeColorLegend() override;
    void recomputeWidgetPositions();
    bool renderGuiTechniqueSettings();
    bool renderGuiLineRenderingSettings();

    /// Create render data.
    TubeRenderDataMultiVar getTubeRenderDataMultiVar();

    enum MultiVarRenderMode {
        MULTIVAR_RENDERMODE_ROLLS,
        MULTIVAR_RENDERMODE_TWISTED_ROLLS,
        MULTIVAR_RENDERMODE_COLOR_BANDS,
        MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS,
        MULTIVAR_RENDERMODE_CHECKERBOARD,
        MULTIVAR_RENDERMODE_FIBERS
    };

    enum MultiVarRadiusMappingMode {
        MULTIVAR_RADIUSMODE_GLOBAL,
        MULTIVAR_RADIUSMODE_LINE
    };

    ///< Use multivariate or single attribute rendering? If true, falls back to code from @see LineDataFlow.
    bool useMultiVarRendering = true;
    BezierTrajectories bezierTrajectories;
    // Rendering modes.
    MultiVarRenderMode multiVarRenderMode = MULTIVAR_RENDERMODE_ROLLS;
    MultiVarRadiusMappingMode multiVarRadiusMappingMode = MULTIVAR_RADIUSMODE_GLOBAL;

    // Line SSBO data.
    sgl::GeometryBufferPtr variableArrayBuffer;
    sgl::GeometryBufferPtr lineDescArrayBuffer;
    sgl::GeometryBufferPtr varDescArrayBuffer;
    sgl::GeometryBufferPtr lineVarDescArrayBuffer;
    sgl::GeometryBufferPtr varSelectedArrayBuffer;
    sgl::GeometryBufferPtr varColorArrayBuffer;

    // GUI window for inspecting variable distributions over lines.
    MultiVarWindow multiVarWindow;
    sgl::Color clearColor;

    // Multi-Variate settings.
    std::vector<std::uint32_t> varSelected;
    std::vector<glm::vec4> varColors;
    std::string comboValue = "";
    int32_t numVariablesSelected = 0;
    int32_t maxNumVariables = 6;
    int32_t numLineSegments = 8;
    int32_t numInstances = 12;
    int32_t rollWidth = 1;
    float separatorWidth = 0.15;
    bool mapTubeDiameter = false;
    float twistOffset = 0.1;
    bool constantTwistOffset = false;
    int32_t checkerboardWidth = 3;
    int32_t checkerboardHeight = 2;
    int32_t checkerboardIterator = 2;

    // Line settings.
    float minRadiusFactor = 0.5f;
    float fiberRadius = 0.0005f;

    // Lighting settings.
    float minColorIntensity = 0.1;
    float materialConstantAmbient = 0.1;
    float materialConstantDiffuse = 0.85;
    float materialConstantSpecular = 0.05;
    float materialConstantSpecularExp = 10;
    bool drawHalo = true;
    float haloFactor = 1.2;
};

#endif //STRESSLINEVIS_LINEDATAMULTIVAR_HPP
