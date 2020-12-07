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
#include "Widgets/MultiVarTransferFunctionWindow.hpp"
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
    //sgl::GeometryBufferPtr varColorArrayBuffer;
};

class LineDataMultiVar : public LineDataFlow {
public:
    LineDataMultiVar(sgl::TransferFunctionWindow &transferFunctionWindow);
    ~LineDataMultiVar();
    virtual bool settingsDiffer(LineData* other) override;
    virtual void update(float dt) override;
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
    /// Whether to use linear RGB when rendering.
    virtual void setUseLinearRGB(bool useLinearRGB) override;
    virtual bool shallRenderTransferFunctionWindow() override { return !useMultiVarRendering; }

private:
    virtual void recomputeHistogram() override;
    virtual void recomputeColorLegend() override;
    void recomputeWidgetPositions();
    bool renderGuiTechniqueSettings();
    bool renderGuiLineRenderingSettings();

    /// Create render data.
    TubeRenderDataMultiVar getTubeRenderDataMultiVar();
    BezierTrajectories bezierTrajectories;

    // GUI window for inspecting variable distributions over lines.
    MultiVarWindow multiVarWindow;
    MultiVarTransferFunctionWindow multiVarTransferFunctionWindow;
    sgl::Color clearColor;

    enum MultiVarRenderMode {
        MULTIVAR_RENDERMODE_ROLLS,
        MULTIVAR_RENDERMODE_TWISTED_ROLLS,
        MULTIVAR_RENDERMODE_COLOR_BANDS,
        MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS,
        MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS_RIBBON,
        MULTIVAR_RENDERMODE_CHECKERBOARD,
        MULTIVAR_RENDERMODE_FIBERS
    };

    enum MultiVarRadiusMappingMode {
        MULTIVAR_RADIUSMODE_GLOBAL,
        MULTIVAR_RADIUSMODE_LINE
    };

    // --- For MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS, MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS_RIBBON ---
    enum OrientedRibbonMode {
        ORIENTED_RIBBON_MODE_FIXED_BAND_WIDTH,
        ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH,
        ORIENTED_RIBBON_MODE_VARYING_BAND_RATIO,
        ORIENTED_RIBBON_MODE_VARYING_RIBBON_WIDTH
    };
    static OrientedRibbonMode orientedRibbonMode;
    static bool mapColorToSaturation; ///< !mapColorToSaturation -> DIRECT_COLOR_MAPPING in gather shader.

    ///< Use multivariate or single attribute rendering? If true, falls back to code from @see LineDataFlow.
    static bool useMultiVarRendering;

    // Rendering modes.
    static MultiVarRenderMode multiVarRenderMode;
    static MultiVarRadiusMappingMode multiVarRadiusMappingMode;

    // Line SSBO data.
    sgl::GeometryBufferPtr variableArrayBuffer;
    sgl::GeometryBufferPtr lineDescArrayBuffer;
    sgl::GeometryBufferPtr varDescArrayBuffer;
    sgl::GeometryBufferPtr lineVarDescArrayBuffer;
    sgl::GeometryBufferPtr varSelectedArrayBuffer;
    //sgl::GeometryBufferPtr varColorArrayBuffer;

    // Multi-Variate settings.
    std::vector<std::uint32_t> varSelected;
    //std::vector<glm::vec4> varColors;
    std::string comboValue = "";
    int32_t numVariablesSelected = 0;
    int32_t maxNumVariables = 6;
    static int32_t numLineSegments;
    static int32_t numInstances;
    static int32_t rollWidth;
    static float separatorWidth;
    static bool mapTubeDiameter;
    static float twistOffset;
    static bool constantTwistOffset;
    static int32_t checkerboardWidth;
    static int32_t checkerboardHeight;
    static int32_t checkerboardIterator;
    /// For orientedRibbonMode == ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH
    static glm::vec4 bandBackgroundColor ;

    // Line settings.
    static float minRadiusFactor;
    static float fiberRadius;

    // Lighting settings.
    static bool useColorIntensity;
    static float materialConstantAmbient;
    static float materialConstantDiffuse;
    static float materialConstantSpecular;
    static float materialConstantSpecularExp;
    static bool drawHalo;
    static float haloFactor;
};

#endif //STRESSLINEVIS_LINEDATAMULTIVAR_HPP
