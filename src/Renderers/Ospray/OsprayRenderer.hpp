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

#ifndef LINEVIS_OSPRAYRENDERER_HPP
#define LINEVIS_OSPRAYRENDERER_HPP

#include <ospray/ospray.h>
#include <ospray/ospray_util.h>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <Math/Geometry/AABB3.hpp>

#include "Renderers/LineRenderer.hpp"

struct CurvesData {
    std::vector<uint32_t> curveIndices;
    std::vector<glm::vec4> vertexPositionsAndRadii;
    std::vector<float> vertexAttributes;
    std::vector<glm::vec4> vertexColors;
};

struct TriangleMesh {
    std::vector<uint32_t> triangleIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<float> vertexAttributes;
    std::vector<glm::vec4> vertexColors;
};

/**
 * A ray tracing renderer using Intel OSPRay (https://www.ospray.org/).
 */
class OsprayRenderer : public LineRenderer {
public:
    OsprayRenderer(
            SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    ~OsprayRenderer();

    static inline bool getIsOsprayInitialized() { return isOsprayInitialized; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Returns whether the triangle representation is used by the renderer.
    bool getIsTriangleRepresentationUsed() const override;

    /// Called when the resolution of the application window has changed.
    void onResolutionChanged() override;
    /// Called when the transfer function was changed.
    void onTransferFunctionMapRebuilt() override;
    // Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    bool needsReRender() override;
    // If the re-rendering was triggered from an outside source, frame accumulation cannot be used.
    void notifyReRenderTriggeredExternally() override;
    /// Called when the camera has moved.
    void onHasMoved() override;

    // Renders the object to the scene framebuffer.
    void render() override;
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    void renderGui() override;

    /// For changing performance measurement modes.
    void setNewState(const InternalState& newState) override;

protected:
    void reloadGatherShader(bool canCopyShaderAttributes = true) override;

private:
    static bool isOsprayInitialized;
    static bool denoiserAvailable;

    sgl::TexturePtr renderImage;

    // The relative change in frame variance is used for deciding whether to stop refining the current (still) frame.
    float lastFrameVariance = 1e9f;
    float frameVarianceRelativeChange = 1e9f;
    float frameVarianceRelativeChangePrev = 1e9f;
    const float FRAME_VARIANCE_RERENDER_THRESHOLD = 1e-3f;

    bool frameParameterChanged = true;
    enum class GeometryMode {
        TRIANGLE_MESH, CURVES
    };
    GeometryMode geometryMode = GeometryMode::CURVES;
    glm::vec4 backgroundColor{};
    float currentLineWidth = 0.0f;
    OSPCurveType curveType = OSP_ROUND;
    OSPCurveBasis curveBasis = OSP_LINEAR;
    void updateDenoiserMode();
    bool useDenoiser = false;
    int numAoSamples = 0;

    // OSPRay render data.
    OSPFrameBuffer ospFrameBuffer = nullptr;
    OSPWorld ospWorld = nullptr;
    OSPRenderer ospRenderer = nullptr;
    OSPCamera ospCamera = nullptr;
    OSPMaterial ospMaterial = nullptr;
    OSPLight headLight = nullptr;
    OSPData ospLights = nullptr;
    std::vector<OSPLight> lights;
    OSPGeometry ospGeometry = nullptr;

    // Utility functions for loading geometry data.
    void loadTriangleMeshData(LineDataPtr& lineData);
    void loadCurvesData(LineDataPtr& lineData);
    void finalizeLoadedData();
    void computeAttributeColors(const std::vector<float>& attributes, std::vector<glm::vec4>& colors);
    void onLineRadiusChanged();
    DataSetType currentDataSetType = DATA_SET_TYPE_FLOW_LINES;
    CurvesData curvesData;
    TriangleMesh triangleMesh;
};

#endif //LINEVIS_OSPRAYRENDERER_HPP
