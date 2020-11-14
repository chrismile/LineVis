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

#ifndef RENDERERS_HEXAHEDRALMESHRENDERER_HPP
#define RENDERERS_HEXAHEDRALMESHRENDERER_HPP

#include "LineData/LineData.hpp"
#include "SceneData.hpp"

namespace sgl {
    class TransferFunctionWindow;
}
struct InternalState;
class SettingsMap;

const float STANDARD_LINE_WIDTH = 0.002f;

/**
 * Super-class for line renderers.
 */
class LineRenderer {
public:
    LineRenderer(SceneData &sceneData, sgl::TransferFunctionWindow &transferFunctionWindow)
        : sceneData(sceneData), transferFunctionWindow(transferFunctionWindow) {}
    virtual ~LineRenderer() {}

    // Returns if the visualization mapping needs to be re-generated.
    inline bool isDirty() { return dirty; }
    // Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { bool tmp = reRender; reRender = false; return tmp; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    virtual void setLineData(LineDataPtr& lineData, bool isNewMesh) =0;

    // Renders the object to the scene framebuffer.
    virtual void render()=0;
    // Renders the GUI. The "dirty" and "reRender" flags might be set depending on the user's actions.
    virtual void renderGui()=0;
    // Updates the internal logic (called once per frame).
    virtual void update(float dt) {}

    // Called when the resolution of the application window has changed.
    virtual void onResolutionChanged() {}

    // Called when the transfer function was changed.
    virtual void onTransferFunctionMapRebuilt() {}

    // Called when the camera has moved.
    virtual void onHasMoved() {}

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState) {}
    virtual void setNewSettings(const SettingsMap& settings) {}

    /// Only for stress lines: Should we use the principal stress direction ID for rendering?
    virtual void setUsePrincipalStressDirectionIndex(bool usePrincipalStressDirectionIndex) {}

    /// Sets the global line width.
    static void setLineWidth(float lineWidth) { LineRenderer::lineWidth = lineWidth; }

protected:
    SceneData& sceneData;
    sgl::TransferFunctionWindow& transferFunctionWindow;
    bool dirty = true;
    bool reRender = true;

    // Minimum and maximum values in the UI.
    const float MIN_LINE_WIDTH = 0.001f;
    const float MAX_LINE_WIDTH = 0.005f;
    static float lineWidth;
};

#endif // RENDERERS_HEXAHEDRALMESHRENDERER_HPP