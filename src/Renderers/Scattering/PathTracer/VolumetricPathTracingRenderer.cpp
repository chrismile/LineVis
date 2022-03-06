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

#include <Utils/File/Logfile.hpp>
#include <Graphics/Window.hpp>

#include <Graphics/Vulkan/Utils/Interop.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/GraphicsPipeline.hpp>
#include <Graphics/Vulkan/Render/CommandBuffer.hpp>

#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "VolumetricPathTracingPass.hpp"
#include "VolumetricPathTracingRenderer.hpp"

using namespace sgl;

VolumetricPathTracingRenderer::VolumetricPathTracingRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Volumetric Path Tracer", sceneData, transferFunctionWindow) {
    isRasterizer = false;
    vptPass = std::make_shared<VolumetricPathTracingPass>(*sceneData->renderer, &sceneData->camera);
}

VolumetricPathTracingRenderer::~VolumetricPathTracingRenderer() = default;

void VolumetricPathTracingRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    dirty = false;
    reRender = true;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in ScatteredLinesRenderer::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");
        return;
    }

    LineDataScatteringPtr lineDataScattering = std::static_pointer_cast<LineDataScattering>(lineData);
    vptPass->setCloudData(lineDataScattering->getCloudData(), isNewData);
}

void VolumetricPathTracingRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    vptPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    vptPass->recreateSwapchain(width, height);
}

bool VolumetricPathTracingRenderer::needsReRender() {
    bool reRenderParent = LineRenderer::needsReRender();
    return vptPass->needsReRender() || reRenderParent;
}

void VolumetricPathTracingRenderer::onHasMoved() {
    LineRenderer::onHasMoved();
    vptPass->onHasMoved();
}

void VolumetricPathTracingRenderer::notifyReRenderTriggeredExternally() {
    LineRenderer::notifyReRenderTriggeredExternally();
    vptPass->onHasMoved();
}

void VolumetricPathTracingRenderer::setUseLinearRGB(bool useLinearRGB) {
    vptPass->setUseLinearRGB(useLinearRGB);
    onResolutionChanged();
}

void VolumetricPathTracingRenderer::setFileDialogInstance(ImGuiFileDialog* fileDialogInstance) {
    vptPass->setFileDialogInstance(fileDialogInstance);
}

void VolumetricPathTracingRenderer::render() {
    LineRenderer::renderBase();

    if (lineData && lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        return;
    }

    vptPass->render();
}

void VolumetricPathTracingRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool somethingChanged = vptPass->renderGuiPropertyEditorNodes(propertyEditor);

    if (somethingChanged) {
        reRender = true;
    }
}

void VolumetricPathTracingRenderer::renderGuiOverlay() {
}
