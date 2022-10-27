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

#ifndef LINEVIS_SPHERICALHEATMAPRENDERER_HPP
#define LINEVIS_SPHERICALHEATMAPRENDERER_HPP

#include "Renderers/LineRenderer.hpp"
#include "LineData/Scattering/DtPathTrace.hpp"

namespace sgl { namespace vk {

class BlitRenderPass;
typedef std::shared_ptr<BlitRenderPass> BlitRenderPassPtr;

}}

class TexturedSphereRasterPass;

enum class SphericalMapType {
    MOLLWEIDE_KD_TREE, MOLLWEIDE, MOLLWEIDE_SPHERE
};

const char* const SPHERICAL_MAP_TYPE_NAMES[] = {
    "Mollweide (k-D Tree)", "Mollweide", "Mollweide (Sphere)"
};

class SphericalHeatMapRenderer : public LineRenderer {
public:
    SphericalHeatMapRenderer(
            SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
    ~SphericalHeatMapRenderer() override;
    [[nodiscard]] RenderingMode getRenderingMode() const override { return RENDERING_MODE_SPHERICAL_HEAT_MAP_RENDERER; }

    /// Returns whether the triangle representation is used by the renderer.
    [[nodiscard]] bool getIsTriangleRepresentationUsed() const override { return false; }

    /// Whether to use a 3D or 2D camera for the renderer.
    [[nodiscard]] bool getUseCamera3d() override { return sphericalMapType == SphericalMapType::MOLLWEIDE_SPHERE; }

    /**
     * Re-generates the visualization mapping.
     * @param lineData The render data.
     */
    void setLineData(LineDataPtr& lineData, bool isNewData) override;

    /// Set heat map data.
    void setHeatMapData(Image heat_map);

    // Renders the object to the scene framebuffer.
    void render() override;
    /// Renders the entries in the property editor.
    void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) override;

private:
    void recreateMapImage();
    void renderImage();
    void renderSphere();
    SphericalMapType sphericalMapType = SphericalMapType::MOLLWEIDE;
    Image heat_map = {};
    sgl::vk::BlitRenderPassPtr blitRenderPass;
    std::shared_ptr<TexturedSphereRasterPass> sphereRasterPass;
    sgl::vk::TexturePtr heatMapTexture;
};


class TexturedSphereRasterPass : public sgl::vk::RasterPass {
public:
    explicit TexturedSphereRasterPass(LineRenderer* lineRenderer);

    // Public interface.
    void setMollweideMapImage(const sgl::vk::TexturePtr& _texture);

    void setAttachmentLoadOp(VkAttachmentLoadOp loadOp);
    void recreateSwapchain(uint32_t width, uint32_t height) override;

protected:
    void loadShader() override;
    void setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) override;
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;

private:
    LineRenderer* lineRenderer;
    SceneData* sceneData;
    sgl::CameraPtr* camera;
    LineDataPtr lineData;
    VkAttachmentLoadOp attachmentLoadOp = VK_ATTACHMENT_LOAD_OP_LOAD;

    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexTexCoordBuffer;
    sgl::vk::TexturePtr mollweideMapImage;
};

#endif //LINEVIS_SPHERICALHEATMAPRENDERER_HPP
