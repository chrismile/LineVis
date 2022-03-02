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
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Vulkan/Utils/Interop.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/GraphicsPipeline.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/Scattering/LineDataScattering.hpp"
#include "LineDensityMapRenderer.hpp"

using namespace sgl;

LineDensityMapRenderer::LineDensityMapRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Scattered Lines Renderer", sceneData, transferFunctionWindow) {
    isRasterizer = false;
    lineDensityFieldDvrPass = std::make_shared<LineDensityFieldDvrPass>(
            this, *sceneData->renderer, &sceneData->camera);
    onClearColorChanged();
}

LineDensityMapRenderer::~LineDensityMapRenderer() = default;

void LineDensityMapRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    dirty = false;
    reRender = true;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in LineDensityMapRenderer::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");
        return;
    }

    lineDensityFieldDvrPass->setLineData(lineData, isNewData);
}

void LineDensityMapRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    lineDensityFieldDvrPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    lineDensityFieldDvrPass->recreateSwapchain(*sceneData->viewportWidth, *sceneData->viewportHeight);
}

void LineDensityMapRenderer::onClearColorChanged() {
}

void LineDensityMapRenderer::render() {
    LineRenderer::renderBase();

    if (lineData && lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        return;
    }

    lineDensityFieldDvrPass->render();
}

void LineDensityMapRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool somethingChanged = lineDensityFieldDvrPass->renderGuiPropertyEditorNodes(propertyEditor);

    if (somethingChanged) {
        reRender = true;
    }
}



LineDensityFieldDvrPass::LineDensityFieldDvrPass(
        LineDensityMapRenderer* lineRenderer, sgl::vk::Renderer* renderer, sgl::CameraPtr* camera)
        : ComputePass(renderer), lineRenderer(lineRenderer), camera(camera) {
    renderSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(RenderSettingsData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void LineDensityFieldDvrPass::setOutputImage(sgl::vk::ImageViewPtr& colorImage) {
    sceneImageView = colorImage;

    if (computeData) {
        computeData->setStaticImageView(sceneImageView, "outputImage");
    }
}

bool LineDensityFieldDvrPass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool changed = false;
    changed |= propertyEditor.addSliderFloat(
            "Attenuation Coefficient", &renderSettingsData.attenuationCoefficient, 0, 500);

    return changed;
}

void LineDensityFieldDvrPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;

    if (lineData->getType() != DATA_SET_TYPE_SCATTERING_LINES) {
        sgl::Logfile::get()->writeError(
                "Error in AabbRenderPassCompute::setLineData: Only data sets of the type "
                "DATA_SET_TYPE_SCATTERING_LINES are supported.");
        return;
    }

    lineDataScatteringRenderData = {};
    std::shared_ptr<LineDataScattering> lineDataScattering = std::static_pointer_cast<LineDataScattering>(lineData);
    lineDataScatteringRenderData = lineDataScattering->getVulkanLineDataScatteringRenderData();
    AABB3 aabb = lineDataScattering->getGridBoundingBox();

    renderSettingsData.minBoundingBox = aabb.min;
    renderSettingsData.maxBoundingBox = aabb.max;
    renderSettingsData.voxelSize = aabb.getDimensions().x / float(lineDataScattering->getGridSizeX());

    dataDirty = true;
}

void LineDensityFieldDvrPass::loadShader() {
    shaderStages = sgl::vk::ShaderManager->getShaderStages({"LineDensityFieldDvrShader.Compute"});
}

void LineDensityFieldDvrPass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticBuffer(renderSettingsBuffer, "RenderSettingsBuffer");
    computeData->setStaticImageView(sceneImageView, "outputImage");
    computeData->setStaticTexture(lineDataScatteringRenderData.lineDensityFieldTexture, "lineDensityField");

    lineData->setVulkanRenderDataDescriptors(std::static_pointer_cast<vk::RenderData>(computeData));
}

void LineDensityFieldDvrPass::_render() {
    renderSettingsBuffer->updateData(
            sizeof(RenderSettingsData), &renderSettingsData, renderer->getVkCommandBuffer());
    lineData->updateVulkanUniformBuffers(lineRenderer, renderer);
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT);

    renderer->insertImageMemoryBarrier(
            sceneImageView->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT);

    int width = int(sceneImageView->getImage()->getImageSettings().width);
    int height = int(sceneImageView->getImage()->getImageSettings().height);
    int groupCountX = sgl::iceil(width, 16);
    int groupCountY = sgl::iceil(height, 16);
    renderer->dispatch(computeData, groupCountX, groupCountY, 1);

    renderer->insertImageMemoryBarrier(
            sceneImageView->getImage(),
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);
}
