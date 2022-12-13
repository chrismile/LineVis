/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#include <utility>

#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/CommandBuffer.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/RayTracingPipeline.hpp>

#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "LineData/TrianglePayload/NodesBVHTreePayload.hpp"
#include "RayTracerCompute.hpp"

using namespace sgl;

RayTracerCompute::RayTracerCompute(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Ray Tracer (Compute)", sceneData, transferFunctionWindow) {
    isRasterizer = false;

    rayTracingComputePass = std::make_shared<RayTracingComputePass>(
            sceneData, this, renderer, &sceneData->camera);

    onClearColorChanged();
}

RayTracerCompute::~RayTracerCompute() {
    sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();

    rayTracingComputePass = {};
    ambientOcclusionBaker = {};
}

void RayTracerCompute::reloadGatherShader() {
    rayTracingComputePass->setShaderDirty();
}

bool RayTracerCompute::getIsTriangleRepresentationUsed() const {
    return LineRenderer::getIsTriangleRepresentationUsed() || true; // !useAnalyticIntersections
}

void RayTracerCompute::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);
    rayTracingComputePass->setLineData(lineData, isNewData);

    dirty = false;
    reRender = true;
}

void RayTracerCompute::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    rayTracingComputePass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    rayTracingComputePass->recreateSwapchain(width, height);
}

void RayTracerCompute::render() {
    LineRenderer::renderBase(VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR);
    if (ambientOcclusionBaker
        && ((ambientOcclusionBaker->getIsStaticPrebaker() && ambientOcclusionBuffersDirty)
            || (!ambientOcclusionBaker->getIsStaticPrebaker() && ambientOcclusionTexturesDirty))) {
        rayTracingComputePass->setDataDirty();
        ambientOcclusionBuffersDirty = false;
        ambientOcclusionTexturesDirty = false;
    }

    rayTracingComputePass->buildIfNecessary();
    if (rayTracingComputePass->getIsAccelerationStructureEmpty()) {
        renderer->transitionImageLayout(
                (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        (*sceneData->sceneTexture)->getImage()->clearColor(
                sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
    } else {
        rayTracingComputePass->render();
    }
}

void RayTracerCompute::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);
}

bool RayTracerCompute::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = LineRenderer::setNewSettings(settings);

    return shallReloadGatherShader;
}

void RayTracerCompute::setNewState(const InternalState& newState) {
}

bool RayTracerCompute::needsReRender() {
    bool reRender = LineRenderer::needsReRender();
    return reRender;
}

void RayTracerCompute::notifyReRenderTriggeredExternally() {
    internalReRender = false;
}

void RayTracerCompute::onHasMoved() {
    LineRenderer::onHasMoved();
}

void RayTracerCompute::update(float dt) {
}



RayTracingComputePass::RayTracingComputePass(
        SceneData* sceneData, RayTracerCompute* rayTracerCompute, sgl::vk::Renderer* renderer, sgl::CameraPtr* camera)
        : ComputePass(renderer), sceneData(sceneData), rayTracerCompute(rayTracerCompute), camera(camera) {
}

void RayTracingComputePass::setOutputImage(sgl::vk::ImageViewPtr& imageView) {
    sceneImageView = imageView;

    if (computeData) {
        computeData->setStaticImageView(sceneImageView, "outputImage");
    }
}

void RayTracingComputePass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;

    TubeTriangleRenderDataPayloadPtr payloadSuperClass(new NodesBVHTreePayload(
            true, 16, 64, false, BvhBuildAlgorithm::SWEEP_SAH_CPU, BvhBuildGeometryMode::TRIANGLES,
            BvhBuildPrimitiveCenterMode::PRIMITIVE_CENTROID, true,
            16, 64, false));
    tubeTriangleRenderData = lineData->getLinePassTubeTriangleMeshRenderDataPayload(
            true, false, payloadSuperClass);
    auto* payload = static_cast<NodesBVHTreePayload*>(payloadSuperClass.get());
    bvhNodes = payload->getNodeDataBuffer();
    bvhTreeHeight = payload->getTreeHeight();

    tubeTriangleRenderData = lineData->getLinePassTubeTriangleMeshRenderData(false, false);
    dataDirty = true;
}

void RayTracingComputePass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    preprocessorDefines.insert(std::make_pair("WORKGROUP_SIZE_X", std::to_string(WORKGROUP_SIZE_X)));
    preprocessorDefines.insert(std::make_pair("WORKGROUP_SIZE_Y", std::to_string(WORKGROUP_SIZE_Y)));
    preprocessorDefines.insert(std::make_pair("MAX_STACK_SIZE", std::to_string(bvhTreeHeight * 2 + 1)));
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines, false);
    rayTracerCompute->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "RayTracingCompute.Compute" }, preprocessorDefines);
}

void RayTracingComputePass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticImageView(sceneImageView, "outputImage");
    computeData->setStaticBuffer(bvhNodes, "NodeBuffer");
    if (tubeTriangleRenderData.indexBuffer) {
        computeData->setStaticBuffer(
                tubeTriangleRenderData.indexBuffer, "TriangleIndexBuffer");
        computeData->setStaticBuffer(
                tubeTriangleRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
        computeData->setStaticBufferOptional(
                tubeTriangleRenderData.linePointDataBuffer, "LinePointDataBuffer");
        computeData->setStaticBufferOptional(
                tubeTriangleRenderData.stressLinePointDataBuffer, "StressLinePointDataBuffer");
        computeData->setStaticBufferOptional(
                tubeTriangleRenderData.stressLinePointPrincipalStressDataBuffer,
                "StressLinePointPrincipalStressDataBuffer");
        if (tubeTriangleRenderData.multiVarAttributeDataBuffer) {
            computeData->setStaticBufferOptional(
                    tubeTriangleRenderData.multiVarAttributeDataBuffer, "AttributeDataArrayBuffer");
        }
    }
    rayTracerCompute->setRenderDataBindings(computeData);
    lineData->setVulkanRenderDataDescriptors(std::static_pointer_cast<vk::RenderData>(computeData));
}

void RayTracingComputePass::_render() {
    lineData->updateVulkanUniformBuffers(rayTracerCompute, renderer);

    if ((*sceneData->performanceMeasurer)) {
        auto renderDataSize = computeData->getRenderDataSize();
        (*sceneData->performanceMeasurer)->setCurrentDataSetBufferSizeBytes(renderDataSize.storageBufferSize);
    }

    const auto& imageSettings = sceneImageView->getImage()->getImageSettings();
    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_GENERAL);
    renderer->dispatch(
            computeData,
            sgl::uiceil(imageSettings.width, WORKGROUP_SIZE_X),
            sgl::uiceil(imageSettings.height, WORKGROUP_SIZE_Y),
            1);
    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}
