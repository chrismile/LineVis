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
#include <Utils/AppSettings.hpp>
#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "MLABBucketRenderer.hpp"

// Opacity threshold for lower back buffer boundary.
static float lowerBackBufferOpacity = 0.2f;

// Opacity threshold for upper back buffer boundary.
static float upperBackBufferOpacity = 0.98f;

MLABBucketRenderer::MLABBucketRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : MLABRenderer("Multi-Layer Alpha Blending Renderer with Depth Buckets",
                       sceneData, transferFunctionWindow) {
}

void MLABBucketRenderer::initialize() {
    MLABRenderer::initialize();
    uniformBucketDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(UniformBucketData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    minDepthRasterPass = std::make_shared<MinDepthRasterPass>(this);

    // Uniform updating is done by the minimum depth raster pass.
    lineRasterPass->setUpdateUniformData(false);
}

MLABBucketRenderer::~MLABBucketRenderer() = default;

void MLABBucketRenderer::reloadGatherShader() {
    MLABRenderer::reloadGatherShader();
    minDepthRasterPass->setShaderDirty();
}

void MLABBucketRenderer::setNewState(const InternalState& newState) {
    MLABRenderer::setNewState(newState);
    lowerBackBufferOpacity = newState.rendererSettings.getFloatValue("lowerOpacity");
    upperBackBufferOpacity = newState.rendererSettings.getFloatValue("upperOpacity");
}

void MLABBucketRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    MLABRenderer::setLineData(lineData, isNewData);
    minDepthRasterPass->setLineData(lineData, isNewData);
}

void MLABBucketRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    MLABRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines["OIT_GATHER_HEADER"] = "\"MLABBucketGather.glsl\"";
    preprocessorDefines.insert(std::make_pair("MLAB_MIN_DEPTH_BUCKETS", ""));
    preprocessorDefines.insert(std::make_pair("USE_SCREEN_SPACE_POSITION", ""));
}

void MLABBucketRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    MLABRenderer::setRenderDataBindings(renderData);
    renderData->setStaticBufferOptional(minDepthBuffer, "MinDepthBuffer");
    renderData->setStaticBufferOptional(uniformBucketDataBuffer, "UniformBucketDataBuffer");
}

void MLABBucketRenderer::updateVulkanUniformBuffers() {
}

void MLABBucketRenderer::reallocateFragmentBuffer() {
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    int paddedWidth = width, paddedHeight = height;
    getScreenSizeWithTiling(paddedWidth, paddedHeight);

    size_t fragmentBufferSizeBytes =
            (sizeof(uint32_t) + sizeof(float)) * size_t(numLayers) * size_t(paddedWidth) * size_t(paddedHeight);
    if (fragmentBufferSizeBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than maxStorageBufferRange ("
                + std::to_string(maxStorageBufferSize) + "). Clamping to maxStorageBufferRange.",
                false);
        fragmentBufferSizeBytes = maxStorageBufferSize / 8ull - 8ull;
    } else {
        sgl::Logfile::get()->writeInfo(
                std::string() + "Fragment buffer size GiB: "
                + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));
    }

    size_t minDepthBufferSizeBytes = sizeof(float) * 2 * size_t(paddedWidth) * size_t(paddedHeight);

    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(
                fragmentBufferSizeBytes + minDepthBufferSizeBytes);
    }

    fragmentBuffer = {}; // Delete old data first (-> refcount 0)
    fragmentBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), fragmentBufferSizeBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    minDepthBuffer = {}; // Delete old data first (-> refcount 0)
    minDepthBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), minDepthBufferSizeBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    updateSyncMode();

    // Buffer has to be cleared again.
    clearBitSet = true;
}

void MLABBucketRenderer::onResolutionChanged() {
    MLABRenderer::onResolutionChanged();

    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    minDepthRasterPass->recreateSwapchain(width, height);
}

void MLABBucketRenderer::render() {
    LineRenderer::renderBase();

    computeDepthRange();
    setUniformData();
    clear();
    gather();
    resolve();
}

void MLABBucketRenderer::setUniformData() {
    uniformBucketData.lowerBackBufferOpacity = lowerBackBufferOpacity;
    uniformBucketData.upperBackBufferOpacity = upperBackBufferOpacity;
    uniformBucketDataBuffer->updateData(
            sizeof(UniformBucketData), &uniformBucketData,
            renderer->getVkCommandBuffer());

    MLABRenderer::setUniformData();
}

void MLABBucketRenderer::gather() {
    //renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    //renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    //renderer->setModelMatrix(sgl::matrixIdentity());

    minDepthRasterPass->buildIfNecessary();
    if (!minDepthRasterPass->getIsDataEmpty()) {
        minDepthRasterPass->render();
    }
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

    lineRasterPass->buildIfNecessary();
    if (!lineRasterPass->getIsDataEmpty()) {
        lineRasterPass->render();
    }
    renderHull();
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void MLABBucketRenderer::computeDepthRange() {
    const sgl::AABB3& boundingBox = lineData->getModelBoundingBox();
    sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed(sceneData->camera->getViewMatrix());

    /*
     * Add offset of 0.1 for e.g. point data sets where additonal vertices may be added in the shader for quads.
     * Vulkan uses a right-handed coordinate system with the negative z axis pointing into the view direction.
     * Consequently, we need to use the negative z position to get the depth and swap minimum and maximum.
     */
    float minViewZ = -screenSpaceBoundingBox.getMaximum().z - 0.1f;
    float maxViewZ = -screenSpaceBoundingBox.getMinimum().z + 0.1f;
    minViewZ = std::max(minViewZ, sceneData->camera->getNearClipDistance());
    maxViewZ = std::min(maxViewZ, sceneData->camera->getFarClipDistance());
    minViewZ = std::min(minViewZ, sceneData->camera->getFarClipDistance());
    maxViewZ = std::max(maxViewZ, sceneData->camera->getNearClipDistance());
    float logmin = std::log(minViewZ);
    float logmax = std::log(maxViewZ);
    uniformBucketData.logDepthMin = logmin;
    uniformBucketData.logDepthMax = logmax;
}

void MLABBucketRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    MLABRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addSliderFloat("Back Bucket Lower Opacity", &lowerBackBufferOpacity, 0.0f, 1.0f)) {
        reRender = true;
    }

    if (propertyEditor.addSliderFloat("Back Bucket Upper Opacity", &upperBackBufferOpacity, 0.0f, 1.0f)) {
        reRender = true;
    }
}



MinDepthRasterPass::MinDepthRasterPass(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
}

void MinDepthRasterPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines["OIT_GATHER_HEADER"] = "\"MinDepthPass.glsl\"";
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            lineData->getShaderModuleNames(), preprocessorDefines);
}
