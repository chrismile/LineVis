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

#include "VulkanRayTracer.hpp"

using namespace sgl;

VulkanRayTracer::VulkanRayTracer(SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Vulkan Ray Tracer", sceneData, transferFunctionWindow) {
    isRasterizer = false;

    rayTracingRenderPass = std::make_shared<RayTracingRenderPass>(
            this, renderer, &sceneData->camera);
    rayTracingRenderPass->setNumSamplesPerFrame(numSamplesPerFrame);
    rayTracingRenderPass->setMaxNumFrames(maxNumAccumulatedFrames);
    rayTracingRenderPass->setMaxDepthComplexity(maxDepthComplexity);
    rayTracingRenderPass->setUseAnalyticIntersections(useAnalyticIntersections);
    rayTracingRenderPass->setUseMlat(useMlat);
    rayTracingRenderPass->setMlatNumNodes(mlatNumNodes);

    onClearColorChanged();
}

VulkanRayTracer::~VulkanRayTracer() {
    sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();

    rayTracingRenderPass = {};
    ambientOcclusionBaker = {};
}

void VulkanRayTracer::reloadGatherShader() {
    rayTracingRenderPass->setShaderDirty();
    rayTracingRenderPass->setUseDepthCues(useDepthCues);
    rayTracingRenderPass->setVisualizeSeedingProcess(visualizeSeedingProcess);
    rayTracingRenderPass->setUseAmbientOcclusion(useAmbientOcclusion);
    rayTracingRenderPass->setAmbientOcclusionBaker(ambientOcclusionBaker);
    accumulatedFramesCounter = 0;
}

bool VulkanRayTracer::getIsTriangleRepresentationUsed() const {
    return LineRenderer::getIsTriangleRepresentationUsed() || true; // !useAnalyticIntersections
}

void VulkanRayTracer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    rayTracingRenderPass->setLineData(lineData, isNewData);

    accumulatedFramesCounter = 0;
    dirty = false;
    reRender = true;
}

void VulkanRayTracer::setRenderSimulationMeshHull(bool shallRenderSimulationMeshHull) {
    rayTracingRenderPass->setRenderSimulationMeshHull(shallRenderSimulationMeshHull);
}

void VulkanRayTracer::setVisualizeSeedingProcess(bool visualizeSeeding) {
    if (this->visualizeSeedingProcess != visualizeSeeding) {
        this->visualizeSeedingProcess = visualizeSeeding;
        if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
            reloadGatherShader();
        }
    }
}

void VulkanRayTracer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    rayTracingRenderPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    rayTracingRenderPass->recreateSwapchain(width, height);

    accumulatedFramesCounter = 0;
}

void VulkanRayTracer::render() {
    LineRenderer::renderBase(VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR);
    if (ambientOcclusionBaker
            && ((ambientOcclusionBaker->getIsStaticPrebaker() && ambientOcclusionBuffersDirty)
            || (!ambientOcclusionBaker->getIsStaticPrebaker() && ambientOcclusionTexturesDirty))) {
        rayTracingRenderPass->setDataDirty();
        ambientOcclusionBuffersDirty = false;
        ambientOcclusionTexturesDirty = false;
    }

    rayTracingRenderPass->setFrameNumber(accumulatedFramesCounter);
    rayTracingRenderPass->setDepthMinMaxBuffer(depthMinMaxBuffers[outputDepthMinMaxBufferIndex]);
    rayTracingRenderPass->render();

    accumulatedFramesCounter++;
}

void VulkanRayTracer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addSliderInt(
            "#Samples/Frame", reinterpret_cast<int*>(&numSamplesPerFrame), 1, 32)) {
        rayTracingRenderPass->setNumSamplesPerFrame(numSamplesPerFrame);
        accumulatedFramesCounter = 0;
    }

    if (propertyEditor.addSliderInt(
            "#Accum. Frames", reinterpret_cast<int*>(&maxNumAccumulatedFrames), 1, 32)) {
        rayTracingRenderPass->setMaxNumFrames(maxNumAccumulatedFrames);
        accumulatedFramesCounter = 0;
    }

    if (propertyEditor.addSliderInt(
            "Max. Depth", reinterpret_cast<int*>(&maxDepthComplexity), 1, 2048)) {
        rayTracingRenderPass->setMaxDepthComplexity(maxDepthComplexity);
        accumulatedFramesCounter = 0;
    }

    if (propertyEditor.addCheckbox("Use Analytic Intersections", &useAnalyticIntersections)) {
        rayTracingRenderPass->setUseAnalyticIntersections(useAnalyticIntersections);
        rayTracingRenderPass->setShaderDirty();
        if (lineData) {
            rayTracingRenderPass->setLineData(lineData, false);
        }
        accumulatedFramesCounter = 0;
    }
    ImGui::SameLine();
    ImGui::HelpMarker("Whether to trace rays against a triangle mesh or analytic tubes using line segment AABBs.");

    if (propertyEditor.addCheckbox("Use MLAT", &useMlat)) {
        rayTracingRenderPass->setUseMlat(useMlat);
        rayTracingRenderPass->setShaderDirty();
        accumulatedFramesCounter = 0;
    }
    ImGui::SameLine();
    ImGui::HelpMarker("Whether to use multi-layer alpha tracing for accelerated transparency rendering.");

    if (useMlat && propertyEditor.addSliderIntPowerOfTwo("#MLAT Nodes", &mlatNumNodes, 1, 32)) {
        rayTracingRenderPass->setMlatNumNodes(mlatNumNodes);
        rayTracingRenderPass->setShaderDirty();
        accumulatedFramesCounter = 0;
    }
}

bool VulkanRayTracer::needsReRender() {
    bool reRender = LineRenderer::needsReRender();
    if (accumulatedFramesCounter < maxNumAccumulatedFrames) {
        reRender = true;
    }
    return reRender;
}

void VulkanRayTracer::notifyReRenderTriggeredExternally() {
    internalReRender = false;
    accumulatedFramesCounter = 0;
}

void VulkanRayTracer::onHasMoved() {
    LineRenderer::onHasMoved();
    accumulatedFramesCounter = 0;
}

void VulkanRayTracer::update(float dt) {
}



RayTracingRenderPass::RayTracingRenderPass(
        VulkanRayTracer* vulkanRayTracer, sgl::vk::Renderer* renderer, sgl::CameraPtr* camera)
        : RayTracingPass(renderer), vulkanRayTracer(vulkanRayTracer), camera(camera) {
    rayTracerSettingsBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(RayTracerSettings),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

void RayTracingRenderPass::setOutputImage(sgl::vk::ImageViewPtr& imageView) {
    sceneImageView = imageView;

    if (rayTracingData) {
        rayTracingData->setStaticImageView(sceneImageView, 1);
    }
}

void RayTracingRenderPass::setLineData(LineDataPtr& lineData, bool isNewData) {
    this->lineData = lineData;
    if (useAnalyticIntersections) {
        tubeTriangleRenderData = TubeTriangleRenderData();
        tubeAabbRenderData = lineData->getVulkanTubeAabbRenderData(vulkanRayTracer);
        if (lineData->getShallRenderSimulationMeshBoundary()) {
            topLevelAS = lineData->getRayTracingTubeAabbAndHullTopLevelAS(vulkanRayTracer);
        } else {
            topLevelAS = lineData->getRayTracingTubeAabbTopLevelAS(vulkanRayTracer);
        }
    } else {
        tubeTriangleRenderData = lineData->getVulkanTubeTriangleRenderData(vulkanRayTracer, true);
        tubeAabbRenderData = TubeAabbRenderData();
        if (lineData->getShallRenderSimulationMeshBoundary()) {
            topLevelAS = lineData->getRayTracingTubeTriangleAndHullTopLevelAS(vulkanRayTracer);
        } else {
            topLevelAS = lineData->getRayTracingTubeTriangleTopLevelAS(vulkanRayTracer);
        }
    }

    if (lineData->getShallRenderSimulationMeshBoundary()) {
        hullTriangleRenderData = lineData->getVulkanHullTriangleRenderData(true);
    }
    dataDirty = true;
}

void RayTracingRenderPass::setRenderSimulationMeshHull(bool shallRenderSimulationMeshHull) {
    setLineData(lineData, false);
}

void RayTracingRenderPass::updateUseJitteredSamples() {
    bool useJitteredSamplesNew = maxNumFrames > 1 || rayTracerSettings.numSamplesPerFrame > 1;
    if (useJitteredSamples != useJitteredSamplesNew) {
        setShaderDirty();
    }
    useJitteredSamples = useJitteredSamplesNew;
}

void RayTracingRenderPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines, false);
    vulkanRayTracer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    if (useJitteredSamples) {
        preprocessorDefines.insert(std::make_pair("USE_JITTERED_RAYS", ""));
    }
    if (useDepthCues) {
        preprocessorDefines.insert(std::make_pair("USE_DEPTH_CUES", ""));
        preprocessorDefines.insert(std::make_pair("COMPUTE_DEPTH_CUES_GPU", ""));
        preprocessorDefines.insert(std::make_pair("USE_SCREEN_SPACE_POSITION", ""));
    }
    if (visualizeSeedingProcess) {
        preprocessorDefines.insert(std::make_pair("VISUALIZE_SEEDING_PROCESS", ""));
    }
    if (useAmbientOcclusion && ambientOcclusionBaker) {
        preprocessorDefines.insert(std::make_pair("USE_AMBIENT_OCCLUSION", ""));
        if (ambientOcclusionBaker->getIsStaticPrebaker()) {
            preprocessorDefines.insert(std::make_pair("STATIC_AMBIENT_OCCLUSION_PREBAKING", ""));
        }
        preprocessorDefines.insert(std::make_pair("GEOMETRY_PASS_TUBE", ""));
    }
    if (useMlat) {
        preprocessorDefines.insert(std::make_pair("USE_MLAT", ""));
        preprocessorDefines.insert(std::make_pair("NUM_NODES", std::to_string(mlatNumNodes)));
    }
    if (useMlat) {
        if (useAnalyticIntersections) {
            shaderStages = sgl::vk::ShaderManager->getShaderStages(
                    {"TubeRayTracing.RayGen", "TubeRayTracing.Miss",
                     "TubeRayTracing.IntersectionTube", "TubeRayTracing.AnyHitTubeAnalytic",
                     "TubeRayTracing.AnyHitHull"},
                    preprocessorDefines);
        } else {
            shaderStages = sgl::vk::ShaderManager->getShaderStages(
                    {"TubeRayTracing.RayGen", "TubeRayTracing.Miss",
                     "TubeRayTracing.AnyHitTubeTriangles", "TubeRayTracing.AnyHitHull"},
                    preprocessorDefines);
        }
    } else {
        if (useAnalyticIntersections) {
            shaderStages = sgl::vk::ShaderManager->getShaderStages(
                    {"TubeRayTracing.RayGen", "TubeRayTracing.Miss",
                     "TubeRayTracing.IntersectionTube", "TubeRayTracing.ClosestHitTubeAnalytic",
                     "TubeRayTracing.ClosestHitHull"},
                    preprocessorDefines);
        } else {
            shaderStages = sgl::vk::ShaderManager->getShaderStages(
                    {"TubeRayTracing.RayGen", "TubeRayTracing.Miss",
                     "TubeRayTracing.ClosestHitTubeTriangles", "TubeRayTracing.ClosestHitHull"},
                    preprocessorDefines);
        }
    }
}

void RayTracingRenderPass::createRayTracingData(
        sgl::vk::Renderer* renderer, sgl::vk::RayTracingPipelinePtr& rayTracingPipeline) {
    rayTracingData = std::make_shared<sgl::vk::RayTracingData>(renderer, rayTracingPipeline);
    rayTracingData->setStaticImageView(sceneImageView, "outputImage");
    rayTracingData->setTopLevelAccelerationStructure(topLevelAS, "topLevelAS");
    rayTracingData->setStaticBuffer(rayTracerSettingsBuffer, "RayTracerSettingsBuffer");
    if (useAnalyticIntersections) {
        if (tubeAabbRenderData.indexBuffer) {
            rayTracingData->setStaticBuffer(
                    tubeAabbRenderData.indexBuffer, "BoundingBoxLinePointIndexBuffer");
            rayTracingData->setStaticBuffer(
                    tubeAabbRenderData.linePointDataBuffer, "LinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    tubeAabbRenderData.stressLinePointDataBuffer, "StressLinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    tubeAabbRenderData.stressLinePointPrincipalStressDataBuffer,
                    "StressLinePointPrincipalStressDataBuffer");
        } else {
            // Just bind anything in order for sgl to not complain...
            rayTracingData->setStaticBuffer(
                    hullTriangleRenderData.indexBuffer, "BoundingBoxLinePointIndexBuffer");
            rayTracingData->setStaticBuffer(
                    hullTriangleRenderData.vertexBuffer, "LinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    hullTriangleRenderData.vertexBuffer, "StressLinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    hullTriangleRenderData.vertexBuffer, "StressLinePointPrincipalStressDataBuffer");
        }
    } else {
        if (tubeTriangleRenderData.indexBuffer) {
            rayTracingData->setStaticBuffer(
                    tubeTriangleRenderData.indexBuffer, "TubeIndexBuffer");
            rayTracingData->setStaticBuffer(
                    tubeTriangleRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
            rayTracingData->setStaticBuffer(
                    tubeTriangleRenderData.linePointDataBuffer, "LinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    tubeTriangleRenderData.stressLinePointDataBuffer, "StressLinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    tubeTriangleRenderData.stressLinePointPrincipalStressDataBuffer,
                    "StressLinePointPrincipalStressDataBuffer");
        } else {
            // Just bind anything in order for sgl to not complain...
            rayTracingData->setStaticBuffer(
                    hullTriangleRenderData.indexBuffer, "TubeIndexBuffer");
            rayTracingData->setStaticBuffer(
                    hullTriangleRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
            rayTracingData->setStaticBuffer(
                    hullTriangleRenderData.vertexBuffer, "LinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    hullTriangleRenderData.vertexBuffer, "StressLinePointDataBuffer");
            rayTracingData->setStaticBufferOptional(
                    hullTriangleRenderData.vertexBuffer, "StressLinePointPrincipalStressDataBuffer");
        }
    }
    if (hullTriangleRenderData.indexBuffer) {
        rayTracingData->setStaticBuffer(hullTriangleRenderData.indexBuffer, "HullIndexBuffer");
        rayTracingData->setStaticBuffer(
                hullTriangleRenderData.vertexBuffer, "HullTriangleVertexDataBuffer");
    } else {
        // Just bind anything in order for sgl to not complain...
        sgl::vk::BufferPtr indexBuffer =
                useAnalyticIntersections ? tubeAabbRenderData.indexBuffer : tubeTriangleRenderData.indexBuffer;
        sgl::vk::BufferPtr vertexBuffer =
                useAnalyticIntersections ? tubeAabbRenderData.linePointDataBuffer : tubeTriangleRenderData.vertexBuffer;
        rayTracingData->setStaticBuffer(indexBuffer, "HullIndexBuffer");
        rayTracingData->setStaticBuffer(vertexBuffer, "HullTriangleVertexDataBuffer");
    }
    vulkanRayTracer->setRenderDataBindings(rayTracingData);
    lineData->setVulkanRenderDataDescriptors(std::static_pointer_cast<vk::RenderData>(rayTracingData));
}

void RayTracingRenderPass::updateLineRenderSettings() {
    rayTracerSettingsBuffer->updateData(
            sizeof(RayTracerSettings), &rayTracerSettings, renderer->getVkCommandBuffer());

    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR);
}

void RayTracingRenderPass::_render() {
    updateLineRenderSettings();

    vk::ShaderGroupSettings shaderGroupSettings{};
    shaderGroupSettings.hitShaderGroupSize = lineData->getShallRenderSimulationMeshBoundary() ? 2 : 1;
    rayTracingData->setShaderGroupSettings(shaderGroupSettings);

    lineData->updateVulkanUniformBuffers(vulkanRayTracer, renderer);

    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_GENERAL);
    renderer->traceRays(rayTracingData, launchSizeX, launchSizeY, launchSizeZ);
    renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

sgl::vk::RayTracingPipelinePtr RayTracingRenderPass::createRayTracingPipeline() {
    //sgl::vk::ShaderBindingTable sbt = sgl::vk::ShaderBindingTable::generateSimpleShaderBindingTable(shaderStages);
    sgl::vk::ShaderBindingTable sbt(shaderStages);
    sbt.addRayGenShaderGroup()->setRayGenShader(0);
    sbt.addMissShaderGroup()->setMissShader(1);
    if (useMlat) {
        if (useAnalyticIntersections) {
            sgl::vk::HitShaderGroup* tubeHitShaderGroup = sbt.addHitShaderGroup(
                    VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR);
            tubeHitShaderGroup->setIntersectionShader(2);
            tubeHitShaderGroup->setAnyHitShader(3);
            sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setAnyHitShader(4);
        } else {
            sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setAnyHitShader(2);
            sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setAnyHitShader(3);
        }
    } else {
        if (useAnalyticIntersections) {
            sgl::vk::HitShaderGroup* tubeHitShaderGroup = sbt.addHitShaderGroup(
                    VK_RAY_TRACING_SHADER_GROUP_TYPE_PROCEDURAL_HIT_GROUP_KHR);
            tubeHitShaderGroup->setIntersectionShader(2);
            tubeHitShaderGroup->setClosestHitShader(3);
            sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setClosestHitShader(4);
        } else {
            sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setClosestHitShader(2);
            sbt.addHitShaderGroup(VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR)->setClosestHitShader(3);
        }
    }

    sgl::vk::RayTracingPipelineInfo rayTracingPipelineInfo(sbt);
    return std::make_shared<sgl::vk::RayTracingPipeline>(device, rayTracingPipelineInfo);
}
