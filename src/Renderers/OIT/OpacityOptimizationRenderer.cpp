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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Utils/AppSettings.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "LineData/LineDataStress.hpp"
#include "Loaders/TrajectoryFile.hpp"
#include "OpacityOptimizationRenderer.hpp"

OpacityOptimizationRenderer::OpacityOptimizationRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opacity Optimization Renderer", sceneData, transferFunctionWindow) {
    // Add events for interaction with line data.
    onOpacityEstimationRecomputeListenerToken = sgl::EventManager::get()->addListener(
            ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT, [this](const sgl::EventPtr&) {
                this->onHasMoved();
            });
}

void OpacityOptimizationRenderer::initialize() {
    LineRenderer::initialize();

    // Get all available multisampling modes.
    maximumNumberOfSamples = (*sceneData->renderer)->getDevice()->getMaxUsableSampleCount();
    if (maximumNumberOfSamples <= 1) {
        useMultisampling = false;
    }
    supportsSampleShadingRate = renderer->getDevice()->getPhysicalDeviceFeatures().sampleRateShading;
    numSampleModes = sgl::intlog2(maximumNumberOfSamples) + 1;
    sampleModeSelection = std::min(sgl::intlog2(numSamples), numSampleModes - 1);
    for (int i = 1; i <= maximumNumberOfSamples; i *= 2) {
        sampleModeNames.push_back(std::to_string(i));
    }

    ppllUniformDataBufferOpacities = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(PpllUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    ppllUniformDataBufferFinal = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(PpllUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    opacityOptimizationUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(OpacityOptimizationUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    attributeRangeUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(AttributeRangeUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    resolvePpllOpacitiesPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this,
            {"LinkedListResolveOpacities.Vertex", "LinkedListResolveOpacities.Fragment"}));
    resolvePpllOpacitiesPass->setColorWriteEnabled(false);
    resolvePpllOpacitiesPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    resolvePpllOpacitiesPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_DONT_CARE);
    resolvePpllOpacitiesPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
    resolvePpllOpacitiesPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    clearPpllOpacitiesPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"LinkedListClearOpacities.Vertex", "LinkedListClearOpacities.Fragment"}));
    clearPpllOpacitiesPass->setColorWriteEnabled(false);
    clearPpllOpacitiesPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    clearPpllOpacitiesPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_DONT_CARE);
    clearPpllOpacitiesPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
    clearPpllOpacitiesPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    resolvePpllFinalPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this,
            {"LinkedListResolveFinal.Vertex", "LinkedListResolveFinal.Fragment"}));
    resolvePpllFinalPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    resolvePpllFinalPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_STRAIGHT_ALPHA);

    clearPpllFinalPass = std::shared_ptr<ResolvePass>(new ResolvePass(
            this, {"LinkedListClearFinal.Vertex", "LinkedListClearFinal.Fragment"}));
    clearPpllFinalPass->setColorWriteEnabled(false);
    clearPpllFinalPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    clearPpllFinalPass->setAttachmentStoreOp(VK_ATTACHMENT_STORE_OP_DONT_CARE);
    clearPpllFinalPass->setOutputImageInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED);
    clearPpllFinalPass->setOutputImageFinalLayout(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    gatherPpllOpacitiesPass = std::make_shared<PpllOpacitiesLineRasterPass>(this);
    gatherPpllOpacitiesPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE);
    gatherPpllOpacitiesPass->setUpdateUniformData(false);
    gatherPpllFinalPass = std::make_shared<PpllFinalLineRasterPass>(this);
    gatherPpllFinalPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
    gatherPpllFinalPass->setUpdateUniformData(false);

    convertPerSegmentOpacitiesPass = std::make_shared<ConvertPerSegmentOpacitiesPass>(this);
    for (int i = 0; i < 2; i++) {
        smoothPerSegmentOpacitiesPasses[i] = std::make_shared<SmoothPerSegmentOpacitiesPass>(this);
    }
    computePerVertexOpacitiesPass = std::make_shared<ComputePerVertexOpacitiesPass>(this);

    // Disable render passes of parent class.
    lineRasterPass = {};
    hullRasterPass = {};

    onClearColorChanged();
}

OpacityOptimizationRenderer::~OpacityOptimizationRenderer() {
    sgl::EventManager::get()->removeListener(
            ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT, onOpacityEstimationRecomputeListenerToken);
}

void OpacityOptimizationRenderer::reloadResolveShader() {
    resolvePpllOpacitiesPass->setShaderDirty();
    resolvePpllFinalPass->setShaderDirty();
}

void OpacityOptimizationRenderer::reloadGatherShader() {
    gatherPpllOpacitiesPass->setShaderDirty();
    gatherPpllFinalPass->setShaderDirty();
}

void OpacityOptimizationRenderer::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    timerDataIsWritten = false;
    if ((*sceneData->performanceMeasurer) && !timerDataIsWritten) {
        timer = {};
        timer = std::make_shared<sgl::vk::Timer>(renderer);
        timer->setStoreFrameTimeList(true);
        (*sceneData->performanceMeasurer)->setPpllTimer(timer);
    }
}

void OpacityOptimizationRenderer::updateLargeMeshMode() {
    // More than one million cells?
    LargeMeshMode newMeshLargeMeshMode = MESH_SIZE_MEDIUM;
    if (lineData->getNumLineSegments() > size_t(1e6)) { // > 1m line segments
        newMeshLargeMeshMode = MESH_SIZE_LARGE;
    }
    if (newMeshLargeMeshMode != largeMeshMode) {
        largeMeshMode = newMeshLargeMeshMode;
        expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_OPOPT[int(largeMeshMode)][0];
        expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES_OPOPT[int(largeMeshMode)][1];
        reallocateFragmentBuffer();
        reloadResolveShader();
    }
}

void OpacityOptimizationRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    gatherPpllOpacitiesPass->setLineData(lineData, isNewData);
    gatherPpllFinalPass->setLineData(lineData, isNewData);
    updateLargeMeshMode();

    lines = lineData->getFilteredLines(this);

    OpacityOptimizationLineDataBuffers buffers{};
    gatherPpllOpacitiesPass->setOpacityOptimizationLineDataBuffers(buffers);
    gatherPpllFinalPass->setOpacityOptimizationLineDataBuffers(buffers);

    if (lineData->getUseBandRendering()) {
        LinePassQuadsRenderData tubeRenderData = lineData->getLinePassQuadsRenderData();
        buffers.indexBuffer = tubeRenderData.indexBuffer;
        buffers.vertexPositionBuffer = tubeRenderData.vertexPositionBuffer;
        buffers.vertexAttributeBuffer = tubeRenderData.vertexAttributeBuffer;
        buffers.vertexNormalBuffer = tubeRenderData.vertexNormalBuffer;
        buffers.vertexTangentBuffer = tubeRenderData.vertexTangentBuffer;
        buffers.vertexOffsetLeftBuffer = tubeRenderData.vertexOffsetLeftBuffer;
        buffers.vertexOffsetRightBuffer = tubeRenderData.vertexOffsetRightBuffer;
        buffers.vertexPrincipalStressIndexBuffer = tubeRenderData.vertexPrincipalStressIndexBuffer;
        buffers.vertexLineHierarchyLevelBuffer = tubeRenderData.vertexLineHierarchyLevelBuffer;
    } else {
        TubeRenderDataOpacityOptimization tubeRenderData = lineData->getTubeRenderDataOpacityOptimization();
        buffers.indexBuffer = tubeRenderData.indexBuffer;
        buffers.vertexPositionBuffer = tubeRenderData.vertexPositionBuffer;
        buffers.vertexAttributeBuffer = tubeRenderData.vertexAttributeBuffer;
        buffers.vertexTangentBuffer = tubeRenderData.vertexTangentBuffer;
        buffers.vertexPrincipalStressIndexBuffer = tubeRenderData.vertexPrincipalStressIndexBuffer;
        buffers.vertexLineHierarchyLevelBuffer = tubeRenderData.vertexLineHierarchyLevelBuffer;
    }

    size_t sizeInBytes = 0;
    if (buffers.vertexPositionBuffer) {
        sizeInBytes = buffers.vertexPositionBuffer->getSizeInBytes();
    }
    numLineVertices = uint32_t(sizeInBytes / sizeof(glm::vec3));
    generateBlendingWeightParametrization(isNewData);

    vertexOpacityBuffer = {};
    if (numLineVertices > 0) {
        vertexOpacityBuffer = std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), numLineVertices * sizeof(float),
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
        vertexOpacityBuffer->fill(0, renderer->getVkCommandBuffer());
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                vertexOpacityBuffer);
    }

    buffers.lineSegmentIdBuffer = lineSegmentIdBuffer;
    buffers.vertexOpacityBuffer = vertexOpacityBuffer;
    gatherPpllOpacitiesPass->setOpacityOptimizationLineDataBuffers(buffers);
    gatherPpllFinalPass->setOpacityOptimizationLineDataBuffers(buffers);

    convertPerSegmentOpacitiesPass->setNumLineSegments(numLineSegments);
    convertPerSegmentOpacitiesPass->setSegmentOpacityBuffer(segmentOpacityBuffers[0]);
    convertPerSegmentOpacitiesPass->setSegmentOpacityUintBuffer(segmentOpacityUintBuffer);

    for (int i = 0; i < 2; i++) {
        smoothPerSegmentOpacitiesPasses[i]->setNumLineSegments(numLineSegments);
        smoothPerSegmentOpacitiesPasses[i]->setSegmentOpacityBufferIn(
                segmentOpacityBuffers[i]);
        smoothPerSegmentOpacitiesPasses[i]->setSegmentOpacityBufferOut(
                segmentOpacityBuffers[(i + 1) % 2]);
        smoothPerSegmentOpacitiesPasses[i]->setSegmentVisibilityBuffer(segmentVisibilityBuffer);
        smoothPerSegmentOpacitiesPasses[i]->setLineSegmentConnectivityBuffer(lineSegmentConnectivityBuffer);
    }

    computePerVertexOpacitiesPass->setNumLineVertices(numLineVertices);
    computePerVertexOpacitiesPass->setVertexOpacityBuffer(vertexOpacityBuffer);
    computePerVertexOpacitiesPass->setSegmentOpacityBuffer(
            segmentOpacityBuffers[opacityOptimizationUniformData.s % 2]);
    computePerVertexOpacitiesPass->setSegmentVisibilityBuffer(segmentVisibilityBuffer);
    computePerVertexOpacitiesPass->setBlendingWeightParametrizationBuffer(blendingWeightParametrizationBuffer);
    segmentOpacityBufferIdx = 0;

    dirty = false;
    reRender = true;
    onHasMoved();
}

void OpacityOptimizationRenderer::generateBlendingWeightParametrization(bool isNewData) {
    // First, compute data necessary for parametrizing the polylines (number of segments, segment lengths).
    linesLengthSum = 0.0f;
    numPolylineSegments = 0;
    polylineLengths.clear();
    polylineLengths.shrink_to_fit();
    polylineLengths.resize(lines.size());

#if _OPENMP >= 201107
    #pragma omp parallel for reduction(+: linesLengthSum) reduction(+: numPolylineSegments) shared(polylineLengths) \
    default(none)
#endif
    for (size_t lineIdx = 0; lineIdx < lines.size(); lineIdx++) {
        std::vector<glm::vec3>& line = lines.at(lineIdx);
        const size_t n = line.size();
        float polylineLength = 0.0f;
        for (size_t i = 1; i < n; i++) {
            polylineLength += glm::length(line[i] - line[i-1]);
        }
        polylineLengths.at(lineIdx) = polylineLength;
        linesLengthSum += polylineLength;
        numPolylineSegments += uint32_t(n - 1);
    }

    if (useViewDependentParametrization) {
        recomputeViewDependentParametrization();
    } else {
        recomputeStaticParametrization();
    }
}

void OpacityOptimizationRenderer::recomputeViewDependentParametrization() {
    // TODO
}

void OpacityOptimizationRenderer::recomputeStaticParametrization() {
    std::vector<uint32_t> lineSegmentIdData(numLineVertices, 0);
    std::vector<float> blendingWeightParametrizationData(numLineVertices, 0);
    std::vector<glm::uvec2> lineSegmentConnectivityData;

    const float EPSILON = 1e-5f;
    const int approximateLineSegmentsTotal = int(lines.size()) * 32; // Avg. discretization of 32 segments per line.
    lineSegmentConnectivityData.reserve(approximateLineSegmentsTotal);

    uint32_t segmentIdOffset = 0;
    size_t vertexIdx = 0;
    for (size_t lineIdx = 0; lineIdx < lines.size(); lineIdx++) {
        std::vector<glm::vec3>& line = lines.at(lineIdx);
        const size_t n = line.size();
        float polylineLength = polylineLengths.at(lineIdx);

        uint32_t numLineSubdivs =
                std::max(1u, uint32_t(std::ceil(float(approximateLineSegmentsTotal) / linesLengthSum * polylineLength)));
        float lineSubdivLength = polylineLength / float(numLineSubdivs);

        // Set the first vertex manually (we can guarantee there is no segment before it).
        assert(line.size() >= 2);
        lineSegmentIdData.at(vertexIdx) = segmentIdOffset;
        blendingWeightParametrizationData.at(vertexIdx) = float(segmentIdOffset);
        vertexIdx++;

        // Compute
        float currentLength = 0.0f;
        for (size_t i = 1; i < n; i++) {
            currentLength += glm::length(line[i] - line[i-1]);
            lineSegmentIdData.at(vertexIdx) =
                    segmentIdOffset + std::min(numLineSubdivs - 1u, uint32_t(
                            std::floor(float(numLineSubdivs) * currentLength / polylineLength)));
            float w = float(numLineSubdivs - 1u) * (currentLength - lineSubdivLength / 2.0f) / (polylineLength - lineSubdivLength);
            blendingWeightParametrizationData.at(vertexIdx) =
                    float(segmentIdOffset) + glm::clamp(w, 0.0f, float(numLineSubdivs - 1u) - EPSILON);
            vertexIdx++;
        }

        if (numLineSubdivs == 1) {
            lineSegmentConnectivityData.emplace_back(segmentIdOffset, segmentIdOffset);
        } else {
            lineSegmentConnectivityData.emplace_back(segmentIdOffset, segmentIdOffset + 1);
            for (size_t i = 1; i < numLineSubdivs - 1; i++) {
                lineSegmentConnectivityData.emplace_back(segmentIdOffset + i - 1u, segmentIdOffset + i + 1u);
            }
            lineSegmentConnectivityData.emplace_back(
                    segmentIdOffset + numLineSubdivs - 2u, segmentIdOffset + numLineSubdivs - 1u);
        }

        segmentIdOffset += numLineSubdivs;
    }
    numLineSegments = uint32_t(lineSegmentConnectivityData.size());

    lineSegmentIdBuffer = {};
    blendingWeightParametrizationBuffer = {};
    lineSegmentConnectivityBuffer = {};
    segmentVisibilityBuffer = {};
    segmentOpacityUintBuffer = {};
    segmentOpacityBuffers[0] = {};
    segmentOpacityBuffers[1] = {};
    if (numLineVertices == 0) {
        return;
    }

    // Upload the data to the GPU.
    lineSegmentIdBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numLineVertices * sizeof(uint32_t), lineSegmentIdData.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    blendingWeightParametrizationBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numLineVertices * sizeof(float), blendingWeightParametrizationData.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    lineSegmentConnectivityBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numLineSegments * sizeof(glm::uvec2), lineSegmentConnectivityData.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    segmentVisibilityBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numLineSegments * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    segmentOpacityUintBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numLineSegments * sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    for (int i = 0; i < 2; i++) {
        segmentOpacityBuffers[i] = std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), numLineSegments * sizeof(float),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    }

    if (resolvePpllOpacitiesPass) {
        resolvePpllOpacitiesPass->setDataDirty();
    }
}

void OpacityOptimizationRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);

    bool isStressLineData = false;
    bool usePrincipalStressDirectionIndex = false;
    bool useLineHierarchy = false;
    if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
        LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
        isStressLineData = true;
        usePrincipalStressDirectionIndex = lineDataStress->getUsePrincipalStressDirectionIndex();
        useLineHierarchy = lineDataStress->getUseLineHierarchy();
    }
    if (isStressLineData) {
        preprocessorDefines.insert(std::make_pair("IS_PSL_DATA", ""));
    }
    if (usePrincipalStressDirectionIndex) {
        preprocessorDefines.insert(std::make_pair("USE_PRINCIPAL_STRESS_DIRECTION_INDEX", ""));
    }
    if (useLineHierarchy) {
        preprocessorDefines.insert(std::make_pair("USE_LINE_HIERARCHY_LEVEL", ""));
    }
    preprocessorDefines.insert(std::make_pair("USE_TRANSPARENCY", ""));

    if (useMultisampling) {
        preprocessorDefines.insert(std::make_pair("USE_COVERAGE_MASK", ""));
    }

    preprocessorDefines.insert(std::make_pair("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity)));
    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
        || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = int(std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4);
        preprocessorDefines.insert(std::make_pair("STACK_SIZE", sgl::toString(stackSize)));
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_PRIORITY_QUEUE) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "frontToBackPQ"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BUBBLE_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "bubbleSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_INSERTION_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "insertionSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_SHELL_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "shellSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_MAX_HEAP) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "heapSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BITONIC_SORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "bitonicSort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "quicksort"));
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        preprocessorDefines.insert(std::make_pair("sortingAlgorithm", "quicksortHybrid"));
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
        || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        preprocessorDefines.insert(std::make_pair("USE_QUICKSORT", ""));
    }
}

void OpacityOptimizationRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setColorWriteEnabled(false);
    pipelineInfo.setDepthWriteEnabled(false);
    pipelineInfo.setDepthTestEnabled(false);
}

void OpacityOptimizationRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);

    renderData->setStaticBufferOptional(
            opacityOptimizationUniformDataBuffer, "OpacityOptimizationUniformDataBuffer");
    renderData->setStaticBufferOptional(
            attributeRangeUniformDataBuffer, "AttributeRangeUniformDataBuffer");

    if (isOpacitiesStep) {
        renderData->setStaticBufferOptional(fragmentBufferOpacities, "FragmentBuffer");
        renderData->setStaticBufferOptional(startOffsetBufferOpacities, "StartOffsetBuffer");
        renderData->setStaticBufferOptional(fragmentCounterBufferOpacities, "FragCounterBuffer");
        renderData->setStaticBufferOptional(ppllUniformDataBufferOpacities, "UniformDataBuffer");
    } else {
        renderData->setStaticBufferOptional(fragmentBufferFinal, "FragmentBuffer");
        renderData->setStaticBufferOptional(startOffsetBufferFinal, "StartOffsetBuffer");
        renderData->setStaticBufferOptional(fragmentCounterBufferFinal, "FragCounterBuffer");
        renderData->setStaticBufferOptional(ppllUniformDataBufferFinal, "UniformDataBuffer");
    }

    // Resolve opacities pass.
    renderData->setStaticBufferOptional(segmentOpacityUintBuffer, "OpacityBufferUint");
    renderData->setStaticBufferOptional(segmentVisibilityBuffer, "LineSegmentVisibilityBuffer");
}

void OpacityOptimizationRenderer::updateVulkanUniformBuffers() {
}

void OpacityOptimizationRenderer::setFramebufferAttachments(
        sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::ImageViewPtr colorRenderTarget;
    sgl::vk::AttachmentState attachmentState;
    if (loadOp == VK_ATTACHMENT_LOAD_OP_DONT_CARE) {
        colorRenderTarget = colorRenderTargetImageOpacities;
    } else {
        colorRenderTarget = colorRenderTargetImageFinal;
        //attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        //attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        //attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        //attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    }
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    framebuffer->setColorAttachment(
            colorRenderTarget, 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());
}

void OpacityOptimizationRenderer::reallocateFragmentBuffer() {
    // Fragment buffer for opacities.
    fragmentBufferSizeOpacity = size_t(expectedAvgDepthComplexity) * size_t(viewportWidthOpacity) * size_t(viewportHeightOpacity);
    size_t fragmentBufferSizeOpacityBytes = 12ull * fragmentBufferSizeOpacity;
    if (fragmentBufferSizeOpacityBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.",
                false);
        fragmentBufferSizeOpacityBytes = (1ull << 32ull) - 12ull;
        fragmentBufferSizeOpacity = fragmentBufferSizeOpacityBytes / 12ull;
    }

    fragmentBufferOpacities = {}; // Delete old data first (-> refcount 0)
    fragmentBufferOpacities = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), fragmentBufferSizeOpacityBytes,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    // Fragment buffer for final color pass.
    fragmentBufferSizeFinal = size_t(expectedAvgDepthComplexity) * size_t(viewportWidthFinal) * size_t(viewportHeightFinal);
    size_t fragmentBufferSizeFinalBytes = 12ull * fragmentBufferSizeFinal;
    if (fragmentBufferSizeFinalBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.",
                false);
        fragmentBufferSizeFinalBytes = (1ull << 32ull) - 12ull;
        fragmentBufferSizeFinal = fragmentBufferSizeFinalBytes / 12ull;
    }

    fragmentBufferFinal = {}; // Delete old data first (-> refcount 0)
    fragmentBufferFinal = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), fragmentBufferSizeFinalBytes,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    // Write info to console and performance measurer.
    size_t fragmentBufferSizeBytes = fragmentBufferSizeOpacityBytes + fragmentBufferSizeFinalBytes;
    sgl::Logfile::get()->writeInfo(
            std::string() + "Fragment buffer size GiB (total): "
            + std::to_string(double(fragmentBufferSizeBytes) / 1024.0 / 1024.0 / 1024.0));

    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }
}

void OpacityOptimizationRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    sgl::vk::Device* device = renderer->getDevice();
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);
    viewportWidthOpacity = int(std::round(float(width) * opacityBufferScaleFactor));
    viewportHeightOpacity = int(std::round(float(height) * opacityBufferScaleFactor));
    paddedViewportWidthOpacity = viewportWidthOpacity;
    paddedViewportHeightOpacity = viewportHeightOpacity;
    getScreenSizeWithTiling(paddedViewportWidthOpacity, paddedViewportHeightOpacity);
    viewportWidthFinal = width;
    viewportHeightFinal = height;
    paddedViewportWidthFinal = viewportWidthFinal;
    paddedViewportHeightFinal = viewportHeightFinal;
    getScreenSizeWithTiling(paddedViewportWidthFinal, paddedViewportHeightFinal);

    reallocateFragmentBuffer();

    startOffsetBufferOpacities = {}; // Delete old data first (-> refcount 0)
    startOffsetBufferOpacities = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t) * paddedViewportWidthOpacity * paddedViewportHeightOpacity,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    fragmentCounterBufferOpacities = {}; // Delete old data first (-> refcount 0)
    fragmentCounterBufferOpacities = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    startOffsetBufferFinal = {}; // Delete old data first (-> refcount 0)
    startOffsetBufferFinal = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t) * paddedViewportWidthFinal * paddedViewportHeightFinal,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    fragmentCounterBufferFinal = {}; // Delete old data first (-> refcount 0)
    fragmentCounterBufferFinal = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (useMultisampling) {
        sgl::vk::ImageSettings imageSettings = (*sceneData->sceneTexture)->getImage()->getImageSettings();
        imageSettings.numSamples = VkSampleCountFlagBits(numSamples);
        imageSettings.usage =
                VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        colorRenderTargetImageFinal = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);
    } else {
        colorRenderTargetImageFinal = (*sceneData->sceneTexture)->getImageView();
    }

    if (viewportWidthOpacity == viewportWidthFinal && viewportHeightOpacity == viewportHeightFinal) {
        colorRenderTargetImageOpacities = (*sceneData->sceneTexture)->getImageView();
    } else {
        sgl::vk::ImageSettings imageSettings = (*sceneData->sceneTexture)->getImage()->getImageSettings();
        imageSettings.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        colorRenderTargetImageOpacities = std::make_shared<sgl::vk::ImageView>(
                std::make_shared<sgl::vk::Image>(device, imageSettings), VK_IMAGE_ASPECT_COLOR_BIT);
    }

    gatherPpllOpacitiesPass->recreateSwapchain(viewportWidthOpacity, viewportHeightOpacity);
    gatherPpllFinalPass->recreateSwapchain(viewportWidthFinal, viewportHeightFinal);

    clearPpllOpacitiesPass->setOutputImage(colorRenderTargetImageOpacities);
    clearPpllOpacitiesPass->recreateSwapchain(viewportWidthOpacity, viewportHeightOpacity);
    resolvePpllOpacitiesPass->setOutputImage(colorRenderTargetImageOpacities);
    resolvePpllOpacitiesPass->recreateSwapchain(viewportWidthOpacity, viewportHeightOpacity);
    clearPpllFinalPass->setOutputImage(colorRenderTargetImageFinal);
    clearPpllFinalPass->recreateSwapchain(viewportWidthFinal, viewportHeightFinal);
    resolvePpllFinalPass->setOutputImage(colorRenderTargetImageFinal);
    resolvePpllFinalPass->recreateSwapchain(viewportWidthFinal, viewportHeightFinal);

    onHasMoved();
}

void OpacityOptimizationRenderer::onClearColorChanged() {
    resolvePpllFinalPass->setAttachmentClearColor(sceneData->clearColor->getFloatColorRGBA());
}

void OpacityOptimizationRenderer::render() {
    LineRenderer::renderBase();

    if (numLineVertices == 0) {
        renderer->transitionImageLayout(
                (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        (*sceneData->sceneTexture)->getImageView()->clearColor(
                sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
        return;
    }

    setUniformData();
    isOpacitiesStep = true;
    clearPpllOpacities();
    gatherPpllOpacities();
    resolvePpllOpacities();
    convertPerSegmentOpacities();
    smoothPerSegmentOpacities();
    computePerVertexOpacities();
    isOpacitiesStep = false;
    clearPpllFinal();
    gatherPpllFinal();
    resolvePpllFinal();

    if (useMultisampling) {
        renderer->transitionImageLayout(
                colorRenderTargetImageFinal->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(
                (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        renderer->resolveImage(
                colorRenderTargetImageFinal, (*sceneData->sceneTexture)->getImageView());
    }
}

void OpacityOptimizationRenderer::setUniformData() {
    lineData->updateVulkanUniformBuffers(this, renderer);
    this->updateVulkanUniformBuffers();

    ppllUniformDataOpacities.viewportW = paddedViewportWidthOpacity;
    ppllUniformDataOpacities.linkedListSize = uint32_t(fragmentBufferSizeOpacity);
    ppllUniformDataBufferOpacities->updateData(
            sizeof(PpllUniformData), &ppllUniformDataOpacities,
            renderer->getVkCommandBuffer());

    ppllUniformDataFinal.viewportW = paddedViewportWidthFinal;
    ppllUniformDataFinal.linkedListSize = uint32_t(fragmentBufferSizeFinal);
    ppllUniformDataBufferFinal->updateData(
            sizeof(PpllUniformData), &ppllUniformDataFinal,
            renderer->getVkCommandBuffer());

    opacityOptimizationUniformDataBuffer->updateData(
            sizeof(OpacityOptimizationUniformData), &opacityOptimizationUniformData,
            renderer->getVkCommandBuffer());

    attributeRangeUniformData.minAttrValue = transferFunctionWindow.getDataRangeMin();
    attributeRangeUniformData.maxAttrValue = transferFunctionWindow.getDataRangeMax();
    attributeRangeUniformDataBuffer->updateData(
            sizeof(AttributeRangeUniformData), &attributeRangeUniformData,
            renderer->getVkCommandBuffer());

    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_VERTEX_SHADER_BIT);
}

void OpacityOptimizationRenderer::clearPpllOpacities() {
    clearPpllOpacitiesPass->render();
    fragmentCounterBufferOpacities->fill(0, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            startOffsetBufferOpacities);
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            fragmentCounterBufferOpacities);
}

void OpacityOptimizationRenderer::gatherPpllOpacities() {
    gatherPpllOpacitiesPass->buildIfNecessary();
    if (!gatherPpllOpacitiesPass->getIsDataEmpty()) {
        gatherPpllOpacitiesPass->render();
    }
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void OpacityOptimizationRenderer::resolvePpllOpacities() {
    // The segment opacity buffer stores the minimum fragment opacity, so initialize with maximum value.
    segmentOpacityUintBuffer->fill(
            std::numeric_limits<uint32_t>::max(), renderer->getVkCommandBuffer());
    segmentVisibilityBuffer->fill(0, renderer->getVkCommandBuffer());
    renderer->insertMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

    resolvePpllOpacitiesPass->render();
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
}

void OpacityOptimizationRenderer::convertPerSegmentOpacities() {
    convertPerSegmentOpacitiesPass->render();
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
}

void OpacityOptimizationRenderer::smoothPerSegmentOpacities() {
    for (int i = 0; i < opacityOptimizationUniformData.s; i++) {
        smoothPerSegmentOpacitiesPasses[i % 2]->render();

        segmentOpacityBufferIdx = (segmentOpacityBufferIdx + 1) % 2;
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
    }
}

void OpacityOptimizationRenderer::computePerVertexOpacities() {
    computePerVertexOpacitiesPass->render();
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_VERTEX_INPUT_BIT);

    segmentOpacityBufferIdx = 0;
}

void OpacityOptimizationRenderer::clearPpllFinal() {
    clearPpllFinalPass->render();
    fragmentCounterBufferFinal->fill(0, renderer->getVkCommandBuffer());
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            startOffsetBufferFinal);
    renderer->insertBufferMemoryBarrier(
            VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            fragmentCounterBufferFinal);
}

void OpacityOptimizationRenderer::gatherPpllFinal() {
    gatherPpllFinalPass->buildIfNecessary();
    if (!gatherPpllFinalPass->getIsDataEmpty()) {
        gatherPpllFinalPass->render();
    }
    renderer->insertMemoryBarrier(
            VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
}

void OpacityOptimizationRenderer::resolvePpllFinal() {
    resolvePpllFinalPass->render();
}


void OpacityOptimizationRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addSliderFloat(
            "q", &opacityOptimizationUniformData.q, 0.0f, 5000.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat(
            "r", &opacityOptimizationUniformData.r, 0.0f, 5000.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderInt(
            "s", &opacityOptimizationUniformData.s, 0, 20)) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat(
            "lambda", &opacityOptimizationUniformData.lambda, 0.1f, 20.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat(
            "rel", &opacityOptimizationUniformData.relaxationConstant,
            0.0f, 1.0f, "%.2f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat(
            "ts", &opacityOptimizationUniformData.temporalSmoothingFactor,
            0.0f, 1.0f, "%.2f")) {
        reRender = true;
        onHasMoved();
    }

    if (propertyEditor.addCombo(
            "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
        reloadResolveShader();
        reRender = true;
        onHasMoved();
    }

    if (maximumNumberOfSamples > 1) {
        if (propertyEditor.addCombo("Samples", &sampleModeSelection, sampleModeNames.data(), numSampleModes)) {
            numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
            useMultisampling = numSamples > 1;
            reloadResolveShader();
            onResolutionChanged();
            reRender = true;
        }
        if (useMultisampling && supportsSampleShadingRate) {
            if (propertyEditor.addCheckbox("Use Sample Shading", &useSamplingShading)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                onResolutionChanged();
                reRender = true;
            }
            if (propertyEditor.addSliderFloatEdit(
                    "Min. Sample Shading", &minSampleShading,
                    0.0f, 1.0f) == ImGui::EditMode::INPUT_FINISHED) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                useMultisampling = numSamples > 1;
                onResolutionChanged();
                reRender = true;
            }
        }
    }
}

bool OpacityOptimizationRenderer::needsReRender() {
    if (smoothingFramesCounter) {
        reRender = true;
    }
    return LineRenderer::needsReRender();
}

void OpacityOptimizationRenderer::onHasMoved() {
    LineRenderer::onHasMoved();
    smoothingFramesCounter = NUM_SMOOTHING_FRAMES;
}

void OpacityOptimizationRenderer::update(float dt) {
    if (smoothingFramesCounter > 0) {
        --smoothingFramesCounter;
    }
}



PpllOpacitiesLineRasterPass::PpllOpacitiesLineRasterPass(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
}

void PpllOpacitiesLineRasterPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    std::vector<std::string> shaderModuleNames;
    if (lineData->getUseBandRendering()) {
        shaderModuleNames = {
                "GeometryPassOpacitiesBand.VBO.Vertex",
                "GeometryPassOpacitiesBand.VBO.Geometry",
                "GeometryPassOpacitiesBand.Fragment"
        };
    } else {
        shaderModuleNames = {
                "GeometryPassOpacities.VBO.Vertex",
                "GeometryPassOpacities.VBO.Geometry",
                "GeometryPassOpacities.Fragment"
        };
    }

    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            shaderModuleNames, preprocessorDefines);
}

void PpllOpacitiesLineRasterPass::setOpacityOptimizationLineDataBuffers(
        const OpacityOptimizationLineDataBuffers& dataBuffers) {
    buffers = dataBuffers;
}

void PpllOpacitiesLineRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::LINE_LIST);

    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexPosition", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexAttribute", sizeof(float));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexTangent", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexOffsetLeft", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexOffsetRight", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexPrincipalStressIndex", sizeof(uint32_t));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexLineHierarchyLevel", sizeof(float));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexLineSegmentId", sizeof(uint32_t));

    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    if ((lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_TUBE_TRIANGLE_MESH && lineData->getUseCappedTubes())
        || (lineRenderer->getIsTransparencyUsed() && lineData->getLinePrimitiveMode() != LineData::LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER)) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }

    pipelineInfo.setColorWriteEnabled(false);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::OVERWRITE);
}

void PpllOpacitiesLineRasterPass::createRasterData(
        sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    lineRenderer->setRenderDataBindings(rasterData);

    if (!buffers.indexBuffer) {
        return;
    }

    rasterData->setIndexBuffer(buffers.indexBuffer);
    rasterData->setVertexBuffer(buffers.vertexPositionBuffer, "vertexPosition");
    rasterData->setVertexBuffer(buffers.vertexAttributeBuffer, "vertexAttribute");
    rasterData->setVertexBuffer(buffers.vertexTangentBuffer, "vertexTangent");
    rasterData->setVertexBufferOptional(buffers.vertexOffsetLeftBuffer, "vertexOffsetLeft");
    rasterData->setVertexBufferOptional(buffers.vertexOffsetRightBuffer, "vertexOffsetRight");
    rasterData->setVertexBufferOptional(
            buffers.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex");
    rasterData->setVertexBufferOptional(buffers.vertexLineHierarchyLevelBuffer, "vertexLineHierarchyLevel");
    rasterData->setVertexBufferOptional(buffers.lineSegmentIdBuffer, "vertexLineSegmentId");
}



PpllFinalLineRasterPass::PpllFinalLineRasterPass(LineRenderer* lineRenderer) : LineRasterPass(lineRenderer) {
}

void PpllFinalLineRasterPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineData->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    std::vector<std::string> shaderModuleNames;
    if (lineData->getUseBandRendering()) {
        shaderModuleNames = {
                "GeometryPassFinalBand.VBO.Vertex",
                "GeometryPassFinalBand.VBO.Geometry",
                "GeometryPassFinalBand.Fragment"
        };
    } else {
        shaderModuleNames = {
                "GeometryPassFinal.VBO.Vertex",
                "GeometryPassFinal.VBO.Geometry",
                "GeometryPassFinal.Fragment"
        };
    }

    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            shaderModuleNames, preprocessorDefines);
}

void PpllFinalLineRasterPass::setOpacityOptimizationLineDataBuffers(
        const OpacityOptimizationLineDataBuffers& dataBuffers) {
    buffers = dataBuffers;
}

void PpllFinalLineRasterPass::setGraphicsPipelineInfo(sgl::vk::GraphicsPipelineInfo& pipelineInfo) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::LINE_LIST);

    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexPosition", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexAttribute", sizeof(float));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional("vertexNormal", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexTangent", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexOffsetLeft", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexOffsetRight", sizeof(glm::vec3));
    pipelineInfo.setVertexBufferBindingByLocationIndexOptional(
            "vertexPrincipalStressIndex", sizeof(uint32_t));
    pipelineInfo.setVertexBufferBindingByLocationIndex("vertexOpacity", sizeof(float));

    lineRenderer->setGraphicsPipelineInfo(pipelineInfo, shaderStages);
    if ((lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_TUBE_TRIANGLE_MESH && lineData->getUseCappedTubes())
        || (lineRenderer->getIsTransparencyUsed() && lineData->getLinePrimitiveMode() != LineData::LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER)) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }

    pipelineInfo.setColorWriteEnabled(true);
    pipelineInfo.setBlendMode(sgl::vk::BlendMode::ONE);
}

void PpllFinalLineRasterPass::createRasterData(
        sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    lineData->setVulkanRenderDataDescriptors(rasterData);
    lineRenderer->setRenderDataBindings(rasterData);

    rasterData->setIndexBuffer(buffers.indexBuffer);
    rasterData->setVertexBuffer(buffers.vertexPositionBuffer, "vertexPosition");
    rasterData->setVertexBuffer(buffers.vertexAttributeBuffer, "vertexAttribute");
    rasterData->setVertexBufferOptional(buffers.vertexNormalBuffer, "vertexNormal");
    rasterData->setVertexBuffer(buffers.vertexTangentBuffer, "vertexTangent");
    rasterData->setVertexBufferOptional(buffers.vertexOffsetLeftBuffer, "vertexOffsetLeft");
    rasterData->setVertexBufferOptional(buffers.vertexOffsetRightBuffer, "vertexOffsetRight");
    rasterData->setVertexBufferOptional(
            buffers.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex");
    rasterData->setVertexBufferOptional(buffers.vertexOpacityBuffer, "vertexOpacity");
}



ConvertPerSegmentOpacitiesPass::ConvertPerSegmentOpacitiesPass(LineRenderer* lineRenderer)
        : ComputePass(*lineRenderer->getSceneData()->renderer), lineRenderer(lineRenderer) {
}

void ConvertPerSegmentOpacitiesPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "ConvertPerSegmentOpacities.Compute" }, preprocessorDefines);
}

void ConvertPerSegmentOpacitiesPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    lineRenderer->setRenderDataBindings(computeData);
    computeData->setStaticBuffer(segmentOpacityBuffer, "OpacityBufferFloat");
    computeData->setStaticBuffer(segmentOpacityUintBuffer, "OpacityBufferUint");
}

void ConvertPerSegmentOpacitiesPass::setSegmentOpacityBuffer(const sgl::vk::BufferPtr& buffer) {
    segmentOpacityBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(segmentOpacityBuffer, "OpacityBufferFloat");
    }
}

void ConvertPerSegmentOpacitiesPass::setSegmentOpacityUintBuffer(const sgl::vk::BufferPtr& buffer) {
    segmentOpacityUintBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(segmentOpacityUintBuffer, "OpacityBufferUint");
    }
}

void ConvertPerSegmentOpacitiesPass::_render() {
    groupCountX = sgl::iceil(int(numLineSegments), int(WORK_GROUP_SIZE_1D));
    renderer->pushConstants(
            getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT, 0, numLineSegments);
    ComputePass::_render();
}



SmoothPerSegmentOpacitiesPass::SmoothPerSegmentOpacitiesPass(LineRenderer* lineRenderer)
        : ComputePass(*lineRenderer->getSceneData()->renderer), lineRenderer(lineRenderer) {
}

void SmoothPerSegmentOpacitiesPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "LaplacianSmoothing.Compute" }, preprocessorDefines);
}

void SmoothPerSegmentOpacitiesPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    lineRenderer->setRenderDataBindings(computeData);
    computeData->setStaticBuffer(segmentOpacityBufferIn, "OpacityBufferIn");
    computeData->setStaticBuffer(segmentOpacityBufferOut, "OpacityBufferOut");
    computeData->setStaticBuffer(segmentVisibilityBuffer, "LineSegmentVisibilityBuffer");
    computeData->setStaticBuffer(lineSegmentConnectivityBuffer, "LineSegmentConnectivityBuffer");
}

void SmoothPerSegmentOpacitiesPass::setSegmentOpacityBufferIn(const sgl::vk::BufferPtr& buffer) {
    segmentOpacityBufferIn = buffer;
    if (computeData) {
        computeData->setStaticBuffer(segmentOpacityBufferIn, "OpacityBufferIn");
    }
}

void SmoothPerSegmentOpacitiesPass::setSegmentOpacityBufferOut(const sgl::vk::BufferPtr& buffer) {
    segmentOpacityBufferOut = buffer;
    if (computeData) {
        computeData->setStaticBuffer(segmentOpacityBufferOut, "OpacityBufferOut");
    }
}

void SmoothPerSegmentOpacitiesPass::setSegmentVisibilityBuffer(const sgl::vk::BufferPtr& buffer) {
    segmentVisibilityBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(segmentVisibilityBuffer, "LineSegmentVisibilityBuffer");
    }
}

void SmoothPerSegmentOpacitiesPass::setLineSegmentConnectivityBuffer(const sgl::vk::BufferPtr& buffer) {
    lineSegmentConnectivityBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(lineSegmentConnectivityBuffer, "LineSegmentConnectivityBuffer");
    }
}

void SmoothPerSegmentOpacitiesPass::_render() {
    groupCountX = sgl::iceil(int(numLineSegments), int(WORK_GROUP_SIZE_1D));
    renderer->pushConstants(
            getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT, 0, numLineSegments);
    ComputePass::_render();
}



ComputePerVertexOpacitiesPass::ComputePerVertexOpacitiesPass(LineRenderer* lineRenderer)
        : ComputePass(*lineRenderer->getSceneData()->renderer), lineRenderer(lineRenderer) {
}

void ComputePerVertexOpacitiesPass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
            { "ComputePerVertexOpacities.Compute" }, preprocessorDefines);
}

void ComputePerVertexOpacitiesPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    lineRenderer->setRenderDataBindings(computeData);
    computeData->setStaticBuffer(vertexOpacityBuffer, "OpacityBufferPerVertex");
    computeData->setStaticBuffer(segmentOpacityBuffer, "OpacityBufferPerSegment");
    computeData->setStaticBuffer(segmentVisibilityBuffer, "LineSegmentVisibilityBuffer");
    computeData->setStaticBuffer(blendingWeightParametrizationBuffer, "BlendingWeightParametrizationBuffer");
}

void ComputePerVertexOpacitiesPass::setVertexOpacityBuffer(const sgl::vk::BufferPtr& buffer) {
    vertexOpacityBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(vertexOpacityBuffer, "OpacityBufferPerVertex");
    }
}

void ComputePerVertexOpacitiesPass::setSegmentOpacityBuffer(const sgl::vk::BufferPtr& buffer) {
    segmentOpacityBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(segmentOpacityBuffer, "OpacityBufferPerSegment");
    }
}

void ComputePerVertexOpacitiesPass::setSegmentVisibilityBuffer(const sgl::vk::BufferPtr& buffer) {
    segmentVisibilityBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(segmentVisibilityBuffer, "LineSegmentVisibilityBuffer");
    }
}

void ComputePerVertexOpacitiesPass::setBlendingWeightParametrizationBuffer(const sgl::vk::BufferPtr& buffer) {
    blendingWeightParametrizationBuffer = buffer;
    if (computeData) {
        computeData->setStaticBuffer(blendingWeightParametrizationBuffer, "BlendingWeightParametrizationBuffer");
    }
}

void ComputePerVertexOpacitiesPass::_render() {
    groupCountX = sgl::iceil(int(numLineVertices), int(WORK_GROUP_SIZE_1D));
    renderer->pushConstants(
            getComputePipeline(), VK_SHADER_STAGE_COMPUTE_BIT, 0, numLineVertices);
    ComputePass::_render();
}
