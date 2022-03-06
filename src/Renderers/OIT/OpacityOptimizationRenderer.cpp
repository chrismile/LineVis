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

/*OpacityOptimizationRenderer::OpacityOptimizationRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opacity Optimization Renderer", sceneData, transferFunctionWindow) {
    // Add events for interaction with line data.
    onOpacityEstimationRecomputeListenerToken = sgl::EventManager::get()->addListener(
            ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT, [this](const sgl::EventPtr&) {
                this->onHasMoved();
            });
}

void OpacityOptimizationRenderer::initialize() {
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

    reloadGatherShader();
    reloadResolveShader();
    clearPpllOpacitiesShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClearOpacities.Vertex", "LinkedListClearOpacities.Fragment"});
    clearPpllFinalShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListClearFinal.Vertex", "LinkedListClearFinal.Fragment"});

    convertPerSegmentOpacitiesShader = sgl::ShaderManager->getShaderProgram(
            {"ConvertPerSegmentOpacities.Compute"});
    smoothPerSegmentOpacitiesShader = sgl::ShaderManager->getShaderProgram(
            {"LaplacianSmoothing.Compute"});
    computePerVertexOpacitiesShader = sgl::ShaderManager->getShaderProgram(
            {"ComputePerVertexOpacities.Compute"});

    onClearColorChanged();
}

OpacityOptimizationRenderer::~OpacityOptimizationRenderer() {
    sgl::EventManager::get()->removeListener(
            ON_OPACITY_OPTIMIZATION_RECOMPUTE_EVENT, onOpacityEstimationRecomputeListenerToken);
}

void OpacityOptimizationRenderer::reloadResolveShader() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_FRAGS", sgl::toString(expectedMaxDepthComplexity));

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
        || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        int stackSize = int(std::ceil(std::log2(expectedMaxDepthComplexity)) * 2 + 4);
        sgl::ShaderManager->addPreprocessorDefine("STACK_SIZE", sgl::toString(stackSize));
    }

    if (useMultisampling) {
        sgl::ShaderManager->addPreprocessorDefine("USE_COVERAGE_MASK", "");
    }

    resolvePpllOpacitiesShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolveOpacities.Vertex", "LinkedListResolveOpacities.Fragment"});
    if (resolvePpllOpacitiesRenderData) {
        resolvePpllOpacitiesRenderData = resolvePpllOpacitiesRenderData->copy(resolvePpllOpacitiesShader);
    }

    resolvePpllFinalShader = sgl::ShaderManager->getShaderProgram(
            {"LinkedListResolveFinal.Vertex", "LinkedListResolveFinal.Fragment"});
    if (resolvePpllFinalRenderData) {
        resolvePpllFinalRenderData = resolvePpllFinalRenderData->copy(resolvePpllFinalShader);
    }

    if (useMultisampling) {
        sgl::ShaderManager->removePreprocessorDefine("USE_COVERAGE_MASK");
    }
}

void OpacityOptimizationRenderer::reloadGatherShader() {
    sgl::ShaderManager->invalidateShaderCache();

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
        sgl::ShaderManager->addPreprocessorDefine("IS_PSL_DATA", "");
    }
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->addPreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX", "");
    }
    if (useLineHierarchy) {
        sgl::ShaderManager->addPreprocessorDefine("USE_LINE_HIERARCHY_LEVEL", "");
    }
    sgl::ShaderManager->addPreprocessorDefine("USE_TRANSPARENCY", "");

    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\"");
    LineRenderer::reloadGatherShader();
    sgl::ShaderManager->removePreprocessorDefine("OIT_GATHER_HEADER");

    if (lineData->getUseBandRendering()) {
        gatherPpllOpacitiesShader = sgl::ShaderManager->getShaderProgram({
                "GeometryPassOpacitiesBand.VBO.Vertex",
                "GeometryPassOpacitiesBand.VBO.Geometry",
                "GeometryPassOpacitiesBand.Fragment"
        });
        gatherPpllFinalShader = sgl::ShaderManager->getShaderProgram({
                "GeometryPassFinalBand.VBO.Vertex",
                "GeometryPassFinalBand.VBO.Geometry",
                "GeometryPassFinalBand.Fragment"
        });
    } else {
        gatherPpllOpacitiesShader = sgl::ShaderManager->getShaderProgram({
                "GeometryPassOpacities.VBO.Vertex",
                "GeometryPassOpacities.VBO.Geometry",
                "GeometryPassOpacities.Fragment"
        });
        gatherPpllFinalShader = sgl::ShaderManager->getShaderProgram({
                "GeometryPassFinal.VBO.Vertex",
                "GeometryPassFinal.VBO.Geometry",
                "GeometryPassFinal.Fragment"
        });
    }

    sgl::ShaderManager->removePreprocessorDefine("USE_TRANSPARENCY");
    if (useLineHierarchy) {
        sgl::ShaderManager->removePreprocessorDefine("USE_LINE_HIERARCHY_LEVEL");
    }
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->removePreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX");
    }
    if (isStressLineData) {
        sgl::ShaderManager->removePreprocessorDefine("IS_PSL_DATA");
    }

    if (gatherPpllOpacitiesRenderData) {
        gatherPpllOpacitiesRenderData = gatherPpllOpacitiesRenderData->copy(gatherPpllOpacitiesShader);
    }
    if (gatherPpllFinalRenderData) {
        gatherPpllFinalRenderData = gatherPpllFinalRenderData->copy(gatherPpllFinalShader);
    }
}

void OpacityOptimizationRenderer::setSortingAlgorithmDefine() {
    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_PRIORITY_QUEUE) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "frontToBackPQ");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BUBBLE_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "bubbleSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_INSERTION_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "insertionSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_SHELL_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "shellSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_MAX_HEAP) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "heapSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_BITONIC_SORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "bitonicSort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "quicksort");
    } else if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        sgl::ShaderManager->addPreprocessorDefine("sortingAlgorithm", "quicksortHybrid");
    }

    if (sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT
        || sortingAlgorithmMode == SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID) {
        sgl::ShaderManager->addPreprocessorDefine("USE_QUICKSORT", "");
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_QUICKSORT");
    }
}

void OpacityOptimizationRenderer::setNewState(const InternalState& newState) {
    currentStateName = newState.name;
    timerDataIsWritten = false;
    if ((*sceneData->performanceMeasurer) && !timerDataIsWritten) {
        if (timer) {
            delete timer;
            timer = nullptr;
        }
        timer = new sgl::TimerGL;
        (*sceneData->performanceMeasurer)->setPpllTimer(timer);
    }
}

void OpacityOptimizationRenderer::updateLargeMeshMode() {
    // More than one million cells?
    LargeMeshMode newMeshLargeMeshMode = MESH_SIZE_MEDIUM;
    if (lineData->getNumLineSegments() > 1e6) { // > 1m line segments
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

    // Unload old data.
    gatherPpllOpacitiesRenderData = sgl::ShaderAttributesPtr();
    gatherPpllFinalRenderData = sgl::ShaderAttributesPtr();
    updateLargeMeshMode();

    lines = lineData->getFilteredLines(this);

    sgl::vk::BufferPtr indexBuffer;
    sgl::vk::BufferPtr vertexPositionBuffer;
    sgl::vk::BufferPtr vertexAttributeBuffer;
    sgl::vk::BufferPtr vertexNormalBuffer;
    sgl::vk::BufferPtr vertexTangentBuffer;
    sgl::vk::BufferPtr vertexOffsetLeftBuffer;
    sgl::vk::BufferPtr vertexOffsetRightBuffer;
    sgl::vk::BufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
    sgl::vk::BufferPtr vertexLineHierarchyLevelBuffer; ///< Empty for flow lines.

    if (lineData->getUseBandRendering()) {
        BandRenderData tubeRenderData = lineData->getBandRenderData();
        indexBuffer = tubeRenderData.indexBuffer;
        vertexPositionBuffer = tubeRenderData.vertexPositionBuffer;
        vertexAttributeBuffer = tubeRenderData.vertexAttributeBuffer;
        vertexNormalBuffer = tubeRenderData.vertexNormalBuffer;
        vertexTangentBuffer = tubeRenderData.vertexTangentBuffer;
        vertexOffsetLeftBuffer = tubeRenderData.vertexOffsetLeftBuffer;
        vertexOffsetRightBuffer = tubeRenderData.vertexOffsetRightBuffer;
        vertexPrincipalStressIndexBuffer = tubeRenderData.vertexPrincipalStressIndexBuffer;
        vertexLineHierarchyLevelBuffer = tubeRenderData.vertexLineHierarchyLevelBuffer;
    } else {
        TubeRenderDataOpacityOptimization tubeRenderData = lineData->getTubeRenderDataOpacityOptimization();
        indexBuffer = tubeRenderData.indexBuffer;
        vertexPositionBuffer = tubeRenderData.vertexPositionBuffer;
        vertexAttributeBuffer = tubeRenderData.vertexAttributeBuffer;
        vertexTangentBuffer = tubeRenderData.vertexTangentBuffer;
        vertexPrincipalStressIndexBuffer = tubeRenderData.vertexPrincipalStressIndexBuffer;
        vertexLineHierarchyLevelBuffer = tubeRenderData.vertexLineHierarchyLevelBuffer;
    }

    gatherPpllOpacitiesRenderData = sgl::ShaderManager->createShaderAttributes(gatherPpllOpacitiesShader);
    gatherPpllFinalRenderData = sgl::ShaderManager->createShaderAttributes(gatherPpllFinalShader);

    numLineVertices = uint32_t(vertexPositionBuffer->getSize() / sizeof(glm::vec3));
    generateBlendingWeightParametrization(isNewData);

    vertexOpacityBuffer = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), numLineVertices * sizeof(float),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    vertexOpacityBuffer->fill(0, renderer->getVkCommandBuffer());
    // TODO: Barrier.

    gatherPpllOpacitiesRenderData->setVertexMode(sgl::VERTEX_MODE_LINES);
    gatherPpllOpacitiesRenderData->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    gatherPpllOpacitiesRenderData->addGeometryBuffer(
            vertexPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    gatherPpllOpacitiesRenderData->addGeometryBufferOptional(
            vertexAttributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);
    gatherPpllOpacitiesRenderData->addGeometryBuffer(
            vertexTangentBuffer, "vertexTangent", sgl::ATTRIB_FLOAT, 3);
    if (vertexOffsetLeftBuffer) {
        gatherPpllOpacitiesRenderData->addGeometryBuffer(
                vertexOffsetLeftBuffer, "vertexOffsetLeft", sgl::ATTRIB_FLOAT, 3);
    }
    if (vertexOffsetRightBuffer) {
        gatherPpllOpacitiesRenderData->addGeometryBuffer(
                vertexOffsetRightBuffer, "vertexOffsetRight", sgl::ATTRIB_FLOAT, 3);
    }
    if (vertexPrincipalStressIndexBuffer) {
        gatherPpllOpacitiesRenderData->addGeometryBufferOptional(
                vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex",
                sgl::ATTRIB_UNSIGNED_INT,
                1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);
    }
    if (vertexLineHierarchyLevelBuffer) {
        gatherPpllOpacitiesRenderData->addGeometryBufferOptional(
                vertexLineHierarchyLevelBuffer, "vertexLineHierarchyLevel",
                sgl::ATTRIB_FLOAT, 1);
    }
    gatherPpllOpacitiesRenderData->addGeometryBuffer(
            this->lineSegmentIdBuffer, "vertexLineSegmentId", sgl::ATTRIB_UNSIGNED_INT,
            1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);

    gatherPpllFinalRenderData->setVertexMode(sgl::VERTEX_MODE_LINES);
    gatherPpllFinalRenderData->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    gatherPpllFinalRenderData->addGeometryBuffer(
            vertexPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    gatherPpllFinalRenderData->addGeometryBufferOptional(
            vertexAttributeBuffer, "vertexAttribute", sgl::ATTRIB_FLOAT, 1);
    if (vertexNormalBuffer) {
        gatherPpllFinalRenderData->addGeometryBuffer(
                vertexNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);
    }
    gatherPpllFinalRenderData->addGeometryBuffer(
            vertexTangentBuffer, "vertexTangent", sgl::ATTRIB_FLOAT, 3);
    if (vertexOffsetLeftBuffer) {
        gatherPpllFinalRenderData->addGeometryBuffer(
                vertexOffsetLeftBuffer, "vertexOffsetLeft", sgl::ATTRIB_FLOAT, 3);
    }
    if (vertexOffsetRightBuffer) {
        gatherPpllFinalRenderData->addGeometryBuffer(
                vertexOffsetRightBuffer, "vertexOffsetRight", sgl::ATTRIB_FLOAT, 3);
    }
    gatherPpllFinalRenderData->addGeometryBuffer(
            this->vertexOpacityBuffer, "vertexOpacity", sgl::ATTRIB_FLOAT, 1);
    if (vertexPrincipalStressIndexBuffer) {
        gatherPpllFinalRenderData->addGeometryBufferOptional(
                vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex",
                sgl::ATTRIB_UNSIGNED_INT,
                1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);
    }

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
}


void OpacityOptimizationRenderer::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string> &preprocessorDefines) {
    LineRenderer::getVulkanShaderPreprocessorDefines(preprocessorDefines);
    preprocessorDefines.insert(std::make_pair("OIT_GATHER_HEADER", "\"LinkedListGather.glsl\""));

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
}

void OpacityOptimizationRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    LineRenderer::setRenderDataBindings(renderData);
    renderData->setStaticBufferOptional(fragmentBuffer, "FragmentBuffer");
    renderData->setStaticBuffer(startOffsetBuffer, "StartOffsetBuffer");
    renderData->setStaticBufferOptional(fragmentCounterBuffer, "FragCounterBuffer");
    renderData->setStaticBufferOptional(uniformDataBuffer, "UniformDataBuffer");
}

void OpacityOptimizationRenderer::updateVulkanUniformBuffers() {
}

void OpacityOptimizationRenderer::setFramebufferAttachments(
        sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachmentState.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachmentState.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachmentState.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    framebuffer->setColorAttachment(
            (*sceneData->sceneTexture)->getImageView(), 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());
}

void OpacityOptimizationRenderer::reallocateFragmentBuffer() {
    // Fragment buffer for opacities.
    fragmentBufferSizeOpacity = size_t(expectedAvgDepthComplexity) * size_t(viewportWidthOpacity) * size_t(viewportHeightOpacity);
    size_t fragmentBufferSizeOpacityBytes = 12ull * fragmentBufferSizeOpacity;
    if (fragmentBufferSizeOpacityBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
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
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
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
            + std::to_string(fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));

    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }
}

void OpacityOptimizationRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

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

    atomicCounterBufferOpacities = {}; // Delete old data first (-> refcount 0)
    atomicCounterBufferOpacities = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    startOffsetBufferFinal = {}; // Delete old data first (-> refcount 0)
    startOffsetBufferOpacities = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t) * paddedViewportWidthFinal * paddedViewportHeightFinal,
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    atomicCounterBufferFinal = {}; // Delete old data first (-> refcount 0)
    atomicCounterBufferFinal = std::make_shared<sgl::vk::Buffer>(
            renderer->getDevice(), sizeof(uint32_t),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (useMultisampling) {
        msaaSceneFBO = sgl::Renderer->createFBO();
        //msaaRenderTexture = sgl::TextureManager->createMultisampledTexture(
        //        viewportWidthFinal, viewportHeightFinal, numSamples,
        //        (*sceneData->sceneTexture)->getSettings().internalFormat, true); // TODO
        msaaDepthRBO = sgl::Renderer->createRBO(
                viewportWidthFinal, viewportHeightFinal, sgl::RBO_DEPTH24_STENCIL8, numSamples);
        msaaSceneFBO->bindTexture(msaaRenderTexture);
        msaaSceneFBO->bindRenderbuffer(msaaDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);
    } else {
        msaaSceneFBO = sgl::FramebufferObjectPtr();
        msaaRenderTexture = sgl::TexturePtr();
        msaaDepthRBO = sgl::RenderbufferObjectPtr();
    }

    onHasMoved();
}

void OpacityOptimizationRenderer::render() {
    LineRenderer::renderBase();

    setUniformData();
    clearPpllOpacities();
    gatherPpllOpacities();
    resolvePpllOpacities();
    convertPerSegmentOpacities();
    smoothPerSegmentOpacities();
    computePerVertexOpacities();
    clearPpllFinal();
    gatherPpllFinal();
    resolvePpllFinal();
}

void OpacityOptimizationRenderer::setUniformData() {
    if (lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
        lineData->setUniformGatherShaderData_AllPasses();
    }

    gatherPpllOpacitiesShader->setUniform("viewportW", paddedViewportWidthOpacity);
    gatherPpllOpacitiesShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSizeOpacity);
    gatherPpllOpacitiesShader->setUniform("cameraPosition", sceneData->camera->getPosition());
    gatherPpllOpacitiesShader->setUniform("lineWidth", lineWidth);
    gatherPpllOpacitiesShader->setUniformOptional("minAttrValue", transferFunctionWindow.getDataRangeMin());
    gatherPpllOpacitiesShader->setUniformOptional("maxAttrValue", transferFunctionWindow.getDataRangeMax());
    lineData->setUniformGatherShaderData_Pass(gatherPpllOpacitiesShader);
    setUniformData_Pass(gatherPpllOpacitiesShader);

    gatherPpllFinalShader->setUniform("viewportW", paddedViewportWidthFinal);
    gatherPpllFinalShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSizeFinal);
    gatherPpllFinalShader->setUniform("cameraPosition", sceneData->camera->getPosition());
    gatherPpllFinalShader->setUniform("lineWidth", lineWidth);
    if (gatherPpllFinalShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        gatherPpllFinalShader->setUniform("backgroundColor", backgroundColor);
    }
    if (gatherPpllFinalShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        gatherPpllFinalShader->setUniform("foregroundColor", foregroundColor);
    }
    lineData->setUniformGatherShaderData_Pass(gatherPpllFinalShader);
    setUniformData_Pass(gatherPpllFinalShader);

    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        gatherShaderHull->setUniform("viewportW", paddedViewportWidthFinal);
        gatherShaderHull->setUniform("linkedListSize", (unsigned int)fragmentBufferSizeFinal);
    }

    resolvePpllOpacitiesShader->setUniform("viewportW", paddedViewportWidthOpacity);
    resolvePpllOpacitiesShader->setUniform("q", q);
    resolvePpllOpacitiesShader->setUniform("r", r);
    //resolvePpllOpacitiesShader->setUniform("s", s);
    resolvePpllOpacitiesShader->setUniform("lambda", lambda);

    resolvePpllFinalShader->setUniform("viewportW", paddedViewportWidthFinal);

    clearPpllOpacitiesShader->setUniform("viewportW", paddedViewportWidthOpacity);
    clearPpllFinalShader->setUniform("viewportW", paddedViewportWidthFinal);

    convertPerSegmentOpacitiesShader->setUniform("numLineSegments", (unsigned int)numLineSegments);

    smoothPerSegmentOpacitiesShader->setUniform("numLineSegments", (unsigned int)numLineSegments);
    smoothPerSegmentOpacitiesShader->setUniform("relaxationConstant", relaxationConstant);

    computePerVertexOpacitiesShader->setUniform("numLineVertices", (unsigned int)numLineVertices);
    computePerVertexOpacitiesShader->setUniform("temporalSmoothingFactor", temporalSmoothingFactor);
}

void OpacityOptimizationRenderer::clearPpllOpacities() {
    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBufferOpacities);
    sgl::ShaderManager->bindShaderStorageBuffer(1, startOffsetBufferOpacities);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBufferOpacities);

    glDepthMask(GL_FALSE);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glViewport(0, 0, viewportWidthOpacity, viewportHeightOpacity);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(clearPpllOpacitiesRenderData);

    // Set atomic counter to zero.
    GLuint bufferId = static_cast<sgl::GeometryBufferGL*>(atomicCounterBufferOpacities.get())->getBuffer();
    uint32_t val = 0;
    glClearNamedBufferData(bufferId, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void OpacityOptimizationRenderer::gatherPpllOpacities() {
    // Enable the depth test, but disable depth write for gathering.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // We can use the stencil buffer to mask used pixels for the resolve pass.
    if (useStencilBuffer) {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);
        glClear(GL_STENCIL_BUFFER_BIT);
    }

    sgl::Renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(gatherPpllOpacitiesRenderData);
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void OpacityOptimizationRenderer::resolvePpllOpacities() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);

    if (useStencilBuffer) {
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilMask(0x00);
    }

    // The segment opacity buffer stores the minimum fragment opacity, so initialize with maximum value.
    sgl::ShaderManager->bindShaderStorageBuffer(3, segmentOpacityUintBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(4, segmentVisibilityBuffer);
    GLuint opacityBufferId = static_cast<sgl::GeometryBufferGL*>(segmentOpacityUintBuffer.get())->getBuffer();
    uint32_t opacityClearVal = std::numeric_limits<uint32_t>::max();//0xFFFFFFFFu;
    glClearNamedBufferData(
            opacityBufferId, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&opacityClearVal);
    GLuint visibilityBufferId = static_cast<sgl::GeometryBufferGL*>(segmentVisibilityBuffer.get())->getBuffer();
    uint32_t visibilityClearVal = 0u;
    glClearNamedBufferData(
            visibilityBufferId, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&visibilityClearVal);

    sgl::Renderer->render(resolvePpllOpacitiesRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}

void OpacityOptimizationRenderer::convertPerSegmentOpacities() {
    sgl::ShaderManager->bindShaderStorageBuffer(2, segmentOpacityBuffers[segmentOpacityBufferIdx]);
    // Already bound by @see resolvePpllOpacities.
    //sgl::ShaderManager->bindShaderStorageBuffer(3, segmentOpacityUintBuffer);

    const uint32_t WORK_GROUP_SIZE_1D = 64;
    uint32_t numWorkGroups = sgl::iceil(uint32_t(numLineSegments), WORK_GROUP_SIZE_1D);
    convertPerSegmentOpacitiesShader->dispatchCompute(numWorkGroups);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void OpacityOptimizationRenderer::smoothPerSegmentOpacities() {
    // Already bound by @see resolvePpllOpacities.
    //sgl::ShaderManager->bindShaderStorageBuffer(4, segmentVisibilityBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(5, lineSegmentConnectivityBuffer);
    const uint32_t WORK_GROUP_SIZE_1D = 64;
    uint32_t numWorkGroups = sgl::iceil(uint32_t(numLineSegments), WORK_GROUP_SIZE_1D);

    for (int i = 0; i < s; i++) {
        if (i != 0) {
            // Already bound by @see convertPerSegmentOpacities if i == 0.
            sgl::ShaderManager->bindShaderStorageBuffer(2, segmentOpacityBuffers[segmentOpacityBufferIdx]);
        }
        sgl::ShaderManager->bindShaderStorageBuffer(3, segmentOpacityBuffers[(segmentOpacityBufferIdx + 1) % 2]);

        smoothPerSegmentOpacitiesShader->dispatchCompute(numWorkGroups);

        segmentOpacityBufferIdx = (segmentOpacityBufferIdx + 1) % 2;
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }

    if (s == 0) {
        sgl::ShaderManager->bindShaderStorageBuffer(3, segmentOpacityBuffers[segmentOpacityBufferIdx]);
    }
}

void OpacityOptimizationRenderer::computePerVertexOpacities() {
    sgl::ShaderManager->bindShaderStorageBuffer(2, vertexOpacityBuffer);
    // Already bound by @see smoothPerSegmentOpacities.
    //sgl::ShaderManager->bindShaderStorageBuffer(3, segmentOpacityBuffers[segmentOpacityBufferIdx]);
    // Already bound by @see resolvePpllOpacities.
    //sgl::ShaderManager->bindShaderStorageBuffer(4, segmentVisibilityBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(5, blendingWeightParametrizationBuffer);

    const uint32_t WORK_GROUP_SIZE_1D = 64;
    uint32_t numWorkGroups = sgl::iceil(uint32_t(numLineVertices), WORK_GROUP_SIZE_1D);
    computePerVertexOpacitiesShader->dispatchCompute(numWorkGroups);
    glMemoryBarrier(GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);

    segmentOpacityBufferIdx = 0;
}

void OpacityOptimizationRenderer::clearPpllFinal() {
    sgl::ShaderManager->bindShaderStorageBuffer(0, fragmentBufferFinal);
    sgl::ShaderManager->bindShaderStorageBuffer(1, startOffsetBufferFinal);
    sgl::ShaderManager->bindAtomicCounterBuffer(0, atomicCounterBufferFinal);

    glDepthMask(GL_FALSE);

    // In the clear and gather pass, we just want to write data to an SSBO.
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glViewport(0, 0, viewportWidthFinal, viewportHeightFinal);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    sgl::Renderer->render(clearPpllFinalRenderData);

    // Set atomic counter to zero.
    GLuint bufferId = static_cast<sgl::GeometryBufferGL*>(atomicCounterBufferFinal.get())->getBuffer();
    uint32_t val = 0;
    glClearNamedBufferData(bufferId, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&val);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_ATOMIC_COUNTER_BARRIER_BIT);
}

void OpacityOptimizationRenderer::gatherPpllFinal() {
    // Enable the depth test, but disable depth write for gathering.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // We can use the stencil buffer to mask used pixels for the resolve pass.
    if (useStencilBuffer) {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);
        glClear(GL_STENCIL_BUFFER_BIT);
    }

    sgl::Renderer->setProjectionMatrix(sceneData->camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData->camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    if (useMultisampling) {
        sgl::Renderer->bindFBO(msaaSceneFBO);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, *sceneData->clearColor);
    }

    // Now, the final gather step.
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }
    sgl::Renderer->render(gatherPpllFinalRenderData);
    renderHull();
    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    if (useMultisampling) {
        //sgl::Renderer->bindFBO(*sceneData->framebuffer); // TODO
    }
}

void OpacityOptimizationRenderer::resolvePpllFinal() {
    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);

    if (useStencilBuffer) {
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilMask(0x00);
    }

    sgl::Renderer->render(resolvePpllFinalRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glDisable(GL_STENCIL_TEST);
    glDepthMask(GL_TRUE);
}


void OpacityOptimizationRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    if (propertyEditor.addSliderFloat("q", &q, 0.0f, 5000.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat("r", &r, 0.0f, 5000.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderInt("s", &s, 0, 20)) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat("lambda", &lambda, 0.1f, 20.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat("rel", &relaxationConstant, 0.0f, 1.0f, "%.2f")) {
        reRender = true;
        onHasMoved();
    }
    if (propertyEditor.addSliderFloat("ts", &temporalSmoothingFactor, 0.0f, 1.0f, "%.2f")) {
        reRender = true;
        onHasMoved();
    }

    if (propertyEditor.addCombo(
            "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
        setSortingAlgorithmDefine();
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
}*/
