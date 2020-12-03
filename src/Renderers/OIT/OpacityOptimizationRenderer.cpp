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
#include <Graphics/Renderer.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <Graphics/OpenGL/Shader.hpp>
#include <Utils/AppSettings.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include <ImGui/imgui_custom.h>

#include "Utils/InternalState.hpp"
#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "LineData/LineDataStress.hpp"
#include "Loaders/TrajectoryFile.hpp"
#include "TilingMode.hpp"
#include "OpacityOptimizationRenderer.hpp"

// Use stencil buffer to mask unused pixels
static bool useStencilBuffer = false;

OpacityOptimizationRenderer::OpacityOptimizationRenderer(SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Opacity Optimization Renderer", sceneData, transferFunctionWindow) {
    sgl::ShaderManager->invalidateShaderCache();
    setSortingAlgorithmDefine();

    // Get all available multisampling modes.
    glGetIntegerv(GL_MAX_SAMPLES, &maximumNumberOfSamples);
    if (maximumNumberOfSamples <= 1) {
        useMultisampling = false;
    }
    numSampleModes = sgl::intlog2(maximumNumberOfSamples) + 1;
    sampleModeSelection = std::min(sgl::intlog2(4), numSampleModes - 1);
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

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), (void*)&fullscreenQuad.front());

    resolvePpllOpacitiesRenderData = sgl::ShaderManager->createShaderAttributes(resolvePpllOpacitiesShader);
    resolvePpllOpacitiesRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    resolvePpllFinalRenderData = sgl::ShaderManager->createShaderAttributes(resolvePpllFinalShader);
    resolvePpllFinalRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    clearPpllOpacitiesRenderData = sgl::ShaderManager->createShaderAttributes(clearPpllOpacitiesShader);
    clearPpllOpacitiesRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    clearPpllFinalRenderData = sgl::ShaderManager->createShaderAttributes(clearPpllFinalShader);
    clearPpllFinalRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    onResolutionChanged();
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

void OpacityOptimizationRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    sgl::ShaderManager->invalidateShaderCache();

    bool usePrincipalStressDirectionIndex = false;
    bool useLineHierarchy = false;
    if (lineData && lineData->getType() == DATA_SET_TYPE_STRESS_LINES) {
        LineDataStress* lineDataStress = static_cast<LineDataStress*>(lineData.get());
        usePrincipalStressDirectionIndex = lineDataStress->getUsePrincipalStressDirectionIndex();
        useLineHierarchy = lineDataStress->getUseLineHierarchy();
    }
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->addPreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX", "");
    }
    if (useLineHierarchy) {
        sgl::ShaderManager->addPreprocessorDefine("USE_LINE_HIERARCHY_LEVEL", "");
    }
    sgl::ShaderManager->addPreprocessorDefine("USE_TRANSPARENCY", "");
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
    sgl::ShaderManager->removePreprocessorDefine("USE_TRANSPARENCY");
    if (useLineHierarchy) {
        sgl::ShaderManager->removePreprocessorDefine("USE_LINE_HIERARCHY_LEVEL");
    }
    if (usePrincipalStressDirectionIndex) {
        sgl::ShaderManager->removePreprocessorDefine("USE_PRINCIPAL_STRESS_DIRECTION_INDEX");
    }

    if (canCopyShaderAttributes && gatherPpllOpacitiesRenderData) {
        gatherPpllOpacitiesRenderData = gatherPpllOpacitiesRenderData->copy(gatherPpllOpacitiesShader);
    }
    if (canCopyShaderAttributes && gatherPpllFinalRenderData) {
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
    if (sceneData.performanceMeasurer && !timerDataIsWritten) {
        if (timer) {
            delete timer;
        }
        timer = new sgl::TimerGL;
        sceneData.performanceMeasurer->setPpllTimer(timer);
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
        expectedAvgDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][0];
        expectedMaxDepthComplexity = MESH_MODE_DEPTH_COMPLEXITIES[int(largeMeshMode)][1];
        reallocateFragmentBuffer();
        reloadResolveShader();
    }
}

void OpacityOptimizationRenderer::setLineData(LineDataPtr& lineData, bool isNewMesh) {
    if (!this->lineData || lineData->getType() != this->lineData->getType()) {
        this->lineData = lineData;
        reloadGatherShader(false);
    }
    this->lineData = lineData;

    // Unload old data.
    gatherPpllOpacitiesRenderData = sgl::ShaderAttributesPtr();
    gatherPpllFinalRenderData = sgl::ShaderAttributesPtr();
    updateLargeMeshMode();

    TubeRenderDataOpacityOptimization tubeRenderData = lineData->getTubeRenderDataOpacityOptimization();
    lines = lineData->getFilteredLines();

    gatherPpllOpacitiesRenderData = sgl::ShaderManager->createShaderAttributes(gatherPpllOpacitiesShader);
    gatherPpllFinalRenderData = sgl::ShaderManager->createShaderAttributes(gatherPpllFinalShader);

    numLineVertices = tubeRenderData.vertexPositionBuffer->getSize() / sizeof(glm::vec3);
    generateBlendingWeightParametrization(isNewMesh);

    vertexOpacityBuffer = sgl::Renderer->createGeometryBuffer(
            numLineVertices * sizeof(float), nullptr, sgl::VERTEX_BUFFER);
    GLuint bufferId = static_cast<sgl::GeometryBufferGL*>(vertexOpacityBuffer.get())->getBuffer();
    float clearVal = 0.0f;
    glClearNamedBufferData(bufferId, GL_R32F, GL_RED, GL_FLOAT, (const void*)&clearVal);

    gatherPpllOpacitiesRenderData->setVertexMode(sgl::VERTEX_MODE_LINES);
    gatherPpllOpacitiesRenderData->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    gatherPpllOpacitiesRenderData->addGeometryBuffer(
            tubeRenderData.vertexPositionBuffer, "vertexPosition",
            sgl::ATTRIB_FLOAT, 3);
    gatherPpllOpacitiesRenderData->addGeometryBufferOptional(
            tubeRenderData.vertexAttributeBuffer, "vertexAttribute",
            sgl::ATTRIB_FLOAT, 1);
    gatherPpllOpacitiesRenderData->addGeometryBuffer(
            tubeRenderData.vertexTangentBuffer, "vertexTangent",
            sgl::ATTRIB_FLOAT, 3);
    if (tubeRenderData.vertexPrincipalStressIndexBuffer) {
        gatherPpllOpacitiesRenderData->addGeometryBufferOptional(
                tubeRenderData.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex",
                sgl::ATTRIB_UNSIGNED_INT,
                1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);
    }
    if (tubeRenderData.vertexLineHierarchyLevelBuffer) {
        gatherPpllOpacitiesRenderData->addGeometryBufferOptional(
                tubeRenderData.vertexLineHierarchyLevelBuffer, "vertexLineHierarchyLevel",
                sgl::ATTRIB_FLOAT, 1);
    }
    gatherPpllOpacitiesRenderData->addGeometryBuffer(
            this->lineSegmentIdBuffer, "vertexLineSegmentId", sgl::ATTRIB_UNSIGNED_INT,
            1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);

    gatherPpllFinalRenderData->setVertexMode(sgl::VERTEX_MODE_LINES);
    gatherPpllFinalRenderData->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    gatherPpllFinalRenderData->addGeometryBuffer(
            tubeRenderData.vertexPositionBuffer, "vertexPosition",
            sgl::ATTRIB_FLOAT, 3);
    gatherPpllFinalRenderData->addGeometryBufferOptional(
            tubeRenderData.vertexAttributeBuffer, "vertexAttribute",
            sgl::ATTRIB_FLOAT, 1);
    gatherPpllFinalRenderData->addGeometryBuffer(
            tubeRenderData.vertexTangentBuffer, "vertexTangent",
            sgl::ATTRIB_FLOAT, 3);
    gatherPpllFinalRenderData->addGeometryBuffer(
            this->vertexOpacityBuffer, "vertexOpacity",
            sgl::ATTRIB_FLOAT, 1);
    if (tubeRenderData.vertexPrincipalStressIndexBuffer) {
        gatherPpllFinalRenderData->addGeometryBufferOptional(
                tubeRenderData.vertexPrincipalStressIndexBuffer, "vertexPrincipalStressIndex",
                sgl::ATTRIB_UNSIGNED_INT,
                1, 0, 0, 0, sgl::ATTRIB_CONVERSION_INT);
    }

    dirty = false;
    reRender = true;
    onHasMoved();
}

void OpacityOptimizationRenderer::generateBlendingWeightParametrization(bool isNewMesh) {
    // First, compute data necessary for parametrizing the polylines (number of segments, segment lengths).
    linesLengthSum = 0.0f;
    numPolylineSegments = 0;
    polylineLengths.clear();
    polylineLengths.shrink_to_fit();
    polylineLengths.resize(lines.size());

    #pragma omp parallel for reduction(+: linesLengthSum) reduction(+: numPolylineSegments) shared(polylineLengths) default(none)
    for (size_t lineIdx = 0; lineIdx < lines.size(); lineIdx++) {
        std::vector<glm::vec3>& line = lines.at(lineIdx);
        const size_t n = line.size();
        float polylineLength = 0.0f;
        for (size_t i = 1; i < n; i++) {
            polylineLength += glm::length(line[i] - line[i-1]);
        }
        polylineLengths.at(lineIdx) = polylineLength;
        linesLengthSum += polylineLength;
        numPolylineSegments += n - 1;
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
    const int approximateLineSegmentsTotal = lines.size() * 32; // Avg. discretization of 8 segments per line.
    lineSegmentConnectivityData.reserve(approximateLineSegmentsTotal);

    size_t segmentIdOffset = 0;
    size_t vertexIdx = 0;
    for (size_t lineIdx = 0; lineIdx < lines.size(); lineIdx++) {
        std::vector<glm::vec3>& line = lines.at(lineIdx);
        const size_t n = line.size();
        float polylineLength = polylineLengths.at(lineIdx);

        uint32_t numLineSubdivs =
                std::max(1u, uint32_t(std::ceil(approximateLineSegmentsTotal / linesLengthSum * polylineLength)));
        float lineSubdivLength = polylineLength / numLineSubdivs;

        // Set the first vertex manually (we can guarantee there is no segment before it).
        assert(line.size() >= 2);
        lineSegmentIdData.at(vertexIdx) = segmentIdOffset;
        blendingWeightParametrizationData.at(vertexIdx) = segmentIdOffset;
        vertexIdx++;

        // Compute
        float currentLength = 0.0f;
        for (size_t i = 1; i < n; i++) {
            currentLength += glm::length(line[i] - line[i-1]);
            lineSegmentIdData.at(vertexIdx) =
                    segmentIdOffset + std::min(numLineSubdivs - 1u, uint32_t(
                            std::floor(numLineSubdivs * currentLength / polylineLength)));
            float w = (numLineSubdivs - 1u) * (currentLength - lineSubdivLength / 2.0f) / (polylineLength - lineSubdivLength);
            blendingWeightParametrizationData.at(vertexIdx) =
                    segmentIdOffset + glm::clamp(w, 0.0f, float(numLineSubdivs - 1u) - EPSILON);
            vertexIdx++;
        }

        if (numLineSubdivs == 1) {
            lineSegmentConnectivityData.push_back(glm::uvec2(segmentIdOffset, segmentIdOffset));
        } else {
            lineSegmentConnectivityData.push_back(glm::uvec2(segmentIdOffset, segmentIdOffset + 1));
            for (size_t i = 1; i < numLineSubdivs - 1; i++) {
                lineSegmentConnectivityData.push_back(
                        glm::uvec2(segmentIdOffset + i - 1u, segmentIdOffset + i + 1u));
            }
            lineSegmentConnectivityData.push_back(
                    glm::uvec2(segmentIdOffset + numLineSubdivs - 2u, segmentIdOffset + numLineSubdivs - 1u));
        }

        segmentIdOffset += numLineSubdivs;
    }
    numLineSegments = lineSegmentConnectivityData.size();

    // Upload the data to the GPU.
    lineSegmentIdBuffer = sgl::Renderer->createGeometryBuffer(
            numLineVertices * sizeof(uint32_t), lineSegmentIdData.data(), sgl::VERTEX_BUFFER);

    blendingWeightParametrizationBuffer = sgl::Renderer->createGeometryBuffer(
            numLineVertices * sizeof(float), blendingWeightParametrizationData.data(), sgl::SHADER_STORAGE_BUFFER);

    lineSegmentConnectivityBuffer = sgl::Renderer->createGeometryBuffer(
            numLineSegments * sizeof(glm::uvec2), lineSegmentConnectivityData.data(), sgl::SHADER_STORAGE_BUFFER);

    segmentVisibilityBuffer = sgl::Renderer->createGeometryBuffer(
            numLineSegments * sizeof(uint32_t), nullptr, sgl::SHADER_STORAGE_BUFFER);

    segmentOpacityUintBuffer = sgl::Renderer->createGeometryBuffer(
            numLineSegments * sizeof(uint32_t), nullptr, sgl::SHADER_STORAGE_BUFFER);

    for (int i = 0; i < 2; i++) {
        segmentOpacityBuffers[i] = sgl::Renderer->createGeometryBuffer(
                numLineSegments * sizeof(float), nullptr, sgl::SHADER_STORAGE_BUFFER);
    }
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

    fragmentBufferOpacities = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentBufferOpacities = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeOpacityBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    // Fragment buffer for final color pass.
    fragmentBufferSizeFinal = size_t(expectedAvgDepthComplexity) * size_t(viewportWidthFinal) * size_t(viewportHeightFinal);
    size_t fragmentBufferSizeFinalBytes = 12ull * fragmentBufferSizeFinal;
    if (fragmentBufferSizeFinalBytes >= (1ull << 32ull)) {
        sgl::Logfile::get()->writeError(
                std::string() + "Fragment buffer size was larger than or equal to 4GiB. Clamping to 4GiB.");
        fragmentBufferSizeFinalBytes = (1ull << 32ull) - 12ull;
        fragmentBufferSizeFinal = fragmentBufferSizeFinalBytes / 12ull;
    }

    fragmentBufferFinal = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    fragmentBufferFinal = sgl::Renderer->createGeometryBuffer(
            fragmentBufferSizeFinalBytes, NULL, sgl::SHADER_STORAGE_BUFFER);

    // Write info to console and performance measurer.
    size_t fragmentBufferSizeBytes = fragmentBufferSizeOpacityBytes + fragmentBufferSizeFinalBytes;
    sgl::Logfile::get()->writeInfo(
            std::string() + "Fragment buffer size GiB (total): "
            + std::to_string(fragmentBufferSizeBytes / 1024.0 / 1024.0 / 1024.0));

    if (sceneData.performanceMeasurer) {
        sceneData.performanceMeasurer->setCurrentAlgorithmBufferSizeBytes(fragmentBufferSizeBytes);
    }
}

void OpacityOptimizationRenderer::onResolutionChanged() {
    sgl::Window *window = sgl::AppSettings::get()->getMainWindow();
    viewportWidthOpacity = std::round(window->getWidth() * opacityBufferScaleFactor);
    viewportHeightOpacity = std::round(window->getHeight() * opacityBufferScaleFactor);
    paddedViewportWidthOpacity = viewportWidthOpacity;
    paddedViewportHeightOpacity = viewportHeightOpacity;
    getScreenSizeWithTiling(paddedViewportWidthOpacity, paddedViewportHeightOpacity);
    viewportWidthFinal = window->getWidth();
    viewportHeightFinal = window->getHeight();
    paddedViewportWidthFinal = viewportWidthFinal;
    paddedViewportHeightFinal = viewportHeightFinal;
    getScreenSizeWithTiling(paddedViewportWidthFinal, paddedViewportHeightFinal);

    reallocateFragmentBuffer();

    startOffsetBufferOpacities = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    startOffsetBufferOpacities = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t) * paddedViewportWidthOpacity * paddedViewportHeightOpacity,
            NULL, sgl::SHADER_STORAGE_BUFFER);

    atomicCounterBufferOpacities = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    atomicCounterBufferOpacities = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);

    startOffsetBufferFinal = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    startOffsetBufferFinal = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t) * paddedViewportWidthFinal * paddedViewportHeightFinal,
            NULL, sgl::SHADER_STORAGE_BUFFER);

    atomicCounterBufferFinal = sgl::GeometryBufferPtr(); // Delete old data first (-> refcount 0)
    atomicCounterBufferFinal = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t), NULL, sgl::ATOMIC_COUNTER_BUFFER);

    if (useMultisampling) {
        msaaSceneFBO = sgl::Renderer->createFBO();
        msaaRenderTexture = sgl::TextureManager->createMultisampledTexture(
                viewportWidthFinal, viewportHeightFinal, numSamples,
                sceneData.sceneTexture->getSettings().internalFormat, true);
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
    gatherPpllOpacitiesShader->setUniform("viewportW", paddedViewportWidthOpacity);
    gatherPpllOpacitiesShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSizeOpacity);
    gatherPpllOpacitiesShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    gatherPpllOpacitiesShader->setUniform("lineWidth", lineWidth);
    lineData->setUniformGatherShaderData_Pass(gatherPpllOpacitiesShader);

    gatherPpllFinalShader->setUniform("viewportW", paddedViewportWidthFinal);
    gatherPpllFinalShader->setUniform("linkedListSize", (unsigned int)fragmentBufferSizeOpacity);
    gatherPpllFinalShader->setUniform("cameraPosition", sceneData.camera->getPosition());
    gatherPpllFinalShader->setUniform("lineWidth", lineWidth);
    if (gatherPpllFinalShader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        gatherPpllFinalShader->setUniform("backgroundColor", backgroundColor);
    }
    if (gatherPpllFinalShader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData.clearColor.getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        gatherPpllFinalShader->setUniform("foregroundColor", foregroundColor);
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

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    // Now, the final gather step.
    sgl::Renderer->render(gatherPpllOpacitiesRenderData);
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
    glClearNamedBufferData(opacityBufferId, GL_R32UI, GL_RED, GL_UNSIGNED_INT, (const void*)&opacityClearVal);
    GLuint visibilityBufferId = static_cast<sgl::GeometryBufferGL*>(segmentVisibilityBuffer.get())->getBuffer();
    uint32_t visibilityClearVal = 0u;
    glClearNamedBufferData(visibilityBufferId, GL_R32UI, GL_RED, GL_UNSIGNED_INT, (const void*)&visibilityClearVal);

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

    sgl::Renderer->setProjectionMatrix(sceneData.camera->getProjectionMatrix());
    sgl::Renderer->setViewMatrix(sceneData.camera->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    if (useMultisampling) {
        sgl::Renderer->bindFBO(msaaSceneFBO);
        sgl::Renderer->clearFramebuffer(
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT, sceneData.clearColor);
    }

    // Now, the final gather step.
    sgl::Renderer->render(gatherPpllFinalRenderData);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    if (useMultisampling) {
        sgl::Renderer->bindFBO(sceneData.framebuffer);
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


void OpacityOptimizationRenderer::renderGui() {
    if (ImGui::SliderFloat("Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f")) {
        reRender = true;
        onHasMoved();
    }
    if (ImGui::SliderFloat("q", &q, 0.0f, 5000.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (ImGui::SliderFloat("r", &r, 0.0f, 5000.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (ImGui::SliderInt("s", &s, 0, 20)) {
        reRender = true;
        onHasMoved();
    }
    if (ImGui::SliderFloat("lambda", &lambda, 0.1f, 20.0f, "%.1f")) {
        reRender = true;
        onHasMoved();
    }
    if (ImGui::SliderFloat("rel", &relaxationConstant, 0.0f, 1.0f, "%.2f")) {
        reRender = true;
        onHasMoved();
    }
    if (ImGui::SliderFloat("ts", &temporalSmoothingFactor, 0.0f, 1.0f, "%.2f")) {
        reRender = true;
        onHasMoved();
    }

    if (ImGui::Combo(
            "Sorting Mode", (int*)&sortingAlgorithmMode, SORTING_MODE_NAMES, NUM_SORTING_MODES)) {
        setSortingAlgorithmDefine();
        reloadResolveShader();
        reRender = true;
        onHasMoved();
    }

    if (maximumNumberOfSamples > 1) {
        if (ImGui::Checkbox("Multisampling", &useMultisampling)) {
            onResolutionChanged();
            reRender = true;
        }
        if (useMultisampling) {
            if (ImGui::Combo("Samples", &sampleModeSelection, sampleModeNames.data(), numSampleModes)) {
                numSamples = sgl::fromString<int>(sampleModeNames.at(sampleModeSelection));
                reloadResolveShader();
                onResolutionChanged();
                reRender = true;
            }
        }
    }
}

// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
bool OpacityOptimizationRenderer::needsReRender() {
    if (smoothingFramesCounter) {
        reRender = true;
    }
    bool tmp = reRender;
    reRender = false;
    return tmp;
}
// Called when the camera has moved.
void OpacityOptimizationRenderer::onHasMoved() {
    smoothingFramesCounter = NUM_SMOOTHING_FRAMES;
}
// Updates the internal logic (called once per frame).
void OpacityOptimizationRenderer::update(float dt) {
    if (smoothingFramesCounter > 0) {
        --smoothingFramesCounter;
    }
}
