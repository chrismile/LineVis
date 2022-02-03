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

#include <iostream>

#include <Math/Geometry/MatrixUtil.hpp>
#include <Utils/Dialog.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/OpenGL/RendererGL.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>
#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include "Renderers/Vulkan/VulkanAmbientOcclusionBaker.hpp"
#include "Renderers/Vulkan/VulkanRayTracedAmbientOcclusion.hpp"
#include "Renderers/Vulkan/VulkanRayTracer.hpp"
#endif

#include "LineRenderer.hpp"

float LineRenderer::lineWidth = STANDARD_LINE_WIDTH;
float LineRenderer::bandWidth = STANDARD_BAND_WIDTH;

// For compute shaders.
constexpr size_t BLOCK_SIZE = 256;

void LineRenderer::initialize() {
    updateDepthCueMode();
    updateAmbientOcclusionMode();
    sgl::ShaderManager->addPreprocessorDefine("COMPUTE_DEPTH_CUES_GPU", "");
    computeDepthValuesShaderProgram = sgl::ShaderManager->getShaderProgram({"ComputeDepthValues.Compute"});
    minMaxReduceDepthShaderProgram = sgl::ShaderManager->getShaderProgram({"MinMaxReduce.Compute"});

    // Add events for interaction with line data.
    onTransferFunctionMapRebuiltListenerToken = sgl::EventManager::get()->addListener(
            ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT, [this](const sgl::EventPtr&) {
                this->onTransferFunctionMapRebuilt();
            });

    isInitialized = true;
}

LineRenderer::~LineRenderer() {
    if (isInitialized) {
        sgl::EventManager::get()->removeListener(
                ON_TRANSFER_FUNCTION_MAP_REBUILT_EVENT, onTransferFunctionMapRebuiltListenerToken);
    }

    if (useDepthCues) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
    if (useAmbientOcclusion) {
        sgl::ShaderManager->removePreprocessorDefine("USE_AMBIENT_OCCLUSION");
        sgl::ShaderManager->removePreprocessorDefine("STATIC_AMBIENT_OCCLUSION_PREBAKING");
    }
    sgl::ShaderManager->removePreprocessorDefine("COMPUTE_DEPTH_CUES_GPU");

    ambientOcclusionBaker = {};
}

bool LineRenderer::needsReRender() {
    if (useAmbientOcclusion && ambientOcclusionBaker) {
        ambientOcclusionReRender = ambientOcclusionBaker->needsReRender();
        reRender |= ambientOcclusionReRender;
    }

    bool tmp = reRender;
    reRender = false;
    return tmp;
}

bool LineRenderer::getIsTriangleRepresentationUsed() const {
    return (lineData && lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_TRIANGLE_MESH)
            || (useAmbientOcclusion && ambientOcclusionBaker);
}

void LineRenderer::update(float dt) {
}

void LineRenderer::onResolutionChanged() {
    if (ambientOcclusionBaker) {
        ambientOcclusionBaker->onResolutionChanged();
    }
}

void LineRenderer::updateDepthCueMode() {
    if (useDepthCues) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SCREEN_SPACE_POSITION", "");
        sgl::ShaderManager->addPreprocessorDefine("USE_DEPTH_CUES", "");

        if (lineData && filteredLines.empty()) {
            updateDepthCueGeometryData();
        }
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
        sgl::ShaderManager->removePreprocessorDefine("USE_DEPTH_CUES");
    }
}

bool LineRenderer::getIsComputationRunning() {
    if (ambientOcclusionBaker) {
        return ambientOcclusionBaker->getIsComputationRunning();
    }
    return false;
}

void LineRenderer::onHasMoved() {
    if (ambientOcclusionBaker) {
        ambientOcclusionBaker->onHasMoved();
    }
}

void LineRenderer::showRayQueriesUnsupportedWarning() {
    std::string warningText = "Ray queries are not supported by the used GPU. Disabling ambient occlusion.";
    sgl::Logfile::get()->writeWarning(
            "Warning in LineRenderer::showRayQueriesUnsupportedWarning: " + warningText, false);
    auto handle = sgl::dialog::openMessageBox(
            "Unsupported Renderer", warningText, sgl::dialog::Icon::WARNING);
    sceneData->nonBlockingMsgBoxHandles->push_back(handle);
}

void LineRenderer::setAmbientOcclusionBaker() {
    AmbientOcclusionBakerType oldAmbientOcclusionBakerType = AmbientOcclusionBakerType::NONE;
    if (ambientOcclusionBaker) {
        oldAmbientOcclusionBakerType = ambientOcclusionBaker->getType();
    }
    ambientOcclusionBaker = {};

#ifdef USE_VULKAN_INTEROP
    if (ambientOcclusionBakerType == AmbientOcclusionBakerType::VULKAN_RTAO_PREBAKER) {
        sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
        if (!device || !device->getRayQueriesSupported()) {
            showRayQueriesUnsupportedWarning();
            return;
        }
        auto* ambientOcclusionRenderer = new sgl::vk::Renderer(sgl::AppSettings::get()->getPrimaryDevice());
        ambientOcclusionBaker = AmbientOcclusionBakerPtr(
                new VulkanAmbientOcclusionBaker(ambientOcclusionRenderer));
    } else if (ambientOcclusionBakerType == AmbientOcclusionBakerType::VULKAN_RTAO) {
        sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
        if (!device || !device->getRayQueriesSupported()) {
            showRayQueriesUnsupportedWarning();
            return;
        }
        if (lineData && lineData->getUseCappedTubes() && (!isVulkanRenderer || isRasterizer)) {
            lineData->setUseCappedTubes(this, false);
        }
        sgl::vk::Renderer* ambientOcclusionRenderer;
        bool isMainRenderer;
        if (getRenderingMode() == RENDERING_MODE_VULKAN_RAY_TRACER) {
            isMainRenderer = true;
            ambientOcclusionRenderer = static_cast<VulkanRayTracer*>(this)->getVulkanRenderer();
        } else {
            isMainRenderer = false;
            ambientOcclusionRenderer = new sgl::vk::Renderer(sgl::AppSettings::get()->getPrimaryDevice());
        }
        ambientOcclusionBaker = AmbientOcclusionBakerPtr(
                new VulkanRayTracedAmbientOcclusion(sceneData, ambientOcclusionRenderer, isMainRenderer));
    }
#endif

    AmbientOcclusionBakerType newAmbientOcclusionBakerType = AmbientOcclusionBakerType::NONE;
    if (ambientOcclusionBaker) {
        newAmbientOcclusionBakerType = ambientOcclusionBaker->getType();
    }

    if (oldAmbientOcclusionBakerType != newAmbientOcclusionBakerType) {
        updateAmbientOcclusionMode();
        reloadGatherShader();
    }
    reRender = true;

    if (useAmbientOcclusion && ambientOcclusionBaker && lineData) {
        ambientOcclusionBaker->startAmbientOcclusionBaking(lineData, true);
    }
}

void LineRenderer::updateAmbientOcclusionMode() {
    if (useAmbientOcclusion && !ambientOcclusionBaker) {
        setAmbientOcclusionBaker();
    }

    if (useAmbientOcclusion && ambientOcclusionBaker) {
        sgl::ShaderManager->addPreprocessorDefine("USE_AMBIENT_OCCLUSION", "");
        if (ambientOcclusionBaker->getIsStaticPrebaker()) {
            sgl::ShaderManager->addPreprocessorDefine("STATIC_AMBIENT_OCCLUSION_PREBAKING", "");
        } else {
            sgl::ShaderManager->removePreprocessorDefine("STATIC_AMBIENT_OCCLUSION_PREBAKING");
        }
    } else {
        sgl::ShaderManager->removePreprocessorDefine("USE_AMBIENT_OCCLUSION");
        sgl::ShaderManager->removePreprocessorDefine("STATIC_AMBIENT_OCCLUSION_PREBAKING");
    }
}

void LineRenderer::updateDepthCueGeometryData() {
    filteredLines = lineData->getFilteredLines(this);
    std::vector<glm::vec4> filteredLinesVertices;
    for (std::vector<glm::vec3>& line : filteredLines) {
        for (const glm::vec3& point : line) {
            filteredLinesVertices.emplace_back(point.x, point.y, point.z, 1.0f);
        }
    }

#ifdef USE_VULKAN_INTEROP
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    if (device) {
        depthMinMaxBuffersVk[0] = std::make_shared<sgl::vk::Buffer>(
                device, sgl::iceil(int(filteredLinesVertices.size()), BLOCK_SIZE) * sizeof(glm::vec2),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
                true, true);
        depthMinMaxBuffers[0] = sgl::GeometryBufferPtr(new sgl::GeometryBufferGLExternalMemoryVk(
                depthMinMaxBuffersVk[0], sgl::SHADER_STORAGE_BUFFER));
        depthMinMaxBuffersVk[1] = std::make_shared<sgl::vk::Buffer>(
                device, sgl::iceil(int(filteredLinesVertices.size()), BLOCK_SIZE * BLOCK_SIZE * 2)
                * sizeof(glm::vec2),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY,
                true, true);
        depthMinMaxBuffers[1] = sgl::GeometryBufferPtr(new sgl::GeometryBufferGLExternalMemoryVk(
                depthMinMaxBuffersVk[1], sgl::SHADER_STORAGE_BUFFER));
    } else {
#endif
    depthMinMaxBuffers[0] = sgl::Renderer->createGeometryBuffer(
            sgl::iceil(int(filteredLinesVertices.size()), BLOCK_SIZE) * sizeof(glm::vec2),
            sgl::SHADER_STORAGE_BUFFER);
    depthMinMaxBuffers[1] = sgl::Renderer->createGeometryBuffer(
            sgl::iceil(int(filteredLinesVertices.size()), BLOCK_SIZE * BLOCK_SIZE * 2) * sizeof(glm::vec2),
            sgl::SHADER_STORAGE_BUFFER);
#ifdef USE_VULKAN_INTEROP
    }
#endif

    filteredLinesVerticesBuffer = sgl::Renderer->createGeometryBuffer(
            filteredLinesVertices.size() * sizeof(glm::vec4), filteredLinesVertices.data(),
            sgl::SHADER_STORAGE_BUFFER);
}

void LineRenderer::computeDepthRange() {
    if (computeDepthCuesOnGpu) {
        sgl::ShaderManager->bindShaderStorageBuffer(12, filteredLinesVerticesBuffer);
        sgl::ShaderManager->bindShaderStorageBuffer(11, depthMinMaxBuffers[0]);
        uint32_t numVertices = uint32_t(filteredLinesVerticesBuffer->getSize() / sizeof(glm::vec4));
        uint32_t numBlocks = sgl::iceil(int(numVertices), BLOCK_SIZE);
        computeDepthValuesShaderProgram->setUniform("numVertices", numVertices);
        computeDepthValuesShaderProgram->setUniform("nearDist", (*sceneData->camera)->getNearClipDistance());
        computeDepthValuesShaderProgram->setUniform("farDist", (*sceneData->camera)->getFarClipDistance());
        computeDepthValuesShaderProgram->setUniform("cameraViewMatrix", (*sceneData->camera)->getViewMatrix());
        computeDepthValuesShaderProgram->setUniform(
                "cameraProjectionMatrix", (*sceneData->camera)->getProjectionMatrix());
        computeDepthValuesShaderProgram->dispatchCompute(int(numBlocks));
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        minMaxReduceDepthShaderProgram->setUniform("nearDist", (*sceneData->camera)->getNearClipDistance());
        minMaxReduceDepthShaderProgram->setUniform("farDist", (*sceneData->camera)->getFarClipDistance());
        int iteration = 0;
        uint32_t inputSize;
        while (numBlocks > 1) {
            if (iteration != 0) {
                // Already bound for computeDepthValuesShaderProgram if i == 0.
                sgl::ShaderManager->bindShaderStorageBuffer(11, depthMinMaxBuffers[iteration % 2]);
            }
            sgl::ShaderManager->bindShaderStorageBuffer(12, depthMinMaxBuffers[(iteration + 1) % 2]);

            inputSize = numBlocks;
            numBlocks = sgl::iceil(int(numBlocks), BLOCK_SIZE*2);
            minMaxReduceDepthShaderProgram->setUniform("sizeOfInput", inputSize);
            minMaxReduceDepthShaderProgram->dispatchCompute(int(numBlocks));
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

            iteration++;
        }

        // Bind the output of ComputeDepthValues.glsl to position 12 for Lighting.glsl if no reduction was necessary.
        if (iteration == 0) {
            sgl::ShaderManager->bindShaderStorageBuffer(12, depthMinMaxBuffers[0]);
        }

        outputDepthMinMaxBufferIndex = iteration % 2;
    }

    if (!computeDepthCuesOnGpu) {
        bool useBoundingBox = lineData->getNumLines() > 1000;
        if (useBoundingBox) {
            const sgl::AABB3& boundingBox = lineData->getModelBoundingBox();
            sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed((*sceneData->camera)->getViewMatrix());
            minDepth = -screenSpaceBoundingBox.getMaximum().z;
            maxDepth = -screenSpaceBoundingBox.getMinimum().z;
        } else {
            glm::mat4 viewMatrix = (*sceneData->camera)->getViewMatrix();
            minDepth = std::numeric_limits<float>::max();
            maxDepth = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
            #pragma omp parallel for default(none) shared(viewMatrix, filteredLines) \
            reduction(min: minDepth) reduction(max: maxDepth)
#endif
            for (size_t lineIdx = 0; lineIdx < filteredLines.size(); lineIdx++) {
                const std::vector<glm::vec3>& line = filteredLines.at(lineIdx);
                for (const glm::vec3& point : line) {
                    float depth = -sgl::transformPoint(viewMatrix, point).z;
                    minDepth = std::min(minDepth, depth);
                    maxDepth = std::max(maxDepth, depth);
                }
            }
        }

        minDepth = glm::clamp(
                minDepth, (*sceneData->camera)->getFarClipDistance(), (*sceneData->camera)->getNearClipDistance());
        maxDepth = glm::clamp(
                maxDepth, (*sceneData->camera)->getFarClipDistance(), (*sceneData->camera)->getNearClipDistance());
    }
}

void LineRenderer::setUniformData_Pass(sgl::ShaderProgramPtr shaderProgram) {
    if (useDepthCues && lineData) {
        computeDepthRange();

        if (!computeDepthCuesOnGpu) {
            shaderProgram->setUniformOptional("minDepth", minDepth);
            shaderProgram->setUniformOptional("maxDepth", maxDepth);
        }
    }

    shaderProgram->setUniformOptional("depthCueStrength", depthCueStrength);

    shaderProgram->setUniformOptional("fieldOfViewY", (*sceneData->camera)->getFOVy());
    shaderProgram->setUniformOptional(
            "viewportSize",
            glm::ivec2((*sceneData->sceneTexture)->getW(), (*sceneData->sceneTexture)->getH()));

    if (useAmbientOcclusion && ambientOcclusionBaker && ambientOcclusionBaker->getIsDataReady()) {
        if (ambientOcclusionBaker->getIsStaticPrebaker() && ambientOcclusionBaker->getAmbientOcclusionBuffer()) {
            sgl::GeometryBufferPtr aoBuffer = ambientOcclusionBaker->getAmbientOcclusionBuffer();
            sgl::GeometryBufferPtr blendingWeightsBuffer = ambientOcclusionBaker->getBlendingWeightsBuffer();
            sgl::ShaderManager->bindShaderStorageBuffer(13, aoBuffer);
            sgl::ShaderManager->bindShaderStorageBuffer(14, blendingWeightsBuffer);
            shaderProgram->setUniformOptional(
                    "numAoTubeSubdivisions", ambientOcclusionBaker->getNumTubeSubdivisions());
            shaderProgram->setUniformOptional(
                    "numLineVertices", ambientOcclusionBaker->getNumLineVertices());
            shaderProgram->setUniformOptional(
                    "numParametrizationVertices", ambientOcclusionBaker->getNumParametrizationVertices());
        } else if (!ambientOcclusionBaker->getIsStaticPrebaker()) {
            shaderProgram->setUniform(
                    "ambientOcclusionTexture", ambientOcclusionBaker->getAmbientOcclusionFrameTexture(),
                    6);
        }
        shaderProgram->setUniformOptional(
                "ambientOcclusionStrength", ambientOcclusionStrength);
        shaderProgram->setUniformOptional(
                "ambientOcclusionGamma", ambientOcclusionGamma);
    }
}

bool LineRenderer::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = false;

    if (settings.getValueOpt("line_width", lineWidth) && lineData) {
        lineData->setTriangleRepresentationDirty();
    }
    if (settings.getValueOpt("band_width", bandWidth) && lineData) {
        lineData->setTriangleRepresentationDirty();
    }

    if (settings.getValueOpt("depth_cue_strength", depthCueStrength)) {
        if (depthCueStrength <= 0.0f && useDepthCues) {
            useDepthCues = false;
            updateDepthCueMode();
            shallReloadGatherShader = true;
        }
        if (depthCueStrength > 0.0f && !useDepthCues) {
            useDepthCues = true;
            updateDepthCueMode();
            shallReloadGatherShader = true;
        }
    }

    if (settings.getValueOpt("ambient_occlusion_strength", ambientOcclusionStrength)) {
        if (ambientOcclusionStrength <= 0.0f && useAmbientOcclusion) {
            useAmbientOcclusion = false;
            updateAmbientOcclusionMode();
            shallReloadGatherShader = true;
        }
        if (ambientOcclusionStrength > 0.0f && !useAmbientOcclusion) {
            useAmbientOcclusion = true;
            updateAmbientOcclusionMode();
            shallReloadGatherShader = true;
        }
    }

    if (settings.getValueOpt("ambient_occlusion_gamma", ambientOcclusionGamma)) {
        reRender = false;
    }

    return shallReloadGatherShader;
}

void LineRenderer::reloadGatherShaderExternal() {
    if (lineData) {
        reloadGatherShader(false);
        setLineData(lineData, false);
    }
}

bool LineRenderer::getCanUseLiveUpdate(LineDataAccessType accessType) const {
    if (lineData) {
        if (accessType == LineDataAccessType::TRIANGLE_MESH) {
            return !getIsTriangleRepresentationUsed() || lineData->getIsSmallDataSet();
        } else if (accessType == LineDataAccessType::FILTERED_LINES) {
            return lineData->getIsSmallDataSet();
        }
    }
    return true;
}

void LineRenderer::render() {
    if (useAmbientOcclusion && ambientOcclusionBaker) {
        if (ambientOcclusionBaker->getBakingMode() == BakingMode::ITERATIVE_UPDATE
                && (ambientOcclusionBaker->getIsComputationRunning() || ambientOcclusionReRender)) {
            ambientOcclusionBaker->updateIterative(isVulkanRenderer);
            reRender = true;
            internalReRender = true;
            ambientOcclusionReRender = false;
        }
        if (ambientOcclusionBaker->getBakingMode() == BakingMode::MULTI_THREADED
                && ambientOcclusionBaker->getHasThreadUpdate()) {
            ambientOcclusionBaker->updateMultiThreaded(isVulkanRenderer);
            reRender = true;
            internalReRender = true;
        }
        if (!ambientOcclusionBaker->getIsStaticPrebaker()) {
            ambientOcclusionTexturesDirty |= ambientOcclusionBaker->getHasTextureResolutionChanged();
        }
    }
}

void LineRenderer::renderGuiOverlay() {
    bool shallReloadGatherShader = false;

    if (lineData && lineData->renderGuiOverlay()) {
        shallReloadGatherShader = true;
    }

    if (shallReloadGatherShader) {
        reloadGatherShaderExternal();
    }
}

void LineRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = false;

    bool canUseLiveUpdate = getCanUseLiveUpdate(LineDataAccessType::TRIANGLE_MESH);
    ImGui::EditMode editMode = propertyEditor.addSliderFloatEdit(
            "Line Width", &lineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f");
    if ((canUseLiveUpdate && editMode != ImGui::EditMode::NO_CHANGE)
        || (!canUseLiveUpdate && editMode == ImGui::EditMode::INPUT_FINISHED)) {
        if (lineData) {
            lineData->setTriangleRepresentationDirty();
        }
        reRender = true;
        internalReRender = true;
    }

    if (lineData) {
        shallReloadGatherShader =
                lineData->renderGuiRenderingSettingsPropertyEditor(propertyEditor) || shallReloadGatherShader;
    }

    if (lineData) {
        RenderingMode mode = getRenderingMode();

        if (lineData->renderGuiPropertyEditorNodesRenderer(propertyEditor, this)) {
            shallReloadGatherShader = true;
        }

        if (mode != RENDERING_MODE_LINE_DENSITY_MAP_RENDERER
                && mode != RENDERING_MODE_VOLUMETRIC_PATH_TRACER) {
            ImGui::EditMode editModeDepthCue = propertyEditor.addSliderFloatEdit(
                    "Depth Cue Strength", &depthCueStrength, 0.0f, 1.0f);
            if (editModeDepthCue != ImGui::EditMode::NO_CHANGE) {
                reRender = true;
                internalReRender = true;
                if (depthCueStrength > 0.0f && !useDepthCues) {
                    useDepthCues = true;
                    updateDepthCueMode();
                    shallReloadGatherShader = true;
                }
            }
            if (editModeDepthCue == ImGui::EditMode::INPUT_FINISHED) {
                if (depthCueStrength <= 0.0f && useDepthCues) {
                    useDepthCues = false;
                    updateDepthCueMode();
                    shallReloadGatherShader = true;
                }
            }
        }

#ifdef USE_VULKAN_INTEROP
        if (mode != RENDERING_MODE_VOXEL_RAY_CASTING
                && mode != RENDERING_MODE_LINE_DENSITY_MAP_RENDERER
                && mode != RENDERING_MODE_VOLUMETRIC_PATH_TRACER) {
            ImGui::EditMode editModeAo = propertyEditor.addSliderFloatEdit(
                    "AO Strength", &ambientOcclusionStrength, 0.0f, 1.0f);
            if (editModeAo != ImGui::EditMode::NO_CHANGE) {
                reRender = true;
                internalReRender = true;
                if (ambientOcclusionStrength > 0.0f && !useAmbientOcclusion) {
                    useAmbientOcclusion = true;
                    updateAmbientOcclusionMode();
                    shallReloadGatherShader = true;
                }
            }
            if (editModeAo == ImGui::EditMode::INPUT_FINISHED) {
                if (ambientOcclusionStrength <= 0.0f && useAmbientOcclusion) {
                    useAmbientOcclusion = false;
                    updateAmbientOcclusionMode();
                    shallReloadGatherShader = true;
                }
            }

            if (ambientOcclusionStrength > 0.0f) {
                if (propertyEditor.addSliderFloat(
                        "AO Gamma", &ambientOcclusionGamma, 0.5f, 3.0f)) {
                    reRender = true;
                    internalReRender = true;
                }

                if (propertyEditor.addCombo(
                        "Ambient Occlusion Mode", (int*)&ambientOcclusionBakerType,
                        AMBIENT_OCCLUSION_BAKER_TYPE_NAMES,
                        IM_ARRAYSIZE(AMBIENT_OCCLUSION_BAKER_TYPE_NAMES))) {
                    setAmbientOcclusionBaker();
                }
            }
        }
#endif
    }

    if (useAmbientOcclusion && ambientOcclusionBaker) {
        if (ambientOcclusionBaker->renderGuiPropertyEditorNodes(propertyEditor)) {
            reRender = true;
            internalReRender = true;
            if (isVulkanRenderer && ambientOcclusionBaker->getIsStaticPrebaker()) {
                ambientOcclusionBuffersDirty = true;
            }
        }
    }

    if (shallReloadGatherShader) {
        reloadGatherShaderExternal();
    }
}

void LineRenderer::updateNewLineData(LineDataPtr& lineData, bool isNewData) {
    if (!this->lineData || lineData->getType() != this->lineData->getType()
            || lineData->settingsDiffer(this->lineData.get())) {
        this->lineData = lineData;
        reloadGatherShader(false);
    }
    this->lineData = lineData;

    if (useAmbientOcclusion && ambientOcclusionBaker) {
        ambientOcclusionBaker->startAmbientOcclusionBaking(lineData, isNewData);
    }

    filteredLines.clear();
    filteredLinesVerticesBuffer = sgl::GeometryBufferPtr();
    depthMinMaxBuffers[0] = sgl::GeometryBufferPtr();
    depthMinMaxBuffers[1] = sgl::GeometryBufferPtr();
    if (useDepthCues) {
        updateDepthCueGeometryData();
    }

    if (!isVulkanRenderer && lineData && lineData->hasSimulationMeshOutline()) {
        shaderAttributesHull = sgl::ShaderAttributesPtr();
        shaderAttributesHull = lineData->getGatherShaderAttributesHull(gatherShaderHull);
    }
}

void LineRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    if (lineData && lineData->hasSimulationMeshOutline() && !isVulkanRenderer) {
        gatherShaderHull = lineData->reloadGatherShaderHull();
        if (canCopyShaderAttributes && shaderAttributesHull) {
            shaderAttributesHull = shaderAttributesHull->copy(gatherShaderHull);
        }
    }
}

void LineRenderer::renderHull() {
    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        if (!gatherShaderHull) {
            reloadGatherShader();
            shaderAttributesHull = lineData->getGatherShaderAttributesHull(gatherShaderHull);
        }
        lineData->setUniformGatherShaderDataHull_Pass(gatherShaderHull);
        gatherShaderHull->setUniformOptional("cameraPosition", (*sceneData->camera)->getPosition());
        glDisable(GL_CULL_FACE);
        sgl::Renderer->render(shaderAttributesHull);
        glEnable(GL_CULL_FACE);
    }
}
