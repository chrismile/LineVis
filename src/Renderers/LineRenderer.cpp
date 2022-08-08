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

#include <Utils/Dialog.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "Renderers/AmbientOcclusion/VulkanAmbientOcclusionBaker.hpp"
#include "Renderers/AmbientOcclusion/VulkanRayTracedAmbientOcclusion.hpp"
#include "Renderers/RayTracing/VulkanRayTracer.hpp"
#include "DepthCues.hpp"
#include "HullRasterPass.hpp"
#include "LineRenderer.hpp"

float LineRenderer::lineWidth = STANDARD_LINE_WIDTH;
float LineRenderer::bandWidth = STANDARD_BAND_WIDTH;
float LineRenderer::displayLineWidth = STANDARD_LINE_WIDTH;
float LineRenderer::displayBandWidth = STANDARD_BAND_WIDTH;
float LineRenderer::displayLineWidthStaging = STANDARD_LINE_WIDTH;
float LineRenderer::displayBandWidthStaging = STANDARD_BAND_WIDTH;

LineRenderer::LineRenderer(
        std::string windowName, SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : windowName(std::move(windowName)), sceneData(sceneData), renderer(*sceneData->renderer),
          transferFunctionWindow(transferFunctionWindow) {
}

void LineRenderer::initialize() {
    updateDepthCueMode();
    updateAmbientOcclusionMode();
    computeDepthValuesPass = std::make_shared<ComputeDepthValuesPass>(sceneData);
    minMaxDepthReductionPass[0] = std::make_shared<MinMaxDepthReductionPass>(sceneData);
    minMaxDepthReductionPass[1] = std::make_shared<MinMaxDepthReductionPass>(sceneData);

    if (isRasterizer) {
        lineRasterPass = std::make_shared<LineRasterPass>(this);
        hullRasterPass = std::make_shared<HullRasterPass>(this);
    }

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
    bool primitiveModeUsesTriMesh =
            lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_TUBE_TRIANGLE_MESH
            || lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH
            || getRenderingMode() == RENDERING_MODE_DEFERRED_SHADING;
    return (lineData && primitiveModeUsesTriMesh) || (useAmbientOcclusion && ambientOcclusionBaker);
}

void LineRenderer::update(float dt) {
}

void LineRenderer::onResolutionChanged() {
    if (ambientOcclusionBaker) {
        ambientOcclusionBaker->onResolutionChanged();
    }
}

void LineRenderer::getVulkanShaderPreprocessorDefines(std::map<std::string, std::string>& preprocessorDefines) {
    sgl::vk::ShaderManager->invalidateShaderCache();

    if (useDepthCues) {
        preprocessorDefines.insert(std::make_pair("USE_DEPTH_CUES", ""));
        preprocessorDefines.insert(std::make_pair("COMPUTE_DEPTH_CUES_GPU", ""));
    }
    if (useDepthCues || (useAmbientOcclusion && ambientOcclusionBaker)) {
        preprocessorDefines.insert(std::make_pair("USE_SCREEN_SPACE_POSITION", ""));
    }
    if (useAmbientOcclusion && ambientOcclusionBaker) {
        preprocessorDefines.insert(std::make_pair("USE_AMBIENT_OCCLUSION", ""));
        if (ambientOcclusionBaker->getIsStaticPrebaker()) {
            preprocessorDefines.insert(std::make_pair("STATIC_AMBIENT_OCCLUSION_PREBAKING", ""));
        }
        preprocessorDefines.insert(std::make_pair("GEOMETRY_PASS_TUBE", ""));
    }

    if (tileWidth == 1 && tileHeight == 1) {
        // No tiling
        tilingModeIndex = 0;
    } else if (tileWidth == 2 && tileHeight == 2) {
        tilingModeIndex = 1;
        preprocessorDefines.insert(std::make_pair("ADDRESSING_TILED_2x2", ""));
    } else if (tileWidth == 2 && tileHeight == 8) {
        tilingModeIndex = 2;
        preprocessorDefines.insert(std::make_pair("ADDRESSING_TILED_2x8", ""));
    } else if (tileWidth == 8 && tileHeight == 2) {
        tilingModeIndex = 3;
        preprocessorDefines.insert(std::make_pair("ADDRESSING_TILED_NxM", ""));
    } else if (tileWidth == 4 && tileHeight == 4) {
        tilingModeIndex = 4;
        preprocessorDefines.insert(std::make_pair("ADDRESSING_TILED_NxM", ""));
    } else if (tileWidth == 8 && tileHeight == 8 && !tilingUseMortonCode) {
        tilingModeIndex = 5;
        preprocessorDefines.insert(std::make_pair("ADDRESSING_TILED_NxM", ""));
    } else if (tileWidth == 8 && tileHeight == 8 && tilingUseMortonCode) {
        tilingModeIndex = 6;
        preprocessorDefines.insert(std::make_pair("ADRESSING_MORTON_CODE_8x8", ""));
    } else {
        // Invalid mode, just set to mode 5, too.
        tilingModeIndex = 5;
        preprocessorDefines.insert(std::make_pair("ADDRESSING_TILED_NxM", ""));
    }

    preprocessorDefines.insert(std::make_pair("TILE_N", sgl::toString(tileWidth)));
    preprocessorDefines.insert(std::make_pair("TILE_M", sgl::toString(tileHeight)));
}

void LineRenderer::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
}

void LineRenderer::setRenderDataBindings(const sgl::vk::RenderDataPtr& renderData) {
    if (useDepthCues) {
        if (!depthMinMaxBuffers[outputDepthMinMaxBufferIndex]
                && renderData->getShaderStages()->hasDescriptorBinding(0, "DepthMinMaxBuffer")) {
            if (!dummyBuffer) {
                dummyBuffer = std::make_shared<sgl::vk::Buffer>(
                        renderer->getDevice(), sizeof(glm::vec4),
                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
            }
            renderData->setStaticBufferOptional(dummyBuffer, "DepthMinMaxBuffer");
        } else {
            renderData->setStaticBufferOptional(
                    depthMinMaxBuffers[outputDepthMinMaxBufferIndex], "DepthMinMaxBuffer");
        }
    }
    if (useAmbientOcclusion && ambientOcclusionBaker) {
        if (ambientOcclusionBaker->getIsStaticPrebaker()) {
            auto ambientOcclusionBuffer = ambientOcclusionBaker->getAmbientOcclusionBufferVulkan();
            auto blendingWeightsBuffer = ambientOcclusionBaker->getBlendingWeightsBufferVulkan();
            if (ambientOcclusionBuffer && blendingWeightsBuffer) {
                renderData->setStaticBufferOptional(ambientOcclusionBuffer, "AmbientOcclusionFactors");
                renderData->setStaticBufferOptional(blendingWeightsBuffer, "AmbientOcclusionBlendingWeights");
            } else {
                // Just bind anything in order for sgl to not complain...
                glm::vec4 zeroData{};
                sgl::vk::BufferPtr buffer = std::make_shared<sgl::vk::Buffer>(
                        sgl::AppSettings::get()->getPrimaryDevice(), sizeof(glm::vec4), &zeroData,
                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                        VMA_MEMORY_USAGE_GPU_ONLY);
                renderData->setStaticBufferOptional(buffer, "AmbientOcclusionFactors");
                renderData->setStaticBufferOptional(buffer, "AmbientOcclusionBlendingWeights");
            }
        } else {
            auto ambientOcclusionTexture =
                    ambientOcclusionBaker->getAmbientOcclusionFrameTextureVulkan();
            renderData->setStaticTextureOptional(ambientOcclusionTexture, "ambientOcclusionTexture");
        }
    }
}

void LineRenderer::updateVulkanUniformBuffers() {
}

void LineRenderer::setFramebufferAttachments(sgl::vk::FramebufferPtr& framebuffer, VkAttachmentLoadOp loadOp) {
    sgl::vk::AttachmentState attachmentState;
    attachmentState.loadOp = loadOp;
    attachmentState.initialLayout =
            loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
            VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    framebuffer->setColorAttachment(
            (*sceneData->sceneTexture)->getImageView(), 0, attachmentState,
            sceneData->clearColor->getFloatColorRGBA());

    sgl::vk::AttachmentState depthAttachmentState;
    depthAttachmentState.loadOp = loadOp;
    depthAttachmentState.initialLayout =
            loadOp == VK_ATTACHMENT_LOAD_OP_CLEAR ?
            VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachmentState.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    framebuffer->setDepthStencilAttachment(
            (*sceneData->sceneDepthTexture)->getImageView(), depthAttachmentState, 1.0f);
}

void LineRenderer::renderBase(VkPipelineStageFlags pipelineStageFlags) {
    if (useDepthCues && lineData && depthMinMaxBuffers[outputDepthMinMaxBufferIndex]) {
        computeDepthRange();
        renderer->insertBufferMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
                depthMinMaxBuffers[outputDepthMinMaxBufferIndex]);
    }

    if (useAmbientOcclusion && ambientOcclusionBaker) {
        if (ambientOcclusionBaker->getBakingMode() == BakingMode::ITERATIVE_UPDATE
                && (ambientOcclusionBaker->getIsComputationRunning() || ambientOcclusionReRender)) {
            ambientOcclusionBaker->updateIterative(pipelineStageFlags);
            reRender = true;
            internalReRender = true;
            ambientOcclusionReRender = false;
        }
        if (ambientOcclusionBaker->getBakingMode() == BakingMode::MULTI_THREADED
                && ambientOcclusionBaker->getHasThreadUpdate()) {
            ambientOcclusionBaker->updateMultiThreaded(pipelineStageFlags);
            reRender = true;
            internalReRender = true;
        }
        if (!ambientOcclusionBaker->getIsStaticPrebaker()) {
            ambientOcclusionTexturesDirty |= ambientOcclusionBaker->getHasTextureResolutionChanged();
        }
    }

    lineData->updateVulkanUniformBuffers(this, renderer);
}

void LineRenderer::updateDepthCueMode() {
    if (useDepthCues && lineData && filteredLines.empty()) {
        updateDepthCueGeometryData();
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

    if (ambientOcclusionBakerType == AmbientOcclusionBakerType::VULKAN_RTAO_PREBAKER) {
        sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
        if (!device || !device->getRayQueriesSupported()) {
            showRayQueriesUnsupportedWarning();
            return;
        }
        ambientOcclusionBaker = AmbientOcclusionBakerPtr(
                new VulkanAmbientOcclusionBaker(renderer));
    } else if (ambientOcclusionBakerType == AmbientOcclusionBakerType::VULKAN_RTAO) {
        sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
        if (!device || !device->getRayQueriesSupported()) {
            showRayQueriesUnsupportedWarning();
            return;
        }
        if (lineData && lineData->getUseCappedTubes() && isRasterizer) {
            lineData->setUseCappedTubes(this, false);
        }
        ambientOcclusionBaker = AmbientOcclusionBakerPtr(
                new VulkanRayTracedAmbientOcclusion(sceneData, renderer));
    }

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
}

void LineRenderer::updateDepthCueGeometryData() {
    filteredLines = lineData->getFilteredLines(this);
    std::vector<glm::vec4> filteredLinesVertices;
    for (std::vector<glm::vec3>& line : filteredLines) {
        for (const glm::vec3& point : line) {
            filteredLinesVertices.emplace_back(point.x, point.y, point.z, 1.0f);
        }
    }

    depthMinMaxBuffers[0] = {};
    depthMinMaxBuffers[1] = {};
    filteredLinesVerticesBuffer = {};
    if (filteredLinesVertices.empty()) {
        return;
    }

    sgl::vk::Device* device = renderer->getDevice();
    depthMinMaxBuffers[0] = std::make_shared<sgl::vk::Buffer>(
            device, sgl::iceil(int(filteredLinesVertices.size()), BLOCK_SIZE_DEPTH_CUES) * sizeof(glm::vec4),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    depthMinMaxBuffers[1] = std::make_shared<sgl::vk::Buffer>(
            device, sgl::iceil(int(filteredLinesVertices.size()), BLOCK_SIZE_DEPTH_CUES * BLOCK_SIZE_DEPTH_CUES * 2)
                    * sizeof(glm::vec4),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_GPU_ONLY);
    filteredLinesVerticesBuffer = std::make_shared<sgl::vk::Buffer>(
            device, filteredLinesVertices.size() * sizeof(glm::vec4), filteredLinesVertices.data(),
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    computeDepthValuesPass->setLineVerticesBuffer(filteredLinesVerticesBuffer);
    computeDepthValuesPass->setDepthMinMaxOutBuffer(depthMinMaxBuffers[0]);
    for (int i = 0; i < 2; i++) {
        minMaxDepthReductionPass[i]->setDepthMinMaxBuffers(
                depthMinMaxBuffers[i], depthMinMaxBuffers[(i + 1) % 2]);
    }
}

void LineRenderer::computeDepthRange() {
    if (!filteredLinesVerticesBuffer) {
        return;
    }

    auto numVertices = uint32_t(filteredLinesVerticesBuffer->getSizeInBytes() / sizeof(glm::vec4));
    uint32_t numBlocks = sgl::iceil(int(numVertices), BLOCK_SIZE_DEPTH_CUES);
    computeDepthValuesPass->render();

    int iteration = 0;
    while (numBlocks > 1) {
        renderer->insertMemoryBarrier(
                VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
        minMaxDepthReductionPass[iteration]->setInputSize(numBlocks);
        minMaxDepthReductionPass[iteration]->render();
        numBlocks = sgl::iceil(int(numBlocks), BLOCK_SIZE_DEPTH_CUES * 2);
        iteration++;
    }

    outputDepthMinMaxBufferIndex = iteration % 2;
}

bool LineRenderer::setNewSettings(const SettingsMap& settings) {
    bool shallReloadGatherShader = false;

    float newLineWidth = lineWidth;
    if (settings.getValueOpt("line_width", lineWidth)) {
        if (newLineWidth != lineWidth && lineData) {
            lineData->setTriangleRepresentationDirty();
        }
    }
    float newBandWidth = bandWidth;
    if (settings.getValueOpt("band_width", bandWidth)) {
        if (newBandWidth != bandWidth && lineData) {
            lineData->setTriangleRepresentationDirty();
        }
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

    if (ambientOcclusionBaker) {
        ambientOcclusionBaker->setNewSettings(settings);
    }

    return shallReloadGatherShader;
}

void LineRenderer::reloadGatherShaderExternal() {
    if (lineData) {
        reloadGatherShader();
        setLineData(lineData, false);
    }
}

bool LineRenderer::getCanUseLiveUpdate(LineDataAccessType accessType) const {
    if (ambientOcclusionStrength > 0.0f) {
        return false;
    }
    if (lineData) {
        if (accessType == LineDataAccessType::TRIANGLE_MESH) {
            return !getIsTriangleRepresentationUsed() || lineData->getIsSmallDataSet();
        } else if (accessType == LineDataAccessType::FILTERED_LINES) {
            return lineData->getIsSmallDataSet();
        }
    }
    return true;
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
    if (lineWidth != displayLineWidthStaging) {
        displayLineWidthStaging = lineWidth;
        displayLineWidth = lineWidth;
        if (lineData) {
            lineData->setTriangleRepresentationDirty();
        }
        reRender = true;
        internalReRender = true;
    }
    ImGui::EditMode editMode = propertyEditor.addSliderFloatEdit(
            "Line Width", &displayLineWidth, MIN_LINE_WIDTH, MAX_LINE_WIDTH, "%.4f");
    if ((canUseLiveUpdate && editMode != ImGui::EditMode::NO_CHANGE)
            || (!canUseLiveUpdate && editMode == ImGui::EditMode::INPUT_FINISHED)) {
        lineWidth = displayLineWidth;
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

                AmbientOcclusionBakerType oldType = ambientOcclusionBakerType;
                if (propertyEditor.addCombo(
                        "Ambient Occlusion Mode", (int*)&ambientOcclusionBakerType,
                        AMBIENT_OCCLUSION_BAKER_TYPE_NAMES,
                        IM_ARRAYSIZE(AMBIENT_OCCLUSION_BAKER_TYPE_NAMES))) {
                    if (ambientOcclusionBakerType != oldType) {
                        setAmbientOcclusionBaker();
                    }
                }
            }
        }
    }

    if (useAmbientOcclusion && ambientOcclusionBaker) {
        if (ambientOcclusionBaker->renderGuiPropertyEditorNodes(propertyEditor)) {
            reRender = true;
            internalReRender = true;
            if (ambientOcclusionBaker->getIsStaticPrebaker()) {
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
        reloadGatherShader();
    }
    this->lineData = lineData;

    if (useAmbientOcclusion && ambientOcclusionBaker) {
        ambientOcclusionBaker->startAmbientOcclusionBaking(lineData, isNewData);
    }

    filteredLines.clear();
    filteredLinesVerticesBuffer = {};
    depthMinMaxBuffers[0] = {};
    depthMinMaxBuffers[1] = {};
    if (useDepthCues) {
        updateDepthCueGeometryData();
    }

    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentDataSetBaseSizeBytes(lineData->getBaseSizeInBytes());
    }

    if (isRasterizer) {
        if (lineRasterPass) {
            lineRasterPass->setLineData(lineData, isNewData);
        }
        if (hullRasterPass && lineData->hasSimulationMeshOutline()) {
            hullRasterPass->setLineData(lineData, isNewData);
        }
    }
}

void LineRenderer::reloadGatherShader() {
    if (isRasterizer) {
        if (lineRasterPass) {
            lineRasterPass->setShaderDirty();
        }
        if (hullRasterPass) {
            hullRasterPass->setShaderDirty();
        }
    }
}

void LineRenderer::renderHull() {
    if (lineData && lineData->hasSimulationMeshOutline() && lineData->getShallRenderSimulationMeshBoundary()) {
        hullRasterPass->render();
    }
}

int LineRenderer::tilingModeIndex = 2;
int LineRenderer::tileWidth = 2;
int LineRenderer::tileHeight = 8;
bool LineRenderer::tilingUseMortonCode = false;

bool LineRenderer::selectTilingModeUI(sgl::PropertyEditor& propertyEditor) {
    const char* indexingModeNames[] = { "1x1", "2x2", "2x8", "8x2", "4x4", "8x8", "8x8 Morton Code" };
    if (propertyEditor.addCombo(
            "Tiling Mode", (int*)&tilingModeIndex,
            indexingModeNames, IM_ARRAYSIZE(indexingModeNames))) {
        // Select new mode
        if (tilingModeIndex == 0) {
            // No tiling
            tileWidth = 1;
            tileHeight = 1;
        } else if (tilingModeIndex == 1) {
            tileWidth = 2;
            tileHeight = 2;
        } else if (tilingModeIndex == 2) {
            tileWidth = 2;
            tileHeight = 8;
        } else if (tilingModeIndex == 3) {
            tileWidth = 8;
            tileHeight = 2;
        } else if (tilingModeIndex == 4) {
            tileWidth = 4;
            tileHeight = 4;
        } else if (tilingModeIndex == 5) {
            tileWidth = 8;
            tileHeight = 8;
        } else if (tilingModeIndex == 6) {
            tileWidth = 8;
            tileHeight = 8;
        }

        return true;
    }
    return false;
}

void LineRenderer::setNewTilingMode(int newTileWidth, int newTileHeight, bool useMortonCode /* = false */) {
    tileWidth = newTileWidth;
    tileHeight = newTileHeight;
    tilingUseMortonCode = useMortonCode;

    // Select new mode.
    if (tileWidth == 1 && tileHeight == 1) {
        // No tiling.
        tilingModeIndex = 0;
    } else if (tileWidth == 2 && tileHeight == 2) {
        tilingModeIndex = 1;
    } else if (tileWidth == 2 && tileHeight == 8) {
        tilingModeIndex = 2;
    } else if (tileWidth == 8 && tileHeight == 2) {
        tilingModeIndex = 3;
    } else if (tileWidth == 4 && tileHeight == 4) {
        tilingModeIndex = 4;
    } else if (tileWidth == 8 && tileHeight == 8 && !useMortonCode) {
        tilingModeIndex = 5;
    } else if (tileWidth == 8 && tileHeight == 8 && useMortonCode) {
        tilingModeIndex = 6;
    } else {
        // Invalid mode, just set to mode 5, too.
        tilingModeIndex = 5;
    }
}

void LineRenderer::getScreenSizeWithTiling(int& screenWidth, int& screenHeight) {
    if (screenWidth % tileWidth != 0) {
        screenWidth = (screenWidth / tileWidth + 1) * tileWidth;
    }
    if (screenHeight % tileHeight != 0) {
        screenHeight = (screenHeight / tileHeight + 1) * tileHeight;
    }
}
