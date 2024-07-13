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

#include <ospray/ospray.h>

#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "OsprayRenderer.hpp"

bool OsprayRenderer::isOsprayInitialized = false;
bool OsprayRenderer::denoiserAvailable = false;

OsprayRenderer::OsprayRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("OSPRay Renderer", sceneData, transferFunctionWindow) {
    isRasterizer = false;

    int argc = sgl::FileUtils::get()->get_argc();
    const char** argv = const_cast<const char**>(sgl::FileUtils::get()->get_argv());

    if (!isOsprayInitialized) {
        OSPError ospError = ospInit(&argc, argv);
        if (ospError != OSP_NO_ERROR) {
            sgl::Logfile::get()->throwError(
                    "Error in OsprayRenderer::OsprayRenderer: Initialization of OSPRay failed.");
        }

        if (ospLoadModule("denoiser") == OSP_NO_ERROR) {
            denoiserAvailable = true;
        }

        isOsprayInitialized = true;
    }

    if (!denoiserAvailable) {
        useDenoiser = false;
    }
    if (useDenoiser) {
        frameBufferFormat = OSP_FB_RGBA32F;
    }

    ospRenderer = ospNewRenderer("scivis");
    backgroundColor = sceneData->clearColor->getFloatColorRGBA();
    ospSetVec4f(
            ospRenderer, "backgroundColor",
            backgroundColor.r, backgroundColor.g, backgroundColor.b, backgroundColor.a);
    ospSetInt(ospRenderer, "aoSamples", numAoSamples);
    ospCommit(ospRenderer);

    ospCamera = ospNewCamera("perspective");

    ospMaterial = ospNewMaterial("", "obj");
    ospSetVec3f(ospMaterial, "kd", 0.9f, 0.9f, 0.9f);
    ospSetVec3f(ospMaterial, "ks", 0.1f, 0.1f, 0.1f);
    ospSetInt(ospMaterial, "ns", 50);
    ospCommit(ospMaterial);

    OSPLight ambientLight = ospNewLight("ambient");
    ospSetFloat(ambientLight, "intensity", 0.7f);
    ospSetVec3f(ambientLight, "color", 1.0f, 1.0f, 1.0f);
    ospCommit(ambientLight);

    headLight = ospNewLight("distant");
    ospSetFloat(headLight, "intensity", 1.5f);
    ospSetVec3f(headLight, "color", 1.0f, 1.0f, 1.0f);
    ospSetVec3f(headLight, "direction", 1.0f, 0.0f, 0.0f);
    ospSetFloat(headLight, "angularDiameter", 0.0f);
    ospCommit(headLight);

    lights = { ambientLight, headLight };
    OSPData ospLightsShared = ospNewSharedData1D(lights.data(), OSP_LIGHT, lights.size());
    ospCommit(ospLightsShared);
    ospLights = ospNewData(OSP_LIGHT, lights.size());
    ospCopyData(ospLightsShared, ospLights);
    ospCommit(ospLights);
    ospRelease(ambientLight);
    ospRelease(ospLightsShared);

    blitRenderPass = std::make_shared<sgl::vk::BlitRenderPass>(
            renderer, std::vector<std::string>{ "Blit.Vertex.MirrorVertical", "Blit.Fragment" });
    // This was "glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);" / pre-multiplied alpha in OpenGL.
    //blitRenderPass->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
    //blitRenderPass->setBlendMode(sgl::vk::BlendMode::BACK_TO_FRONT_PREMUL_ALPHA);

    onClearColorChanged();
}

OsprayRenderer::~OsprayRenderer() {
    if (ospFrameBuffer) {
        ospRelease(ospFrameBuffer);
    }
    if (ospRenderer) {
        ospRelease(ospRenderer);
    }
    if (ospCamera) {
        ospRelease(ospCamera);
    }
    if (ospGeometry) {
        ospRelease(ospGeometry);
        ospGeometry = nullptr;
    }
    if (ospWorld) {
        ospRelease(ospWorld);
    }
    if (ospMaterial) {
        ospRelease(ospMaterial);
    }
    if (ospLights) {
        ospRelease(ospLights);
    }
    if (headLight) {
        ospRelease(headLight);
    }
}

void OsprayRenderer::setNewState(const InternalState& newState) {
    std::string geometryModeName;
    if (newState.rendererSettings.getValueOpt("geometryMode", geometryModeName)) {
        sgl::toLower(geometryModeName);
        if (geometryModeName == "triangle_mesh") {
            geometryMode = GeometryMode::TRIANGLE_MESH;
        } else if (geometryModeName == "curves") {
            geometryMode = GeometryMode::CURVES;
        }
        if (lineData) {
            setLineData(lineData, false);
        }
    }
}

void OsprayRenderer::reloadGatherShader() {
}

bool OsprayRenderer::getIsTriangleRepresentationUsed() const {
    return lineData && geometryMode == GeometryMode::TRIANGLE_MESH;
}

void OsprayRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);
    currentLineWidth = LineRenderer::getLineWidth();

    // Clear old curve and triangle mesh data.
    curvesData.curveIndices.clear();
    curvesData.vertexPositionsAndRadii.clear();
    curvesData.vertexAttributes.clear();
    curvesData.vertexColors.clear();
    triangleMesh.triangleIndices.clear();
    triangleMesh.vertexPositions.clear();
    triangleMesh.vertexNormals.clear();
    triangleMesh.vertexAttributes.clear();
    triangleMesh.vertexColors.clear();

    if (ospWorld) {
        ospRelease(ospWorld);
    }
    ospWorld = ospNewWorld();

    size_t memoryPrev = 0;
    if ((*sceneData->performanceMeasurer)) {
        memoryPrev = getUsedSystemMemoryBytes();
    }
    if (geometryMode == GeometryMode::TRIANGLE_MESH) {
        loadTriangleMeshData(lineData);
        if (triangleMesh.triangleIndices.empty()) {
            dirty = false;
            reRender = true;
            return;
        }
    } else if (geometryMode == GeometryMode::CURVES) {
        loadCurvesData(lineData);
        if (curvesData.curveIndices.empty()) {
            dirty = false;
            reRender = true;
            return;
        }
    }

    onHasMoved();

    ospSetObject(ospWorld, "light", ospLights);
    ospCommit(ospWorld);
    if ((*sceneData->performanceMeasurer)) {
        size_t memoryAfter = getUsedSystemMemoryBytes();
        (*sceneData->performanceMeasurer)->setCurrentDataSetBufferSizeBytes(memoryAfter - memoryPrev);
    }

    if (lineData->getType() != currentDataSetType) {
        //ospSetInt(ospMaterial, "ns", lineData->getType() == DATA_SET_TYPE_STRESS_LINES ? 30 : 50);
        ospCommit(ospMaterial);
        currentDataSetType = lineData->getType();
    }

    frameParameterChanged = true;
    dirty = false;
    reRender = true;
}

void OsprayRenderer::computeAttributeColors(
        const std::vector<float>& attributes, std::vector<glm::vec4>& colors) {
    const std::vector<sgl::Color16>& colorLookupTable = transferFunctionWindow.getTransferFunctionMap_sRGB();
    glm::vec2 range = transferFunctionWindow.getSelectedRange();
    const float delta = 1.0f / (range.y - range.x);

    colors.resize(attributes.size());
    for (size_t i = 0; i < attributes.size(); i++) {
        const float t = (attributes.at(i) - range.x) * delta;
        const int index = int(t * float(colorLookupTable.size() - 1));
        const float residual = t * float(colorLookupTable.size() - 1) - float(index);

        if (index < 0) {
            colors.at(i) = colorLookupTable.front().getFloatColorRGBA();
        } else if (index >= int(colorLookupTable.size() - 1)) {
            colors.at(i) = colorLookupTable.back().getFloatColorRGBA();
        } else {
            glm::vec4 color0 = colorLookupTable.at(index).getFloatColorRGBA();
            glm::vec4 color1 = colorLookupTable.at(index + 1).getFloatColorRGBA();
            colors.at(i) = glm::mix(color0, color1, residual);
        }
    }
}

void OsprayRenderer::loadTriangleMeshData(LineDataPtr& lineData) {
    lineData->getTriangleMesh(
            this,
            triangleMesh.triangleIndices, triangleMesh.vertexPositions, triangleMesh.vertexNormals,
            triangleMesh.vertexAttributes);
    computeAttributeColors(triangleMesh.vertexAttributes, triangleMesh.vertexColors);

    if (ospGeometry) {
        ospRelease(ospGeometry);
        ospGeometry = nullptr;
    }

    if (triangleMesh.triangleIndices.empty()) {
        return;
    }

    ospGeometry = ospNewGeometry("mesh");

    OSPData triangleIndexData = ospNewSharedData1D(
            triangleMesh.triangleIndices.data(), OSP_VEC3UI, triangleMesh.triangleIndices.size() / 3);
    OSPData vertexPositionData = ospNewSharedData1D(
            triangleMesh.vertexPositions.data(), OSP_VEC3F, triangleMesh.vertexPositions.size());
    OSPData vertexNormalData = ospNewSharedData1D(
            triangleMesh.vertexNormals.data(), OSP_VEC3F, triangleMesh.vertexNormals.size());
    OSPData vertexColorData = ospNewSharedData1D(
            triangleMesh.vertexColors.data(), OSP_VEC4F, triangleMesh.vertexColors.size());
    ospCommit(triangleIndexData);
    ospCommit(vertexPositionData);
    ospCommit(vertexNormalData);
    ospCommit(vertexColorData);

    ospSetObject(ospGeometry, "index", triangleIndexData);
    ospSetObject(ospGeometry, "vertex.position", vertexPositionData);
    ospSetObject(ospGeometry, "vertex.normal", vertexNormalData);
    ospSetObject(ospGeometry, "vertex.color", vertexColorData);
    ospCommit(ospGeometry);

    ospRelease(triangleIndexData);
    ospRelease(vertexPositionData);
    ospRelease(vertexNormalData);
    ospRelease(vertexColorData);

    finalizeLoadedData();
}

void OsprayRenderer::loadCurvesData(LineDataPtr& lineData) {
    Trajectories trajectories = lineData->filterTrajectoryData();

    const int selectedAttributeIndex = lineData->getSelectedAttributeIndex();
    const float radius = LineRenderer::getLineWidth() / 2.0f;

    for (const Trajectory& trajectory : trajectories) {
        auto offset = uint32_t(curvesData.vertexPositionsAndRadii.size());
        auto numPoints = int(trajectory.positions.size());
        if (curveBasis == OSP_LINEAR) {
            for (int i = 0; i <= numPoints - 2; i++) {
                curvesData.curveIndices.emplace_back(offset + uint32_t(i));
            }
        } else if (curveBasis == OSP_BEZIER) {
            for (int i = 0; i <= numPoints - 4; i += 3) {
                curvesData.curveIndices.emplace_back(offset + uint32_t(i));
            }
        } else if (curveBasis == OSP_BSPLINE) {
            for (int i = 0; i <= numPoints - 4; i++) {
                curvesData.curveIndices.emplace_back(offset + uint32_t(i));
            }
        } else if (curveBasis == OSP_HERMITE) {
            for (int i = 0; i <= numPoints - 4; i++) {
                curvesData.curveIndices.emplace_back(offset + uint32_t(i));
            }
        } else if (curveBasis == OSP_CATMULL_ROM) {
            for (int i = 0; i <= numPoints - 4; i++) {
                curvesData.curveIndices.emplace_back(offset + uint32_t(i));
            }
        }
        numPoints = std::max(numPoints, 0);
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            const glm::vec3& pos = trajectory.positions.at(i);
            curvesData.vertexPositionsAndRadii.emplace_back(pos.x, pos.y, pos.z, radius);
            curvesData.vertexAttributes.emplace_back(trajectory.attributes.at(selectedAttributeIndex).at(i));
        }
    }
    computeAttributeColors(curvesData.vertexAttributes, curvesData.vertexColors);

    if (ospGeometry) {
        ospRelease(ospGeometry);
        ospGeometry = nullptr;
    }

    if (curvesData.curveIndices.empty()) {
        return;
    }

    ospGeometry = ospNewGeometry("curve");
    ospSetParam(ospGeometry, "type", OSP_UCHAR, &curveType);
    ospSetParam(ospGeometry, "basis", OSP_UCHAR, &curveBasis);

    OSPData curveIndexData = ospNewSharedData1D(
            curvesData.curveIndices.data(), OSP_UINT, curvesData.curveIndices.size());
    OSPData vertexPositionData = ospNewSharedData1D(
            curvesData.vertexPositionsAndRadii.data(), OSP_VEC4F,
            curvesData.vertexPositionsAndRadii.size());
    OSPData vertexColorData = ospNewSharedData1D(
            curvesData.vertexColors.data(), OSP_VEC4F, curvesData.vertexColors.size());
    ospCommit(curveIndexData);
    ospCommit(vertexPositionData);
    ospCommit(vertexColorData);

    ospSetObject(ospGeometry, "index", curveIndexData);
    ospSetObject(ospGeometry, "vertex.position_radius", vertexPositionData);
    ospSetObject(ospGeometry, "vertex.color", vertexColorData);
    ospCommit(ospGeometry);

    ospRelease(curveIndexData);
    ospRelease(vertexPositionData);
    ospRelease(vertexColorData);

    finalizeLoadedData();
}

void OsprayRenderer::finalizeLoadedData() {
    OSPGeometricModel ospGeometricModel = ospNewGeometricModel(ospGeometry);
    ospSetObject(ospGeometricModel, "material", ospMaterial);
    ospCommit(ospGeometricModel);

    OSPGroup ospGroup = ospNewGroup();
    ospSetObjectAsData(ospGroup, "geometry", OSP_GEOMETRIC_MODEL, ospGeometricModel);
    ospCommit(ospGroup);
    ospRelease(ospGeometricModel);

    OSPInstance ospInstance = ospNewInstance(ospGroup);
    ospCommit(ospInstance);
    ospRelease(ospGroup);

    ospSetObjectAsData(ospWorld, "instance", OSP_INSTANCE, ospInstance);
    ospCommit(ospWorld);
    ospRelease(ospInstance);
}

void OsprayRenderer::onLineRadiusChanged() {
    frameParameterChanged = true;

    if (geometryMode == GeometryMode::TRIANGLE_MESH) {
        return;
    }

    if (geometryMode == GeometryMode::CURVES) {
        const float radius = LineRenderer::getLineWidth() / 2.0f;
        for (glm::vec4& pt : curvesData.vertexPositionsAndRadii) {
            pt.w = radius;
        }

        OSPData vertexPositionData = ospNewSharedData1D(
                curvesData.vertexPositionsAndRadii.data(), OSP_VEC4F,
                curvesData.vertexPositionsAndRadii.size());
        ospCommit(vertexPositionData);
        ospSetObject(ospGeometry, "vertex.position_radius", vertexPositionData);
        ospCommit(ospGeometry);
        ospRelease(vertexPositionData);
        finalizeLoadedData();
    }
}

void OsprayRenderer::onResolutionChanged() {
    LineRenderer::onResolutionChanged();

    uint32_t width = *sceneData->viewportWidth;
    uint32_t height = *sceneData->viewportHeight;

    if (previousWidth != width || previousHeight != height) {
        onHasMoved();
    }

    size_t bufferSize = width * height;
    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.usage =
            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    if (frameBufferFormat == OSP_FB_RGBA32F) {
        imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
        bufferSize *= sizeof(float) * 4;
    } else {
        imageSettings.format = VK_FORMAT_R8G8B8A8_UNORM;
        bufferSize *= sizeof(uint32_t);
    }
    renderTexture = std::make_shared<sgl::vk::Texture>(renderer->getDevice(), imageSettings, samplerSettings);

    blitRenderPass->setInputTexture(renderTexture);
    blitRenderPass->setOutputImage((*sceneData->sceneTexture)->getImageView());
    blitRenderPass->recreateSwapchain(width, height);

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    stagingBuffers.clear();
    stagingBuffers.reserve(swapchain->getNumImages());
    for (size_t i = 0; i < swapchain->getNumImages(); i++) {
        stagingBuffers.push_back(std::make_shared<sgl::vk::Buffer>(
                renderer->getDevice(), bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU));
    }

    if (ospFrameBuffer) {
        ospRelease(ospFrameBuffer);
    }
    ospFrameBuffer = ospNewFrameBuffer(
            int(width), int(height), frameBufferFormat,
            OSP_FB_COLOR | OSP_FB_ACCUM | OSP_FB_VARIANCE);
    if (useDenoiser) {
        updateDenoiserMode();
    }
}

void OsprayRenderer::onClearColorChanged() {
    glm::vec4 newBackgroundColor = sceneData->clearColor->getFloatColorRGBA();
    if (newBackgroundColor != backgroundColor) {
        backgroundColor = newBackgroundColor;
        ospSetVec4f(
                ospRenderer, "backgroundColor",
                backgroundColor.r, backgroundColor.g, backgroundColor.b, backgroundColor.a);
        ospCommit(ospRenderer);
        frameParameterChanged = true;
    }

    blitRenderPass->setAttachmentClearColor(newBackgroundColor);
}

void OsprayRenderer::onTransferFunctionMapRebuilt() {
    frameParameterChanged = true;

    if (geometryMode == GeometryMode::TRIANGLE_MESH) {
        computeAttributeColors(triangleMesh.vertexAttributes, triangleMesh.vertexColors);
        OSPData vertexColorData = ospNewSharedData1D(
                triangleMesh.vertexColors.data(), OSP_VEC4F, triangleMesh.vertexColors.size());
        ospCommit(vertexColorData);
        ospSetObject(ospGeometry, "vertex.color", vertexColorData);
        ospCommit(ospGeometry);
        ospRelease(vertexColorData);
    } else if (geometryMode == GeometryMode::CURVES) {
        computeAttributeColors(curvesData.vertexAttributes, curvesData.vertexColors);
        OSPData vertexColorData = ospNewSharedData1D(
                curvesData.vertexColors.data(), OSP_VEC4F, curvesData.vertexColors.size());
        ospCommit(vertexColorData);
        ospSetObject(ospGeometry, "vertex.color", vertexColorData);
        ospCommit(ospGeometry);
        ospRelease(vertexColorData);
    }
}

bool OsprayRenderer::needsReRender() {
    return LineRenderer::needsReRender() || frameParameterChanged
            || frameVarianceRelativeChange > FRAME_VARIANCE_RERENDER_THRESHOLD
            || frameVarianceRelativeChangePrev > FRAME_VARIANCE_RERENDER_THRESHOLD
            || currentLineWidth != LineRenderer::getLineWidth();
}

void OsprayRenderer::notifyReRenderTriggeredExternally() {
    internalReRender = false;
    frameParameterChanged = true;
}

void OsprayRenderer::onHasMoved() {
    LineRenderer::onHasMoved();

    previousWidth = *sceneData->viewportWidth;
    previousHeight = *sceneData->viewportHeight;
    int width = std::max(int(*sceneData->viewportWidth), 1);
    int height = std::max(int(*sceneData->viewportHeight), 1);
    glm::mat4 invViewMatrix = glm::inverse(sceneData->camera->getViewMatrix());
    glm::vec3 upDir = invViewMatrix[1];
    glm::vec3 lookDir = -invViewMatrix[2];
    glm::vec3 position = invViewMatrix[3];

    // Update the camera settings.
    float camPosition[] = { position.x, position.y, position.z };
    float camDirection[] = { lookDir.x, lookDir.y, lookDir.z };
    float camUp[] = { upDir.x, upDir.y, upDir.z };
    ospSetFloat(ospCamera, "aspect", float(width) / float(height));
    ospSetFloat(ospCamera, "fovy", glm::degrees(sceneData->camera->getFOVy()));
    ospSetParam(ospCamera, "position", OSP_VEC3F, camPosition);
    ospSetParam(ospCamera, "direction", OSP_VEC3F, camDirection);
    ospSetParam(ospCamera, "up", OSP_VEC3F, camUp);
    ospCommit(ospCamera);

    // Update the headlight direction.
    ospSetVec3f(headLight, "direction", lookDir.x, lookDir.y, lookDir.z);
    ospCommit(headLight);

    frameParameterChanged = true;
}

void OsprayRenderer::render() {
    LineRenderer::renderBase();

    if (!ospGeometry) {
        renderer->transitionImageLayout(
                (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        (*sceneData->sceneTexture)->getImageView()->clearColor(
                sceneData->clearColor->getFloatColorRGBA(), renderer->getVkCommandBuffer());
        return;
    }

    if (currentLineWidth != LineRenderer::getLineWidth()) {
        currentLineWidth = LineRenderer::getLineWidth();
        onLineRadiusChanged();
    }

    if (frameParameterChanged) {
        ospResetAccumulation(ospFrameBuffer);
        frameParameterChanged = false;
        lastFrameVariance = 1e9f;
    }

    //OSPFuture future = ospRenderFrame(ospFrameBuffer, ospRenderer, ospCamera, ospWorld);
    //ospWait(future, OSP_TASK_FINISHED);
    //ospRelease(future);
    float frameVariance = ospRenderFrameBlocking(ospFrameBuffer, ospRenderer, ospCamera, ospWorld);
    frameVariance = std::min(frameVariance, 1e8f);
    const void* imageData = ospMapFrameBuffer(ospFrameBuffer, OSP_FB_COLOR);

    auto* swapchain = sgl::AppSettings::get()->getSwapchain();
    auto& stagingBuffer = stagingBuffers.at(swapchain->getImageIndex());
    void* stagingBufferData = stagingBuffer->mapMemory();
    memcpy(stagingBufferData, imageData, stagingBuffer->getSizeInBytes());
    stagingBuffer->unmapMemory();

    renderer->transitionImageLayout(renderTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    renderTexture->getImage()->copyFromBuffer(stagingBuffer, renderer->getVkCommandBuffer());

    ospUnmapFrameBuffer(ospFrameBuffer, ospFrameBuffer);

    //float frameVariance = std::min(ospGetVariance(ospFrameBuffer), 1e8f);
    frameVarianceRelativeChangePrev = frameVarianceRelativeChange;
    frameVarianceRelativeChange = lastFrameVariance / frameVariance - 1.0f;
    lastFrameVariance = frameVariance;

    //renderer->transitionImageLayout(
    //        renderTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    //renderer->transitionImageLayout(
    //        (*sceneData->sceneTexture)->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    //renderTexture->getImage()->blit(
    //        (*sceneData->sceneTexture)->getImage(), renderer->getVkCommandBuffer());

    renderer->transitionImageLayout(renderTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    blitRenderPass->render();
}

void OsprayRenderer::updateDenoiserMode() {
    if (useDenoiser) {
        OSPImageOperation denoiserImageOperation = ospNewImageOperation("denoiser");
        ospCommit(denoiserImageOperation);
        ospSetObjectAsData(ospFrameBuffer, "imageOperation", OSP_IMAGE_OPERATION, denoiserImageOperation);
        ospRelease(denoiserImageOperation);
    } else {
        ospRemoveParam(ospFrameBuffer, "imageOperation");
    }
    ospCommit(ospFrameBuffer);
}

const char* const GEOMETRY_MODE_NAMES[] = {
        "Triangle Mesh", "Curves"
};
const char* const CURVE_TYPE_NAMES[] = {
        "Round", "Flat", "Ribbon", "Disjoint"
};
const char* const CURVE_BASIS_NAMES[] = {
        "Linear", "Bezier", "B-Spline", "Hermite", "Catmull-Rom"
};
const char* const FRAME_BUFFER_FORMAT_NAMES[] = {
        "RGBA8", "sRGBA8", "RGBA32F",
};

void OsprayRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    bool isDataDirty = false;

    if (propertyEditor.addCombo(
            "Geometry Mode", (int*)&geometryMode, GEOMETRY_MODE_NAMES,
            IM_ARRAYSIZE(GEOMETRY_MODE_NAMES))) {
        isDataDirty = true;
    }

    if (geometryMode == GeometryMode::CURVES) {
        int curveTypeInt = int(curveType);
        if (propertyEditor.addCombo(
                "Curve Type", &curveTypeInt, CURVE_TYPE_NAMES,
                IM_ARRAYSIZE(CURVE_TYPE_NAMES))) {
            curveType = OSPCurveType(curveTypeInt);
            if (curveType == OSP_RIBBON) {
                sgl::Logfile::get()->writeError(
                        "Warning in OsprayRenderer::renderGuiPropertyEditorNodes: The curve type 'Ribbon' is currently not "
                        "supported, as it would need additional normal data.");
            }
            isDataDirty = true;
        }
        int curveBasisInt = int(curveBasis);
        if (propertyEditor.addCombo(
                "Curve Basis", &curveBasisInt, CURVE_BASIS_NAMES,
                IM_ARRAYSIZE(CURVE_BASIS_NAMES))) {
            curveBasis = OSPCurveBasis(curveBasisInt);
            if (curveBasis == OSP_HERMITE) {
                sgl::Logfile::get()->writeError(
                        "Warning in OsprayRenderer::renderGuiPropertyEditorNodes: The curve basis type 'Hermite' is currently not "
                        "supported, as it would need additional tangent data.");
            }
            isDataDirty = true;
        }
    }

    int frameBufferFormatInt = int(frameBufferFormat) - 1;
    if (propertyEditor.addCombo(
            "Framebuffer Format", &frameBufferFormatInt, FRAME_BUFFER_FORMAT_NAMES,
            IM_ARRAYSIZE(FRAME_BUFFER_FORMAT_NAMES))) {
        frameBufferFormat = OSPFrameBufferFormat(frameBufferFormatInt + 1);
        onResolutionChanged();
        frameParameterChanged = true;
    }

    if (propertyEditor.addSliderInt("#AO Samples", &numAoSamples, 0, 64)) {
        ospSetInt(ospRenderer, "aoSamples", numAoSamples);
        ospCommit(ospRenderer);
        frameParameterChanged = true;
    }
    if (numAoSamples > 0 && denoiserAvailable) {
        if (propertyEditor.addCheckbox("Use Denoiser", &useDenoiser)) {
            if (useDenoiser && frameBufferFormat != OSP_FB_RGBA32F) {
                frameBufferFormat = OSP_FB_RGBA32F;
                onResolutionChanged();
            }
            updateDenoiserMode();
            frameParameterChanged = true;
        }
    }

    if (isDataDirty) {
        setLineData(lineData, false);
        reRender = true;
    }
}
