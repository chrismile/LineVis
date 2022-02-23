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

#include <Utils/File/Logfile.hpp>
#include <Graphics/Shader/ShaderManager.hpp> // TODO
#include <Graphics/Shader/ShaderAttributes.hpp> // TODO
#include <Graphics/Renderer.hpp> // TODO
#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/AccelerationStructure.hpp>

#include <ImGui/imgui.h>
#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/TriangleNormals.hpp"
#include "Utils/MeshSmoothing.hpp"
#include "Renderers/LineRenderer.hpp"
#include "Mesh/MeshBoundarySurface.hpp"
#include "LineData.hpp"

LineData::LinePrimitiveMode LineData::linePrimitiveMode = LineData::LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER;
int LineData::tubeNumSubdivisions = 6;
bool LineData::renderThickBands = true;
float LineData::minBandThickness = 0.15f;

const char *const LINE_PRIMITIVE_MODE_DISPLAYNAMES[] = {
        "Ribbon (Programmable Fetch)", "Ribbon (Geometry Shader)", "Tube (Geometry Shader)",
        "Ribbon Bands", "Tube Bands"
};

LineData::LineData(sgl::TransferFunctionWindow &transferFunctionWindow, DataSetType dataSetType)
        : dataSetType(dataSetType), transferFunctionWindow(transferFunctionWindow) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    lineUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(lineUniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
}

LineData::~LineData() = default;

bool LineData::setNewSettings(const SettingsMap& settings) {
    std::string attributeName;
    if (settings.getValueOpt("attribute", attributeName)) {
        int i;
        for (i = 0; i < int(attributeNames.size()); i++) {
            if (attributeNames.at(i) == attributeName) {
                selectedAttributeIndexUi = i;
                break;
            }
        }
        if (i != int(attributeNames.size())) {
            setSelectedAttributeIndex(selectedAttributeIndexUi);
        } else {
            sgl::Logfile::get()->writeError(
                    "LineData::setNewSettings: Invalid attribute name \"" + attributeName + "\".");
        }
    }

    return false;
}

bool LineData::getCanUseLiveUpdate(LineDataAccessType accessType) const {
    if (accessType == LineDataAccessType::FILTERED_LINES) {
        return getIsSmallDataSet();
    }
    bool canUseLiveUpdate = std::all_of(
            lineRenderersCached.cbegin(), lineRenderersCached.cend(), [accessType](LineRenderer* lineRenderer){
                return lineRenderer->getCanUseLiveUpdate(accessType);
            });
    return canUseLiveUpdate;
}

bool LineData::setUseCappedTubes(LineRenderer* lineRenderer, bool cappedTubes) {
    bool useCappedTubesOld = useCappedTubes;
    useCappedTubes = cappedTubes;
    if (useCappedTubesOld != cappedTubes) {
        triangleRepresentationDirty = true;
        if (lineRenderer->isVulkanRenderer && !lineRenderer->isRasterizer) {
            return true;
        }
    }
    return false;
}

bool LineData::renderGuiPropertyEditorNodesRenderer(sgl::PropertyEditor& propertyEditor, LineRenderer* lineRenderer) {
    bool shallReloadGatherShader = false;

    if (lineRenderer->getIsRasterizer()) {
        int numPrimitiveModes = IM_ARRAYSIZE(LINE_PRIMITIVE_MODE_DISPLAYNAMES);
        if (!hasBandsData) {
            numPrimitiveModes -= 2;
        }
        if (lineRenderer->getRenderingMode() != RENDERING_MODE_OPACITY_OPTIMIZATION && propertyEditor.addCombo(
                "Line Primitives", (int*)&linePrimitiveMode,
                LINE_PRIMITIVE_MODE_DISPLAYNAMES, numPrimitiveModes)) {
            dirty = true;
            shallReloadGatherShader = true;
        }
    }

    bool isTriangleRepresentationUsed = lineRenderer && lineRenderer->getIsTriangleRepresentationUsed();
    if (isTriangleRepresentationUsed
        || (lineRenderer->getIsRasterizer() && lineRenderer->getRenderingMode() != RENDERING_MODE_OPACITY_OPTIMIZATION
            && (linePrimitiveMode == LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER
                || linePrimitiveMode == LINE_PRIMITIVES_TUBE_BAND))) {
        if (propertyEditor.addSliderInt("Tube Subdivisions", &tubeNumSubdivisions, 3, 8)) {
            if (lineRenderer->getIsRasterizer()) {
                shallReloadGatherShader = true;
            }
            setTriangleRepresentationDirty();
        }
    }

    if (lineRenderer && (linePrimitiveMode == LINE_PRIMITIVES_TRIANGLE_MESH || lineRenderer->isVulkanRenderer)) {
        if (propertyEditor.addCheckbox("Capped Tubes", &useCappedTubes)) {
            triangleRepresentationDirty = true;
            if (lineRenderer->isVulkanRenderer && !lineRenderer->isRasterizer) {
                shallReloadGatherShader = true;
            }
        }
    }

    propertyEditor.addCheckbox("Render Color Legend", &shallRenderColorLegendWidgets);

    if (!simulationMeshOutlineTriangleIndices.empty()) {
        ImGui::EditMode editModeHullOpacity = propertyEditor.addSliderFloatEdit(
                "Hull Opacity", &hullOpacity, 0.0f, 1.0f, "%.4f");
        if (editModeHullOpacity != ImGui::EditMode::NO_CHANGE) {
            shallRenderSimulationMeshBoundary = hullOpacity > 0.0f;
            reRender = true;
        }
        if (lineRenderer && lineRenderer->isVulkanRenderer && !lineRenderer->isRasterizer
                && editModeHullOpacity == ImGui::EditMode::INPUT_FINISHED) {
            lineRenderer->setRenderSimulationMeshHull(shallRenderSimulationMeshBoundary);
        }
        if (shallRenderSimulationMeshBoundary) {
            if (propertyEditor.addColorEdit3("Hull Color", &hullColor.r)) {
                reRender = true;
            }
        }
    }

    return shallReloadGatherShader;
}

bool LineData::renderGuiRenderingSettingsPropertyEditor(sgl::PropertyEditor& propertyEditor) {
    if (getUseBandRendering()) {
        bool canUseLiveUpdate = getCanUseLiveUpdate(LineDataAccessType::TRIANGLE_MESH);
        ImGui::EditMode editMode = propertyEditor.addSliderFloatEdit(
                "Band Width", &LineRenderer::bandWidth,
                LineRenderer::MIN_BAND_WIDTH, LineRenderer::MAX_BAND_WIDTH, "%.4f");
        if ((canUseLiveUpdate && editMode != ImGui::EditMode::NO_CHANGE)
            || (!canUseLiveUpdate && editMode == ImGui::EditMode::INPUT_FINISHED)) {
            reRender = true;
            setTriangleRepresentationDirty();
        }
    }
    return false;
}

bool LineData::renderGuiWindowSecondary() {
    return false;
}

bool LineData::renderGuiOverlay() {
    bool shallReloadGatherShader = false;

    if (shallRenderColorLegendWidgets && !colorLegendWidgets.empty()) {
        colorLegendWidgets.at(selectedAttributeIndex).setAttributeMinValue(
                transferFunctionWindow.getSelectedRangeMin());
        colorLegendWidgets.at(selectedAttributeIndex).setAttributeMaxValue(
                transferFunctionWindow.getSelectedRangeMax());
        colorLegendWidgets.at(selectedAttributeIndex).renderGui();
    }

    return shallReloadGatherShader;
}

void LineData::setLineRenderers(const std::vector<LineRenderer*>& lineRenderers) {
    lineRenderersCached = lineRenderers;
}

bool LineData::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = false;

    // Switch importance criterion.
    if (propertyEditor.addCombo(
            "Attribute", (int*)&selectedAttributeIndexUi,
            attributeNames.data(), int(attributeNames.size()))) {
        setSelectedAttributeIndex(selectedAttributeIndexUi);
    }

    return shallReloadGatherShader;
}

void LineData::setClearColor(const sgl::Color& clearColor) {
    for (auto& colorLegendWidget : colorLegendWidgets) {
        colorLegendWidget.setClearColor(clearColor);
    }
}

void LineData::setSelectedAttributeIndex(int attributeIndex) {
    if (this->selectedAttributeIndex != attributeIndex) {
        dirty = true;
        this->selectedAttributeIndex = attributeIndex;
    }
    recomputeHistogram();
}

int LineData::getAttributeNameIndex(const std::string& attributeName) {
    auto it = std::find(attributeNames.begin(), attributeNames.end(), attributeName);
    if (it != attributeNames.end()) {
        return int(it - attributeNames.begin());
    } else {
        sgl::Logfile::get()->throwError(
                "Error in LineData::getAttributeNameIndex: Couldn't find attribute with name \""
                + attributeName + "\".");
        return -1;
    }
}

void LineData::onTransferFunctionMapRebuilt() {
    recomputeColorLegend();
}

void LineData::recomputeColorLegend() {
    for (auto& colorLegendWidget : colorLegendWidgets) {
        colorLegendWidget.setTransferFunctionColorMap(
                transferFunctionWindow.getTransferFunctionMap_sRGB());
        colorLegendWidget.setAttributeMinValue(
                transferFunctionWindow.getSelectedRangeMin());
        colorLegendWidget.setAttributeMinValue(
                transferFunctionWindow.getSelectedRangeMax());
    }
}

void LineData::rebuildInternalRepresentationIfNecessary() {
    if (dirty || triangleRepresentationDirty) {
        sgl::AppSettings::get()->getPrimaryDevice()->waitIdle();
        //updateMeshTriangleIntersectionDataStructure();

        vulkanTubeTriangleRenderData = {};
        vulkanTubeAabbRenderData = {};
        vulkanHullTriangleRenderData = {};
        tubeTriangleBottomLevelAS = {};
        tubeAabbBottomLevelAS = {};
        hullTriangleBottomLevelAS = {};
        tubeTriangleTopLevelAS = {};
        tubeTriangleAndHullTopLevelAS = {};
        tubeAabbTopLevelAS = {};
        tubeAabbAndHullTopLevelAS = {};

        dirty = false;
        triangleRepresentationDirty = false;
    }
}

std::vector<std::string> LineData::getShaderModuleNames() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
        return {
                "GeometryPassNormal.Programmable.Vertex",
                "GeometryPassNormal.Fragment"
        };
    } else if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_GEOMETRY_SHADER) {
        return {
                "GeometryPassNormal.VBO.Vertex",
                "GeometryPassNormal.VBO.Geometry",
                "GeometryPassNormal.Fragment"
        };
    } else if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER) {
        return {
                "GeometryPassNormalTube.VBO.Vertex",
                "GeometryPassNormalTube.VBO.Geometry",
                "GeometryPassNormalTube.Fragment"
        };
    } else if (linePrimitiveMode == LINE_PRIMITIVES_BAND) {
        return {
                "GeometryPassNormalBand.VBO.Vertex",
                "GeometryPassNormalBand.VBO.Geometry",
                "GeometryPassNormalBand.Fragment"
        };
    } else if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_BAND) {
        return {
                "GeometryPassNormalTube.VBO.Vertex",
                "GeometryPassNormalTube.VBO.Geometry",
                "GeometryPassNormalTube.Fragment"
        };
    } else {
        sgl::Logfile::get()->writeError("Error in LineData::reloadGatherShaderOpenGL: Invalid line primitive mode.");
        return {};
    }
}

sgl::ShaderProgramPtr LineData::reloadGatherShaderOpenGL() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderProgramPtr shaderProgramPtr;
    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
        shaderProgramPtr = sgl::ShaderManager->getShaderProgram({
                "GeometryPassNormal.Programmable.Vertex",
                "GeometryPassNormal.Fragment"
        });
    } else if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_GEOMETRY_SHADER) {
        shaderProgramPtr = sgl::ShaderManager->getShaderProgram({
                "GeometryPassNormal.VBO.Vertex",
                "GeometryPassNormal.VBO.Geometry",
                "GeometryPassNormal.Fragment"
        });
    } else if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER) {
        sgl::ShaderManager->addPreprocessorDefine("NUM_TUBE_SUBDIVISIONS", tubeNumSubdivisions);
        shaderProgramPtr = sgl::ShaderManager->getShaderProgram({
                "GeometryPassNormalTube.VBO.Vertex",
                "GeometryPassNormalTube.VBO.Geometry",
                "GeometryPassNormalTube.Fragment"
        });
        sgl::ShaderManager->removePreprocessorDefine("NUM_TUBE_SUBDIVISIONS");
    } else if (linePrimitiveMode == LINE_PRIMITIVES_BAND) {
        shaderProgramPtr = sgl::ShaderManager->getShaderProgram({
                "GeometryPassNormalBand.VBO.Vertex",
                "GeometryPassNormalBand.VBO.Geometry",
                "GeometryPassNormalBand.Fragment"
        });
    } else if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_BAND) {
        sgl::ShaderManager->addPreprocessorDefine("NUM_TUBE_SUBDIVISIONS", tubeNumSubdivisions);
        shaderProgramPtr = sgl::ShaderManager->getShaderProgram({
                "GeometryPassNormalTube.VBO.Vertex",
                "GeometryPassNormalTube.VBO.Geometry",
                "GeometryPassNormalTube.Fragment"
        });
        sgl::ShaderManager->removePreprocessorDefine("NUM_TUBE_SUBDIVISIONS");
    } else {
        sgl::Logfile::get()->writeError("Error in LineData::reloadGatherShaderOpenGL: Invalid line primitive mode.");
    }
    return shaderProgramPtr;
}

void LineData::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
        pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);
    } else {
        pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::LINE_LIST);
    }

    if (linePrimitiveMode != LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
        uint32_t vertexPositionBinding = shaderStages->getInputVariableLocation("vertexPosition");
        pipelineInfo.setVertexBufferBinding(vertexPositionBinding, sizeof(glm::vec3));
        pipelineInfo.setInputAttributeDescription(
                vertexPositionBinding, 0, "vertexPosition");

        uint32_t vertexAttributeBinding = shaderStages->getInputVariableLocation("vertexAttribute");
        pipelineInfo.setVertexBufferBinding(vertexAttributeBinding, sizeof(float));
        pipelineInfo.setInputAttributeDescription(
                vertexAttributeBinding, 0, "vertexAttribute");

        uint32_t vertexNormalBinding = shaderStages->getInputVariableLocation("vertexNormal");
        pipelineInfo.setVertexBufferBinding(vertexNormalBinding, sizeof(glm::vec3));
        pipelineInfo.setInputAttributeDescription(
                vertexNormalBinding, 0, "vertexNormal");

        uint32_t vertexTangentBinding = shaderStages->getInputVariableLocation("vertexTangent");
        pipelineInfo.setVertexBufferBinding(vertexTangentBinding, sizeof(glm::vec3));
        pipelineInfo.setInputAttributeDescription(
                vertexTangentBinding, 0, "vertexTangent");
    }
}

void LineData::setRasterDataBindings(sgl::vk::RasterDataPtr& rasterData) {
    setVulkanRenderDataDescriptors(rasterData);

    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
        TubeRenderDataProgrammableFetch tubeRenderData = this->getTubeRenderDataProgrammableFetch();
        linePointDataSSBO = tubeRenderData.linePointsBuffer;
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setStaticBuffer(linePointDataSSBO, "LinePoints");
    } else {
        TubeRenderData tubeRenderData = this->getTubeRenderData();
        linePointDataSSBO = {};
        rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
        rasterData->setVertexBuffer(tubeRenderData.vertexPositionBuffer, "vertexPosition");
        rasterData->setVertexBuffer(tubeRenderData.vertexAttributeBuffer, "vertexAttribute");
        rasterData->setVertexBuffer(tubeRenderData.vertexNormalBuffer, "vertexNormal");
        rasterData->setVertexBuffer(tubeRenderData.vertexTangentBuffer, "vertexTangent");
    }
}

sgl::ShaderAttributesPtr LineData::getGatherShaderAttributesOpenGL(sgl::ShaderProgramPtr& gatherShader) {
    sgl::ShaderAttributesPtr shaderAttributes;

    /*if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
        TubeRenderDataProgrammableFetch tubeRenderData = this->getTubeRenderDataProgrammableFetch();
        linePointDataSSBO = tubeRenderData.linePointsBuffer;

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    } else {
        TubeRenderData tubeRenderData = this->getTubeRenderData();
        linePointDataSSBO = sgl::GeometryBufferPtr();

        shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);

        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);
        shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        shaderAttributes->addGeometryBuffer(
                tubeRenderData.vertexPositionBuffer, "vertexPosition",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexAttributeBuffer, "vertexAttribute",
                sgl::ATTRIB_FLOAT, 1);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexNormalBuffer, "vertexNormal",
                sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBufferOptional(
                tubeRenderData.vertexTangentBuffer, "vertexTangent",
                sgl::ATTRIB_FLOAT, 3);
    }*/

    return shaderAttributes;
}

sgl::ShaderProgramPtr LineData::reloadGatherShaderHull() {
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderProgramPtr shaderProgramPtr = sgl::ShaderManager->getShaderProgram({
            "MeshHull.Vertex", "MeshHull.Fragment"
    });
    return shaderProgramPtr;
}

sgl::ShaderAttributesPtr LineData::getGatherShaderAttributesHull(sgl::ShaderProgramPtr& gatherShader) {
    SimulationMeshOutlineRenderData renderData = this->getSimulationMeshOutlineRenderData();

    sgl::ShaderAttributesPtr shaderAttributes;
    /*if (gatherShader) {
        shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
        shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
        shaderAttributes->setIndexGeometryBuffer(renderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        shaderAttributes->addGeometryBuffer(
                renderData.vertexPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
        shaderAttributes->addGeometryBuffer(
                renderData.vertexNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);
    }*/

    return shaderAttributes;
}

SimulationMeshOutlineRenderData LineData::getSimulationMeshOutlineRenderData() {
    SimulationMeshOutlineRenderData renderData;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    // Add the index buffer.
    renderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, simulationMeshOutlineTriangleIndices.size() * sizeof(uint32_t),
            simulationMeshOutlineTriangleIndices.data(),
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the position buffer.
    renderData.vertexPositionBuffer = std::make_shared<sgl::vk::Buffer>(
            device, simulationMeshOutlineVertexPositions.size() * sizeof(glm::vec3),
            simulationMeshOutlineVertexPositions.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    // Add the normal buffer.
    renderData.vertexNormalBuffer = std::make_shared<sgl::vk::Buffer>(
            device, simulationMeshOutlineVertexNormals.size() * sizeof(glm::vec3),
            simulationMeshOutlineVertexNormals.data(),
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    return renderData;
}

void LineData::loadSimulationMeshOutlineFromFile(
        const std::string& simulationMeshFilename, const sgl::AABB3& oldAABB, glm::mat4* transformationMatrixPtr) {
    loadMeshBoundarySurfaceFromFile(
            simulationMeshFilename, simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions);
    normalizeVertexPositions(simulationMeshOutlineVertexPositions, oldAABB, transformationMatrixPtr);
    laplacianSmoothing(simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions);
    computeSmoothTriangleNormals(
            simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions,
            simulationMeshOutlineVertexNormals);
}

void LineData::setUniformGatherShaderDataOpenGL(sgl::ShaderProgramPtr& gatherShader) {
    setUniformGatherShaderData_AllPasses();
    setUniformGatherShaderData_Pass(gatherShader);
}

void LineData::setUniformGatherShaderData_AllPasses() {
    /*if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH && linePointDataSSBO) {
        sgl::ShaderManager->bindShaderStorageBuffer(2, linePointDataSSBO);
    }*/
}

void LineData::setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader) {
    gatherShader->setUniformOptional(
            "transferFunctionTexture",
            transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    gatherShader->setUniformOptional("minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShader->setUniformOptional("maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());

    if (getUseBandRendering()) {
        gatherShader->setUniformOptional("bandWidth", LineRenderer::bandWidth);
    }
}

void LineData::setUniformGatherShaderDataHull_Pass(sgl::ShaderProgramPtr& gatherShader) {
    gatherShader->setUniformOptional("color", glm::vec4(hullColor.r, hullColor.g, hullColor.b, hullOpacity));
    gatherShader->setUniformOptional("useShading", int(hullUseShading));
}


#ifdef USE_VULKAN_INTEROP
sgl::vk::BottomLevelAccelerationStructurePtr LineData::getTubeTriangleBottomLevelAS(LineRenderer* lineRenderer) {
    rebuildInternalRepresentationIfNecessary();
    if (tubeTriangleBottomLevelAS) {
        return tubeTriangleBottomLevelAS;
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    VulkanTubeTriangleRenderData tubeTriangleRenderData = getVulkanTubeTriangleRenderData(lineRenderer, true);

    if (!tubeTriangleRenderData.indexBuffer) {
        return tubeTriangleBottomLevelAS;
    }

    auto asTubeInput = new sgl::vk::TrianglesAccelerationStructureInput(
            device, VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR);
    asTubeInput->setIndexBuffer(tubeTriangleRenderData.indexBuffer);
    asTubeInput->setVertexBuffer(
            tubeTriangleRenderData.vertexBuffer, VK_FORMAT_R32G32B32_SFLOAT,
            sizeof(TubeTriangleVertexData));
    auto asTubeInputPtr = sgl::vk::BottomLevelAccelerationStructureInputPtr(asTubeInput);
    sgl::Logfile::get()->writeInfo("Building tube triangle bottom level ray tracing acceleration structure...");
    size_t inputVerticesSize = tubeTriangleRenderData.indexBuffer->getSizeInBytes();
    size_t inputIndicesSize =
            tubeTriangleRenderData.vertexBuffer->getSizeInBytes() / sizeof(TubeTriangleVertexData) * sizeof(glm::vec3);
    sgl::Logfile::get()->writeInfo(
            "Input vertices size: " + sgl::toString(double(inputVerticesSize) / 1024.0 / 1024.0) + "MiB");
    sgl::Logfile::get()->writeInfo(
            "Input indices size: " + sgl::toString(double(inputIndicesSize) / 1024.0 / 1024.0) + "MiB");
    tubeTriangleBottomLevelAS = buildBottomLevelAccelerationStructureFromInput(
            asTubeInputPtr,
            VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
            | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR, true);

    return tubeTriangleBottomLevelAS;
}

sgl::vk::BottomLevelAccelerationStructurePtr LineData::getTubeAabbBottomLevelAS(LineRenderer* lineRenderer) {
    rebuildInternalRepresentationIfNecessary();
    if (tubeAabbBottomLevelAS) {
        return tubeAabbBottomLevelAS;
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    VulkanTubeAabbRenderData tubeAabbRenderData = getVulkanTubeAabbRenderData(lineRenderer);

    if (!tubeAabbRenderData.indexBuffer) {
        return tubeAabbBottomLevelAS;
    }

    auto asAabbInput = new sgl::vk::AabbsAccelerationStructureInput(
            device, VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR);
    asAabbInput->setAabbsBuffer(tubeAabbRenderData.aabbBuffer);
    auto asAabbInputPtr = sgl::vk::BottomLevelAccelerationStructureInputPtr(asAabbInput);
    sgl::Logfile::get()->writeInfo("Building tube AABB bottom level ray tracing acceleration structure...");
    size_t inputSize = tubeAabbRenderData.aabbBuffer->getSizeInBytes();
    sgl::Logfile::get()->writeInfo(
            "Input AABBs size: " + sgl::toString(double(inputSize) / 1024.0 / 1024.0) + "MiB");
    tubeAabbBottomLevelAS = buildBottomLevelAccelerationStructureFromInput(
            asAabbInputPtr,
            VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
            | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR, true);

    return tubeAabbBottomLevelAS;
}

sgl::vk::BottomLevelAccelerationStructurePtr LineData::getHullTriangleBottomLevelAS() {
    rebuildInternalRepresentationIfNecessary();
    if (hullTriangleBottomLevelAS) {
        return hullTriangleBottomLevelAS;
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    VulkanHullTriangleRenderData hullTriangleRenderData = getVulkanHullTriangleRenderData(true);

    if (!hullTriangleRenderData.indexBuffer) {
        return hullTriangleBottomLevelAS;
    }

    auto asHullInput = new sgl::vk::TrianglesAccelerationStructureInput(
            device, VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR);
    asHullInput->setIndexBuffer(hullTriangleRenderData.indexBuffer);
    asHullInput->setVertexBuffer(
            hullTriangleRenderData.vertexBuffer, VK_FORMAT_R32G32B32_SFLOAT,
            sizeof(HullTriangleVertexData));
    auto asHullInputPtr = sgl::vk::BottomLevelAccelerationStructureInputPtr(asHullInput);

    sgl::Logfile::get()->writeInfo("Building hull triangle bottom level ray tracing acceleration structure...");
    size_t inputVerticesSize = hullTriangleRenderData.indexBuffer->getSizeInBytes();
    size_t inputIndicesSize =
            hullTriangleRenderData.vertexBuffer->getSizeInBytes() / sizeof(HullTriangleVertexData) * sizeof(glm::vec3);
    sgl::Logfile::get()->writeInfo(
            "Input vertices size: " + sgl::toString(double(inputVerticesSize) / 1024.0 / 1024.0) + "MiB");
    sgl::Logfile::get()->writeInfo(
            "Input indices size: " + sgl::toString(double(inputIndicesSize) / 1024.0 / 1024.0) + "MiB");
    hullTriangleBottomLevelAS = buildBottomLevelAccelerationStructureFromInput(
            asHullInputPtr,
            VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
            | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR, true);

    return hullTriangleBottomLevelAS;
}

sgl::vk::TopLevelAccelerationStructurePtr LineData::getRayTracingTubeTriangleTopLevelAS(LineRenderer* lineRenderer) {
    rebuildInternalRepresentationIfNecessary();
    if (tubeTriangleTopLevelAS) {
        return tubeTriangleTopLevelAS;
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    tubeTriangleBottomLevelAS = getTubeTriangleBottomLevelAS(lineRenderer);

    if (!tubeTriangleBottomLevelAS) {
        return tubeTriangleTopLevelAS;
    }

    tubeTriangleTopLevelAS = std::make_shared<sgl::vk::TopLevelAccelerationStructure>(device);
    tubeTriangleTopLevelAS->build({ tubeTriangleBottomLevelAS }, { sgl::vk::BlasInstance() });

    return tubeTriangleTopLevelAS;
}

sgl::vk::TopLevelAccelerationStructurePtr LineData::getRayTracingTubeTriangleAndHullTopLevelAS(
        LineRenderer* lineRenderer) {
    rebuildInternalRepresentationIfNecessary();
    if (tubeTriangleAndHullTopLevelAS) {
        return tubeTriangleAndHullTopLevelAS;
    }
    if (simulationMeshOutlineTriangleIndices.empty()) {
        return getRayTracingTubeTriangleTopLevelAS(lineRenderer);
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    tubeTriangleBottomLevelAS = getTubeTriangleBottomLevelAS(lineRenderer);
    hullTriangleBottomLevelAS = getHullTriangleBottomLevelAS();

    if (!tubeTriangleBottomLevelAS && !hullTriangleBottomLevelAS) {
        return tubeTriangleAndHullTopLevelAS;
    }

    sgl::vk::BlasInstance tubeBlasInstance, hullBlasInstance;
    hullBlasInstance.shaderBindingTableRecordOffset = 1;
    tubeTriangleAndHullTopLevelAS = std::make_shared<sgl::vk::TopLevelAccelerationStructure>(device);
    if (tubeTriangleBottomLevelAS) {
        hullBlasInstance.blasIdx = 1;
        tubeTriangleAndHullTopLevelAS->build(
                { tubeTriangleBottomLevelAS, hullTriangleBottomLevelAS },
                { tubeBlasInstance, hullBlasInstance });
    } else {
        hullBlasInstance.blasIdx = 0;
        tubeTriangleAndHullTopLevelAS->build({ hullTriangleBottomLevelAS }, { hullBlasInstance });
    }

    return tubeTriangleAndHullTopLevelAS;
}

sgl::vk::TopLevelAccelerationStructurePtr LineData::getRayTracingTubeAabbTopLevelAS(LineRenderer* lineRenderer) {
    rebuildInternalRepresentationIfNecessary();
    if (tubeAabbTopLevelAS) {
        return tubeAabbTopLevelAS;
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    tubeAabbBottomLevelAS = getTubeAabbBottomLevelAS(lineRenderer);

    if (!tubeAabbBottomLevelAS) {
        return tubeAabbTopLevelAS;
    }

    tubeAabbTopLevelAS = std::make_shared<sgl::vk::TopLevelAccelerationStructure>(device);
    tubeAabbTopLevelAS->build({ tubeAabbBottomLevelAS }, { sgl::vk::BlasInstance() });

    return tubeAabbTopLevelAS;
}

sgl::vk::TopLevelAccelerationStructurePtr LineData::getRayTracingTubeAabbAndHullTopLevelAS(LineRenderer* lineRenderer) {
    rebuildInternalRepresentationIfNecessary();
    if (tubeAabbAndHullTopLevelAS) {
        return tubeAabbAndHullTopLevelAS;
    }
    if (simulationMeshOutlineTriangleIndices.empty()) {
        return getRayTracingTubeAabbTopLevelAS(lineRenderer);
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    tubeAabbBottomLevelAS = getTubeAabbBottomLevelAS(lineRenderer);
    hullTriangleBottomLevelAS = getHullTriangleBottomLevelAS();

    if (!tubeAabbBottomLevelAS && !hullTriangleBottomLevelAS) {
        return tubeAabbAndHullTopLevelAS;
    }

    sgl::vk::BlasInstance tubeBlasInstance, hullBlasInstance;
    hullBlasInstance.shaderBindingTableRecordOffset = 1;
    tubeAabbAndHullTopLevelAS = std::make_shared<sgl::vk::TopLevelAccelerationStructure>(device);
    if (tubeAabbBottomLevelAS) {
        hullBlasInstance.blasIdx = 1;
        tubeAabbAndHullTopLevelAS->build(
                { tubeAabbBottomLevelAS, hullTriangleBottomLevelAS },
                { tubeBlasInstance, hullBlasInstance });
    } else {
        hullBlasInstance.blasIdx = 0;
        tubeAabbAndHullTopLevelAS->build({ hullTriangleBottomLevelAS }, { hullBlasInstance });
    }

    return tubeAabbAndHullTopLevelAS;
}

VulkanHullTriangleRenderData LineData::getVulkanHullTriangleRenderData(bool raytracing) {
    rebuildInternalRepresentationIfNecessary();
    if (vulkanHullTriangleRenderData.vertexBuffer) {
        return vulkanHullTriangleRenderData;
    }
    if (simulationMeshOutlineTriangleIndices.empty()) {
        return {};
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    vulkanHullTriangleRenderData = {};

    std::vector<HullTriangleVertexData> vertexDataList;
    vertexDataList.reserve(simulationMeshOutlineVertexPositions.size());
    for (size_t i = 0; i < simulationMeshOutlineVertexPositions.size(); i++) {
        HullTriangleVertexData vertex{};
        vertex.vertexPosition = simulationMeshOutlineVertexPositions.at(i);
        vertex.vertexNormal = simulationMeshOutlineVertexNormals.at(i);
        vertexDataList.push_back(vertex);
    }

    uint32_t indexBufferFlags = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    uint32_t vertexBufferFlags = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    if (raytracing) {
        indexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
        vertexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
    }

    vulkanHullTriangleRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, simulationMeshOutlineTriangleIndices.size() * sizeof(uint32_t),
            simulationMeshOutlineTriangleIndices.data(),
            indexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    vulkanHullTriangleRenderData.vertexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexDataList.size() * sizeof(HullTriangleVertexData), vertexDataList.data(),
            vertexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    return vulkanHullTriangleRenderData;
}

void LineData::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string>& preprocessorDefines, bool isRasterizer) {
    if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER || linePrimitiveMode == LINE_PRIMITIVES_BAND
            || linePrimitiveMode == LINE_PRIMITIVES_TUBE_BAND) {
        preprocessorDefines.insert(std::make_pair(
                "NUM_TUBE_SUBDIVISIONS", std::to_string(tubeNumSubdivisions)));
    }
    if (useCappedTubes) {
        preprocessorDefines.insert(std::make_pair("USE_CAPPED_TUBES", ""));
    }
    if (renderThickBands) {
        preprocessorDefines.insert(std::make_pair("MIN_THICKNESS", std::to_string(minBandThickness)));
    } else {
        preprocessorDefines.insert(std::make_pair("MIN_THICKNESS", std::to_string(1e-2f)));
    }
}

void LineData::setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData) {
    renderData->setStaticBufferOptional(lineUniformDataBuffer, "LineUniformDataBuffer");

    if (renderData->getShaderStages()->hasDescriptorBinding(0, "transferFunctionTexture")) {
        const sgl::vk::DescriptorInfo& descriptorInfo = renderData->getShaderStages()->getDescriptorInfoByName(
                0, "transferFunctionTexture");
        if (descriptorInfo.image.arrayed == 0) {
            renderData->setStaticTexture(
                    transferFunctionWindow.getTransferFunctionMapTextureVulkan(),
                    "transferFunctionTexture");
            renderData->setStaticBuffer(
                    transferFunctionWindow.getMinMaxUboVulkan(),
                    "MinMaxUniformBuffer");
        }
    }
}

void LineData::updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer) {
    SceneData* sceneData = nullptr;
    if (lineRenderer) {
        sceneData = lineRenderer->getSceneData();
    }

    if (sceneData) {
        glm::vec4 backgroundColor = sceneData->clearColor->getFloatColorRGBA();
        glm::vec4 foregroundColor = glm::vec4(1.0f) - backgroundColor;
        lineUniformData.cameraPosition = (*sceneData->camera)->getPosition();
        lineUniformData.fieldOfViewY = (*sceneData->camera)->getFOVy();
        lineUniformData.viewMatrix = (*sceneData->camera)->getViewMatrix();
        lineUniformData.projectionMatrix = (*sceneData->camera)->getProjectionMatrix();
        lineUniformData.inverseViewMatrix = glm::inverse((*sceneData->camera)->getViewMatrix());
        lineUniformData.inverseProjectionMatrix = glm::inverse((*sceneData->camera)->getProjectionMatrix());
        lineUniformData.backgroundColor = backgroundColor;
        lineUniformData.foregroundColor = foregroundColor;
    }

    lineUniformData.lineWidth = LineRenderer::getLineWidth();
    lineUniformData.bandWidth = LineRenderer::getBandWidth();
    lineUniformData.minBandThickness = minBandThickness;
    lineUniformData.depthCueStrength = lineRenderer ? lineRenderer->depthCueStrength : 0.0f;
    lineUniformData.ambientOcclusionStrength = lineRenderer ? lineRenderer->ambientOcclusionStrength : 0.0f;
    lineUniformData.ambientOcclusionGamma = lineRenderer ? lineRenderer->ambientOcclusionGamma : 1.0f;
    if (lineRenderer && lineRenderer->useAmbientOcclusion && lineRenderer->ambientOcclusionBaker) {
        lineUniformData.numAoTubeSubdivisions = lineRenderer->ambientOcclusionBaker->getNumTubeSubdivisions();
        lineUniformData.numLineVertices = lineRenderer->ambientOcclusionBaker->getNumLineVertices();
        lineUniformData.numParametrizationVertices =
                lineRenderer->ambientOcclusionBaker->getNumParametrizationVertices();
    }

    lineUniformData.hasHullMesh = simulationMeshOutlineTriangleIndices.empty() ? 0 : 1;
    lineUniformData.hullColor = glm::vec4(hullColor.r, hullColor.g, hullColor.b, hullOpacity);
    lineUniformData.hullUseShading = uint32_t(hullUseShading);

    if (sceneData) {
        lineUniformData.viewportSize = glm::uvec2(*sceneData->viewportWidth, *sceneData->viewportHeight);
    }

    lineUniformDataBuffer->updateData(
            sizeof(LineUniformData), &lineUniformData, renderer->getVkCommandBuffer());
}
#endif
