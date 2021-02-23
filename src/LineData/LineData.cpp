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
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>
#include <Graphics/Renderer.hpp>

#include <ImGui/imgui.h>
#include <ImGui/imgui_custom.h>

#include "Utils/TriangleNormals.hpp"
#include "Renderers/LineRenderer.hpp"
#include "Mesh/MeshBoundarySurface.hpp"
#include "LineData.hpp"

LineData::LinePrimitiveMode LineData::linePrimitiveMode = LineData::LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH;
int LineData::tubeNumSubdivisions = 5;

const char *const LINE_PRIMITIVE_MODE_DISPLAYNAMES[] = {
        "Ribbon (Programmable Fetch)", "Ribbon (Geometry Shader)", "Tube (Geometry Shader)", "Band"
};

LineData::LineData(sgl::TransferFunctionWindow &transferFunctionWindow, DataSetType dataSetType)
        : dataSetType(dataSetType), transferFunctionWindow(transferFunctionWindow) {
}

LineData::~LineData() {
}

bool LineData::renderGui(bool isRasterizer) {
    bool shallReloadGatherShader = false;
    if (isRasterizer) {
        size_t numPrimitiveModes = IM_ARRAYSIZE(LINE_PRIMITIVE_MODE_DISPLAYNAMES);
        if (getType() != DATA_SET_TYPE_STRESS_LINES) {
            numPrimitiveModes -= 1;
        }
        if (renderingMode != RENDERING_MODE_OPACITY_OPTIMIZATION && ImGui::Combo(
                "Line Primitives", (int*)&linePrimitiveMode,
                LINE_PRIMITIVE_MODE_DISPLAYNAMES, numPrimitiveModes)) {
            dirty = true;
            shallReloadGatherShader = true;
        }

        if (renderingMode != RENDERING_MODE_OPACITY_OPTIMIZATION
                && linePrimitiveMode == LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER) {
            if (ImGui::SliderInt("Tube Subdivisions", &tubeNumSubdivisions, 3, 8)) {
                shallReloadGatherShader = true;
            }
        }
    }

    // Switch importance criterion.
    if (ImGui::Combo(
            "Attribute", (int*)&selectedAttributeIndexUi,
            attributeNames.data(), attributeNames.size())) {
        setSelectedAttributeIndex(selectedAttributeIndexUi);
    }
    ImGui::Checkbox("Render Color Legend", &shallRenderColorLegendWidgets);

    if (!simulationMeshOutlineTriangleIndices.empty()) {
        ImGui::Checkbox("Render Mesh Boundary", &shallRenderSimulationMeshBoundary);

        if (shallRenderSimulationMeshBoundary) {
            if (ImGui::SliderFloat("Hull Opacity", &hullOpacity, 0.0f, 1.0f, "%.4f")) {
                reRender = true;
            }
            if (ImGui::ColorEdit3("Hull Color", &hullColor.r)) {
                reRender = true;
            }
            if (ImGui::Checkbox("Use Shading for Hull", &hullUseShading)) {
                reRender = true;
            }
        }
    }

    return shallReloadGatherShader;
}

bool LineData::renderGuiWindow(bool isRasterizer) {
    if (shallRenderColorLegendWidgets) {
        colorLegendWidgets.at(selectedAttributeIndex).setAttributeMinValue(
                transferFunctionWindow.getSelectedRangeMin());
        colorLegendWidgets.at(selectedAttributeIndex).setAttributeMaxValue(
                transferFunctionWindow.getSelectedRangeMax());
        colorLegendWidgets.at(selectedAttributeIndex).renderGui();
    }
    return false;
}

void LineData::setClearColor(const sgl::Color& clearColor) {
    for (int i = 0; i < colorLegendWidgets.size(); i++) {
        colorLegendWidgets.at(i).setClearColor(clearColor);
    }
}

void LineData::setSelectedAttributeIndex(int qualityMeasureIdx) {
    if (this->selectedAttributeIndex != qualityMeasureIdx) {
        dirty = true;
        this->selectedAttributeIndex = qualityMeasureIdx;
    }
    recomputeHistogram();
}

void LineData::onTransferFunctionMapRebuilt() {
    recomputeColorLegend();
}

void LineData::recomputeColorLegend() {
    for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
        colorLegendWidgets[i].setTransferFunctionColorMap(
                transferFunctionWindow.getTransferFunctionMap_sRGB());
    }
}

void LineData::rebuildInternalRepresentationIfNecessary() {
    if (dirty) {
        //updateMeshTriangleIntersectionDataStructure();
        dirty = false;
    }
}

sgl::ShaderProgramPtr LineData::reloadGatherShader() {
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
    } else { //if (linePrimitiveMode == LINE_PRIMITIVES_TUBE_GEOMETRY_SHADER) {
        sgl::ShaderManager->addPreprocessorDefine("NUM_TUBE_SUBDIVISIONS", tubeNumSubdivisions);
        shaderProgramPtr = sgl::ShaderManager->getShaderProgram({
                "GeometryPassNormalTube.VBO.Vertex",
                "GeometryPassNormalTube.VBO.Geometry",
                "GeometryPassNormalTube.Fragment"
        });
        sgl::ShaderManager->removePreprocessorDefine("NUM_TUBE_SUBDIVISIONS");
    }
    return shaderProgramPtr;
}

sgl::ShaderAttributesPtr LineData::getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader) {
    sgl::ShaderAttributesPtr shaderAttributes;

    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH) {
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
    }

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
    linePointDataSSBO = sgl::GeometryBufferPtr();

    sgl::ShaderAttributesPtr shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);
    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
    shaderAttributes->setIndexGeometryBuffer(renderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    shaderAttributes->addGeometryBuffer(
            renderData.vertexPositionBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);
    shaderAttributes->addGeometryBuffer(
            renderData.vertexNormalBuffer, "vertexNormal", sgl::ATTRIB_FLOAT, 3);

    return shaderAttributes;
}

SimulationMeshOutlineRenderData LineData::getSimulationMeshOutlineRenderData() {
    SimulationMeshOutlineRenderData renderData;

    // Add the index buffer.
    renderData.indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*simulationMeshOutlineTriangleIndices.size(),
            (void*)&simulationMeshOutlineTriangleIndices.front(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    renderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            simulationMeshOutlineVertexPositions.size()*sizeof(glm::vec3),
            (void*)&simulationMeshOutlineVertexPositions.front(), sgl::VERTEX_BUFFER);

    // Add the normal buffer.
    renderData.vertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
            simulationMeshOutlineVertexNormals.size()*sizeof(glm::vec3),
            (void*)&simulationMeshOutlineVertexNormals.front(), sgl::VERTEX_BUFFER);

    return renderData;
}

void LineData::loadSimulationMeshOutlineFromFile(
        const std::string& simulationMeshFilename, const sgl::AABB3& oldAABB, glm::mat4* transformationMatrixPtr) {
    loadMeshBoundarySurfaceFromFile(
            simulationMeshFilename, simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions);
    normalizeVertexPositions(simulationMeshOutlineVertexPositions, oldAABB, transformationMatrixPtr);
    computeSmoothTriangleNormals(
            simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions,
            simulationMeshOutlineVertexNormals);
}

void LineData::setUniformGatherShaderData(sgl::ShaderProgramPtr& gatherShader) {
    setUniformGatherShaderData_AllPasses();
    setUniformGatherShaderData_Pass(gatherShader);
}

void LineData::setUniformGatherShaderData_AllPasses() {
    if (linePrimitiveMode == LINE_PRIMITIVES_RIBBON_PROGRAMMABLE_FETCH && linePointDataSSBO) {
        sgl::ShaderManager->bindShaderStorageBuffer(2, linePointDataSSBO);
    }
}

void LineData::setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader) {
    gatherShader->setUniformOptional(
            "transferFunctionTexture",
            transferFunctionWindow.getTransferFunctionMapTexture(), 0);
    gatherShader->setUniformOptional("minAttributeValue", transferFunctionWindow.getSelectedRangeMin());
    gatherShader->setUniformOptional("maxAttributeValue", transferFunctionWindow.getSelectedRangeMax());
}

void LineData::setUniformGatherShaderDataHull_Pass(sgl::ShaderProgramPtr& gatherShader) {
    gatherShader->setUniformOptional("color", glm::vec4(hullColor.r, hullColor.g, hullColor.b, hullOpacity));
    gatherShader->setUniformOptional("useShading", int(hullUseShading));
}