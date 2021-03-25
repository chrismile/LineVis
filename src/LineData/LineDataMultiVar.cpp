/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser, Michael Kern
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

#include <Graphics/Renderer.hpp>
#include <Graphics/Shader/ShaderManager.hpp>

#include "Loaders/TrajectoryFile.hpp"
#include "Renderers/Tubes/Tubes.hpp"

#include <Utils/File/Logfile.hpp>

#include "MultiVar/HsvColor.hpp"
#include "LineDataMultiVar.hpp"

const std::string MULTI_VAR_RENDER_MODE_SHADER_NAMES[] = {
        "MultiVarRolls", "MultiVarTwistedRolls", "MultiVarColorBands", "MultiVarOrientedColorBands",
        "MultiVarOrientedColorBandsRibbon", "MultiVarCheckerboard", "MultiVarFibers"
};

LineDataMultiVar::OrientedRibbonMode LineDataMultiVar::orientedRibbonMode =
        LineDataMultiVar::ORIENTED_RIBBON_MODE_FIXED_BAND_WIDTH;
bool LineDataMultiVar::mapColorToSaturation = true; ///< !mapColorToSaturation -> DIRECT_COLOR_MAPPING in gather shader.

///< Use multivariate or single attribute rendering? If true, falls back to code from @see LineDataFlow.
bool LineDataMultiVar::useMultiVarRendering = true;

// Rendering modes.
LineDataMultiVar::MultiVarRenderMode LineDataMultiVar::multiVarRenderMode =
        LineDataMultiVar::MULTIVAR_RENDERMODE_ROLLS;
LineDataMultiVar::MultiVarRadiusMappingMode LineDataMultiVar::multiVarRadiusMappingMode =
        LineDataMultiVar::MULTIVAR_RADIUSMODE_GLOBAL;

// Multi-Variate settings.
int32_t LineDataMultiVar::numLineSegments = 8;
int32_t LineDataMultiVar::numInstances = 12;
int32_t LineDataMultiVar::rollWidth = 1;
float LineDataMultiVar::separatorWidth = 0.15f;
bool LineDataMultiVar::mapTubeDiameter = false;
float LineDataMultiVar::twistOffset = 0.1f;
bool LineDataMultiVar::constantTwistOffset = false;
int32_t LineDataMultiVar::checkerboardWidth = 3;
int32_t LineDataMultiVar::checkerboardHeight = 2;
int32_t LineDataMultiVar::checkerboardIterator = 2;
/// For orientedRibbonMode == ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH
glm::vec4 LineDataMultiVar::bandBackgroundColor = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);

// Line settings.
float LineDataMultiVar::minRadiusFactor = 0.5f;
float LineDataMultiVar::fiberRadius = 0.0005f;

// Lighting settings.
bool LineDataMultiVar::useColorIntensity = true;
float LineDataMultiVar::materialConstantAmbient = 0.1f;
float LineDataMultiVar::materialConstantDiffuse = 0.85f;
float LineDataMultiVar::materialConstantSpecular = 0.05f;
float LineDataMultiVar::materialConstantSpecularExp = 10.0f;
bool LineDataMultiVar::drawHalo = true;
float LineDataMultiVar::haloFactor = 1.2f;

/*const std::vector<glm::vec4> defaultColors = {
        // RED
        glm::vec4(228 / 255.0, 26 / 255.0, 28 / 255.0, 1.0),
        // BLUE
        glm::vec4(55 / 255.0, 126 / 255.0, 184 / 255.0, 1.0),
        // GREEN
        glm::vec4(5 / 255.0, 139 / 255.0, 69 / 255.0, 1.0),
        // PURPLE
        glm::vec4(129 / 255.0, 15 / 255.0, 124 / 255.0, 1.0),
        // ORANGE
        glm::vec4(217 / 255.0, 72 / 255.0, 1 / 255.0, 1.0),
        // PINK
        glm::vec4(231 / 255.0, 41 / 255.0, 138 / 255.0, 1.0),
        // GOLD
        glm::vec4(254 / 255.0, 178 / 255.0, 76 / 255.0, 1.0),
        // DARK BLUE
        glm::vec4(0 / 255.0, 7 / 255.0, 255 / 255.0, 1.0)
};*/

const int MAX_NUM_VARIABLES = 20;


LineDataMultiVar::LineDataMultiVar(sgl::TransferFunctionWindow &transferFunctionWindow)
        : LineDataFlow(transferFunctionWindow),
          multiVarTransferFunctionWindow("multivar", {
              "reds.xml", "blues.xml", "greens.xml", "purples.xml", "oranges.xml", "pinks.xml", "golds.xml",
              "dark-blues.xml" }) {
    dataSetType = DATA_SET_TYPE_FLOW_LINES_MULTIVAR;
    lineDataWindowName = "Line Data (MultiVar)";
}

bool LineDataMultiVar::settingsDiffer(LineData* other) {
    return false;
}

void LineDataMultiVar::update(float dt) {
    multiVarTransferFunctionWindow.update(dt);
}

LineDataMultiVar::~LineDataMultiVar() {
}

void LineDataMultiVar::recomputeHistogram() {
    const size_t numAttributes = attributeNames.size();
    std::vector<std::vector<float>> attributesList(numAttributes);
    for (const Trajectory& trajectory : trajectories) {
        for (size_t attrIdx = 0; attrIdx < numAttributes; attrIdx++) {
            std::vector<float>& attributeList = attributesList.at(attrIdx);
            for (float val : trajectory.attributes.at(attrIdx)) {
                attributeList.push_back(val);
            }
        }
    }
    multiVarTransferFunctionWindow.setAttributesValues(attributeNames, attributesList);
    //multiVarWindow.setAttributes(attributesList, attributeNames);

    LineDataFlow::recomputeHistogram();
}

sgl::ShaderProgramPtr LineDataMultiVar::reloadGatherShader() {
    if (!useMultiVarRendering) {
        return LineData::reloadGatherShader();
    }

    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine(
            "NUM_INSTANCES", static_cast<uint32_t>(numVariablesSelected));
    sgl::ShaderManager->addPreprocessorDefine("NUM_SEGMENTS", numLineSegments);
    sgl::ShaderManager->addPreprocessorDefine("NUM_LINESEGMENTS", numInstances);
    sgl::ShaderManager->addPreprocessorDefine("MAX_NUM_VARIABLES", MAX_NUM_VARIABLES);
    sgl::ShaderManager->addPreprocessorDefine("USE_MULTI_VAR_TRANSFER_FUNCTION", "");
    sgl::ShaderManager->addPreprocessorDefine("IS_MULTIVAR_DATA", "");

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS_RIBBON) {
        if (!mapColorToSaturation) {
            sgl::ShaderManager->addPreprocessorDefine("DIRECT_COLOR_MAPPING", numInstances);
        }
        sgl::ShaderManager->addPreprocessorDefine("ORIENTED_RIBBON_MODE", (int)orientedRibbonMode);
    }

    int idx = int(multiVarRenderMode);
    std::list<std::string> gatherShaderIDs = {
            MULTI_VAR_RENDER_MODE_SHADER_NAMES[idx] + ".Vertex",
            MULTI_VAR_RENDER_MODE_SHADER_NAMES[idx] + ".Geometry",
            MULTI_VAR_RENDER_MODE_SHADER_NAMES[idx] + ".Fragment",
    };
    sgl::ShaderProgramPtr gatherShader = sgl::ShaderManager->getShaderProgram(gatherShaderIDs);

    if (multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS
        || multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS_RIBBON) {
        if (!mapColorToSaturation) {
            sgl::ShaderManager->removePreprocessorDefine("DIRECT_COLOR_MAPPING");
        }
        sgl::ShaderManager->removePreprocessorDefine("ORIENTED_RIBBON_MODE");
    }

    sgl::ShaderManager->removePreprocessorDefine("NUM_INSTANCES");
    sgl::ShaderManager->removePreprocessorDefine("NUM_SEGMENTS");
    sgl::ShaderManager->removePreprocessorDefine("NUM_LINESEGMENTS");
    sgl::ShaderManager->removePreprocessorDefine("MAX_NUM_VARIABLES");
    sgl::ShaderManager->removePreprocessorDefine("USE_MULTI_VAR_TRANSFER_FUNCTION");
    sgl::ShaderManager->removePreprocessorDefine("IS_MULTIVAR_DATA");

    return gatherShader;
}

sgl::ShaderAttributesPtr LineDataMultiVar::getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader) {
    if (!useMultiVarRendering) {
        return LineData::getGatherShaderAttributes(gatherShader);
    }

    TubeRenderDataMultiVar tubeRenderData = this->getTubeRenderDataMultiVar();
    variableArrayBuffer = sgl::GeometryBufferPtr();
    lineDescArrayBuffer = sgl::GeometryBufferPtr();
    varDescArrayBuffer = sgl::GeometryBufferPtr();
    lineVarDescArrayBuffer = sgl::GeometryBufferPtr();
    varSelectedArrayBuffer = sgl::GeometryBufferPtr();
    //varColorArrayBuffer = sgl::GeometryBufferPtr();

    sgl::ShaderAttributesPtr shaderAttributes = sgl::ShaderManager->createShaderAttributes(gatherShader);

    shaderAttributes->setVertexMode(sgl::VERTEX_MODE_LINES);
    shaderAttributes->setIndexGeometryBuffer(tubeRenderData.indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
    shaderAttributes->addGeometryBuffer(
            tubeRenderData.vertexPositionBuffer, "vertexPosition",
            sgl::ATTRIB_FLOAT, 3);
    shaderAttributes->addGeometryBufferOptional(
            tubeRenderData.vertexNormalBuffer, "vertexLineNormal",
            sgl::ATTRIB_FLOAT, 3);
    shaderAttributes->addGeometryBuffer(
            tubeRenderData.vertexTangentBuffer, "vertexLineTangent",
            sgl::ATTRIB_FLOAT, 3);
    shaderAttributes->addGeometryBufferOptional(
            tubeRenderData.vertexMultiVariableBuffer, "multiVariable",
            sgl::ATTRIB_FLOAT, 4);
    shaderAttributes->addGeometryBuffer(
            tubeRenderData.vertexVariableDescBuffer, "variableDesc",
            sgl::ATTRIB_FLOAT, 4);

    variableArrayBuffer = tubeRenderData.variableArrayBuffer;
    lineDescArrayBuffer = tubeRenderData.lineDescArrayBuffer;
    varDescArrayBuffer = tubeRenderData.varDescArrayBuffer;
    lineVarDescArrayBuffer = tubeRenderData.lineVarDescArrayBuffer;
    varSelectedArrayBuffer = tubeRenderData.varSelectedArrayBuffer;
    //varColorArrayBuffer = tubeRenderData.varColorArrayBuffer;

    return shaderAttributes;
}

void LineDataMultiVar::setUniformGatherShaderData(sgl::ShaderProgramPtr& gatherShader) {
    LineData::setUniformGatherShaderData(gatherShader);
}

void LineDataMultiVar::setUniformGatherShaderData_AllPasses() {
    if (!useMultiVarRendering) {
        LineData::setUniformGatherShaderData_AllPasses();
        return;
    }

    sgl::ShaderManager->bindShaderStorageBuffer(2, variableArrayBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(4, lineDescArrayBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(5, varDescArrayBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(6, lineVarDescArrayBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(7, varSelectedArrayBuffer);
    //sgl::ShaderManager->bindShaderStorageBuffer(8, varColorArrayBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(9, multiVarTransferFunctionWindow.getMinMaxSsbo());
}

void LineDataMultiVar::setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader) {
    if (!useMultiVarRendering) {
        LineData::setUniformGatherShaderData_Pass(gatherShader);
        return;
    }

    gatherShader->setUniformOptional(
            "transferFunctionTexture",
            multiVarTransferFunctionWindow.getTransferFunctionMapTexture(), 0);

    gatherShader->setUniformOptional("numVariables", std::min(numVariablesSelected, MAX_NUM_VARIABLES));
    gatherShader->setUniformOptional("maxNumVariables", static_cast<int32_t>(attributeNames.size()));
    gatherShader->setUniformOptional("materialAmbient", materialConstantAmbient);
    gatherShader->setUniformOptional("materialDiffuse", materialConstantDiffuse);
    gatherShader->setUniformOptional("materialSpecular", materialConstantSpecular);
    gatherShader->setUniformOptional("materialSpecularExp", materialConstantSpecularExp);
    gatherShader->setUniformOptional("drawHalo", drawHalo);
    gatherShader->setUniformOptional("haloFactor", haloFactor);
    gatherShader->setUniformOptional("separatorWidth", separatorWidth);
    gatherShader->setUniformOptional("mapTubeDiameter", mapTubeDiameter);
    gatherShader->setUniformOptional("mapTubeDiameterMode", multiVarRadiusMappingMode);
    gatherShader->setUniformOptional("rollWidth", rollWidth);
    gatherShader->setUniformOptional("twistOffset", twistOffset);
    gatherShader->setUniformOptional("fiberRadius", fiberRadius);
    gatherShader->setUniformOptional("minRadiusFactor", minRadiusFactor);
    //gatherShader->setUniformOptional("minColorIntensity", minColorIntensity);
    gatherShader->setUniformOptional("useColorIntensity", int(useColorIntensity));
    gatherShader->setUniformOptional("checkerboardWidth", checkerboardWidth);
    gatherShader->setUniformOptional("checkerboardHeight", checkerboardHeight);
    gatherShader->setUniformOptional("checkerboardIterator", checkerboardIterator);
    gatherShader->setUniformOptional("constantTwistOffset", constantTwistOffset);
    if ((multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS
            || multiVarRenderMode == MULTIVAR_RENDERMODE_ORIENTED_COLOR_BANDS_RIBBON)
            && orientedRibbonMode == ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH) {
        gatherShader->setUniformOptional("bandBackgroundColor", bandBackgroundColor);
    }
}

void LineDataMultiVar::setTrajectoryData(const Trajectories& trajectories) {
    LineDataFlow::setTrajectoryData(trajectories);
    bezierTrajectories = convertTrajectoriesToBezierCurves(filterTrajectoryData());

    // Reset selected attributes.
    varSelected = std::vector<uint32_t>(attributeNames.size(), false);
    numVariablesSelected = 0;

    // Set starting colors based on default color array.
    /*varColors = std::vector<glm::vec4>(attributeNames.size());
    for (auto c = 0; c < varColors.size(); ++c) {
        varColors[c] = defaultColors[c % defaultColors.size()];
    }*/
}

void LineDataMultiVar::recomputeColorLegend() {
    if (useMultiVarRendering) {
        for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
            /*glm::vec3 baseColor_sRGB = varColors.at(i);
            glm::vec3 baseColor_linearRGB = sgl::TransferFunctionWindow::sRGBToLinearRGB(baseColor_sRGB);

            std::vector<sgl::Color> transferFunctionColorMap;
            transferFunctionColorMap.reserve(ColorLegendWidget::STANDARD_MAP_RESOLUTION);
            for (int i = 0; i < ColorLegendWidget::STANDARD_MAP_RESOLUTION; i++) {
                float posFloat = float(i) / float(ColorLegendWidget::STANDARD_MAP_RESOLUTION - 1);

                glm::vec3 hsvCol = rgbToHsv(baseColor_linearRGB);
                hsvCol.g = hsvCol.g * (minColorIntensity + (1.0 - minColorIntensity) * posFloat);
                glm::vec3 color_linearRGB = hsvToRgb(hsvCol);
                glm::vec3 color_sRGB = sgl::TransferFunctionWindow::linearRGBTosRGB(color_linearRGB);

                transferFunctionColorMap.push_back(color_sRGB);
            }
            colorLegendWidgets.at(i).setTransferFunctionColorMap(transferFunctionColorMap);*/

            std::vector<sgl::Color16> transferFunctionColorMap =
                    multiVarTransferFunctionWindow.getTransferFunctionMap_sRGB(i);
            if (!useColorIntensity) {
                for (size_t i = 0; i < transferFunctionColorMap.size(); i++) {
                    transferFunctionColorMap.at(i) = transferFunctionColorMap.back();
                }
            }
            colorLegendWidgets.at(i).setTransferFunctionColorMap(transferFunctionColorMap);
        }
    } else {
        LineData::recomputeColorLegend();
    }
}


struct LineDescData {
    float startIndex;
};

struct VarDescData {
    glm::vec4 info;
};

struct LineVarDescData {
    glm::vec2 minMax;
};

TubeRenderDataMultiVar LineDataMultiVar::getTubeRenderDataMultiVar() {
    rebuildInternalRepresentationIfNecessary();

    std::vector<std::vector<glm::vec3>> lineCentersList;
    std::vector<std::vector<std::vector<float>>> lineAttributesList;
    std::vector<uint32_t> lineIndices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<glm::vec3> vertexNormals;
    std::vector<glm::vec3> vertexTangents;
    std::vector<std::vector<float>> vertexAttributes;

    lineCentersList.resize(bezierTrajectories.size());
    lineAttributesList.resize(bezierTrajectories.size());
    for (size_t trajectoryIdx = 0; trajectoryIdx < bezierTrajectories.size(); trajectoryIdx++) {
        BezierTrajectory& trajectory = bezierTrajectories.at(trajectoryIdx);
        std::vector<std::vector<float>>& attributes = trajectory.attributes;
        std::vector<glm::vec3>& lineCenters = lineCentersList.at(trajectoryIdx);
        std::vector<std::vector<float>>& lineAttributes = lineAttributesList.at(trajectoryIdx);
        for (size_t i = 0; i < trajectory.positions.size(); i++) {
            lineCenters.push_back(trajectory.positions.at(i));

            std::vector<float> attrs;
            attrs.reserve(attributes.size());
            for (size_t attrIdx = 0; attrIdx < attributes.size(); attrIdx++) {
                assert(trajectory.positions.size() == attributes.at(attrIdx).size());
                attrs.push_back(attributes.at(attrIdx).at(i));
            }
            lineAttributes.push_back(attrs);
        }
    }

    createLineTubesRenderDataCPU(
            lineCentersList, lineAttributesList,
            lineIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);

    std::vector<glm::vec4> vertexMultiVariableArray;
    std::vector<glm::vec4> vertexVariableDescArray;
    vertexMultiVariableArray.reserve(vertexAttributes.size());
    vertexVariableDescArray.reserve(vertexAttributes.size());
    for (size_t vertexIdx = 0; vertexIdx < vertexAttributes.size(); vertexIdx++) {
        std::vector<float>& attrList = vertexAttributes.at(vertexIdx);
        vertexMultiVariableArray.push_back(glm::vec4(
                attrList.at(0), attrList.at(1), attrList.at(2), attrList.at(3)));
        vertexVariableDescArray.push_back(glm::vec4(
                attrList.at(4), attrList.at(5), attrList.at(6), attrList.at(7)));
    }


    TubeRenderDataMultiVar tubeRenderData;

    // Add the index buffer.
    tubeRenderData.indexBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(uint32_t)*lineIndices.size(), (void*)&lineIndices.front(), sgl::INDEX_BUFFER);

    // Add the position buffer.
    tubeRenderData.vertexPositionBuffer = sgl::Renderer->createGeometryBuffer(
            vertexPositions.size()*sizeof(glm::vec3), (void*)&vertexPositions.front(), sgl::VERTEX_BUFFER);

    // Add the normal buffer.
    tubeRenderData.vertexNormalBuffer = sgl::Renderer->createGeometryBuffer(
            vertexNormals.size()*sizeof(glm::vec3), (void*)&vertexNormals.front(), sgl::VERTEX_BUFFER);

    // Add the tangent buffer.
    tubeRenderData.vertexTangentBuffer = sgl::Renderer->createGeometryBuffer(
            vertexTangents.size()*sizeof(glm::vec3), (void*)&vertexTangents.front(), sgl::VERTEX_BUFFER);

    // Add the attribute buffers.
    tubeRenderData.vertexMultiVariableBuffer = sgl::Renderer->createGeometryBuffer(
            vertexMultiVariableArray.size()*sizeof(glm::vec4), (void*)&vertexMultiVariableArray.front(),
            sgl::VERTEX_BUFFER);
    tubeRenderData.vertexVariableDescBuffer = sgl::Renderer->createGeometryBuffer(
            vertexVariableDescArray.size()*sizeof(glm::vec4), (void*)&vertexVariableDescArray.front(),
            sgl::VERTEX_BUFFER);



    // ------------------------------------------ Create SSBOs. ------------------------------------------

    size_t numLines = bezierTrajectories.size();
    std::vector<float> varData;
    std::vector<LineDescData> lineDescData(numLines);
    std::vector<VarDescData> varDescData;
    std::vector<LineVarDescData> lineVarDescData;

    for (BezierTrajectory& bezierTrajectory : bezierTrajectories) {
        for (float attrVal : bezierTrajectory.multiVarData) {
            varData.push_back(attrVal);
        }
    }

    for (BezierTrajectory& bezierTrajectory : bezierTrajectories) {
        for (float attrVal : bezierTrajectory.multiVarData) {
            varData.push_back(attrVal);
        }
    }
    for (size_t lineIdx = 0; lineIdx < numLines; ++lineIdx) {
        lineDescData[lineIdx].startIndex = bezierTrajectories.at(lineIdx).lineDesc.startIndex;
    }

    const size_t numVars = attributeNames.size();
    std::vector<float> attributesMinValues(numVars, 0.0f);
    std::vector<float> attributesMaxValues(numVars, 0.0f);
    for (size_t lineIdx = 0; lineIdx < numLines; ++lineIdx) {
        BezierTrajectory& bezierTrajectory = bezierTrajectories.at(lineIdx);
        for (auto varIdx = 0; varIdx < numVars; ++varIdx) {
            attributesMinValues.at(varIdx) = std::min(
                    attributesMinValues.at(varIdx), bezierTrajectory.multiVarDescs.at(varIdx).minMax.x);
            attributesMaxValues.at(varIdx) = std::max(
                    attributesMaxValues.at(varIdx), bezierTrajectory.multiVarDescs.at(varIdx).minMax.y);
        }
    }

    for (size_t lineIdx = 0; lineIdx < numLines; ++lineIdx) {
        for (auto varIdx = 0; varIdx < numVars; ++varIdx) {
            BezierTrajectory& bezierTrajectory = bezierTrajectories.at(lineIdx);

            VarDescData descData;
            descData.info = glm::vec4(
                    bezierTrajectory.multiVarDescs.at(varIdx).startIndex,
                    attributesMinValues.at(varIdx), attributesMaxValues.at(varIdx), 0.0);
            varDescData.push_back(descData);

            LineVarDescData lineVarDesc;
            lineVarDesc.minMax = glm::vec2(
                    bezierTrajectory.multiVarDescs.at(varIdx).minMax.x,
                    bezierTrajectory.multiVarDescs.at(varIdx).minMax.y);
            lineVarDescData.push_back(lineVarDesc);
        }
    }

    tubeRenderData.variableArrayBuffer = sgl::Renderer->createGeometryBuffer(
            varData.size()*sizeof(float), (void*)&varData.front(),
            sgl::SHADER_STORAGE_BUFFER);
    tubeRenderData.lineDescArrayBuffer = sgl::Renderer->createGeometryBuffer(
            lineDescData.size()*sizeof(LineDescData), (void*)&lineDescData.front(),
            sgl::SHADER_STORAGE_BUFFER);
    tubeRenderData.varDescArrayBuffer = sgl::Renderer->createGeometryBuffer(
            varDescData.size()*sizeof(VarDescData), (void*)&varDescData.front(),
            sgl::SHADER_STORAGE_BUFFER);
    tubeRenderData.lineVarDescArrayBuffer = sgl::Renderer->createGeometryBuffer(
            lineVarDescData.size()*sizeof(LineVarDescData), (void*)&lineVarDescData.front(),
            sgl::SHADER_STORAGE_BUFFER);
    tubeRenderData.varSelectedArrayBuffer = sgl::Renderer->createGeometryBuffer(
            varSelected.size()*sizeof(uint32_t), (void*)&varSelected.front(),
            sgl::SHADER_STORAGE_BUFFER);
    /*tubeRenderData.varColorArrayBuffer = sgl::Renderer->createGeometryBuffer(
            varColors.size()*sizeof(glm::vec4), (void*)&varColors.front(),
            sgl::SHADER_STORAGE_BUFFER);*/

    return tubeRenderData;
}
