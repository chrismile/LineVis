/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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
#include <Utils/Parallel/Reduction.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>

#include "Loaders/TriangleMesh/BinaryObjLoader.hpp"
#include "Loaders/TriangleMesh/ObjLoader.hpp"
#include "Loaders/TriangleMesh/StlLoader.hpp"
#include "TriangleMeshData.hpp"

TriangleMeshData::TriangleMeshData(sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineData(transferFunctionWindow, DATA_SET_TYPE_TRIANGLE_MESH) {
    lineDataWindowName = "Triangle Mesh";
}

TriangleMeshData::~TriangleMeshData() = default;

bool TriangleMeshData::getIsSmallDataSet() const {
    return vertexPositions.size() <= SMALL_DATASET_VERTICES_MAX && triangleIndices.size() <= SMALL_DATASET_INDICES_MAX;
}

bool TriangleMeshData::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool shallReloadGatherShader = LineData::renderGuiPropertyEditorNodes(propertyEditor);

    // TODO: Add UI options here.

    return shallReloadGatherShader;
}

void TriangleMeshData::recomputeHistogram() {
    const std::vector<float>& attributeList = vertexAttributesList.at(selectedAttributeIndex);
    glm::vec2 minMaxAttributes;
    if (selectedAttributeIndex < int(minMaxAttributeValues.size())) {
        minMaxAttributes = minMaxAttributeValues.at(selectedAttributeIndex);
    } else {
        minMaxAttributes = glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
    }
    transferFunctionWindow.computeHistogram(
            attributeList, minMaxAttributes.x, minMaxAttributes.y);

    //multiVarTransferFunctionWindow.setAttributesValues(attributeNames, attributesList);

    recomputeColorLegend();
}

bool TriangleMeshData::loadFromFile(
        const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
        glm::mat4* transformationMatrixPtr) {
    this->fileNames = fileNames;
    attributeNames = dataSetInformation.attributeNames;
    std::string extension = sgl::FileUtils::get()->getFileExtensionLower(fileNames.front());
    if (extension == "bobj") {
        loadBinaryObjTriangleMesh(
                fileNames.front(), triangleIndices, vertexPositions, vertexNormals, vertexAttributesList, attributeNames,
                true, false, nullptr, transformationMatrixPtr);
    } if (extension == "obj") {
        loadObjTriangleMesh(
                fileNames.front(), triangleIndices, vertexPositions, vertexNormals, vertexAttributesList, attributeNames,
                true, false, nullptr, transformationMatrixPtr);
    } if (extension == "stl") {
        loadStlTriangleMesh(
                fileNames.front(), triangleIndices, vertexPositions, vertexNormals, vertexAttributesList, attributeNames,
                true, false, nullptr, transformationMatrixPtr);
    }
    bool dataLoaded = !triangleIndices.empty();

    if (!dataLoaded) {
        triangleIndices = {};
        vertexPositions = {};
        vertexNormals = {};
        vertexAttributesList = {};
        attributeNames = {};
    } else {
        for (size_t attrIdx = attributeNames.size(); attrIdx < getNumAttributes(); attrIdx++) {
            attributeNames.push_back(std::string() + "Attribute #" + std::to_string(attrIdx + 1));
        }

        colorLegendWidgets.clear();
        colorLegendWidgets.resize(attributeNames.size());
        for (size_t i = 0; i < colorLegendWidgets.size(); i++) {
            colorLegendWidgets.at(i).setPositionIndex(0, 1);
        }

        minMaxAttributeValues.clear();
        for (size_t varIdx = 0; varIdx < colorLegendWidgets.size(); varIdx++) {
            std::vector<float>& vertexAttributes = vertexAttributesList.at(varIdx);
            auto [minAttr, maxAttr] = sgl::reduceFloatArrayMinMax(vertexAttributes);
            minMaxAttributeValues.emplace_back(minAttr, maxAttr);
            colorLegendWidgets[varIdx].setAttributeMinValue(minAttr);
            colorLegendWidgets[varIdx].setAttributeMaxValue(maxAttr);
            colorLegendWidgets[varIdx].setAttributeDisplayName(
                    std::string() + attributeNames.at(varIdx));
        }

        modelBoundingBox = sgl::reduceVec3ArrayAabb(vertexPositions);
        focusBoundingBox = modelBoundingBox;
    }

    dirty = true;
    return dataLoaded;
}

void TriangleMeshData::onMainThreadDataInit(){
    linePrimitiveMode = LINE_PRIMITIVES_TUBE_TRIANGLE_MESH;
}

sgl::MultiVarTransferFunctionWindow& TriangleMeshData::getMultiVarTransferFunctionWindow() {
    sgl::Logfile::get()->throwError(
            "Error in TriangleMeshData::getMultiVarTransferFunctionWindow: "
            "Multi-parameter visualizations not supported for this data type.");
    return multiVarTransferFunctionWindow;
}



// --- Retrieve data for rendering. Preferred way. ---
void TriangleMeshData::setGraphicsPipelineInfo(
        sgl::vk::GraphicsPipelineInfo& pipelineInfo, const sgl::vk::ShaderStagesPtr& shaderStages) {
    pipelineInfo.setInputAssemblyTopology(sgl::vk::PrimitiveTopology::TRIANGLE_LIST);
    if (useBackfaceCulling) {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_BACK);
    } else {
        pipelineInfo.setCullMode(sgl::vk::CullMode::CULL_NONE);
    }
}

void TriangleMeshData::setVulkanRenderDataDescriptors(const sgl::vk::RenderDataPtr& renderData) {
    LineData::setVulkanRenderDataDescriptors(renderData);
}

void TriangleMeshData::updateVulkanUniformBuffers(LineRenderer* lineRenderer, sgl::vk::Renderer* renderer) {
    // TODO: Add options here.
    //lineUniformData.helicityRotationFactor = helicityRotationFactor;

    LineData::updateVulkanUniformBuffers(lineRenderer, renderer);
}

void TriangleMeshData::setRasterDataBindings(sgl::vk::RasterDataPtr& rasterData) {
    setVulkanRenderDataDescriptors(rasterData);

    if (!getLinePrimitiveModeUsesTriangleMesh(linePrimitiveMode)) {
        return;
    }

    TubeTriangleRenderData tubeRenderData = this->getLinePassTubeTriangleMeshRenderData(
            true, false);
    if (!tubeRenderData.indexBuffer) {
        return;
    }
    rasterData->setIndexBuffer(tubeRenderData.indexBuffer);
    rasterData->setStaticBuffer(tubeRenderData.vertexBuffer, "TubeTriangleVertexDataBuffer");
}



// --- Retrieve data for rendering. ---

TubeTriangleRenderData TriangleMeshData::getLinePassTubeTriangleMeshRenderDataPayload(
        bool isRasterizer, bool vulkanRayTracing, TubeTriangleRenderDataPayloadPtr& payload) {
    rebuildInternalRepresentationIfNecessary();
    bool payloadCompatible = true;
    if (payload && cachedTubeTriangleRenderDataPayload) {
        payloadCompatible = payload->settingsEqual(cachedTubeTriangleRenderDataPayload.get());
    } else if (payload && !cachedTubeTriangleRenderDataPayload) {
        payloadCompatible = false;
    }
    if (cachedTubeTriangleRenderData.vertexBuffer && cachedTubeTriangleRenderDataIsRayTracing == vulkanRayTracing
        && payloadCompatible) {
        if (cachedTubeTriangleRenderDataPayload) {
            payload = cachedTubeTriangleRenderDataPayload;
        }
        return cachedTubeTriangleRenderData;
    }
    removeOtherCachedDataTypes(RequestMode::TRIANGLES);
    cachedTubeTriangleRenderDataPayload = payload;

    std::vector<uint32_t> triangleIndexData = triangleIndices;
    std::vector<TubeTriangleVertexData> vertexDataList;
    std::vector<float>& vertexAttributes = vertexAttributesList.at(selectedAttributeIndex);
    vertexDataList.resize(vertexNormals.size());
    for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
        TubeTriangleVertexData& vertexData = vertexDataList.at(vertexIdx);
        vertexData.vertexPosition = vertexPositions.at(vertexIdx);
        vertexData.vertexNormal = vertexNormals.at(vertexIdx);
        vertexData.phi = vertexAttributes.at(vertexIdx);
        vertexData.vertexLinePointIndex = 0;
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    cachedTubeTriangleRenderData = {};
    cachedTubeTriangleRenderDataIsRayTracing = vulkanRayTracing;

    if (triangleIndexData.empty()) {
        return cachedTubeTriangleRenderData;
    }

    if (generateSplitTriangleData) {
        splitTriangleIndices(triangleIndexData, vertexDataList);
    }
    if (payload) {
        payload->createPayloadPre(
                device, uint32_t(tubeNumSubdivisions), triangleIndexData, vertexDataList, {});
    }

    uint32_t indexBufferFlags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    uint32_t vertexBufferFlags =
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    if (vulkanRayTracing) {
        indexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
        vertexBufferFlags |=
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT
                | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
    }

    cachedTubeTriangleRenderData.indexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, triangleIndexData.size() * sizeof(uint32_t), triangleIndexData.data(),
            indexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    cachedTubeTriangleRenderData.vertexBuffer = std::make_shared<sgl::vk::Buffer>(
            device, vertexDataList.size() * sizeof(TubeTriangleVertexData), vertexDataList.data(),
            vertexBufferFlags, VMA_MEMORY_USAGE_GPU_ONLY);

    cachedTubeTriangleRenderData.linePointDataBuffer = {};

    if (generateSplitTriangleData) {
        std::vector<uint32_t> instanceTriangleIndexOffsets;
        uint32_t batchIndexBufferOffset = 0;
        for (const uint32_t& batchNumIndices : tubeTriangleSplitData.numBatchIndices) {
            instanceTriangleIndexOffsets.push_back(batchIndexBufferOffset);
            batchIndexBufferOffset += uint32_t(batchNumIndices) / 3u;
        }
        cachedTubeTriangleRenderData.instanceTriangleIndexOffsetBuffer = std::make_shared<sgl::vk::Buffer>(
                device, instanceTriangleIndexOffsets.size() * sizeof(uint32_t),
                instanceTriangleIndexOffsets.data(),
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
    }

    if (payload) {
        payload->createPayloadPost(device, cachedTubeTriangleRenderData);
    }

    return cachedTubeTriangleRenderData;
}

void TriangleMeshData::getVulkanShaderPreprocessorDefines(
        std::map<std::string, std::string>& preprocessorDefines, bool isRasterizer) {
    LineData::getVulkanShaderPreprocessorDefines(preprocessorDefines, isRasterizer);
    preprocessorDefines.insert(std::make_pair("GENERAL_TRIANGLE_MESH", ""));
}



// --- Retrieve triangle mesh on the CPU. ---

void TriangleMeshData::getTriangleMesh(
        LineRenderer* lineRenderer,
        std::vector<uint32_t>& _triangleIndices, std::vector<glm::vec3>& _vertexPositions,
        std::vector<glm::vec3>& _vertexNormals, std::vector<float>& _vertexAttributes) {
    rebuildInternalRepresentationIfNecessary();

    _triangleIndices = triangleIndices;
    _vertexPositions = vertexPositions;
    _vertexNormals = vertexNormals;
    _vertexAttributes = vertexAttributesList.at(selectedAttributeIndex);
}

void TriangleMeshData::getTriangleMesh(
        LineRenderer* lineRenderer,
        std::vector<uint32_t>& _triangleIndices, std::vector<glm::vec3>& _vertexPositions) {
    rebuildInternalRepresentationIfNecessary();

    _triangleIndices = triangleIndices;
    _vertexPositions = vertexPositions;
}
