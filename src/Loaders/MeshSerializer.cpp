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

#include <fstream>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>

#include <boost/algorithm/string/predicate.hpp>
#include <glm/glm.hpp>

#include <Utils/Events/Stream/Stream.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/Convert.hpp>
#include <Utils/SciVis/ImportanceCriteria.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>
#include <Graphics/Renderer.hpp>

#include "MeshSerializer.hpp"

using namespace std;

const uint32_t MESH_FORMAT_VERSION = 4u;

void writeMesh3D(const std::string &filename, const BinaryMesh &mesh) {
#ifndef __MINGW32__
    std::ofstream file(filename.c_str(), std::ofstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in writeMesh3D: File \"" + filename + "\" could not be opened for writing.");
        return;
    }
 #else
    FILE *fileptr = fopen(filename.c_str(), "wb");
    if (fileptr == NULL) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in writeMesh3D: File \"" + filename + "\" could not be opened for writing.");
        return;
    }
 #endif

    sgl::BinaryWriteStream stream;
    stream.write((uint32_t)MESH_FORMAT_VERSION);
    stream.write((uint32_t)mesh.submeshes.size());

    for (const BinarySubMesh &submesh : mesh.submeshes) {
        stream.write(submesh.material);
        stream.write((uint32_t)submesh.vertexMode);
        stream.writeArray(submesh.indices);

        // Write attributes
        stream.write((uint32_t)submesh.attributes.size());
        for (const BinaryMeshAttribute &attribute : submesh.attributes) {
            stream.write(attribute.name);
            stream.write((uint32_t)attribute.attributeFormat);
            stream.write((uint32_t)attribute.numComponents);
            stream.writeArray(attribute.data);
        }

        // Write uniforms
        stream.write((uint32_t)submesh.uniforms.size());
        for (const BinaryMeshUniform &uniform : submesh.uniforms) {
            stream.write(uniform.name);
            stream.write((uint32_t)uniform.attributeFormat);
            stream.write((uint32_t)uniform.numComponents);
            stream.writeArray(uniform.data);
        }
    }

#ifndef __MINGW32__
    file.write((const char*)stream.getBuffer(), stream.getSize());
    file.close();
#else
    fwrite((const void*)stream.getBuffer(), stream.getSize(), 1, fileptr);
    fclose(fileptr);
#endif
}

void readMesh3D(const std::string &filename, BinaryMesh &mesh) {
#ifndef __MINGW32__
    std::ifstream file(filename.c_str(), std::ifstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(std::string() + "Error in readMesh3D: File \"" + filename + "\" not found.");
        return;
    }

    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0);
    char *buffer = new char[size];
    file.read(buffer, size);
    file.close();
#else
    /*
     * Using MinGW-w64, the code above doesn't work, as streamsize == ptrdiff_t == __PTRDIFF_TYPE__ == long int
     * is only 32-bits long, and thus is is faulty for large files > 2GiB.
     */
    FILE *fileptr = fopen(filename.c_str(), "rb");
    if (fileptr == NULL) {
        sgl::Logfile::get()->writeError(std::string() + "Error in readMesh3D: File \"" + filename + "\" not found.");
        return;
    }
    fseeko64(fileptr, 0L, SEEK_END);
    size_t size = ftello64 (fileptr);
    fseeko64(fileptr, 0L, SEEK_SET);
    char *buffer = new char[size];
    fseek(fileptr, 0L, SEEK_SET);
    uint64_t readSize = fread(buffer, 1, size, fileptr);
    assert(size == readSize);
    fclose(fileptr);
#endif

    sgl::BinaryReadStream stream(buffer, size);
    uint32_t version;
    stream.read(version);
    if (version != MESH_FORMAT_VERSION) {
        sgl::Logfile::get()->writeError(std::string() + "Error in readMesh3D: Invalid version in file \""
                + filename + "\".");
        return;
    }

    uint32_t numSubmeshes;
    stream.read(numSubmeshes);
    mesh.submeshes.resize(numSubmeshes);

    for (uint32_t i = 0; i < numSubmeshes; i++) {
        BinarySubMesh &submesh = mesh.submeshes.at(i);
        stream.read(submesh.material);
        uint32_t vertexMode;
        stream.read(vertexMode);
        submesh.vertexMode = (sgl::VertexMode)vertexMode;
        stream.readArray(mesh.submeshes.at(i).indices);

        // Read attributes
        uint32_t numAttributes;
        stream.read(numAttributes);
        submesh.attributes.resize(numAttributes);

        for (uint32_t j = 0; j < numAttributes; j++) {
            BinaryMeshAttribute &attribute = submesh.attributes.at(j);
            stream.read(attribute.name);
            uint32_t format;
            stream.read(format);
            attribute.attributeFormat = (sgl::VertexAttributeFormat)format;
            stream.read(attribute.numComponents);
            stream.readArray(attribute.data);
        }

        // Read uniforms
        uint32_t numUniforms;
        stream.read(numUniforms);
        submesh.uniforms.resize(numUniforms);

        for (uint32_t j = 0; j < numUniforms; j++) {
            BinaryMeshUniform &uniform = submesh.uniforms.at(j);
            stream.read(uniform.name);
            uint32_t format;
            stream.read(format);
            uniform.attributeFormat = (sgl::VertexAttributeFormat)format;
            stream.read(uniform.numComponents);
            stream.readArray(uniform.data);
        }
    }

    //delete[] buffer; // BinaryReadStream does deallocation
}





void MeshRenderer::render(sgl::ShaderProgramPtr passShader, bool isGBufferPass, int attributeIndex) {
    if (useProgrammableFetch) {
        for (SSBOEntry &ssboEntry : ssboEntries) {
            if (ssboEntry.bindingPoint >= 0 && (!boost::starts_with(ssboEntry.attributeName, "vertexAttribute")
                    || attributeIndex == sgl::fromString<int>(ssboEntry.attributeName.substr(15)))) {
                sgl::ShaderManager->bindShaderStorageBuffer(ssboEntry.bindingPoint, ssboEntry.attributeBuffer);
            }
        }
    }

    for (size_t i = 0; i < shaderAttributes.size(); i++) {
        //ShaderProgram *shader = shaderAttributes.at(i)->getShaderProgram();
        if (!boost::starts_with(passShader->getShaderList().front()->getFileID(), "PseudoPhongVorticity")
                && !boost::starts_with(passShader->getShaderList().front()->getFileID(), "DepthPeelingGatherDepthComplexity")
                && !isGBufferPass) {
            if (passShader->hasUniform("ambientColor")) {
                passShader->setUniform("ambientColor", materials.at(i).ambientColor);
            }
            if (passShader->hasUniform("diffuseColor")) {
                passShader->setUniform("diffuseColor", materials.at(i).diffuseColor);
            }
            if (passShader->hasUniform("specularColor")) {
                passShader->setUniform("specularColor", materials.at(i).specularColor);
            }
            if (passShader->hasUniform("specularExponent")) {
                passShader->setUniform("specularExponent", materials.at(i).specularExponent);
            }
            if (passShader->hasUniform("opacity")) {
                passShader->setUniform("opacity", materials.at(i).opacity);
            }
        }
        sgl::Renderer->render(shaderAttributes.at(i), passShader);
    }
}

void MeshRenderer::setNewShader(sgl::ShaderProgramPtr newShader) {
    for (size_t i = 0; i < shaderAttributes.size(); i++) {
        shaderAttributes.at(i) = shaderAttributes.at(i)->copy(newShader, false);
    }
}


sgl::AABB3 computeAABB(const std::vector<glm::vec3> &vertices) {
    if (vertices.size() < 1) {
        sgl::Logfile::get()->writeError("computeAABB: vertices.size() < 1");
        return sgl::AABB3();
    }

    glm::vec3 minV = glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    glm::vec3 maxV = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (const glm::vec3 &pt : vertices) {
        minV.x = std::min(minV.x, pt.x);
        minV.y = std::min(minV.y, pt.y);
        minV.z = std::min(minV.z, pt.z);
        maxV.x = std::max(maxV.x, pt.x);
        maxV.y = std::max(maxV.y, pt.y);
        maxV.z = std::max(maxV.z, pt.z);
    }

    return sgl::AABB3(minV, maxV);
}

std::vector<uint32_t> shuffleIndicesLines(const std::vector<uint32_t> &indices) {
    size_t numSegments = indices.size() / 2;
    std::vector<size_t> shuffleOffsets;
    for (size_t i = 0; i < numSegments; i++) {
        shuffleOffsets.push_back(i);
    }
    auto rng = std::default_random_engine{};
    rng.seed(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::shuffle(std::begin(shuffleOffsets), std::end(shuffleOffsets), rng);

    std::vector<uint32_t> shuffledIndices;
    shuffledIndices.reserve(numSegments*2);
    for (size_t i = 0; i < numSegments; i++) {
        size_t lineIndex = shuffleOffsets.at(i);
        shuffledIndices.push_back(indices.at(lineIndex*2));
        shuffledIndices.push_back(indices.at(lineIndex*2+1));
    }

    return shuffledIndices;
}

std::vector<uint32_t> shuffleLineOrder(const std::vector<uint32_t> &indices) {
    size_t numSegments = indices.size() / 2;

    // 1. Compute list of all lines
    std::vector<std::vector<uint32_t>> lines;
    std::vector<uint32_t> currentLine;
    for (size_t i = 0; i < numSegments; i++) {
        uint32_t idx0 = indices.at(i*2);
        uint32_t idx1 = indices.at(i*2+1);

        // Start new line?
        if (i > 0 && idx0 != indices.at((i-1)*2+1)) {
            lines.push_back(currentLine);
            currentLine.clear();
        }

        // Add indices to line
        currentLine.push_back(idx0);
        currentLine.push_back(idx1);
    }
    if (!currentLine.empty()) {
        lines.push_back(currentLine);
    }

    // 2. Shuffle line list
    auto rng = std::default_random_engine{};
    rng.seed(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::shuffle(std::begin(lines), std::end(lines), rng);

    // 3. Reconstruct line list from shuffled lines
    std::vector<uint32_t> shuffledIndices;
    shuffledIndices.reserve(indices.size());
    for (const std::vector<uint32_t> &line : lines) {
        for (uint32_t idx : line) {
            shuffledIndices.push_back(idx);
        }
    }

    return shuffledIndices;
}

std::vector<uint32_t> shuffleIndicesTriangles(const std::vector<uint32_t> &indices) {
    size_t numSegments = indices.size() / 3;
    std::vector<size_t> shuffleOffsets;
    for (size_t i = 0; i < numSegments; i++) {
        shuffleOffsets.push_back(i);
    }
    auto rng = std::default_random_engine{};
    rng.seed(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::shuffle(std::begin(shuffleOffsets), std::end(shuffleOffsets), rng);

    std::vector<uint32_t> shuffledIndices;
    shuffledIndices.reserve(numSegments*3);
    for (size_t i = 0; i < numSegments; i++) {
        size_t lineIndex = shuffleOffsets.at(i);
        shuffledIndices.push_back(indices.at(lineIndex*3));
        shuffledIndices.push_back(indices.at(lineIndex*3+1));
        shuffledIndices.push_back(indices.at(lineIndex*3+2));
    }

    return shuffledIndices;
}


struct LinePointData
{
    glm::vec3 vertexPosition;
    float vertexAttribute;
    glm::vec3 vertexTangent;
    float padding;
};

MeshRenderer parseMesh3d(const std::string &filename, sgl::ShaderProgramPtr shader, bool shuffleData,
        bool useProgrammableFetch, bool programmableFetchUseAoS, float lineRadius) {
    MeshRenderer meshRenderer(useProgrammableFetch);
    BinaryMesh mesh;
    readMesh3D(filename, mesh);

    if (!shader) {
        shader = sgl::ShaderManager->getShaderProgram({"PseudoPhong.Vertex", "PseudoPhong.Fragment"});
    }

    std::vector<sgl::ShaderAttributesPtr> &shaderAttributes = meshRenderer.shaderAttributes;
    std::vector<ObjMaterial> &materials = meshRenderer.materials;
    shaderAttributes.reserve(mesh.submeshes.size());
    materials.reserve(mesh.submeshes.size());

    // Bounding box of all submeshes combined
    sgl::AABB3 totalBoundingBox(glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX), glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX));

    // Importance criterion attributes are bound to location 3 and onwards in vertex shader
    //int importanceCriterionLocationCounter = 3;


    // Iterate over all submeshes and create rendering data
    for (size_t i = 0; i < mesh.submeshes.size(); i++) {
        BinarySubMesh &submesh = mesh.submeshes.at(i);
        sgl::ShaderAttributesPtr renderData = sgl::ShaderManager->createShaderAttributes(shader);
        if (!useProgrammableFetch) {
            renderData->setVertexMode(submesh.vertexMode);
        } else {
            renderData->setVertexMode(sgl::VERTEX_MODE_TRIANGLES);
        }

        if (submesh.indices.size() > 0 && !useProgrammableFetch) {
            if (shuffleData && (submesh.vertexMode == sgl::VERTEX_MODE_LINES
                    || submesh.vertexMode == sgl::VERTEX_MODE_TRIANGLES)) {
                std::vector<uint32_t> shuffledIndices;
                if (submesh.vertexMode == sgl::VERTEX_MODE_LINES) {
                    //shuffledIndices = shuffleIndicesLines(submesh.indices);
                    shuffledIndices = shuffleLineOrder(submesh.indices);
                } else if (submesh.vertexMode == sgl::VERTEX_MODE_TRIANGLES) {
                    shuffledIndices = shuffleIndicesTriangles(submesh.indices);
                } else {
                    sgl::Logfile::get()->writeError(
                            "ERROR in parseMesh3D: shuffleData and unsupported vertex mode!");
                    shuffledIndices = submesh.indices;
                }
                sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
                        sizeof(uint32_t)*shuffledIndices.size(), shuffledIndices.data(),
                        sgl::INDEX_BUFFER);
                renderData->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
            } else {
                sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
                        sizeof(uint32_t)*submesh.indices.size(), submesh.indices.data(),
                        sgl::INDEX_BUFFER);
                renderData->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
            }
        }
        if (submesh.indices.size() > 0 && useProgrammableFetch) {
            // Modify indices
            std::vector<uint32_t> fetchIndices;
            fetchIndices.reserve(submesh.indices.size()*3);
            // Iterate over all line segments
            for (size_t i = 0; i < submesh.indices.size(); i += 2) {
                uint32_t base0 = submesh.indices.at(i)*2;
                uint32_t base1 = submesh.indices.at(i+1)*2;
                // 0,2,3,0,3,1
                fetchIndices.push_back(base0);
                fetchIndices.push_back(base1);
                fetchIndices.push_back(base1+1);
                fetchIndices.push_back(base0);
                fetchIndices.push_back(base1+1);
                fetchIndices.push_back(base0+1);
            }
            sgl::GeometryBufferPtr indexBuffer = sgl::Renderer->createGeometryBuffer(
                    sizeof(uint32_t)*fetchIndices.size(), fetchIndices.data(), sgl::INDEX_BUFFER);
            renderData->setIndexGeometryBuffer(indexBuffer, sgl::ATTRIB_UNSIGNED_INT);
        }

        // For programmableFetchUseAoS
        std::vector<glm::vec3> vertexPositionData;
        std::vector<std::vector<float>> vertexAttributeData;
        std::vector<glm::vec3> vertexTangentData;

        for (size_t j = 0; j < submesh.attributes.size(); j++) {
            BinaryMeshAttribute &meshAttribute = submesh.attributes.at(j);
            sgl::GeometryBufferPtr attributeBuffer;

            // Assume only one component means importance criterion like vorticity, line width, ...
            if (meshAttribute.numComponents == 1) {
                ImportanceCriterionAttribute importanceCriterionAttribute;
                importanceCriterionAttribute.name = meshAttribute.name;

                // Copy values to mesh renderer data structure
                uint16_t *attributeValuesUnorm = (uint16_t*)meshAttribute.data.data();
                size_t numAttributeValues = meshAttribute.data.size() / sizeof(uint16_t);
                sgl::unpackUnorm16Array(
                        attributeValuesUnorm, numAttributeValues,
                        importanceCriterionAttribute.attributes);

                // Compute minimum and maximum value
                float minValue = FLT_MAX, maxValue = 0.0f;
#if _OPENMP >= 201107
                #pragma omp parallel for reduction(min:minValue) reduction(max:maxValue) default(none) \
                shared(numAttributeValues, importanceCriterionAttribute)
#endif
                for (size_t k = 0; k < numAttributeValues; k++) {
                    minValue = std::min(minValue, importanceCriterionAttribute.attributes[k]);
                    maxValue = std::max(maxValue, importanceCriterionAttribute.attributes[k]);
                }
                importanceCriterionAttribute.minAttribute = minValue;
                importanceCriterionAttribute.maxAttribute = maxValue;

                meshRenderer.importanceCriterionAttributes.push_back(importanceCriterionAttribute);

                // SSBOs can't directly perform process uint16_t -> float :(
                if (useProgrammableFetch && !programmableFetchUseAoS) {
                    attributeBuffer = sgl::Renderer->createGeometryBuffer(
                            numAttributeValues*sizeof(float), importanceCriterionAttribute.attributes.data(),
                            sgl::SHADER_STORAGE_BUFFER);
                } else if (useProgrammableFetch) {
                    int attributeIndex = sgl::fromString<int>(meshAttribute.name.substr(15));
                    if (attributeIndex >= int(vertexAttributeData.size())) {
                        vertexAttributeData.resize(attributeIndex+1);
                    }
                    vertexAttributeData.at(attributeIndex).resize(numAttributeValues);
                    for (size_t k = 0; k < numAttributeValues; k++) {
                        vertexAttributeData.at(attributeIndex).at(k) = importanceCriterionAttribute.attributes[k];
                    }
                }
            }

            sgl::BufferType bufferType = useProgrammableFetch ? sgl::SHADER_STORAGE_BUFFER : sgl::VERTEX_BUFFER;

            if (!(useProgrammableFetch && programmableFetchUseAoS)
                && !(meshAttribute.numComponents == 1 && useProgrammableFetch)
                && !(meshAttribute.numComponents == 3 && useProgrammableFetch)) {
                attributeBuffer = sgl::Renderer->createGeometryBuffer(
                        meshAttribute.data.size(), meshAttribute.data.data(), bufferType);
            }
            if (meshAttribute.numComponents == 3 && (useProgrammableFetch && !programmableFetchUseAoS)) {
                // vec3 problematic in std430 struct
                glm::vec3* attributeValues = (glm::vec3*)meshAttribute.data.data();
                size_t numAttributeValues = meshAttribute.data.size() / sizeof(glm::vec3);
                std::vector<glm::vec4> vec4AttributeValues;
                vec4AttributeValues.reserve(numAttributeValues);
                for (size_t i = 0; i < numAttributeValues; i++) {
                    glm::vec3 vec3Value = attributeValues[i];
                    vec4AttributeValues.push_back(glm::vec4(vec3Value.x, vec3Value.y, vec3Value.z, 1.0f));
                }
                attributeBuffer = sgl::Renderer->createGeometryBuffer(
                        vec4AttributeValues.size()*sizeof(glm::vec4), vec4AttributeValues.data(), bufferType);
            }

            if (!useProgrammableFetch) {
                if (meshAttribute.numComponents == 1) {
                    // Importance criterion attributes are bound to location 3 and onwards in vertex shader
                    renderData->addGeometryBufferOptional(
                            attributeBuffer, meshAttribute.name.c_str(),
                            meshAttribute.attributeFormat, meshAttribute.numComponents,
                            0, 0, 0, sgl::ATTRIB_CONVERSION_FLOAT_NORMALIZED);
                } else {
                    bool isNormalizedColor = (meshAttribute.name == "vertexColor");
                    renderData->addGeometryBufferOptional(
                            attributeBuffer, meshAttribute.name.c_str(),
                            meshAttribute.attributeFormat, meshAttribute.numComponents,
                            0, 0, 0, isNormalizedColor ?
                            sgl::ATTRIB_CONVERSION_FLOAT_NORMALIZED : sgl::ATTRIB_CONVERSION_FLOAT);
                }
                meshRenderer.shaderAttributeNames.insert(meshAttribute.name);
            } else {
                if (programmableFetchUseAoS) {
                    if (meshAttribute.name == "vertexPosition") {
                        glm::vec3 *attributeValues = (glm::vec3*)meshAttribute.data.data();
                        size_t numAttributeValues = meshAttribute.data.size() / sizeof(glm::vec3);
                        vertexPositionData.reserve(numAttributeValues);
                        for (size_t i = 0; i < numAttributeValues; i++) {
                            vertexPositionData.push_back(attributeValues[i]);
                        }
                    } else if (meshAttribute.name == "vertexLineTangent") {
                        glm::vec3* attributeValues = (glm::vec3*)meshAttribute.data.data();
                        size_t numAttributeValues = meshAttribute.data.size() / sizeof(glm::vec3);
                        vertexTangentData.reserve(numAttributeValues);
                        for (size_t i = 0; i < numAttributeValues; i++) {
                            vertexTangentData.push_back(attributeValues[i]);
                        }
                    }
                } else {
                    int bindingPoint = -1;
                    if (meshAttribute.name == "vertexPosition") {
                        bindingPoint = 2;
                    } else if (meshAttribute.name == "vertexLineTangent") {
                        bindingPoint = 3;
                    } else if (boost::starts_with(meshAttribute.name, "vertexAttribute")) {
                        bindingPoint = 4;
                    }
                    meshRenderer.ssboEntries.emplace_back(
                            bindingPoint, meshAttribute.name, attributeBuffer);
                }
            }

            if (meshAttribute.name == "vertexPosition") {
                std::vector<glm::vec3> vertices;
                vertices.resize(meshAttribute.data.size() / sizeof(glm::vec3));
                memcpy(vertices.data(), meshAttribute.data.data(), meshAttribute.data.size());
                totalBoundingBox.combine(computeAABB(vertices));
            }
        }

        if (useProgrammableFetch && programmableFetchUseAoS) {
            for (size_t attributeIndex = 0; attributeIndex < vertexAttributeData.size(); attributeIndex++) {
                std::vector<LinePointData> linePointData;
                linePointData.resize(vertexPositionData.size());

                for (size_t i = 0; i < vertexPositionData.size(); i++) {
                    linePointData.at(i).vertexPosition = vertexPositionData.at(i);
                    linePointData.at(i).vertexAttribute = vertexAttributeData.at(attributeIndex).at(i);
                    linePointData.at(i).vertexTangent = vertexTangentData.at(i);
                    linePointData.at(i).padding = 0.0f;
                }

                sgl::GeometryBufferPtr attributeBuffer = sgl::Renderer->createGeometryBuffer(
                        linePointData.size()*sizeof(LinePointData), linePointData.data(),
                        sgl::SHADER_STORAGE_BUFFER);
                meshRenderer.ssboEntries.emplace_back(
                        2, "vertexAttribute" + sgl::toString(attributeIndex),
                        attributeBuffer);
            }
        }

        shaderAttributes.push_back(renderData);
        materials.push_back(mesh.submeshes.at(i).material);
    }

    meshRenderer.boundingBox = totalBoundingBox;
    meshRenderer.boundingSphere = sgl::Sphere(totalBoundingBox.getCenter(), glm::length(totalBoundingBox.getExtent()));

    return meshRenderer;
}
