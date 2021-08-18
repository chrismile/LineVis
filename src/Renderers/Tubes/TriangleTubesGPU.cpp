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

#include <chrono>
#include <cstring>
#include <GL/glew.h>

#include <Utils/File/Logfile.hpp>
#include <Math/Math.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Renderer.hpp>

#include "Tubes.hpp"

/*
struct InputLinePoint {
    glm::vec3 linePoint;
    float lineAttribute;
};
struct OutputLinePoint {
    glm::vec3 linePoint;
    float lineAttribute;
    glm::vec3 lineTangent;
    uint32_t valid; // 0 or 1
    glm::vec3 lineNormal;
    float padding;
};
struct PathLinePoint {
    glm::vec3 linePointPosition;
    float linePointAttribute;
    glm::vec3 lineTangent;
    float padding0;
    glm::vec3 lineNormal;
    float padding1;
};

struct TubeVertex {
    glm::vec3 vertexPosition;
    float vertexAttribute;
    glm::vec3 vertexNormal;
    float padding0;
    glm::vec3 vertexTangent;
    float padding1;
};

void createTriangleTubesRenderDataGPU(
        const std::vector<std::vector<glm::vec3>>& lineCentersList,
        const std::vector<std::vector<float>>& lineAttributesList,
        float tubeRadius,
        int numCircleSubdivisions,
        std::vector<uint32_t>& triangleIndices,
        std::vector<glm::vec3>& vertexPositions,
        std::vector<glm::vec3>& vertexNormals,
        std::vector<glm::vec3>& vertexTangents,
        std::vector<float>& vertexAttributes) {
    auto start = std::chrono::system_clock::now();
    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("NUM_CIRCLE_SEGMENTS", numCircleSubdivisions);
    sgl::ShaderManager->addPreprocessorDefine("CIRCLE_RADIUS", tubeRadius);

    std::vector<uint32_t> lineOffsetsInput;
    uint64_t numLinesInput = 0;
    uint64_t numLinePointsInput = 0;

    std::vector<uint32_t> lineOffsetsOutput;
    uint64_t numLinesOutput = 0;
    uint64_t numLinePointsOutput = 0;

    std::vector<InputLinePoint> inputLinePoints;
    std::vector<OutputLinePoint> outputLinePoints;
    std::vector<PathLinePoint> pathLinePoints;

    assert(lineCentersList.size() == lineAttributesList.size());
    lineOffsetsInput.push_back(0);
    for (size_t lineId = 0; lineId < lineCentersList.size(); lineId++) {
        const std::vector<glm::vec3> &lineCenters = lineCentersList.at(lineId);
        const std::vector<float> &lineAttributes = lineAttributesList.at(lineId);
        assert(lineCenters.size() == lineAttributes.size());
        size_t n = lineCenters.size();

        InputLinePoint inputLinePoint;
        for (size_t j = 0; j < n; j++) {
            inputLinePoint.linePoint = lineCenters.at(j);
            inputLinePoint.lineAttribute = lineAttributes.at(j);
            inputLinePoints.push_back(inputLinePoint);
        }

        if (lineCenters.size() > 0) {
            numLinePointsInput += lineCenters.size();
            numLinesInput++;
        } else {
            continue;
        }
        lineOffsetsInput.push_back(numLinePointsInput);
    }

    const unsigned int WORK_GROUP_SIZE_1D = 256;
    sgl::ShaderManager->addPreprocessorDefine("WORK_GROUP_SIZE_1D", WORK_GROUP_SIZE_1D);
    unsigned int numWorkGroupsOld;
    uint32_t numWorkGroups;
    void *bufferMemory;

    // PART 1: Create line normals & mask invalid line points
    auto startNormals = std::chrono::system_clock::now();
    sgl::GeometryBufferPtr lineOffsetBufferInput = sgl::Renderer->createGeometryBuffer(
            (numLinesInput+1) * sizeof(uint32_t), lineOffsetsInput.data(),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    sgl::GeometryBufferPtr inputLinePointBuffer = sgl::Renderer->createGeometryBuffer(
            inputLinePoints.size() * sizeof(InputLinePoint), inputLinePoints.data(),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    sgl::GeometryBufferPtr outputLinePointBuffer = sgl::Renderer->createGeometryBuffer(
            inputLinePoints.size() * sizeof(OutputLinePoint),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);

    sgl::ShaderProgramPtr createLineNormalsShader = sgl::ShaderManager->getShaderProgram(
            {"CreateLineNormals.Compute"});
    sgl::ShaderManager->bindShaderStorageBuffer(2, lineOffsetBufferInput);
    sgl::ShaderManager->bindShaderStorageBuffer(3, inputLinePointBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(4, outputLinePointBuffer);
    createLineNormalsShader->setUniform("numLines", static_cast<uint32_t>(numLinesInput));
    numWorkGroupsOld = sgl::iceil(numLinesInput, WORK_GROUP_SIZE_1D); // last vector: local work group size
    numWorkGroups = (numLinesInput - 1) / WORK_GROUP_SIZE_1D + 1;

    createLineNormalsShader->dispatchCompute(numWorkGroups);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    bufferMemory = outputLinePointBuffer->mapBuffer(sgl::BUFFER_MAP_READ_ONLY);
    outputLinePoints.resize(inputLinePoints.size());
    memcpy(outputLinePoints.data(), bufferMemory, outputLinePoints.size() * sizeof(OutputLinePoint));
    outputLinePointBuffer->unmapBuffer();
    auto endNormals = std::chrono::system_clock::now();
    auto elapsedNormals = std::chrono::duration_cast<std::chrono::milliseconds>(endNormals - startNormals);
    sgl::Logfile::get()->writeInfo(std::string() + "Computational time to create normals: "
                              + std::to_string(elapsedNormals.count()));


    // PART 1.2: OutputLinePoint -> PathLinePoint (while removing invalid points)
    auto startCompact = std::chrono::system_clock::now();
    pathLinePoints.reserve(outputLinePoints.size());
    lineOffsetsOutput.push_back(0);
    for (size_t lineID = 0; lineID < numLinesInput; lineID++) {
        size_t linePointsOffset = lineOffsetsInput.at(lineID);
        size_t numLinePoints = lineOffsetsInput.at(lineID+1)-linePointsOffset;

        size_t currentLineNumPointsOutput = 0;

        for (size_t linePointID = 0; linePointID < numLinePoints; linePointID++) {
            OutputLinePoint &outputLinePoint = outputLinePoints.at(linePointsOffset+linePointID);
            if (outputLinePoint.valid == 1) {
                PathLinePoint pathLinePoint;
                pathLinePoint.linePointPosition = outputLinePoint.linePoint;
                pathLinePoint.linePointAttribute = outputLinePoint.lineAttribute;
                pathLinePoint.lineTangent = outputLinePoint.lineTangent;
                pathLinePoint.lineNormal = outputLinePoint.lineNormal;
                pathLinePoints.push_back(pathLinePoint);
                currentLineNumPointsOutput++;
                numLinePointsOutput++;
            }
        }

        if (currentLineNumPointsOutput > 0) {
            numLinesOutput++;
            lineOffsetsOutput.push_back(numLinePointsOutput);
        }
    }
    auto endCompact = std::chrono::system_clock::now();
    auto elapsedCompact = std::chrono::duration_cast<std::chrono::milliseconds>(endCompact - startCompact);
    sgl::Logfile::get()->writeInfo(
            std::string() + "Computational time to compact: " + std::to_string(elapsedCompact.count()));


    // PART 2: CreateTubePoints.Compute
    auto startTube = std::chrono::system_clock::now();
    std::vector<TubeVertex> tubeVertices;
    tubeVertices.resize(numCircleSubdivisions * pathLinePoints.size());

    sgl::GeometryBufferPtr pathLinePointsBuffer = sgl::Renderer->createGeometryBuffer(
            pathLinePoints.size() * sizeof(PathLinePoint), pathLinePoints.data(),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    sgl::GeometryBufferPtr tubeVertexBuffer = sgl::Renderer->createGeometryBuffer(
            numCircleSubdivisions * pathLinePoints.size() * sizeof(TubeVertex),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);

    int maxNumWorkGroupsSupported = 0;
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &maxNumWorkGroupsSupported);

    sgl::ShaderProgramPtr createTubePointsShader = sgl::ShaderManager->getShaderProgram(
            {"CreateTubePoints.Compute"});
    sgl::ShaderManager->bindShaderStorageBuffer(2, pathLinePointsBuffer);
    sgl::ShaderManager->bindShaderStorageBuffer(3, tubeVertexBuffer);
    createTubePointsShader->setUniform("numLinePoints", static_cast<uint32_t>(numLinePointsOutput));
    numWorkGroups = sgl::iceil(pathLinePoints.size(), WORK_GROUP_SIZE_1D);
    if (numWorkGroups > maxNumWorkGroupsSupported) {
        sgl::Logfile::get()->writeInfo(
                "Info: numWorkGroups > MAX_COMPUTE_WORK_GROUP_COUNT. Switching to CPU fallback.");
        createTriangleTubesRenderDataCPU(
                lineCentersList, lineAttributesList, tubeRadius, numCircleSubdivisions,
                triangleIndices, vertexPositions, vertexNormals, vertexTangents, vertexAttributes);
        return;
    }
    createTubePointsShader->dispatchCompute(numWorkGroups);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    bufferMemory = tubeVertexBuffer->mapBuffer(sgl::BUFFER_MAP_READ_ONLY);
    memcpy(tubeVertices.data(), bufferMemory, numCircleSubdivisions * pathLinePoints.size() * sizeof(TubeVertex));
    tubeVertexBuffer->unmapBuffer();

    vertexPositions.reserve(tubeVertices.size());
    vertexNormals.reserve(tubeVertices.size());
    vertexTangents.reserve(tubeVertices.size());
    vertexAttributes.reserve(tubeVertices.size());
    for (TubeVertex &tubeVertex : tubeVertices) {
        vertexPositions.push_back(tubeVertex.vertexPosition);
        vertexNormals.push_back(tubeVertex.vertexNormal);
        vertexTangents.push_back(tubeVertex.vertexTangent);
        vertexAttributes.push_back(tubeVertex.vertexAttribute);
    }
    auto endTube = std::chrono::system_clock::now();
    auto elapsedTube = std::chrono::duration_cast<std::chrono::milliseconds>(endTube - startTube);
    sgl::Logfile::get()->writeInfo(
            std::string() + "Computational time to create tube vertices: " + std::to_string(elapsedTube.count()));




    // PART 3: CreateTubeIndices.Compute
    auto startIndices = std::chrono::system_clock::now();
    size_t numLineSegments = numLinePointsOutput - numLinesOutput;
    size_t numIndices = numLineSegments*numCircleSubdivisions*6;
    triangleIndices.resize(numIndices);

    sgl::GeometryBufferPtr lineOffsetBufferOutput = sgl::Renderer->createGeometryBuffer(
            (numLinesOutput+1) * sizeof(uint32_t), lineOffsetsOutput.data(),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);
    sgl::GeometryBufferPtr tubeIndexBuffer = sgl::Renderer->createGeometryBuffer(
            numIndices * sizeof(uint32_t),
            sgl::SHADER_STORAGE_BUFFER, sgl::BUFFER_STATIC);

    sgl::ShaderProgramPtr createTubeIndicesShader = sgl::ShaderManager->getShaderProgram({"CreateTubeIndices.Compute"});
    sgl::ShaderManager->bindShaderStorageBuffer(2, lineOffsetBufferOutput);
    sgl::ShaderManager->bindShaderStorageBuffer(3, tubeIndexBuffer);
    createTubeIndicesShader->setUniform("numLines", static_cast<uint32_t>(numLinesOutput));
    numWorkGroups = sgl::iceil(numLinesOutput, WORK_GROUP_SIZE_1D); // last vector: local work group size
    createTubeIndicesShader->dispatchCompute(numWorkGroups);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    bufferMemory = tubeIndexBuffer->mapBuffer(sgl::BUFFER_MAP_READ_ONLY);
    memcpy(triangleIndices.data(), bufferMemory, numIndices * sizeof(uint32_t));
    tubeIndexBuffer->unmapBuffer();
    auto endIndices = std::chrono::system_clock::now();
    auto elapsedIndices = std::chrono::duration_cast<std::chrono::milliseconds>(endIndices - startIndices);
    sgl::Logfile::get()->writeInfo(std::string() + "Computational time to create tube indices: "
                              + std::to_string(elapsedIndices.count()));

    sgl::ShaderManager->removePreprocessorDefine("WORK_GROUP_SIZE_1D");
    sgl::ShaderManager->removePreprocessorDefine("NUM_CIRCLE_SEGMENTS");
    sgl::ShaderManager->removePreprocessorDefine("CIRCLE_RADIUS");
    sgl::ShaderManager->unbindShader();



    sgl::Logfile::get()->writeInfo(
            std::string() + "Summary: " + sgl::toString(vertexPositions.size()) + " vertices, "
            + sgl::toString(triangleIndices.size() / 3) + " faces.");

    // Compute size of renderable geometry.
    float byteSize =
            triangleIndices.size() * sizeof(uint32_t) +
            vertexPositions.size() * sizeof(glm::vec3) +
            vertexNormals.size() * sizeof(glm::vec3) +
            vertexTangents.size() * sizeof(glm::vec3) +
            vertexAttributes.size() * sizeof(float);

    float sizeMiB = byteSize / 1024.0 / 1024.0;

    sgl::Logfile::get()->writeInfo(
            std::string() +  "Byte Size Mesh Structure: " + std::to_string(sizeMiB) + " MiB");
    sgl::Logfile::get()->writeInfo(
            std::string() +  "Num Lines: " + std::to_string(numLinesOutput * 1e-3) + " tsd.") ;
    sgl::Logfile::get()->writeInfo(
            std::string() +  "Num Line Points: " + std::to_string(numLinePointsOutput * 1e-6) + " mio.");

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    sgl::Logfile::get()->writeInfo(
            std::string() + "Computational time to create tubes on GPU: " + std::to_string(elapsed.count()));
}

*/
