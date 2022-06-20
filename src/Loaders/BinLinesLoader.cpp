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

#include <fstream>

#include <Utils/Events/Stream/Stream.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/File/Logfile.hpp>

#include "BinLinesLoader.hpp"

/**
 * Version 1 of the .binlines file format. It stores only the pure trajectory data.
 */
void loadTrajectoriesFromBinLinesV1(
        const std::string& filename, sgl::BinaryReadStream& stream, BinLinesData& binLinesData) {
    Trajectories& trajectories = binLinesData.trajectories;

    // Rest of header after format version.
    uint32_t numTrajectories, numAttributes, trajectoryNumPoints;
    stream.read(numTrajectories);
    stream.read(numAttributes);
    trajectories.resize(numTrajectories);

    for (uint32_t trajectoryIndex = 0; trajectoryIndex < numTrajectories; trajectoryIndex++) {
        Trajectory& currentTrajectory = trajectories.at(trajectoryIndex);
        stream.read(trajectoryNumPoints);
        currentTrajectory.positions.resize(trajectoryNumPoints);
        stream.read(currentTrajectory.positions.data(), sizeof(glm::vec3) * trajectoryNumPoints);
        currentTrajectory.attributes.resize(numAttributes);
        for (uint32_t attributeIndex = 0; attributeIndex < numAttributes; attributeIndex++) {
            std::vector<float>& currentAttribute = currentTrajectory.attributes.at(attributeIndex);
            currentAttribute.resize(trajectoryNumPoints);
            stream.read(currentAttribute.data(), sizeof(float) * trajectoryNumPoints);
        }
    }
}

/**
 * Version 2 of the .binlines file format. It can additionally store ribbon directions, and the simulation grid outline
 * mesh.
 */
void loadTrajectoriesFromBinLinesV2(
        const std::string& filename, sgl::BinaryReadStream& stream, BinLinesData& binLinesData) {
    auto numTrajectories = uint32_t(binLinesData.trajectories.size());

    uint32_t verticesNormalized;
    stream.read(verticesNormalized);
    binLinesData.verticesNormalized = verticesNormalized != 0;

    // Attribute name data.
    uint32_t hasAttributeNames;
    stream.read(hasAttributeNames);
    if (hasAttributeNames != 0) {
        uint32_t numAttributes =
                binLinesData.trajectories.empty() ? 0 : uint32_t(binLinesData.trajectories.front().attributes.size());
        binLinesData.attributeNames.resize(numAttributes);
        for (uint32_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
            stream.read(binLinesData.attributeNames.at(attributeIdx));
        }
    }

    // Ribbon direction data.
    uint32_t hasRibbonData;
    stream.read(hasRibbonData);
    if (hasRibbonData != 0) {
        binLinesData.ribbonsDirections.resize(numTrajectories);
        for (uint32_t trajectoryIdx = 0; trajectoryIdx < numTrajectories; trajectoryIdx++) {
            size_t trajectoryNumPoints = binLinesData.trajectories.at(trajectoryIdx).positions.size();
            std::vector<glm::vec3>& ribbonDirections = binLinesData.ribbonsDirections.at(trajectoryIdx);
            ribbonDirections.resize(trajectoryNumPoints);
            stream.read(ribbonDirections.data(), sizeof(glm::vec3) * trajectoryNumPoints);
        }
    }

    // Simulation grid outline mesh data.
    uint32_t numMeshOutlineTriangleIndices, numMeshOutlineTriangleVertices, numMeshOutlineTriangleNormals;
    stream.read(numMeshOutlineTriangleIndices);
    stream.read(numMeshOutlineTriangleVertices);
    stream.read(numMeshOutlineTriangleNormals);
    if (numMeshOutlineTriangleIndices != 0) {
        binLinesData.simulationMeshOutlineTriangleIndices.resize(numMeshOutlineTriangleIndices);
        stream.read(
                binLinesData.simulationMeshOutlineTriangleIndices.data(),
                sizeof(uint32_t) * numMeshOutlineTriangleIndices);
    }
    if (numMeshOutlineTriangleVertices != 0) {
        binLinesData.simulationMeshOutlineVertexPositions.resize(numMeshOutlineTriangleVertices);
        stream.read(
                binLinesData.simulationMeshOutlineVertexPositions.data(),
                sizeof(glm::vec3) * numMeshOutlineTriangleVertices);
    }
    if (numMeshOutlineTriangleNormals != 0) {
        binLinesData.simulationMeshOutlineVertexNormals.resize(numMeshOutlineTriangleNormals);
        stream.read(
                binLinesData.simulationMeshOutlineVertexNormals.data(),
                sizeof(glm::vec3) * numMeshOutlineTriangleNormals);
    }
}

BinLinesData loadTrajectoriesFromBinLines(const std::string& filename) {
    Trajectories trajectories;

    uint8_t* buffer = nullptr; //< BinaryReadStream does deallocation.
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(filename, buffer, length, true);
    if (!loaded) {
        return {};
    }

    // Read format version
    sgl::BinaryReadStream stream(buffer, length);
    uint32_t versionNumber;
    stream.read(versionNumber);
    if (versionNumber != 1u && versionNumber != 2u) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in loadTrajectoriesFromBinLines: Invalid magic number in file \"" + filename + "\".");
        return {};
    }

    BinLinesData binLinesData;
    loadTrajectoriesFromBinLinesV1(filename, stream, binLinesData);
    if (versionNumber == 2u) {
        loadTrajectoriesFromBinLinesV2(filename, stream, binLinesData);
    }
    return binLinesData;
}

void saveTrajectoriesAsBinLines(const std::string& filename, const BinLinesData& binLinesData) {
#ifndef __MINGW32__
    std::ofstream file(filename.c_str(), std::ofstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in saveTrajectoriesAsBinLines: File \""
                + filename + "\" could not be opened for writing.");
        return;
    }
#else
    FILE* fileptr = fopen(filename.c_str(), "wb");
    if (fileptr == NULL) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in saveTrajectoriesAsBinLines: File \""
                + filename + "\" could not be opened for writing.");
        return;
    }
#endif

    sgl::BinaryWriteStream stream;
    stream.write(2u); //< Version number.

    auto numTrajectories = uint32_t(binLinesData.trajectories.size());
    uint32_t numAttributes =
            binLinesData.trajectories.empty() ? 0 : uint32_t(binLinesData.trajectories.front().attributes.size());
    stream.write(numTrajectories);
    stream.write(numAttributes);

    // Write 'Trajectories' base data.
    for (uint32_t trajectoryIndex = 0; trajectoryIndex < numTrajectories; trajectoryIndex++) {
        const Trajectory& currentTrajectory = binLinesData.trajectories.at(trajectoryIndex);
        auto trajectoryNumPoints = uint32_t(currentTrajectory.positions.size());
        stream.write(trajectoryNumPoints);
        stream.write(currentTrajectory.positions.data(), sizeof(glm::vec3) * trajectoryNumPoints);
        for (uint32_t attributeIndex = 0; attributeIndex < numAttributes; attributeIndex++) {
            const std::vector<float>& currentAttribute = currentTrajectory.attributes.at(attributeIndex);
            stream.write(currentAttribute.data(), sizeof(float) * trajectoryNumPoints);
        }
    }

    uint32_t verticesNormalized = binLinesData.verticesNormalized ? 1 : 0;
    stream.write(verticesNormalized);

    // Attribute name data.
    uint32_t hasAttributeNames = binLinesData.attributeNames.empty() ? 0 : 1;
    stream.write(hasAttributeNames);
    if (hasAttributeNames != 0) {
        for (uint32_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
            stream.write(binLinesData.attributeNames.at(attributeIdx));
        }
    }

    // Ribbon direction data.
    uint32_t hasRibbonData = binLinesData.ribbonsDirections.empty() ? 0 : 1;
    stream.write(hasRibbonData);
    if (hasRibbonData != 0) {
        for (uint32_t trajectoryIdx = 0; trajectoryIdx < numTrajectories; trajectoryIdx++) {
            size_t trajectoryNumPoints = binLinesData.trajectories.at(trajectoryIdx).positions.size();
            const std::vector<glm::vec3>& ribbonDirections = binLinesData.ribbonsDirections.at(trajectoryIdx);
            stream.write(ribbonDirections.data(), sizeof(glm::vec3) * trajectoryNumPoints);
        }
    }

    // Simulation grid outline mesh data.
    auto numMeshOutlineTriangleIndices = uint32_t(binLinesData.simulationMeshOutlineTriangleIndices.size());
    auto numMeshOutlineTriangleVertices = uint32_t(binLinesData.simulationMeshOutlineVertexPositions.size());
    auto numMeshOutlineTriangleNormals = uint32_t(binLinesData.simulationMeshOutlineVertexNormals.size());
    stream.write(numMeshOutlineTriangleIndices);
    stream.write(numMeshOutlineTriangleVertices);
    stream.write(numMeshOutlineTriangleNormals);
    if (numMeshOutlineTriangleIndices != 0) {
        stream.write(
                binLinesData.simulationMeshOutlineTriangleIndices.data(),
                sizeof(uint32_t) * numMeshOutlineTriangleIndices);
    }
    if (numMeshOutlineTriangleVertices != 0) {
        stream.write(
                binLinesData.simulationMeshOutlineVertexPositions.data(),
                sizeof(glm::vec3) * numMeshOutlineTriangleVertices);
    }
    if (numMeshOutlineTriangleNormals != 0) {
        stream.write(
                binLinesData.simulationMeshOutlineVertexNormals.data(),
                sizeof(glm::vec3) * numMeshOutlineTriangleNormals);
    }

#ifndef __MINGW32__
    file.write((const char*)stream.getBuffer(), stream.getSize());
    file.close();
#else
    fwrite((const void*)stream.getBuffer(), stream.getSize(), 1, fileptr);
    fclose(fileptr);
#endif
}
