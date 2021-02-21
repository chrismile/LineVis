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

#ifndef LOADERS_HEXAHEDRALMESHLOADER_HPP
#define LOADERS_HEXAHEDRALMESHLOADER_HPP

#include <string>
#include <vector>
#include <functional>

#include <glm/vec3.hpp>

class HexahedralMeshLoader {
public:
    // Reads the mesh from the specified file
    /**
     * Reads the mesh from the specified file. The vertices and the cell indices are required.
     * All other attributes are optional.
     * @param filename
     * @param vertices
     * @param cellIndices
     * @param deformations
     * @param anisotropyMetricList
     * @return
     */
    virtual bool loadHexahedralMeshFromFile(
            const std::string& filename,
            std::vector<glm::vec3>& vertices, std::vector<uint32_t>& cellIndices,
            std::vector<glm::vec3>& deformations, std::vector<float>& anisotropyMetricList)=0;
};

/**
 * Reads a text file line by line.
 * @param filename The name of the text file.
 * @param readLineCallback A function that is called when a new (non-empty) line was read. It is passed the line string
 * and the tokenized words (seperated by whitespace characters) and is expected to return false if an error occurred
 * while reading the line and true otherwise.
 * @return Whether loading has succeeded.
 */
bool readFileLineByLine(
        const std::string& filename,
        std::function<bool(const std::string&, const std::vector<std::string>&)> readLineCallback);

#endif // LOADERS_HEXAHEDRALMESHLOADER_HPP