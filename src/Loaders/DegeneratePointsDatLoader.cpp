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

#include <cstdio>
#include <iostream>

#include <Utils/File/Logfile.hpp>
#include <Utils/File/LineReader.hpp>

#include "DegeneratePointsDatLoader.hpp"

void loadDegeneratePointsFromDat(
        const std::string& filename,
        std::vector<glm::vec3>& degeneratePoints) {
    LineReader lineReader(filename);

    uint32_t numDegeneratePoints = lineReader.readScalarLine<uint32_t>();
    degeneratePoints.reserve(numDegeneratePoints);
    for (uint32_t pointIdx = 0; pointIdx < numDegeneratePoints; pointIdx++) {
        std::vector<float> positionData = lineReader.readVectorLine<float>(3);
        degeneratePoints.push_back(glm::vec3(
                positionData.at(0),
                positionData.at(1),
                positionData.at(2)));
    }

    sgl::Logfile::get()->writeInfo(
            std::string() + "Number of degenerate points: " + std::to_string(degeneratePoints.size()));
}
