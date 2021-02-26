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

#ifndef LINEVIS_MESHSMOOTHING_HPP
#define LINEVIS_MESHSMOOTHING_HPP

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <glm/vec3.hpp>

/**
 * For more information on Laplacian smoothing see:
 * https://graphics.stanford.edu/courses/cs468-12-spring/LectureSlides/06_smoothing.pdf
 */

void createNeighborMap(
        const std::vector<uint32_t>& triangleIndices,
        std::unordered_map<uint32_t, std::unordered_set<uint32_t>>& neighborsMap);

void laplacianSmoothing(
        const std::vector<glm::vec3>& pointsIn, std::vector<glm::vec3>& pointsOut,
        const std::unordered_map<uint32_t, std::unordered_set<uint32_t>>& neighborsMap, float lambda = 0.8f);

void laplacianSmoothing(
        const std::vector<uint32_t>& triangleIndices, std::vector<glm::vec3>& vertexPositions,
        int numIterations = 4, float lambda = 0.8f);

#endif //LINEVIS_MESHSMOOTHING_HPP
