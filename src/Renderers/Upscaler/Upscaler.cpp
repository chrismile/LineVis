/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2024-2026, Christoph Neuhauser
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

#include <cmath>
#include <Math/Geometry/MatrixUtil.hpp>

#include "Upscaler.hpp"

#ifdef SUPPORT_DLSS
#include "DLSS.hpp"
#endif
#ifdef SUPPORT_XESS
#include "XeSS.hpp"
#endif

Upscaler* createNewUpscaler(UpscalerType upscalerType) {
#ifdef SUPPORT_DLSS
    if (upscalerType == UpscalerType::DLSS) {
        return new DlssSupersampler;
    }
#endif
#ifdef SUPPORT_XESS
    if (upscalerType == UpscalerType::XESS) {
        return new XessSupersampler;
    }
#endif
    return nullptr;
}

/// Creates Halton sequence with specified base. The pointer is accessed with stride 2.
void evaluateHaltonSequence(float* sequencePointer, int numSamples, int base) {
    int n = 0, d = 1;
    for (int i = 0; i < numSamples; i++) {
        int x = d - n;
        if (x == 1) {
            n = 1;
            d *= base;
        } else {
            int y = d / base;
            while (x <= y) {
                y /= base;
            }
            n = (base + 1) * y - x;
        }
        // Samples lie in range [-0.5, 0.5]^2. Use stride of 2 for 2D points.
        sequencePointer[i * 2] = float(n) / float(d) - 0.5f;
    }
}

void computeJitteredSamples(
        std::vector<glm::vec2>& jitteredSamples,
        uint32_t renderWidth, uint32_t renderHeight,
        uint32_t displayWidth, uint32_t displayHeight) {
    const float scaleFraction = float(displayWidth * displayHeight) / float(renderWidth * renderHeight);
    const int numSamples = std::max(8 * int(std::ceil(scaleFraction * scaleFraction)), 1);
    jitteredSamples.resize(numSamples);
    auto* samplesPtr = &jitteredSamples.front().x;
    constexpr int baseX = 2, baseY = 3;
    evaluateHaltonSequence(samplesPtr, numSamples, baseX);
    evaluateHaltonSequence(samplesPtr + 1, numSamples, baseY);
}

void adaptProjectionMatrixJitterSample(
        glm::mat4& projectionMatrix,
        const glm::vec2& jitterSample,
        uint32_t renderWidth, uint32_t renderHeight) {
    glm::vec2 translation = glm::vec2(
            2.0f * jitterSample.x / float(renderWidth), 2.0f * jitterSample.y / float(renderHeight));
    // Sign is same as value of projectionMatrix[2][3].
    // DLSS integration guide uses "+" due to different conventions.
    projectionMatrix[2][0] -= translation.x;
    projectionMatrix[2][1] += translation.y;

}

glm::mat4 computeJitterSampleMatrix(
        const glm::vec2& jitterSample,
        uint32_t renderWidth, uint32_t renderHeight) {
    // y-axis may be negative if y points down.
    return sgl::matrixTranslation(glm::vec2(
            2.0f * jitterSample.x / float(renderWidth), -2.0f * jitterSample.y / float(renderHeight)));
}
