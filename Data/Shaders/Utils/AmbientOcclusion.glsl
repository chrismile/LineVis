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

layout (std430, binding = 13) readonly buffer AmbientOcclusionFactors {
    float ambientOcclusionFactors[];
};
layout (std430, binding = 14) readonly buffer AmbientOcclusionBlendingWeights {
    float ambientOcclusionBlendingWeights[];
};

#ifndef VULKAN
uniform float ambientOcclusionStrength;
uniform uint numAoTubeSubdivisions;
uniform uint numLineVertices;
uniform uint numParametrizationVertices;
#endif

#define M_PI 3.14159265358979323846

float getAoFactor(float interpolatedVertexId, float phi) {
    uint lastLinePointIdx = uint(interpolatedVertexId);
    uint nextLinePointIdx = min(lastLinePointIdx + 1, numLineVertices - 1);
    float interpolationFactor = fract(interpolatedVertexId);

    float blendingWeightLast = ambientOcclusionBlendingWeights[lastLinePointIdx];
    float blendingWeightNext = ambientOcclusionBlendingWeights[nextLinePointIdx];
    float blendingWeight = mix(blendingWeightLast, blendingWeightNext, interpolationFactor);
    uint lastVertexIdx = uint(blendingWeight);
    uint nextVertexIdx = min(lastVertexIdx + 1, numParametrizationVertices - 1);
    float interpolationFactorLine = fract(blendingWeight);

    float circleIdxFlt = clamp(phi / (2.0 * M_PI) * float(numAoTubeSubdivisions), 0.0, float(numAoTubeSubdivisions));
    uint circleIdxLast = (uint(floor(circleIdxFlt)) + numAoTubeSubdivisions) % numAoTubeSubdivisions;
    uint circleIdxNext = (circleIdxLast + 1) % numAoTubeSubdivisions;
    float interpolationFactorCircle = fract(circleIdxFlt);

    float aoFactor00 = ambientOcclusionFactors[circleIdxLast + numAoTubeSubdivisions * lastVertexIdx];
    float aoFactor01 = ambientOcclusionFactors[circleIdxLast + numAoTubeSubdivisions * nextVertexIdx];
    float aoFactor10 = ambientOcclusionFactors[circleIdxNext + numAoTubeSubdivisions * lastVertexIdx];
    float aoFactor11 = ambientOcclusionFactors[circleIdxNext + numAoTubeSubdivisions * nextVertexIdx];
    float aoFactor0 = mix(aoFactor00, aoFactor01, interpolationFactorLine);
    float aoFactor1 = mix(aoFactor10, aoFactor11, interpolationFactorLine);
    float aoFactor = mix(aoFactor0, aoFactor1, interpolationFactorCircle);
    return 1.0 - ambientOcclusionStrength + ambientOcclusionStrength * aoFactor;
}
