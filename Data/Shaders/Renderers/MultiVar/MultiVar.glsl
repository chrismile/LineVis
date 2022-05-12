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

#ifdef USE_MULTI_VAR_RENDERING

layout(binding = MULTI_VAR_UNIFORM_DATA_BUFFER_BINDING) uniform MultiVarUniformDataBuffer {
    uint numSelectedAttributes;
    uint totalNumAttributes;
    uvec2 multiVarPadding;
};

layout(std430, binding = MULTI_VAR_ATTRIBUTE_DATA_BUFFER_BINDING) readonly buffer AttributeDataArrayBuffer {
    float attributeDataArray[];
};

layout(std430, binding = MULTI_VAR_ATTRIBUTE_SELECTION_MAP_BUFFER_BINDING) readonly buffer AttributeSelectionMapBuffer {
    uint attributeSelectedIdxToRealIdxMap[];
};

uint getRealAttributeIndex(uint attributeIdx) {
    return attributeSelectedIdxToRealIdxMap[attributeIdx];
}

float sampleAttribute(uint vertexIdx, uint attributeIdxReal) {
    return attributeDataArray[totalNumAttributes * vertexIdx + attributeIdxReal];
}

float sampleAttributeLinear(float fragmentVertexIdxInterpolated, uint attributeIdxReal) {
    uint vertexIdx0 = uint(floor(fragmentVertexIdxInterpolated));
    uint vertexIdx1 = uint(ceil(fragmentVertexIdxInterpolated));
    float attr0 = sampleAttribute(vertexIdx0, attributeIdxReal);
    float attr1 = sampleAttribute(vertexIdx1, attributeIdxReal);
    return mix(attr0, attr1, fract(fragmentVertexIdxInterpolated));
}

#endif
