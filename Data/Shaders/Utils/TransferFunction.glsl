/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2021, Christoph Neuhauser
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

#ifndef TRANSFER_FUNCTION_GLSL
#define TRANSFER_FUNCTION_GLSL

// Transfer function color lookup table.
#if defined(USE_MULTI_VAR_TRANSFER_FUNCTION) || defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX)

#ifdef VULKAN
layout (std430, binding = 15) readonly buffer MinMaxBuffer {
    vec2 minMaxValues[];
};
layout(binding = 16) uniform sampler1DArray transferFunctionTexture;
#else
layout (std430, binding = 9) readonly buffer MinMaxBuffer {
    vec2 minMaxValues[];
};
uniform sampler1DArray transferFunctionTexture;
#endif

#ifdef IS_MULTIVAR_DATA
uniform int useColorIntensity = 1;
#endif

vec4 transferFunction(float attr, uint variableIndex) {
    vec2 minMaxValue = minMaxValues[variableIndex];
    float minAttributeValue = minMaxValue.x;
    float maxAttributeValue = minMaxValue.y;

    // Transfer to range [0,1].
    float posFloat = clamp((attr - minAttributeValue) / (maxAttributeValue - minAttributeValue), 0.0, 1.0);

#ifdef IS_MULTIVAR_DATA
    if (useColorIntensity == 0) {
        posFloat = 1.0;
    }
#endif

    // Look up the color value.
    return texture(transferFunctionTexture, vec2(posFloat, variableIndex));
}

#else

#ifdef VULKAN
layout (binding = 15) uniform MinMaxUniformBuffer {
    float minAttributeValue;
    float maxAttributeValue;
};
layout(binding = 16) uniform sampler1D transferFunctionTexture;
#else
uniform float minAttributeValue = 0.0;
uniform float maxAttributeValue = 1.0;
uniform sampler1D transferFunctionTexture;
#endif

vec4 transferFunction(float attr) {
    // Transfer to range [0,1].
    float posFloat = clamp((attr - minAttributeValue) / (maxAttributeValue - minAttributeValue), 0.0, 1.0);
    // Look up the color value.
    return texture(transferFunctionTexture, posFloat);
}

#endif

#endif
