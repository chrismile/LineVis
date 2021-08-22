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
