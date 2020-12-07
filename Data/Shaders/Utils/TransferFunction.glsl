
// Transfer function color lookup table.
#if defined(USE_MULTI_VAR_TRANSFER_FUNCTION) || defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX)

uniform sampler1DArray transferFunctionTexture;
layout (std430, binding = 9) readonly buffer MinMaxBuffer {
    vec2 minMaxValues[];
};

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

uniform float minAttributeValue = 0.0;
uniform float maxAttributeValue = 1.0;
uniform sampler1D transferFunctionTexture;

vec4 transferFunction(float attr) {
    // Transfer to range [0,1].
    float posFloat = clamp((attr - minAttributeValue) / (maxAttributeValue - minAttributeValue), 0.0, 1.0);
    // Look up the color value.
    return texture(transferFunctionTexture, posFloat);
}

#endif

//#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
//vec4 transferFunction(float attr, uint fragmentPrincipalStressIndex) {
//    // Transfer to range [0,1].
//    float posFloat = clamp((attr - minAttributeValue) / (maxAttributeValue - minAttributeValue), 0.0, 1.0);
//    vec4 color;
//    float alpha = texture(transferFunctionTexture, posFloat).a;
//    if (fragmentPrincipalStressIndex == 0) {
//        color = vec4(1.0, 0.0, 0.0, alpha);
//    } else if (fragmentPrincipalStressIndex == 1) {
//        color = vec4(0.0, 1.0, 0.0, alpha);
//    } else {
//        color = vec4(0.0, 0.0, 1.0, alpha);
//    }
//#ifdef DIRECT_BLIT_GATHER
//    color.rgb = mix(color.rgb, vec3(1.0), pow((1.0 - posFloat), 10.0) * 0.5);
//#endif
//    return color;
//}
//#endif
