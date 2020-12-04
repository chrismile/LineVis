uniform float minCriterionValue = 0.0;
uniform float maxCriterionValue = 1.0;

// Transfer function color lookup table.
#if defined(USE_MULTI_VAR_TRANSFER_FUNCTION) || defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX)

uniform sampler1DArray transferFunctionTexture;

vec4 transferFunction(float attr, uint variableIndex) {
    // Transfer to range [0,1].
    float posFloat = clamp((attr - minCriterionValue) / (maxCriterionValue - minCriterionValue), 0.0, 1.0);
    // Look up the color value.
    return texture(transferFunctionTexture, vec2(posFloat, variableIndex));
}

#else

uniform sampler1D transferFunctionTexture;

vec4 transferFunction(float attr) {
    // Transfer to range [0,1].
    float posFloat = clamp((attr - minCriterionValue) / (maxCriterionValue - minCriterionValue), 0.0, 1.0);
    // Look up the color value.
    return texture(transferFunctionTexture, posFloat);
}

#endif

//#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
//vec4 transferFunction(float attr, uint fragmentPrincipalStressIndex) {
//    // Transfer to range [0,1].
//    float posFloat = clamp((attr - minCriterionValue) / (maxCriterionValue - minCriterionValue), 0.0, 1.0);
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
