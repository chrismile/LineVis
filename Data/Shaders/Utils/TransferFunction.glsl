uniform float minCriterionValue = 0.0;
uniform float maxCriterionValue = 1.0;

// Transfer function color lookup table.
uniform sampler1D transferFunctionTexture;

vec4 transferFunction(float attr) {
    // Transfer to range [0,1].
    float posFloat = clamp((attr - minCriterionValue) / (maxCriterionValue - minCriterionValue), 0.0, 1.0);
    // Look up the color value.
    return texture(transferFunctionTexture, posFloat);
}
