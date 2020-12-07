//uniform float minColorIntensity;
uniform int useColorIntensity = 1;

#include "TransferFunction.glsl"

///////////////////////////////////////////
// RGB <-> HSV color space mapping

/*layout (std430, binding = 8) buffer VarColorArray {
    vec4 colorVars[];
};*/

vec3 rgbToHSV(in vec3 color) {
    float minValue = min(color.r, min(color.g, color.b));
    float maxValue = max(color.r, max(color.g, color.b));

    float C = maxValue - minValue;

    // 1) Compute hue H
    float H = 0;
    if (maxValue == color.r) {
        H = mod((color.g - color.b) / C, 6.0);
    } else if (maxValue == color.g) {
        H = (color.b - color.r) / C + 2;
    } else if (maxValue == color.b) {
        H = (color.r - color.g) / C + 4;
    } else {
        H = 0;
    }

    H *= 60; // hue is in degree

    // 2) Compute the value V
    float V = maxValue;

    // 3) Compute saturation S
    float S = 0;
    if (V == 0) {
        S = 0;
    } else {
        S = C / V;
        //        S = C / (1 - abs(maxValue + minValue - 1));
    }

    return vec3(H, S, V);
}

// https://en.wikipedia.org/wiki/HSL_and_HSV
// https://de.wikipedia.org/wiki/HSV-Farbraum

vec3 hsvToRGB(in vec3 color) {
    const float H = color.r;
    const float S = color.g;
    const float V = color.b;

    float h = H / 60.0;

    int hi = int(floor(h));
    float f = (h - float(hi));

    float p = V * (1.0 - S);
    float q = V * (1.0 - S * f);
    float t = V * (1.0 - S * (1.0 - f));

    if (hi == 1) {
        return vec3(q, V, p);
    } else if (hi == 2) {
        return vec3(p, V, t);
    }
    else if (hi == 3) {
        return vec3(p, q, V);
    } else if (hi == 4) {
        return vec3(t, p, V);
    } else if (hi == 5) {
        return vec3(V, p, q);
    } else {
        return vec3(V, t, p);
    }
}

///////////////////////////////////////////
// RGB <-> sRGB

vec3 linearRGBTosRGB(in vec3 color_sRGB) {
    //float factor = 1.0f / 2.2f;
    //return glm::pow(color_sRGB, glm::vec3(factor));
    // See https://en.wikipedia.org/wiki/SRGB
    return mix(1.055f * pow(color_sRGB, vec3(1.0f / 2.4f)) - 0.055f, color_sRGB * 12.92f,
    lessThanEqual(color_sRGB, vec3(0.0031308f)));
}

vec3 sRGBToLinearRGB(in vec3 color_LinearRGB) {
    //float factor = 2.2f;
    //return glm::pow(color_LinearRGB, glm::vec3(factor));
    // See https://en.wikipedia.org/wiki/SRGB
    return mix(pow((color_LinearRGB + 0.055f) / 1.055f, vec3(2.4f)),
    color_LinearRGB / 12.92f, lessThanEqual(color_LinearRGB, vec3(0.04045f)));
}

///////////////////////////////////////////
// Variable color mapping


vec4 mapColor(in float value, uint index) {
    if (index == 0) {
        return mix(vec4(vec3(253,219,199)/ 255.0, 1), vec4(vec3(178,24,43)/ 255.0, 1), value);
    } else if (index == 1) {
        return mix(vec4(vec3(209,229,240)/ 255.0, 1), vec4(vec3(33,102,172)/ 255.0, 1), value);
    } else if (index == 2) {
        return mix(vec4(vec3(217,240,211)/ 255.0, 1), vec4(vec3(27,120,55)/ 255.0, 1), value);
    } else if (index == 3) {
        return mix(vec4(vec3(216,218,235) / 255.0, 1), vec4(vec3(84,39,136) / 255.0, 1), value);
    } else if (index == 5) {
        return mix(vec4(vec3(254,224,182)/ 255.0, 1), vec4(vec3(179,88,6)/ 255.0, 1), value);
    } else if (index == 6) {
        return mix(vec4(vec3(199,234,229)/ 255.0, 1), vec4(vec3(1,102,94)/ 255.0, 1), value);
    } else if (index == 4) {
        return mix(vec4(vec3(253,224,239)/ 255.0, 1), vec4(vec3(197,27,125)/ 255.0, 1), value);
    }

    return vec4(0);
}

/*vec4 determineVariableColor(in int varID) {
    vec4 surfaceColor = vec4(0.2, 0.2, 0.2, 1);

    if (varID >= maxNumVariables || varID < 0) {
        surfaceColor = vec4(0.4, 0.4, 0.4, 1);
    }
    else {
        surfaceColor = colorVars[varID];
    }

//    if (varID == 0) { surfaceColor = vec4(vec3(228,26,28)/ 255.0, 1); } // red
//    else if (varID == 1) { surfaceColor = vec4(vec3(55,126,184)/ 255.0, 1); } // blue
//    else if (varID == 2) { surfaceColor = vec4(vec3(5,139,69)/ 255.0, 1); } // green
//    else if (varID == 3) { surfaceColor = vec4(vec3(129,15,124)/ 255.0, 1); } // lila / purple
//    else if (varID == 4) { surfaceColor = vec4(vec3(217,72,1)/ 255.0, 1); } // orange
//    else if (varID == 5) { surfaceColor = vec4(vec3(231,41,138)/ 255.0, 1); } // pink
//    else { surfaceColor = vec4(0.4, 0.4, 0.4, 1); }

    surfaceColor.rgb = sRGBToLinearRGB(surfaceColor.rgb);

    return surfaceColor;
}*/


vec4 determineColor(in int varID, in float variableValue) {
    // Determine variable color
    /*vec4 surfaceColor = determineVariableColor(varID);

    if (varID >= 0)
    {
        vec3 hsvCol = rgbToHSV(surfaceColor.rgb);
        float rate = variableValue;
        hsvCol.g = hsvCol.g * (minColorIntensity + (1.0 - minColorIntensity) * rate);
        surfaceColor.rgb = hsvCol.rgb;
        surfaceColor.rgb = hsvToRGB(surfaceColor.rgb);
    }*/
    if (varID >= maxNumVariables || varID < 0) {
        return vec4(0.4, 0.4, 0.4, 1);
    }

    //vec4 surfaceColor = transferFunction(variableValue * (1.0 - minColorIntensity) + minColorIntensity, varID);
    float value = 1.0;
    if (useColorIntensity == 1) {
        value = variableValue;
    }
    vec4 surfaceColor = transferFunction(value, varID);
    return surfaceColor;
}


vec4 determineColorLinearInterpolate(
        in int varID, in float variableValue, in float variableNextValue, in float interpolant) {
    // Determine variable color
    /*vec4 surfaceColor = determineVariableColor(varID);

#ifndef DIRECT_COLOR_MAPPING
    vec3 hsvCol = rgbToHSV(surfaceColor.rgb);
    float curMapping = variableValue;
    float nextMapping = variableNextValue;
    float rate = mix(curMapping, nextMapping, interpolant);
    //
    float ambientFactor = 0.1;
    hsvCol.g = hsvCol.g * (minColorIntensity + (1.0 - minColorIntensity) * rate);
    surfaceColor.rgb = hsvCol.rgb;
    surfaceColor.rgb = hsvToRGB(surfaceColor.rgb);

    //surfaceColor = mapColor(variableValue, varID);
    //vec4 surfaceColor2 = mapColor(variableNextValue, varID);
    //surfaceColor = mix(surfaceColor, surfaceColor2, fragElementInterpolant);
#endif
    */

    if (varID >= maxNumVariables || varID < 0) {
        return vec4(0.4, 0.4, 0.4, 1);
    }

    float rate = mix(variableValue, variableNextValue, interpolant);
    float value = rate;
    vec4 surfaceColor = transferFunction(value, varID);
    return surfaceColor;
}

void drawSeparatorBetweenStripes(
        inout vec4 surfaceColor, in float varFraction, in float separatorWidth) {
    float borderWidth = separatorWidth;
    float alphaBorder = 0.5;
    if (varFraction <= borderWidth || varFraction >= (1.0 - borderWidth))
    {
        if (varFraction > 0.5) { varFraction = 1.0 - varFraction; }

        surfaceColor.rgb = surfaceColor.rgb * (alphaBorder + (1 - alphaBorder) * varFraction / borderWidth);
    }
}


///////////////////////////////////////////
// Phong lighting model

uniform float materialAmbient;
uniform float materialDiffuse;
uniform float materialSpecular;
uniform float materialSpecularExp;
uniform bool drawHalo;
uniform float haloFactor;

vec4 computePhongLighting(
        in vec4 surfaceColor, in float occlusionFactor, in float shadowFactor,
        in vec3 worldPos, in vec3 normal, in vec3 tangent) {
    const vec3 lightColor = vec3(1,1,1);
    const vec3 ambientColor = surfaceColor.rgb;
    const vec3 diffuseColor = surfaceColor.rgb;

    const float kA = materialAmbient * occlusionFactor * shadowFactor;
    const vec3 Ia = kA * ambientColor;
    const float kD = materialDiffuse;
    const float kS = materialSpecular;
    const float s = materialSpecularExp;

    const vec3 n = normalize(normal);
    const vec3 v = normalize(cameraPosition - worldPos);
    const vec3 l = normalize(v);
    const vec3 h = normalize(v + l);
    vec3 t = normalize(tangent);
    //    vec3 t = normalize(cross(vec3(0, 0, 1), n));


    vec3 Id = kD * clamp((dot(n, l)), 0.0, 1.0) * diffuseColor;
    vec3 Is = kS * pow(clamp((dot(n, h)), 0.0, 1.0), s) * lightColor;
    vec3 colorShading = Ia + Id + Is;

    if (drawHalo) {
        //    float haloParameter = 0.5;
        //    float angle1 = abs( dot( v, n)) * 0.8;
        //    float angle2 = abs( dot( v, normalize(t))) * 0.2;
        //    float halo = min(1.0,mix(1.0f,angle1 + angle2,haloParameter));//((angle1)+(angle2)), haloParameter);

        vec3 hV = normalize(cross(t, v));
        vec3 vNew = normalize(cross(hV, t));

        float angle = pow(abs((dot(vNew, n))), haloFactor); // 1.8 + 1.5
        float angleN = pow(abs((dot(v, n))), haloFactor);
        //    float EPSILON = 0.8f;
        //    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, angle);

        float haloNew = min(1.0, mix(1.0f, angle + angleN, 0.9)) * 0.9 + 0.1;
        colorShading *= (haloNew) * (haloNew);
    }

    ////////////

    return vec4(colorShading.rgb, surfaceColor.a);
}