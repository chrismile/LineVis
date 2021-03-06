-- Vertex

#version 430 core

#include "MultiVarGlobalVertexInput.glsl"
#include "MultiVarGlobalVariables.glsl"

out VertexData {
    vec3 vPosition;// Position in world space
    vec3 vTangent;// Tangent of line in world space
//    int variableID; // variable index
    int vLineID;// number of line
    int vElementID;// number of line element (original line vertex index)
    int vElementNextID; // number of next line element (original next line vertex index)
    float vElementInterpolant; // curve parameter t along curve between element and next element
};
//} GeomOut; // does not work

void main() {
    //    uint varID = gl_InstanceID % 6;
    const int lineID = int(variableDesc.y);
    const int elementID = int(variableDesc.x);

    vPosition = vertexPosition;
    vTangent = normalize(vertexLineTangent);

    vLineID = lineID;
    vElementID = elementID;

    vElementNextID = int(variableDesc.z);
    vElementInterpolant = variableDesc.w;
}

-- Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

#include "MultiVarGlobalVariables.glsl"

uniform vec3 cameraPosition;

// Input from vertex buffer
in VertexData {
    vec3 vPosition;// Position in world space
    vec3 vTangent;// Tangent of line in world space
    int vLineID;// number of line
    int vElementID;// number of line element (original line vertex index)
    int vElementNextID; // number of next line element (original next line vertex index)
    float vElementInterpolant; // curve parameter t along curve between element and next element
} vertexOutput[];

// Output to fragments
out vec3 fragWorldPos;
out float fragNormalFloat; // Between -1 and 1
out vec3 fragNormal0;
out vec3 fragNormal1;
out vec3 fragTangent;
// "Oriented Stripes"-specfic outputs
flat out int fragElementID; // Actual per-line vertex index --> required for sampling from global buffer
flat out int fragElementNextID; // Actual next per-line vertex index --> for linear interpolation
flat out int fragLineID; // Line index --> required for sampling from global buffer
out float fragElementInterpolant; // current number of curve parameter t (in [0;1]) within one line segment

void main() {
    vec3 linePosition0 = vertexOutput[0].vPosition;
    vec3 linePosition1 = vertexOutput[1].vPosition;
    vec3 tangent0 = normalize(vertexOutput[0].vTangent);
    vec3 tangent1 = normalize(vertexOutput[1].vTangent);

    vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
    vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
    vec3 offsetDirection0 = normalize(cross(tangent0, viewDirection0));
    vec3 offsetDirection1 = normalize(cross(tangent1, viewDirection1));
    vec3 vertexPosition;

#if ORIENTED_RIBBON_MODE != 3 // ORIENTED_RIBBON_MODE_VARYING_RIBBON_WIDTH
    const float lineRadius = lineWidth * 0.5;
#endif
    const mat4 pvMatrix = pMatrix * vMatrix;

    // Emit the vertices and attributes to the fragment shader
    fragElementID = vertexOutput[0].vElementID;
    fragLineID = vertexOutput[0].vLineID;

    // Vertex 0.
    fragNormal0 = normalize(cross(offsetDirection0, tangent0));
    fragNormal1 = offsetDirection0;
    fragTangent = tangent0;
    fragElementNextID = vertexOutput[0].vElementNextID;
    fragElementInterpolant = vertexOutput[0].vElementInterpolant;

#if ORIENTED_RIBBON_MODE == 3 // ORIENTED_RIBBON_MODE_VARYING_RIBBON_WIDTH
    // Sample ALL variables.
    float variables[MAX_NUM_VARIABLES];
    for (int varID = 0; varID < numVariables; varID++) {
        // 1) Sample variables from buffers.
        float variableValue, variableNextValue;
        vec2 variableMinMax, variableNextMinMax;
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementID, variableValue, variableMinMax);
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementNextID, variableNextValue, variableNextMinMax);

        // 2) Normalize values.
        variableValue = clamp((variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x), 0.0, 1.0);
        variableNextValue = clamp((variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x), 0.0, 1.0);

        // 3) Interpolate linearly.
        variables[varID] = mix(variableValue, variableNextValue, fragElementInterpolant);
    }

    // Compute the sum of all variables
    float variablesSumTotal = 0.0;
    for (int varID = 0; varID < numVariables; varID++) {
        variablesSumTotal += variables[varID];
    }

    float lineRadius = lineWidth * 0.5 * (variablesSumTotal / float(max(numVariables, 1)));
#endif

    vertexPosition = linePosition0 - lineRadius * offsetDirection0;
    fragWorldPos = vertexPosition;
    fragNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + lineRadius * offsetDirection0;
    fragWorldPos = vertexPosition;
    fragNormalFloat = 1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();


    // Vertex 1.
    fragNormal0 = normalize(cross(offsetDirection1, tangent1));
    fragNormal1 = offsetDirection1;
    fragTangent = tangent1;
    if (vertexOutput[1].vElementInterpolant < vertexOutput[0].vElementInterpolant) {
        fragElementInterpolant = 1.0f;
        fragElementNextID = int(vertexOutput[0].vElementNextID);
    } else {
        fragElementInterpolant = vertexOutput[1].vElementInterpolant;
        fragElementNextID = int(vertexOutput[1].vElementNextID);
    }

#if ORIENTED_RIBBON_MODE == 3 // ORIENTED_RIBBON_MODE_VARYING_RIBBON_WIDTH
    // Sample ALL variables.
    for (int varID = 0; varID < numVariables; varID++) {
        // 1) Sample variables from buffers.
        float variableValue, variableNextValue;
        vec2 variableMinMax, variableNextMinMax;
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementID, variableValue, variableMinMax);
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementNextID, variableNextValue, variableNextMinMax);

        // 2) Normalize values.
        variableValue = clamp((variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x), 0.0, 1.0);
        variableNextValue = clamp((variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x), 0.0, 1.0);

        // 3) Interpolate linearly.
        variables[varID] = mix(variableValue, variableNextValue, fragElementInterpolant);
    }

    // Compute the sum of all variables
    variablesSumTotal = 0.0;
    for (int varID = 0; varID < numVariables; varID++) {
        variablesSumTotal += variables[varID];
    }

    lineRadius = lineWidth * 0.5 * (variablesSumTotal / float(max(numVariables, 1)));
#endif

    vertexPosition = linePosition1 - lineRadius * offsetDirection1;
    fragWorldPos = vertexPosition;
    fragNormalFloat = -1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + lineRadius * offsetDirection1;
    fragWorldPos = vertexPosition;
    fragNormalFloat = 1.0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}


-- Fragment

#version 430 core

// Output to fragments
in vec3 fragWorldPos;
in float fragNormalFloat; // Between -1 and 1
in vec3 fragNormal0;
in vec3 fragNormal1;
in vec3 fragTangent;
// "Oriented Stripes"-specfic outputs
flat in int fragElementID; // Actual per-line vertex index --> required for sampling from global buffer
flat in int fragElementNextID; // Actual next per-line vertex index --> for linear interpolation
flat in int fragLineID; // Line index --> required for sampling from global buffer
in float fragElementInterpolant; // current number of curve parameter t (in [0;1]) within one line segment

uniform vec3 cameraPosition; // world space

#if !defined(DIRECT_BLIT_GATHER)
#define fragmentPositionWorld fragWorldPos
#include OIT_GATHER_HEADER
#endif

#ifdef DIRECT_BLIT_GATHER
out vec4 fragColor;
#endif

// "Color bands"-specific uniforms
uniform float separatorWidth;

#if ORIENTED_RIBBON_MODE == 1 // ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH
uniform vec4 bandBackgroundColor = vec4(0.5, 0.5, 0.5, 1.0);
#endif

#include "MultiVarGlobalVariables.glsl"
#include "MultiVarShadingUtils.glsl"

#define M_PI 3.14159265358979323846

void main() {
    // Compute the normal of the billboard tube for shading.
    vec3 fragNormal;
    float interpolationFactor = fragNormalFloat;
    vec3 normalCos = normalize(fragNormal0);
    vec3 normalSin = normalize(fragNormal1);
    if (interpolationFactor < 0.0) {
        normalSin = -normalSin;
        interpolationFactor = -interpolationFactor;
    }
    float angle = interpolationFactor * M_PI * 0.5;
    fragNormal = cos(angle) * normalCos + sin(angle) * normalSin;

    float ribbonPosition = 0.5 * fragNormalFloat + 0.5;

#if ORIENTED_RIBBON_MODE == 0 || ORIENTED_RIBBON_MODE == 1 // ORIENTED_RIBBON_MODE_FIXED_BAND_WIDTH || ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH
    // 1) Compute the variable ID for this ribbon position and the variable fraction.
    const int varID = int(floor(ribbonPosition * numVariables));
    float bandPos = ribbonPosition * numVariables - float(varID);
#elif ORIENTED_RIBBON_MODE == 2 || ORIENTED_RIBBON_MODE == 3 // ORIENTED_RIBBON_MODE_VARYING_BAND_RATIO || ORIENTED_RIBBON_MODE_VARYING_RIBBON_WIDTH
    // Sample ALL variables.
    float variables[MAX_NUM_VARIABLES];
    for (int varID = 0; varID < numVariables; varID++) {
        // 1) Sample variables from buffers.
        float variableValue, variableNextValue;
        vec2 variableMinMax, variableNextMinMax;
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementID, variableValue, variableMinMax);
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementNextID, variableNextValue, variableNextMinMax);

        // 2) Normalize values.
        variableValue = clamp((variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x), 0.0, 1.0);
        variableNextValue = clamp((variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x), 0.0, 1.0);

        // 3) Interpolate linearly.
        variables[varID] = mix(variableValue, variableNextValue, fragElementInterpolant);
    }

    // Compute the sum of all variables
    float variablesSumTotal = 0.0;
    for (int varID = 0; varID < numVariables; varID++) {
        variablesSumTotal += variables[varID];
    }

    // Compute which color we are at
    float variablesSumLast = 0.0, variablesSumCurrent = 0.0;
    int varID = 0;
    float bandPos = 1.0;
    for (varID = 0; varID < numVariables; varID++) {
        variablesSumCurrent += variables[varID] / variablesSumTotal;
        if (ribbonPosition < variablesSumCurrent) {
            bandPos = (ribbonPosition - variablesSumLast) / (variablesSumCurrent - variablesSumLast);
            break;
        }
        variablesSumLast = variablesSumCurrent;
    }
#endif

    // 2) Sample variables from buffers
    float variableValue, variableNextValue;
    vec2 variableMinMax, variableNextMinMax;
    sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementID, variableValue, variableMinMax);
    sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementNextID, variableNextValue, variableNextMinMax);

    // 3) Normalize values
    //variableValue = (variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x);
    //variableNextValue = (variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x);

    // 4) Determine variable color
    vec4 surfaceColor = determineColorLinearInterpolate(
            sampleActualVarID(varID), variableValue, variableNextValue, fragElementInterpolant);

#if ORIENTED_RIBBON_MODE == 1 // ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH
    float symBandPos = abs(bandPos * 2.0 - 1.0);

    // 1.2) Normalize values.
    float variableValuePrime = clamp((variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x), 0.0, 1.0);
    float variableNextValuePrime = clamp((variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x), 0.0, 1.0);

    float interpolatedVariableValue = mix(variableValuePrime, variableNextValuePrime, fragElementInterpolant);
    bandPos = (-symBandPos / interpolatedVariableValue + 1.0) * 0.5;
#endif

    // 4.1) Draw black separators between single stripes.
    if (separatorWidth > 0) {
        drawSeparatorBetweenStripes(surfaceColor, bandPos, separatorWidth);
    }

#if ORIENTED_RIBBON_MODE == 1 // ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH
    if (symBandPos > interpolatedVariableValue) {
        surfaceColor = bandBackgroundColor;
    }
#endif

    ////////////
    // 5) Phong Lighting
    float occlusionFactor = 1.0f;
    float shadowFactor = 1.0f;

    vec4 color = computePhongLighting(
            surfaceColor, occlusionFactor, shadowFactor, fragWorldPos, fragNormal, fragTangent);

    if (color.a < 1.0/255.0) {
        discard;
    }

#if defined(DIRECT_BLIT_GATHER)
    // Direct rendering
    fragColor = color;
#elif defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK)
    // Area of mutual exclusion for fragments mapping to the same pixel
    beginInvocationInterlockARB();
    gatherFragment(color);
    endInvocationInterlockARB();
#elif defined(USE_SYNC_SPINLOCK)
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));
    /**
     * Spinlock code below based on code in:
     * Brüll, Felix. (2018). Order-Independent Transparency Acceleration. 10.13140/RG.2.2.17568.84485.
     */
    if (!gl_HelperInvocation) {
        bool keepWaiting = true;
        while (keepWaiting) {
            if (atomicCompSwap(spinlockViewportBuffer[pixelIndex], 0, 1) == 0) {
                gatherFragment(color);
                memoryBarrier();
                atomicExchange(spinlockViewportBuffer[pixelIndex], 0);
                keepWaiting = false;
            }
        }
    }
#else
    gatherFragment(color);
#endif
}