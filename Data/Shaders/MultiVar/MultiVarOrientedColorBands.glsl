/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2021, Christoph Neuhauser, Michael Kern
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

-- Vertex

#version 430 core

#include "MultiVarGlobalVertexInput.glsl"
#include "MultiVarGlobalVariables.glsl"

out VertexData {
    vec3 vPosition;// Position in world space
    vec3 vNormal;// Orientation normal along line in world space
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
    vNormal = normalize(vertexLineNormal);
    vTangent = normalize(vertexLineTangent);

    vLineID = lineID;
    vElementID = elementID;

    vElementNextID = int(variableDesc.z);
    vElementInterpolant = variableDesc.w;
}

-- Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 128) out;

#include "MultiVarGlobalVariables.glsl"

//#define NUM_SEGMENTS 10

// Input from vertex buffer
in VertexData {
    vec3 vPosition;// Position in world space
    vec3 vNormal;// Orientation normal along line in world space
    vec3 vTangent;// Tangent of line in world space
    int vLineID;// number of line
    int vElementID;// number of line element (original line vertex index)
    int vElementNextID; // number of next line element (original next line vertex index)
    float vElementInterpolant; // curve parameter t along curve between element and next element
} vertexOutput[];

// Output to fragments
out vec3 fragWorldPos;
out vec3 fragNormal;
out vec3 fragTangent;
out vec3 screenSpacePosition; // screen space position for depth in view space (to sort for buckets...)
out vec2 fragTexCoord;
// "Oriented Stripes"-specfic outputs
flat out int fragElementID; // Actual per-line vertex index --> required for sampling from global buffer
flat out int fragElementNextID; // Actual next per-line vertex index --> for linear interpolation
flat out int fragLineID; // Line index --> required for sampling from global buffer
out float fragElementInterpolant; // current number of curve parameter t (in [0;1]) within one line segment

#if !defined(NUM_SEGMENTS)
    #define NUM_SEGMENTS 10
#endif

#include "MultiVarGeometryUtils.glsl"

void main() {
    vec3 currentPoint = (mMatrix * vec4(vertexOutput[0].vPosition, 1.0)).xyz;
    vec3 nextPoint = (mMatrix * vec4(vertexOutput[1].vPosition, 1.0)).xyz;

    vec3 circlePointsCurrent[NUM_SEGMENTS];
    vec3 circlePointsNext[NUM_SEGMENTS];

    vec3 vertexNormalsCurrent[NUM_SEGMENTS];
    vec3 vertexNormalsNext[NUM_SEGMENTS];

    vec2 vertexTexCoordsCurrent[NUM_SEGMENTS];
    vec2 vertexTexCoordsNext[NUM_SEGMENTS];

    vec3 normalCurrent = vertexOutput[0].vNormal;
    vec3 tangentCurrent = vertexOutput[0].vTangent;
    vec3 binormalCurrent = cross(tangentCurrent, normalCurrent);
    vec3 normalNext = vertexOutput[1].vNormal;
    vec3 tangentNext = vertexOutput[1].vTangent;
    vec3 binormalNext = cross(tangentNext, normalNext);

    vec3 tangent = normalize(nextPoint - currentPoint);

    // 1) Create tube circle vertices for current and next point
#if ORIENTED_RIBBON_MODE != 3 // ORIENTED_RIBBON_MODE_VARYING_RIBBON_WIDTH
    createTubeSegments(
            circlePointsCurrent, vertexNormalsCurrent, currentPoint, normalCurrent, tangentCurrent, lineWidth / 2.0);
    createTubeSegments(
            circlePointsNext, vertexNormalsNext, nextPoint, normalNext, tangentNext, lineWidth / 2.0);
#else
    // Sample ALL variables.
    float variables[MAX_NUM_VARIABLES];
    for (int varID = 0; varID < numVariables; varID++) {
        // 1) Sample variables from buffers.
        float variableValue, variableNextValue;
        vec2 variableMinMax, variableNextMinMax;
        sampleVariableFromLineSSBO(vertexOutput[0].vLineID, sampleActualVarID(varID), vertexOutput[0].vElementID, variableValue, variableMinMax);
        sampleVariableFromLineSSBO(vertexOutput[0].vLineID, sampleActualVarID(varID), vertexOutput[0].vElementNextID, variableNextValue, variableNextMinMax);

        // 2) Normalize values.
        variableValue = clamp((variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x), 0.0, 1.0);
        variableNextValue = clamp((variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x), 0.0, 1.0);

        // 3) Interpolate linearly.
        variables[varID] = mix(variableValue, variableNextValue, vertexOutput[0].vElementInterpolant);
    }

    // Compute the sum of all variables
    float variablesSumTotal = 0.0;
    for (int varID = 0; varID < numVariables; varID++) {
        variablesSumTotal += variables[varID];
    }
    const float lineRadius0 = lineWidth * 0.5 * (variablesSumTotal / float(max(numVariables, 1)));


    float _fragElementInterpolant;
    int _fragElementNextID;
    if (vertexOutput[1].vElementInterpolant < vertexOutput[0].vElementInterpolant) {
        _fragElementInterpolant = 1.0f;
        _fragElementNextID = int(vertexOutput[0].vElementNextID);
    } else {
        _fragElementInterpolant = vertexOutput[1].vElementInterpolant;
        _fragElementNextID = int(vertexOutput[1].vElementNextID);
    }

    // Sample ALL variables.
    for (int varID = 0; varID < numVariables; varID++) {
        // 1) Sample variables from buffers.
        float variableValue, variableNextValue;
        vec2 variableMinMax, variableNextMinMax;
        sampleVariableFromLineSSBO(vertexOutput[0].vLineID, sampleActualVarID(varID), vertexOutput[0].vElementID, variableValue, variableMinMax);
        sampleVariableFromLineSSBO(vertexOutput[0].vLineID, sampleActualVarID(varID), _fragElementNextID, variableNextValue, variableNextMinMax);

        // 2) Normalize values.
        variableValue = clamp((variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x), 0.0, 1.0);
        variableNextValue = clamp((variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x), 0.0, 1.0);

        // 3) Interpolate linearly.
        variables[varID] = mix(variableValue, variableNextValue, _fragElementInterpolant);
    }

    // Compute the sum of all variables
    variablesSumTotal = 0.0;
    for (int varID = 0; varID < numVariables; varID++) {
        variablesSumTotal += variables[varID];
    }
    const float lineRadius1 = lineWidth * 0.5 * (variablesSumTotal / float(max(numVariables, 1)));


    createTubeSegments(
            circlePointsCurrent, vertexNormalsCurrent, currentPoint, normalCurrent, tangentCurrent, lineRadius0);
    createTubeSegments(
            circlePointsNext, vertexNormalsNext, nextPoint, normalNext, tangentNext, lineRadius1);
#endif

    // 2) Create NDC AABB for stripe -> tube mapping
    // 2.1) Define orientation of local NDC frame-of-reference
    vec4 currentPointNDC = mvpMatrix * vec4(currentPoint, 1);
    currentPointNDC.xyz /= currentPointNDC.w;
    vec4 nextPointNDC = mvpMatrix * vec4(nextPoint, 1);
    nextPointNDC.xyz /= nextPointNDC.w;

    vec2 ndcOrientation = normalize(nextPointNDC.xy - currentPointNDC.xy);
    vec2 ndcOrientNormal = vec2(-ndcOrientation.y, ndcOrientation.x);
    // 2.2) Matrix to rotate every NDC point back to the local frame-of-reference
    // (such that tangent is aligned with the x-axis)
    mat2 invMatNDC = inverse(mat2(ndcOrientation, ndcOrientNormal));

    // 2.3) Compute AABB in NDC space
    vec2 bboxMin = vec2(10,10);
    vec2 bboxMax = vec2(-10,-10);
    vec2 tangentNDC = vec2(0);
    vec2 normalNDC = vec2(0);
    vec2 refPointNDC = vec2(0);

    computeAABBFromNDC(circlePointsCurrent, invMatNDC, bboxMin, bboxMax, tangentNDC, normalNDC, refPointNDC);
    computeAABBFromNDC(circlePointsNext, invMatNDC, bboxMin, bboxMax, tangentNDC, normalNDC, refPointNDC);

    // 3) Compute texture coordinates
    computeTexCoords(vertexTexCoordsCurrent, circlePointsCurrent, invMatNDC, tangentNDC, normalNDC, refPointNDC);
    computeTexCoords(vertexTexCoordsNext, circlePointsNext, invMatNDC, tangentNDC, normalNDC, refPointNDC);

    // 4) Emit the tube triangle vertices and attributes to the fragment shader
    fragElementID = vertexOutput[0].vElementID;
    fragLineID = vertexOutput[0].vLineID;

    for (int i = 0; i < NUM_SEGMENTS; i++) {
        int iN = (i + 1) % NUM_SEGMENTS;

        vec3 segmentPointCurrent0 = circlePointsCurrent[i];
        vec3 segmentPointNext0 = circlePointsNext[i];
        vec3 segmentPointCurrent1 = circlePointsCurrent[iN];
        vec3 segmentPointNext1 = circlePointsNext[iN];

        fragElementNextID = vertexOutput[0].vElementNextID;
        fragElementInterpolant = vertexOutput[0].vElementInterpolant;

        gl_Position = mvpMatrix * vec4(segmentPointCurrent0, 1.0);
        fragNormal = vertexNormalsCurrent[i];
        fragTangent = tangentCurrent;
        fragWorldPos = (mMatrix * vec4(segmentPointCurrent0, 1.0)).xyz;
        fragTexCoord = vertexTexCoordsCurrent[i];
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent0, 1.0)).xyz;
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointCurrent1, 1.0);
        fragNormal = vertexNormalsCurrent[iN];
        fragTangent = tangentCurrent;
        fragWorldPos = (mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
        fragTexCoord = vertexTexCoordsCurrent[iN];
        EmitVertex();

        if (vertexOutput[1].vElementInterpolant < vertexOutput[0].vElementInterpolant) {
            fragElementInterpolant = 1.0f;
            fragElementNextID = int(vertexOutput[0].vElementNextID);
        } else {
            fragElementInterpolant = vertexOutput[1].vElementInterpolant;
            fragElementNextID = int(vertexOutput[1].vElementNextID);
        }

        gl_Position = mvpMatrix * vec4(segmentPointNext0, 1.0);
        fragNormal = vertexNormalsNext[i];
        fragTangent = tangentNext;
        fragWorldPos = (mMatrix * vec4(segmentPointNext0, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointNext0, 1.0)).xyz;
        fragTexCoord = vertexTexCoordsNext[i];
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointNext1, 1.0);
        fragNormal = vertexNormalsNext[iN];
        fragTangent = tangentNext;
        fragWorldPos = (mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
        fragTexCoord = vertexTexCoordsNext[iN];
        EmitVertex();

        EndPrimitive();
    }
}


-- Fragment

#version 430 core

in vec3 screenSpacePosition; // Required for transparency rendering techniques

in vec3 fragWorldPos;
in vec3 fragNormal;
in vec3 fragTangent;
in vec2 fragTexCoord;
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

void main() {
    // 1) Determine variable ID along tube geometry
    const vec3 n = normalize(fragNormal);
    const vec3 v = normalize(cameraPosition - fragWorldPos);
    const vec3 t = normalize(fragTangent);
    // Project v into plane perpendicular to t to get newV.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));
    // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
    // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
    vec3 crossProdVn = cross(newV, n);
    float ribbonPosition = length(crossProdVn);

    // Side note: We can also use the code below, as for a, b with length 1:
    // sqrt(1 - dot^2(a,b)) = len(cross(a,b))
    // Due to:
    // - dot(a,b) = ||a|| ||b|| cos(phi)
    // - len(cross(a,b)) = ||a|| ||b|| |sin(phi)|
    // - sin^2(phi) + cos^2(phi) = 1
    //ribbonPosition = dot(newV, n);
    //ribbonPosition = sqrt(1 - ribbonPosition * ribbonPosition);

    // Get the winding of newV relative to n, taking into account that t is the normal of the plane both vectors lie in.
    // NOTE: dot(a, cross(b, c)) = det(a, b, c), which is the signed volume of the parallelepiped spanned by a, b, c.
    if (dot(t, crossProdVn) < 0.0) {
        ribbonPosition = -ribbonPosition;
    }
    // Normalize the ribbon position: [-1, 1] -> [0, 1].
    ribbonPosition = ribbonPosition / 2.0 + 0.5;

    // Old code:
    //const int varID = int(floor(fragTexCoord.y * numVariables));
    //float bandPos = fragTexCoord.y * numVariables - float(varID);

#if ORIENTED_RIBBON_MODE == 0 || ORIENTED_RIBBON_MODE == 1 // ORIENTED_RIBBON_MODE_FIXED_BAND_WIDTH || ORIENTED_RIBBON_MODE_VARYING_BAND_WIDTH
    // 1) Compute the variable ID for this ribbon position and the variable fraction.
    const int varID = int(floor(ribbonPosition * numVariables));
    float bandPos = ribbonPosition * numVariables - float(varID);
#elif ORIENTED_RIBBON_MODE == 2 || ORIENTED_RIBBON_MODE == 3 // ORIENTED_RIBBON_MODE_VARYING_BAND_RATIO || ORIENTED_RIBBON_MODE_VARYING_RIBBON_WIDTH
    // Sample ALL variables.
    float variables[MAX_NUM_VARIABLES];
    for (int varID = 0; varID < numVariables; varID++) {
        // 1.1) Sample variables from buffers.
        float variableValue, variableNextValue;
        vec2 variableMinMax, variableNextMinMax;
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementID, variableValue, variableMinMax);
        sampleVariableFromLineSSBO(fragLineID, sampleActualVarID(varID), fragElementNextID, variableNextValue, variableNextMinMax);

        // 1.2) Normalize values.
        variableValue = clamp((variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x), 0.0, 1.0);
        variableNextValue = clamp((variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x), 0.0, 1.0);

        // 1.3) Interpolate linearly.
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