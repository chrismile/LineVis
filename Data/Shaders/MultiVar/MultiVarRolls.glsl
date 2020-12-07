-- Vertex

#version 430 core

#include "MultiVarGlobalVertexInput.glsl"
#include "MultiVarGlobalVariables.glsl"

out VertexData {
    vec3 vPosition;// Position in world space
    vec3 vNormal;// Orientation normal along line in world space
    vec3 vTangent;// Tangent of line in world space
    int vVariableID; // variable index (for alternating variable indices per line curve)
    int vLineID;// number of line
    int vElementID;// number of line element (original line vertex index)
    int vElementNextID; // number of next line element (original next line vertex index)
    float vElementInterpolant; // curve parameter t along curve between element and next element
    vec4 lineVariable;
};
//} GeomOut; // does not work

void main() {
    int varID = int(multiVariable.w);
    const int lineID = int(variableDesc.y);
    const int elementID = int(variableDesc.x);

    vPosition = vertexPosition;
    vNormal = normalize(vertexLineNormal);
    vTangent = normalize(vertexLineTangent);

    vVariableID = varID;
    vLineID = lineID;
    vElementID = elementID;

    vElementNextID = int(variableDesc.z);
    vElementInterpolant = variableDesc.w;

    lineVariable = multiVariable;
}

-- Geometry

#version 430 core

#if !defined(NUM_LINESEGMENTS)
    #define NUM_LINESEGMENTS 10
#endif

#if !defined(NUM_CIRCLE_POINTS_PER_INSTANCE)
    #define NUM_CIRCLE_POINTS_PER_INSTANCE 2
#endif

#include "MultiVarGlobalVariables.glsl"

layout(lines, invocations = NUM_LINESEGMENTS) in;
layout(triangle_strip, max_vertices = 128) out;



//#define NUM_SEGMENTS 10


// Input from vertex buffer
in VertexData {
    vec3 vPosition;// Position in world space
    vec3 vNormal;// Orientation normal along line in world space
    vec3 vTangent;// Tangent of line in world space
    int vVariableID; // variable index (for alternating variable indices per line curve)
    int vLineID;// number of line
    int vElementID;// number of line element (original line vertex index)
    int vElementNextID; // number of next line element (original next line vertex index)
    float vElementInterpolant; // curve parameter t along curve between element and next element
    vec4 lineVariable;
} vertexOutput[];

in int gl_PrimitiveIDIn;

// Output to fragments
out vec3 fragWorldPos;
out vec3 fragNormal;
out vec3 fragTangent;
out vec3 screenSpacePosition; // screen space position for depth in view space (to sort for buckets...)
out vec2 fragTexCoord;
// "Rolls"-specfic outputs
flat out int fragVariableID;
flat out float fragVariableValue;
out float fragBorderInterpolant;

#include "MultiVarGeometryUtils.glsl"

// "Rolls" specific uniforms
uniform bool mapTubeDiameter;
uniform int rollWidth;

void main() {
    vec3 currentPoint = (mMatrix * vec4(vertexOutput[0].vPosition, 1.0)).xyz;
    vec3 nextPoint = (mMatrix * vec4(vertexOutput[1].vPosition, 1.0)).xyz;

    vec3 circlePointsCurrent[NUM_CIRCLE_POINTS_PER_INSTANCE];
    vec3 circlePointsNext[NUM_CIRCLE_POINTS_PER_INSTANCE];

    vec3 vertexNormalsCurrent[NUM_CIRCLE_POINTS_PER_INSTANCE];
    vec3 vertexNormalsNext[NUM_CIRCLE_POINTS_PER_INSTANCE];

    vec3 normalCurrent = vertexOutput[0].vNormal;
    vec3 tangentCurrent = vertexOutput[0].vTangent;
    vec3 binormalCurrent = cross(tangentCurrent, normalCurrent);
    vec3 normalNext = vertexOutput[1].vNormal;
    vec3 tangentNext = vertexOutput[1].vTangent;
    vec3 binormalNext = cross(tangentNext, normalNext);

    vec3 tangent = normalize(nextPoint - currentPoint);

    // 1) Sample variables at each tube roll
    const int instanceID = gl_InvocationID;
    const float mappedVarIDRatio = mod(float(vertexOutput[0].vVariableID) / float(rollWidth), 1.0);
    const int mappedVarID = (vertexOutput[0].vVariableID >= 0) ? vertexOutput[0].vVariableID / rollWidth : -1;
    const int varID = sampleActualVarID(mappedVarID); // instanceID % numVariables for stripes
    const int elementID = vertexOutput[0].vElementID;
    const int lineID = vertexOutput[0].vLineID;

    //float variableValueOrig = 0;
    float variableValue = 0;
    vec2 variableMinMax = vec2(0);

//    vec4 varInfo = vertexOutput[0].lineVariable;

    if (varID >= 0) {
        sampleVariableFromLineSSBO(lineID, varID, elementID, variableValue, variableMinMax);
        // Normalize value
        //variableValue = (variableValueOrig - variableMinMax.x) / (variableMinMax.y - variableMinMax.x);
//        variableValue = (varInfo.x - varInfo.y) / (varInfo.z - varInfo.y);
    }

    // 2.1) Radius mapping
    float curRadius = lineWidth / 2.0;
    float minRadius = minRadiusFactor * lineWidth / 2.0;
    float histOffset = 0.0;

    if (mapTubeDiameter) {
        minRadius = minRadiusFactor * lineWidth / 2.0;
        curRadius = minRadius;
        if (varID >= 0) {
            vec2 lineVarMinMax = vec2(0);
            if (mapTubeDiameterMode == 0) {
                lineVarMinMax = variableMinMax;
            } else {
                sampleVariableDistributionFromLineSSBO(lineID, varID, lineVarMinMax);
            }

            if (lineVarMinMax.x != lineVarMinMax.y) {
                float interpolant = (variableValue - lineVarMinMax.x) / (lineVarMinMax.y - lineVarMinMax.x);
//                interpolant = max(0.0, min(1.0, interpolant));
                curRadius = mix(minRadius, lineWidth / 2.0, interpolant);
            }
        }

    } else {
        if (varID < 0) {
            curRadius = minRadius;
        }
    }

    // 2) Create tube circle vertices for current and next point
    createPartialTubeSegments(
            circlePointsCurrent, vertexNormalsCurrent, currentPoint,
            normalCurrent, tangentCurrent, curRadius, curRadius, instanceID, 0, 0);
    createPartialTubeSegments(
            circlePointsNext, vertexNormalsNext, nextPoint,
            normalNext, tangentNext, curRadius, curRadius, instanceID, 0, 0);

    // 3) Draw Tube Front Sides
    fragVariableValue = variableValue;
    fragVariableID = varID;

    for (int i = 0; i < NUM_CIRCLE_POINTS_PER_INSTANCE - 1; i++) {
        int iN = (i + 1) % NUM_CIRCLE_POINTS_PER_INSTANCE;

        vec3 segmentPointCurrent0 = circlePointsCurrent[i];
        vec3 segmentPointNext0 = circlePointsNext[i];
        vec3 segmentPointCurrent1 = circlePointsCurrent[iN];
        vec3 segmentPointNext1 = circlePointsNext[iN];

        gl_Position = mvpMatrix * vec4(segmentPointCurrent0, 1.0);
        fragNormal = vertexNormalsCurrent[i];
        fragTangent = tangentCurrent;
        fragWorldPos = (mMatrix * vec4(segmentPointCurrent0, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent0, 1.0)).xyz;
        fragBorderInterpolant = mappedVarIDRatio;
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointCurrent1, 1.0);
        fragNormal = vertexNormalsCurrent[iN];
        fragTangent = tangentCurrent;
        fragWorldPos = (mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointNext0, 1.0);
        fragNormal = vertexNormalsNext[i];
        fragTangent = tangentNext;
        fragWorldPos = (mMatrix * vec4(segmentPointNext0, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointNext0, 1.0)).xyz;
        fragBorderInterpolant = mappedVarIDRatio + 1.0f / float(rollWidth);
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointNext1, 1.0);
        fragNormal = vertexNormalsNext[iN];
        fragTangent = tangentNext;
        fragWorldPos = (mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
        EmitVertex();

        EndPrimitive();
    }

    // Render lids

    // 3) Ŕender lids
    for (int i = 0; i < NUM_CIRCLE_POINTS_PER_INSTANCE - 1; i++) {
        fragNormal = normalize(-tangent);
        //! TODO compute tangent
        int iN = (i + 1) % NUM_CIRCLE_POINTS_PER_INSTANCE;

        vec3 segmentPointCurrent0 = circlePointsCurrent[i];
        vec3 segmentPointCurrent1 = circlePointsCurrent[iN];

        drawTangentLid(segmentPointCurrent0, currentPoint, segmentPointCurrent1);
    }

    for (int i = 0; i < NUM_CIRCLE_POINTS_PER_INSTANCE - 1; i++) {
        fragNormal = normalize(tangent);
        //! TODO compute tangent
        int iN = (i + 1) % NUM_CIRCLE_POINTS_PER_INSTANCE;

        vec3 segmentPointCurrent0 = circlePointsNext[i];
        vec3 segmentPointCurrent1 = circlePointsNext[iN];

        drawTangentLid(segmentPointCurrent0, segmentPointCurrent1, nextPoint);
    }

    // 3) Render partial circle lids
    drawPartialCircleLids(
            circlePointsCurrent, vertexNormalsCurrent,
            circlePointsNext, vertexNormalsNext,
            currentPoint, nextPoint, tangent);
}


-- Fragment

#version 430 core

in vec3 screenSpacePosition; // Required for transparency rendering techniques

in vec3 fragWorldPos;
in vec3 fragNormal;
in vec3 fragTangent;
in vec2 fragTexCoord;
// "Rolls"-specfic inputs
flat in int fragVariableID;
flat in float fragVariableValue;
in float fragBorderInterpolant;

uniform vec3 cameraPosition; // world space

#if !defined(DIRECT_BLIT_GATHER)
#define fragmentPositionWorld fragWorldPos
#include OIT_GATHER_HEADER
#endif

#ifdef DIRECT_BLIT_GATHER
out vec4 fragColor;
#endif

#include "MultiVarGlobalVariables.glsl"
#include "MultiVarShadingUtils.glsl"

// "Rolls"-specific uniforms
uniform float separatorWidth;

void main() {
    // 1) Determine variable color
    vec4 surfaceColor = determineColor(fragVariableID, fragVariableValue);
    // 1.1) Draw black separators between single stripes.
    float varFraction = fragBorderInterpolant;
    if (separatorWidth > 0) {
        drawSeparatorBetweenStripes(surfaceColor, varFraction, separatorWidth);
    }

    ////////////
    // 2) Phong Lighting
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