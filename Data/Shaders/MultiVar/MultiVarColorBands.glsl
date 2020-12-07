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
//flat out int fragVariableNextID;
flat out float fragVariableNextValue;
out float fragElementInterpolant;

#include "MultiVarGeometryUtils.glsl"

// "Color Bands" specific uniforms
uniform bool mapTubeDiameter;


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
    const int varID = sampleActualVarID(instanceID % numVariables); // for stripes
    const int elementID = vertexOutput[0].vElementID;
    const int lineID = vertexOutput[0].vLineID;

    //float variableValueOrig = 0;
    float variableValue = 0;
    vec2 variableMinMax = vec2(0);

    if (varID >= 0) {
        sampleVariableFromLineSSBO(lineID, varID, elementID, variableValue, variableMinMax);
        // Normalize value
        //variableValue = (variableValueOrig - variableMinMax.x) / (variableMinMax.y - variableMinMax.x);
        //        variableValue = (varInfo.x - varInfo.y) / (varInfo.z - varInfo.y);
    }

    // 2.1) Radius mapping
    float minRadius = minRadiusFactor * lineWidth / 2.0;
    float curRadius = lineWidth / 2.0;
    float nextRadius = lineWidth / 2.0;

    if (mapTubeDiameter) {
        curRadius = minRadius;
        nextRadius = minRadius;
        if (varID >= 0) {
            curRadius = computeRadius(
                    lineID, varID, elementID, vertexOutput[0].vElementNextID,
                    minRadius, lineWidth / 2.0, vertexOutput[0].vElementInterpolant);
//            nextRadius = curRadius;
            nextRadius = computeRadius(
                    lineID, varID, vertexOutput[1].vElementID, vertexOutput[1].vElementNextID,
                    minRadius, lineWidth / 2.0, vertexOutput[1].vElementInterpolant);
        }
    } else {
        if (varID < 0) {
            curRadius = minRadius;
            nextRadius = minRadius;
        }
    }

    // 2) Create tube circle vertices for current and next point
    createPartialTubeSegments(
            circlePointsCurrent, vertexNormalsCurrent, currentPoint,
            normalCurrent, tangentCurrent, curRadius, -1.0, instanceID, 0, 0);
    createPartialTubeSegments(
            circlePointsNext, vertexNormalsNext, nextPoint,
            normalNext, tangentNext, nextRadius, -1.0, instanceID, 0, 0);


    // 3) Draw Tube Front Sides
    fragVariableValue = variableValue;
    fragVariableID = varID;

    float interpIncrement = 1.0 / (NUM_CIRCLE_POINTS_PER_INSTANCE - 1);
    float curInterpolant = 0.0f;

    for (int i = 0; i < NUM_CIRCLE_POINTS_PER_INSTANCE - 1; i++) {
        int iN = (i + 1) % NUM_CIRCLE_POINTS_PER_INSTANCE;

        vec3 segmentPointCurrent0 = circlePointsCurrent[i];
        vec3 segmentPointNext0 = circlePointsNext[i];
        vec3 segmentPointCurrent1 = circlePointsCurrent[iN];
        vec3 segmentPointNext1 = circlePointsNext[iN];

        ////////////////////////
        // For linear interpolation: define next element and interpolant on curve
        int elementNextID = vertexOutput[0].vElementNextID;

        float variableNextValue = 0;

        if (elementNextID >= 0) {
            sampleVariableFromLineSSBO(lineID, varID, elementNextID, variableNextValue, variableMinMax);
            // Normalize value
            //fragVariableNextValue = (variableNextValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x);
            fragVariableNextValue = variableNextValue;
        }
        fragElementInterpolant = vertexOutput[0].vElementInterpolant;
        ////////////////////////

        gl_Position = mvpMatrix * vec4(segmentPointCurrent0, 1.0);
        fragTangent = normalize(segmentPointNext0 - segmentPointCurrent0);//tangentCurrent;
        if (mapTubeDiameter) {
            vec3 intermediateBinormal = normalize(cross(vertexNormalsCurrent[i], fragTangent));
            fragNormal = normalize(cross(fragTangent, intermediateBinormal));//vertexNormalsCurrent[i];
        } else {
            fragNormal = vertexNormalsCurrent[i];
        }
        fragWorldPos = (mMatrix * vec4(segmentPointCurrent0, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent0, 1.0)).xyz;
        fragBorderInterpolant = curInterpolant;
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointCurrent1, 1.0);
        fragTangent = normalize(segmentPointNext1 - segmentPointCurrent1);//tangentCurrent;
        if (mapTubeDiameter) {
            vec3 intermediateBinormal = normalize(cross(vertexNormalsCurrent[iN], fragTangent));
            fragNormal = normalize(cross(fragTangent, intermediateBinormal) + vertexNormalsCurrent[iN]);//vertexNormalsCurrent[iN];
        } else {
            fragNormal = vertexNormalsCurrent[iN];
        }
        fragWorldPos = (mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
        fragBorderInterpolant = curInterpolant + interpIncrement;
        EmitVertex();

        ////////////////////////
        // For linear interpolation: define next element and interpolant on curve
        elementNextID = vertexOutput[0].vElementNextID;

        if (vertexOutput[1].vElementInterpolant < vertexOutput[0].vElementInterpolant) {
            fragElementInterpolant = 1.0f;
//            fragElementNextID = int(vertexOutput[0].vElementNextID);
        } else {
//            fragElementNextID = int(vertexOutput[1].vElementNextID);
            fragElementInterpolant = vertexOutput[1].vElementInterpolant;
        }

//        fragVariableNextID = elementNextID;
        if (elementNextID >= 0) {
            sampleVariableFromLineSSBO(lineID, varID, elementNextID, variableNextValue, variableMinMax);
            // Normalize value
            //fragVariableNextValue = (variableNextValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x);
            fragVariableNextValue = variableNextValue;
        }
        ////////////////////////

        gl_Position = mvpMatrix * vec4(segmentPointNext0, 1.0);
        fragTangent = normalize(segmentPointNext0 - segmentPointCurrent0);//tangentNext;
        if (mapTubeDiameter) {
            vec3 intermediateBinormal = normalize(cross(vertexNormalsNext[i], fragTangent));
            fragNormal = normalize(cross(fragTangent, intermediateBinormal) + vertexNormalsNext[i]);//vertexNormalsNext[i];
        } else {
            fragNormal = vertexNormalsNext[i];
        }
        fragWorldPos = (mMatrix * vec4(segmentPointNext0, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointNext0, 1.0)).xyz;
        fragBorderInterpolant = curInterpolant;
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointNext1, 1.0);
        fragTangent = normalize(segmentPointNext1 - segmentPointCurrent1);//tangentNext;
        if (mapTubeDiameter) {
            vec3 intermediateBinormal = normalize(cross(vertexNormalsNext[iN], fragTangent));
            fragNormal = normalize(cross(fragTangent, intermediateBinormal) + vertexNormalsNext[iN]);//vertexNormalsNext[iN];
        } else {
            fragNormal = vertexNormalsNext[iN];
        }
        fragWorldPos = (mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
        fragBorderInterpolant = curInterpolant + interpIncrement;
        EmitVertex();

        EndPrimitive();

        curInterpolant += interpIncrement;
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
            circlePointsCurrent, vertexNormalsCurrent, circlePointsNext, vertexNormalsNext,
            currentPoint, nextPoint, normalize(circlePointsNext[0] - circlePointsCurrent[0]));
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
//flat in int fragVariableNextID;
flat in float fragVariableNextValue;
in float fragElementInterpolant;

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
//    vec4 surfaceColor = determineColor(fragVariableID, fragVariableValue);
    vec4 surfaceColor = determineColorLinearInterpolate(fragVariableID, fragVariableValue,
                                                        fragVariableNextValue, fragElementInterpolant);
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