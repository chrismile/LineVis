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

#if !defined(NUM_INSTANCES)
#define NUM_INSTANCES 10
#endif

#if !defined(NUM_SEGMENTS)
#define NUM_SEGMENTS 5
#endif

layout(lines, invocations = NUM_INSTANCES) in;
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
flat out int fragVarID;
out float fragElementInterpolant; // current number of curve parameter t (in [0;1]) within one line segment

uniform float fiberRadius;
uniform bool mapTubeDiameter;

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

    // 0) Setup temp fiber offset on imaginary circle
    const int instanceID = gl_InvocationID;
    const int varID = sampleActualVarID(instanceID % maxNumVariables); // for stripes
    const int elementID = vertexOutput[0].vElementID;
    const int lineID = vertexOutput[0].vLineID;

    float thetaInc = 2 * 3.1415926 / (NUM_INSTANCES);
    float thetaCur = thetaInc * instanceID;

    currentPoint = currentPoint + (cos(thetaCur) * normalCurrent + sin(thetaCur) * binormalCurrent) * lineWidth / 2.0;
    nextPoint = nextPoint + (cos(thetaCur) * normalNext + sin(thetaCur) * binormalNext) * lineWidth / 2.0;

    // 0.5) Map diamter if enabled
    float minRadius = minRadiusFactor * fiberRadius;
    float curRadius = fiberRadius;
    float nextRadius = fiberRadius;

    if (mapTubeDiameter) {
        curRadius = minRadius;
        nextRadius = minRadius;

        if (varID >= 0) {
            curRadius = computeRadius(lineID, varID, elementID, vertexOutput[0].vElementNextID,
                                      minRadius, fiberRadius, vertexOutput[0].vElementInterpolant);
            //            nextRadius = curRadius;
            nextRadius = computeRadius(lineID, varID, vertexOutput[1].vElementID, vertexOutput[1].vElementNextID,
                                       minRadius, fiberRadius, vertexOutput[1].vElementInterpolant);
        }
    }

    // 1) Create tube circle vertices for current and next point
    createTubeSegments(circlePointsCurrent, vertexNormalsCurrent, currentPoint,
                       normalCurrent, tangentCurrent, curRadius);
    createTubeSegments(circlePointsNext, vertexNormalsNext, nextPoint,
                       normalNext, tangentNext, nextRadius);

    // 2) Create NDC AABB for stripe -> tube mapping
    // 2.1) Define orientation of local NDC frame-of-reference
    vec4 currentPointNDC = mvpMatrix * vec4(currentPoint, 1);
    currentPointNDC.xyz /= currentPointNDC.w;
    vec4 nextPointNDC = mvpMatrix * vec4(nextPoint, 1);
    nextPointNDC.xyz /= nextPointNDC.w;

//    vec2 ndcOrientation = normalize(nextPointNDC.xy - currentPointNDC.xy);
//    vec2 ndcOrientNormal = vec2(-ndcOrientation.y, ndcOrientation.x);
//    // 2.2) Matrix to rotate every NDC point back to the local frame-of-reference
//    // (such that tangent is aligned with the x-axis)
//    mat2 invMatNDC = inverse(mat2(ndcOrientation, ndcOrientNormal));

//    // 2.3) Compute AABB in NDC space
//    vec2 bboxMin = vec2(10,10);
//    vec2 bboxMax = vec2(-10,-10);
//    vec2 tangentNDC = vec2(0);
//    vec2 normalNDC = vec2(0);
//    vec2 refPointNDC = vec2(0);
//
//    computeAABBFromNDC(circlePointsCurrent, invMatNDC, bboxMin, bboxMax, tangentNDC, normalNDC, refPointNDC);
//    computeAABBFromNDC(circlePointsNext, invMatNDC, bboxMin, bboxMax, tangentNDC, normalNDC, refPointNDC);

//    // 3) Compute texture coordinates
//    computeTexCoords(vertexTexCoordsCurrent, circlePointsCurrent,
//    invMatNDC, tangentNDC, normalNDC, refPointNDC);
//    computeTexCoords(vertexTexCoordsNext, circlePointsNext,
//    invMatNDC, tangentNDC, normalNDC, refPointNDC);

    // 4) Emit the tube triangle vertices and attributes to the fragment shader
    fragElementID = vertexOutput[0].vElementID;
    fragLineID = vertexOutput[0].vLineID;
    fragVarID = varID;

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
//        fragTexCoord = vertexTexCoordsCurrent[i];
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent0, 1.0)).xyz;
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointCurrent1, 1.0);
        fragNormal = vertexNormalsCurrent[iN];
        fragTangent = tangentCurrent;
        fragWorldPos = (mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointCurrent1, 1.0)).xyz;
//        fragTexCoord = vertexTexCoordsCurrent[iN];
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
//        fragTexCoord = vertexTexCoordsNext[i];
        EmitVertex();

        gl_Position = mvpMatrix * vec4(segmentPointNext1, 1.0);
        fragNormal = vertexNormalsNext[iN];
        fragTangent = tangentNext;
        fragWorldPos = (mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
        screenSpacePosition = (vMatrix * mMatrix * vec4(segmentPointNext1, 1.0)).xyz;
//        fragTexCoord = vertexTexCoordsNext[iN];
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
flat in int fragVarID;
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

#include "MultiVarGlobalVariables.glsl"
#include "MultiVarShadingUtils.glsl"


void main()
{
    float variableValue;
    vec2 variableMinMax;

    float variableNextValue;
    vec2 variableNextMinMax;

    const int varID = fragVarID;
    // 1) Determine variable ID along tube geometry
//    const int varID = int(floor(fragTexCoord.y * numVariables));
//    float varFraction = fragTexCoord.y * numVariables - float(varID);

//
//    // 2) Sample variables from buffers
    sampleVariableFromLineSSBO(fragLineID, varID, fragElementID, variableValue, variableMinMax);
    sampleVariableFromLineSSBO(fragLineID, varID, fragElementNextID, variableNextValue, variableNextMinMax);
//
//    // 3) Normalize values
    //variableValue = (variableValue - variableMinMax.x) / (variableMinMax.y - variableMinMax.x);
    //variableNextValue = (variableNextValue - variableNextMinMax.x) / (variableNextMinMax.y - variableNextMinMax.x);
//
//    // 4) Determine variable color
    vec4 surfaceColor = determineColorLinearInterpolate(
            varID, variableValue, variableNextValue, fragElementInterpolant);
//    // 4.1) Draw black separators between single stripes.
//    if (separatorWidth > 0)
//    {
//        drawSeparatorBetweenStripes(surfaceColor, varFraction, separatorWidth);
//    }


    ////////////
    // 5) Phong Lighting
    float occlusionFactor = 1.0f;
    float shadowFactor = 1.0f;

    vec4 color = computePhongLighting(surfaceColor, occlusionFactor, shadowFactor,
    fragWorldPos, fragNormal, fragTangent);

//    vec4 color = determineColor(fragVarID, 100);
//    vec4 color = vec4(1, 0, 0, 1);

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
