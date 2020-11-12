-- Programmable.Vertex

#version 430 core

struct LinePointData {
    vec3 vertexPosition;
    float vertexAttribute;
    vec3 vertexTangent;
    uint vertexPrincipalStressIndex;
};

layout (std430, binding = 2) buffer LinePoints {
    LinePointData linePoints[];
};

out vec3 fragmentPositionWorld;
out float fragmentAttribute;
out float fragmentNormalFloat; // Between -1 and 1
out vec3 normal0;
out vec3 normal1;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
flat out uint fragmentPrincipalStressIndex;
#endif

uniform vec3 cameraPosition;
uniform float lineWidth;

void main() {

    uint pointIndex = gl_VertexID/2;
    LinePointData linePointData = linePoints[pointIndex];
    vec3 linePoint = (mMatrix * vec4(linePointData.vertexPosition, 1.0)).xyz;

    vec3 viewDirection = normalize(cameraPosition - linePoint);
    vec3 offsetDirection = normalize(cross(viewDirection, normalize(linePointData.vertexTangent)));
    vec3 vertexPosition;
    float shiftSign = 1.0f;
    if (gl_VertexID % 2 == 0) {
        shiftSign = -1.0;
    }
    vertexPosition = linePoint + shiftSign * lineWidth * 0.5 * offsetDirection;
    fragmentNormalFloat = shiftSign;
    normal0 = viewDirection;
    normal1 = offsetDirection;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = linePointData.vertexPrincipalStressIndex;
#endif

    fragmentPositionWorld = vertexPosition;
    //screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
    fragmentAttribute = linePointData.vertexAttribute;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexTangent;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
layout(location = 3) in uint vertexPrincipalStressIndex;
#endif

out VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    uint linePrincipalStressIndex;
#endif
};

#include "TransferFunction.glsl"

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    linePrincipalStressIndex = vertexPrincipalStressIndex;
#endif
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float lineWidth;

out vec3 fragmentPositionWorld;
out float fragmentAttribute;
out float fragmentNormalFloat; // Between -1 and 1
out vec3 normal0;
out vec3 normal1;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
flat out uint fragmentPrincipalStressIndex;
#endif

in VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    uint linePrincipalStressIndex;
#endif
} v_in[];

void main() {
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;
    float lineAttribute0 = v_in[0].lineAttribute;
    float lineAttribute1 = v_in[1].lineAttribute;

    vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
    vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
    vec3 offsetDirection0 = normalize(cross(viewDirection0, normalize(v_in[0].lineTangent)));
    vec3 offsetDirection1 = normalize(cross(viewDirection1, normalize(v_in[1].lineTangent)));
    vec3 vertexPosition;

    const float lineRadius = lineWidth * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    vertexPosition = linePosition0 - lineRadius * offsetDirection0;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = lineAttribute0;
    fragmentNormalFloat = -1.0;
    normal0 = viewDirection0;
    normal1 = offsetDirection0;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[0].linePrincipalStressIndex;
#endif
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - lineRadius * offsetDirection1;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = lineAttribute1;
    fragmentNormalFloat = -1.0;
    normal0 = viewDirection1;
    normal1 = offsetDirection1;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
#endif
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + lineRadius * offsetDirection0;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = lineAttribute0;
    fragmentNormalFloat = 1.0;
    normal0 = viewDirection0;
    normal1 = offsetDirection0;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[0].linePrincipalStressIndex;
#endif
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + lineRadius * offsetDirection1;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = lineAttribute1;
    fragmentNormalFloat = 1.0;
    normal0 = viewDirection1;
    normal1 = offsetDirection1;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
#endif
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 450 core

in vec3 fragmentPositionWorld;
in float fragmentAttribute;
in float fragmentNormalFloat;
in vec3 normal0;
in vec3 normal1;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
flat in uint fragmentPrincipalStressIndex;
#endif

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;
uniform float lineWidth;
uniform vec3 backgroundColor;
uniform vec3 foregroundColor;

#define M_PI 3.14159265358979323846

#include "TransferFunction.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"
#include "Lighting.glsl"

void main() {
    // Compute the normal of the billboard tube for shading.
    vec3 fragmentNormal;
    float interpolationFactor = fragmentNormalFloat;
    vec3 normalCos = normalize(normal0);
    vec3 normalSin = normalize(normal1);
    if (interpolationFactor < 0.0) {
        normalSin = -normalSin;
        interpolationFactor = -interpolationFactor;
    }
    float angle = interpolationFactor * M_PI * 0.5;
    fragmentNormal = cos(angle) * normalCos + sin(angle) * normalSin;

#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    vec4 fragmentColor = transferFunction(fragmentAttribute, fragmentPrincipalStressIndex);
#else
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#endif
    fragmentColor = blinnPhongShading(fragmentColor, fragmentNormal);

    float absCoords = abs(fragmentNormalFloat);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth * 0.0015 / lineWidth, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(fragmentNormalFloat));
    vec4 colorOut = vec4(mix(fragmentColor.rgb, foregroundColor,
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, absCoords)),
            fragmentColor.a * coverage);

    #if defined(DIRECT_BLIT_GATHER)
    // To counteract depth fighting with overlay wireframe.
    float depthOffset = -0.00001;
    if (absCoords >= WHITE_THRESHOLD - EPSILON) {
        depthOffset = 0.002;
    }
    //gl_FragDepth = clamp(gl_FragCoord.z + depthOffset, 0.0, 0.999);
    gl_FragDepth = convertLinearDepthToDepthBufferValue(
            convertDepthBufferValueToLinearDepth(gl_FragCoord.z) + fragmentDepth - length(fragmentPositionWorld - cameraPosition) - 0.0001);
    if (colorOut.a < 0.01) {
        discard;
    }
    colorOut.a = 1.0;
    fragColor = colorOut;
    #else

    #if defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK)
    // Area of mutual exclusion for fragments mapping to the same pixel
    beginInvocationInterlockARB();
    gatherFragmentCustomDepth(colorOut, fragmentDepth);
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
                gatherFragmentCustomDepth(colorOut, fragmentDepth);
                memoryBarrier();
                atomicExchange(spinlockViewportBuffer[pixelIndex], 0);
                keepWaiting = false;
            }
        }
    }
    #else
    gatherFragmentCustomDepth(colorOut, fragmentDepth);
    #endif

    #endif
}
