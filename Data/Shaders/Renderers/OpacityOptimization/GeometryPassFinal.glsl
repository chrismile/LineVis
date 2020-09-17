-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexTangent;
layout(location = 3) in float vertexOpacity;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
layout(location = 4) in uint vertexPrincipalStressIndex;
#endif

out VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    float lineOpacity;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    uint linePrincipalStressIndex;
#endif
};

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
    lineOpacity = vertexOpacity;
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
out float fragmentOpacity;
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
    float lineOpacity;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    uint linePrincipalStressIndex;
#endif
} v_in[];

void main() {
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;

    vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
    vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
    vec3 offsetDirection0 = normalize(cross(viewDirection0, normalize(v_in[0].lineTangent)));
    vec3 offsetDirection1 = normalize(cross(viewDirection1, normalize(v_in[1].lineTangent)));
    vec3 vertexPosition;

    const float lineRadius = lineWidth * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    vertexPosition = linePosition0 - lineRadius * offsetDirection0;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = v_in[0].lineAttribute;
    fragmentOpacity = v_in[0].lineOpacity;
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
    fragmentAttribute = v_in[1].lineAttribute;
    fragmentOpacity = v_in[1].lineOpacity;
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
    fragmentAttribute = v_in[0].lineAttribute;
    fragmentOpacity = v_in[0].lineOpacity;
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
    fragmentAttribute = v_in[1].lineAttribute;
    fragmentOpacity = v_in[1].lineOpacity;
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

#version 430 core

in vec3 fragmentPositionWorld;
in float fragmentAttribute;
in float fragmentOpacity;
in float fragmentNormalFloat;
in vec3 normal0;
in vec3 normal1;
#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
flat in uint fragmentPrincipalStressIndex;
#endif

#ifdef USE_COVERAGE_MASK
in int gl_SampleMaskIn[];
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

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"
#include "FloatPack.glsl"
#include "Lighting.glsl"
#include "LinkedListHeaderFinal.glsl"

void gatherFragmentCustomDepth(vec4 color, float fragmentDepth) {
    if (color.a < 0.001) {
        discard;
    }

    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.color = packUnorm4x8(color);
    frag.next = -1;

    //float depthNormalized = gl_FragCoord.z;
    float depthNormalized = convertLinearDepthToDepthBufferValue(
            convertDepthBufferValueToLinearDepth(gl_FragCoord.z) + fragmentDepth - length(fragmentPositionWorld - cameraPosition) - 0.0001);

    #ifdef USE_COVERAGE_MASK
    //packFloat24Uint8(frag.depth, gl_FragCoord.z, gl_SampleMaskIn[0]);
    float coverageRatio = float(bitCount(gl_SampleMaskIn[0])) / float(gl_NumSamples);
    packFloat24Float8(frag.depth, depthNormalized, coverageRatio);
    #else
    frag.depth = convertNormalizedFloatToUint32(depthNormalized);
    #endif

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}

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
    fragmentColor.a = 1.0; // Ignore transparency mapping.
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

    colorOut.a *= fragmentOpacity;
    gatherFragmentCustomDepth(colorOut, fragmentDepth);
}
