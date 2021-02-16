-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in vec3 vertexOffsetLeft;
layout(location = 5) in vec3 vertexOffsetRight;
layout(location = 6) in uint vertexLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
layout(location = 7) in uint vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 8) in float vertexLineHierarchyLevel;
#endif

out VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    vec3 lineOffsetLeft;
    vec3 lineOffsetRight;
    uint lineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    float lineLineHierarchyLevel;
#endif
};

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
    lineOffsetLeft = vertexOffsetLeft;
    lineOffsetRight = vertexOffsetRight;
    lineSegmentId = vertexLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    linePrincipalStressIndex = vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    lineLineHierarchyLevel = vertexLineHierarchyLevel;
#endif

    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float lineWidth;
uniform float bandWidth;
uniform ivec3 psUseBands;

out vec3 fragmentPositionWorld;
out float fragmentAttribute;
flat out uint fragmentLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
flat out uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
flat out float fragmentLineHierarchyLevel;
#endif

in VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    vec3 lineOffsetLeft;
    vec3 lineOffsetRight;
    uint lineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    float lineLineHierarchyLevel;
#endif
} v_in[];

void main() {
    vec3 linePosition0 = v_in[0].linePosition;
    vec3 linePosition1 = v_in[1].linePosition;

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    int useBand = psUseBands[v_in[0].linePrincipalStressIndex];
#else
    int useBand = 1;
#endif

    vec3 offsetDirectionLeft0;
    vec3 offsetDirectionRight0;
    vec3 offsetDirectionLeft1;
    vec3 offsetDirectionRight1;
    if (useBand != 0) {
        offsetDirectionLeft0 = v_in[0].lineOffsetLeft;
        offsetDirectionRight0 = v_in[0].lineOffsetRight;
        offsetDirectionLeft1 = v_in[1].lineOffsetLeft;
        offsetDirectionRight1 = v_in[1].lineOffsetRight;
    } else {
        vec3 viewDirection0 = normalize(cameraPosition - linePosition0);
        vec3 viewDirection1 = normalize(cameraPosition - linePosition1);
        offsetDirectionRight0 = normalize(cross(v_in[0].lineTangent, viewDirection0));
        offsetDirectionLeft0 = -offsetDirectionRight0;
        offsetDirectionRight1 = normalize(cross(v_in[1].lineTangent, viewDirection1));
        offsetDirectionLeft1 = -offsetDirectionRight1;
    }

    vec3 vertexPosition;

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
    const mat4 pvMatrix = pMatrix * vMatrix;

    // Vertex 0
    fragmentAttribute = v_in[0].lineAttribute;
    fragmentLineSegmentId = v_in[0].lineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = v_in[0].linePrincipalStressIndex;
    fragmentLineHierarchyLevel = v_in[0].lineLineHierarchyLevel;
#endif

    vertexPosition = linePosition0 + lineRadius * offsetDirectionLeft0;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + lineRadius * offsetDirectionRight0;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    // Vertex 1
    fragmentAttribute = v_in[1].lineAttribute;
    fragmentLineSegmentId = v_in[1].lineSegmentId;
#ifdef USE_LINE_HIERARCHY_LEVEL
    fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
    fragmentLineHierarchyLevel = v_in[1].lineLineHierarchyLevel;
#endif

    vertexPosition = linePosition1 + lineRadius * offsetDirectionLeft1;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + lineRadius * offsetDirectionRight1;
    fragmentPositionWorld = vertexPosition;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in float fragmentAttribute;
flat in uint fragmentLineSegmentId;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
flat in uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
flat in float fragmentLineHierarchyLevel;
//uniform vec3 lineHierarchySliderLower;
//uniform vec3 lineHierarchySliderUpper;
uniform sampler1DArray lineHierarchyImportanceMap;
#endif

uniform vec3 cameraPosition;

#ifndef USE_LINE_HIERARCHY_LEVEL
uniform float minAttrValue = 0.0f;
uniform float maxAttrValue = 1.0f;
#endif

#include "FloatPack.glsl"
#include "LinkedListHeaderOpacities.glsl"

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.lineSegmentId = fragmentLineSegmentId;
    frag.next = -1;
#ifdef USE_LINE_HIERARCHY_LEVEL
    //float lower = lineHierarchySliderLower[fragmentPrincipalStressIndex];
    //float upper = lineHierarchySliderUpper[fragmentPrincipalStressIndex];
    //float fragmentAttributeHierarchy = (upper - lower) * fragmentLineHierarchyLevel + lower;
    float fragmentAttributeHierarchy = texture(
            lineHierarchyImportanceMap, vec2(fragmentLineHierarchyLevel, float(fragmentPrincipalStressIndex))).r;
    packFloat22Float10(frag.depth, gl_FragCoord.z, fragmentAttributeHierarchy);
#else
    packFloat22Float10(frag.depth, gl_FragCoord.z, (fragmentAttribute - minAttrValue) / (maxAttrValue - minAttrValue));
#endif

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}
