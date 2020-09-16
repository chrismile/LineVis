-- Programmable.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexTangent;

struct LinePointData {
    vec3 vertexPosition;
    float vertexAttribute;
    vec3 vertexTangent;
    float padding;
};

layout (std430, binding = 2) buffer LinePoints {
    LinePointData linePoints[];
};

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

    //screenSpacePosition = (vMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexTangent;

out VertexData {
    vec3 linePosition;
    vec3 lineTangent;
};

#include "TransferFunction.glsl"

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineTangent = vertexTangent;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float lineWidth;

in VertexData {
    vec3 linePosition;
    vec3 lineTangent;
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
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - lineRadius * offsetDirection1;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + lineRadius * offsetDirection0;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + lineRadius * offsetDirection1;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

#include "DepthComplexityHeader.glsl"

void main()
{
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));
    atomicAdd(fragmentCounterBuffer[pixelIndex], 1u);
}
