-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexTangent;
layout(location = 3) in uint vertexLineSegmentId;

out VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    uint lineSegmentId;
};

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
    lineSegmentId = vertexLineSegmentId;
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
flat out uint fragmentLineSegmentId;

in VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    uint lineSegmentId;
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
    fragmentLineSegmentId = v_in[0].lineSegmentId;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 - lineRadius * offsetDirection1;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = lineAttribute1;
    fragmentLineSegmentId = v_in[1].lineSegmentId;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition0 + lineRadius * offsetDirection0;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = lineAttribute0;
    fragmentLineSegmentId = v_in[0].lineSegmentId;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = linePosition1 + lineRadius * offsetDirection1;
    fragmentPositionWorld = vertexPosition;
    fragmentAttribute = lineAttribute1;
    fragmentLineSegmentId = v_in[1].lineSegmentId;
    gl_Position = pvMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in float fragmentAttribute;
flat in uint fragmentLineSegmentId;

uniform vec3 cameraPosition;

#include "FloatPack.glsl"
#include "LinkedListHeaderOpacities.glsl"

void main() {
    int x = int(gl_FragCoord.x);
    int y = int(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    LinkedListFragmentNode frag;
    frag.lineSegmentId = fragmentLineSegmentId;
    frag.next = -1;
    packFloat24Float8(frag.depth, gl_FragCoord.z, fragmentAttribute);

    uint insertIndex = atomicCounterIncrement(fragCounter);

    if (insertIndex < linkedListSize) {
        // Insert the fragment into the linked list
        frag.next = atomicExchange(startOffset[pixelIndex], insertIndex);
        fragmentBuffer[insertIndex] = frag;
    }
}
