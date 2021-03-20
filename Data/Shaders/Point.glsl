-- Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;

out VertexData {
    vec3 pointPosition;
};

void main() {
    pointPosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Geometry

#version 430 core

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec3 cameraPosition;
uniform float pointWidth;

out vec3 fragmentPositionWorld;
out vec2 quadCoords; // Between -1 and 1

in VertexData {
    vec3 pointPosition;
} v_in[];

void main()
{
    vec3 pointPosition = v_in[0].pointPosition;

    vec3 quadNormal = normalize(cameraPosition - pointPosition);
    vec3 vertexPosition;

    vec3 right = cross(quadNormal, vec3(0, 1, 0));
    vec3 top = cross(quadNormal, right);

    vertexPosition = pointPosition + pointWidth / 2.0 * (right - top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(1, -1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + pointWidth / 2.0 * (right + top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(1, 1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + pointWidth / 2.0 * (-right - top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(-1, -1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    vertexPosition = pointPosition + pointWidth / 2.0 * (-right + top);
    fragmentPositionWorld = vertexPosition;
    quadCoords = vec2(-1, 1);
    gl_Position = pMatrix * vMatrix * vec4(vertexPosition, 1.0);
    EmitVertex();

    EndPrimitive();
}

-- Fragment

#version 430 core

in vec3 fragmentPositionWorld;
in vec2 quadCoords; // Between -1 and 1
out vec4 fragColor;

uniform vec4 pointColor;
uniform vec3 foregroundColor;
uniform vec3 cameraPosition;

void main()
{
    float lengthCoords = length(quadCoords);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth / 2.0, 0.0, 0.49);
    //float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lengthCoords);
    float coverage = 1.0 - step(1.0, lengthCoords);

    if (coverage < 0.999) {
        discard;
    }

    fragColor = vec4(mix(pointColor.rgb, foregroundColor,
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lengthCoords)),
            pointColor.a * coverage);
}

