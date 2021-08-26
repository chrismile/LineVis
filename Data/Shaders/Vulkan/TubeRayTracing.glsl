-- RayGen

#version 460
#extension GL_EXT_ray_tracing : require

#ifdef USE_JITTERED_RAYS
#include "RayTracingUtilities.glsl"
#endif

layout (binding = 0) uniform CameraSettingsBuffer {
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
} camera;

layout(binding = 1, rgba8) uniform image2D outputImage;

layout(binding = 2) uniform accelerationStructureEXT topLevelAS;

layout (binding = 3) uniform RayTracerSettingsBuffer {
    vec3 cameraPosition;
    uint maxDepthComplexity;
    vec4 backgroundColor;
    vec4 foregroundColor;

    uvec3 paddingVec;

    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;
};

layout (binding = 4) uniform LineRenderSettingsBuffer {
    float lineWidth;
    int hasHullMesh;
    float depthCueStrength;
    float ambientOcclusionStrength;

    // Ambient occlusion settings.
    uint numAoTubeSubdivisions;
    uint numLineVertices;
    uint numParametrizationVertices;
    uint paddingLineSettings;
};

layout (binding = 9) uniform HullRenderSettingsBuffer {
    vec4 color;
    ivec3 padding;
    int useShading;
} hullRenderSettings;

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadEXT RayPayload payload;

// Minimum distance between two consecutive hits.
const float HIT_DISTANCE_EPSILON = 1e-5;

#define USE_TRANSPARENCY

vec4 traceRayOpaque(vec3 rayOrigin, vec3 rayDirection) {
    float tMin = 0.0001f;
    float tMax = 1000.0f;

    traceRayEXT(topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0);

    return payload.hitColor;
}

vec4 traceRayTransparent(vec3 rayOrigin, vec3 rayDirection) {
    vec4 fragmentColor = vec4(0.0);

    float tMin = 0.0001f;
    float tMax = 1000.0f;

    for (uint hitIdx = 0; hitIdx < maxDepthComplexity; hitIdx++) {
        traceRayEXT(topLevelAS, gl_RayFlagsOpaqueEXT, 0xFF, 0, 0, 0, rayOrigin, tMin, rayDirection, tMax, 0); // gl_RayFlagsNoneEXT

        tMin = payload.hitT + HIT_DISTANCE_EPSILON;

        // Front-to-back blending (hitColor uses post-multiplied, fragmentColor uses pre-multiplied alpha).
        fragmentColor.rgb = fragmentColor.rgb + (1.0 - fragmentColor.a) * payload.hitColor.a * payload.hitColor.rgb;
        fragmentColor.a = fragmentColor.a + (1.0 - fragmentColor.a) * payload.hitColor.a;

        if (!payload.hasHit || fragmentColor.a > 0.99) {
            break;
        }
    }

    return fragmentColor;
}

void main() {
    ivec2 outputImageSize = imageSize(outputImage);

#ifdef USE_JITTERED_RAYS
    uint seed = tea(gl_LaunchIDEXT.x + gl_LaunchIDEXT.y * outputImageSize.x, frameNumber);
    vec2 xi = vec2(rnd(seed), rnd(seed));
    vec2 fragNdc = 2.0 * ((vec2(gl_LaunchIDEXT.xy) + xi) / vec2(gl_LaunchSizeEXT.xy)) - 1.0;
#else
    vec2 fragNdc = 2.0 * ((vec2(gl_LaunchIDEXT.xy) + vec2(0.5)) / vec2(gl_LaunchSizeEXT.xy)) - 1.0;
#endif

    vec3 rayOrigin = (camera.inverseViewMatrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec3 rayTarget = (camera.inverseProjectionMatrix * vec4(fragNdc.xy, 1.0, 1.0)).xyz;
    vec3 rayDirection = (camera.inverseViewMatrix * vec4(normalize(rayTarget.xyz), 0.0)).xyz;


    vec4 fragmentColor;

#ifdef USE_TRANSPARENCY
    fragmentColor = traceRayTransparent(rayOrigin, rayDirection);
#else
    if (hasHullMesh == 1) {
        fragmentColor = traceRayTransparent(rayOrigin, rayDirection);
    } else {
        fragmentColor = traceRayOpaque(rayOrigin, rayDirection);
    }
#endif

    ivec2 writePos = ivec2(gl_LaunchIDEXT.xy);
    writePos.y = outputImageSize.y - writePos.y - 1;
    if (frameNumber != 0) {
        vec4 fragmentColorPrev = imageLoad(outputImage, writePos);
        fragmentColor = mix(fragmentColorPrev, fragmentColor, 1.0 / float(frameNumber + 1));
    }
    imageStore(outputImage, writePos, fragmentColor);
}


-- Miss

#version 460
#extension GL_EXT_ray_tracing : require

layout (binding = 3) uniform RayTracerSettingsBuffer {
    vec3 cameraPosition;
    uint maxDepthComplexity;
    vec4 backgroundColor;
    vec4 foregroundColor;

    uvec3 paddingVec;

    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;
};

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadInEXT RayPayload payload;

void main() {
    payload.hitColor = backgroundColor;
    payload.hitT = 0.0f;
    payload.hasHit = false;
}


-- ClosestHit

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "BarycentricInterpolation.glsl"
#include "TransferFunction.glsl"

layout (binding = 0) uniform CameraSettingsBuffer {
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
} camera;

layout (binding = 3) uniform RayTracerSettingsBuffer {
    vec3 cameraPosition;
    uint maxDepthComplexity;
    vec4 backgroundColor;
    vec4 foregroundColor;

    uvec3 paddingVec;

    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;
};

layout (binding = 4) uniform LineRenderSettingsBuffer {
    float lineWidth;
    int hasHullMesh;
    float depthCueStrength;
    float ambientOcclusionStrength;

    // Ambient occlusion settings.
    uint numAoTubeSubdivisions;
    uint numLineVertices;
    uint numParametrizationVertices;
    uint paddingLineSettings;
};

#ifdef STRESS_LINE_DATA
layout (binding = 5) uniform StressLineRenderSettingsBuffer {
    vec3 lineHierarchySlider;
    float bandWidth;
    ivec3 psUseBands;
    int currentSeedIdx;
};
#endif

struct TubeTriangleVertexData {
    vec3 vertexPosition;
    uint vertexLinePointIndex; ///< Pointer to TubeLinePointData entry.
    vec3 vertexNormal;
    float phi; ///< Angle.
};

struct TubeLinePointData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    float lineHierarchyLevel; ///< Zero for flow lines.
    vec3 lineNormal;
    float lineAppearanceOrder; ///< Zero for flow lines.
    uvec3 padding;
    uint principalStressIndex; ///< Zero for flow lines.
};

layout(scalar, binding = 6) readonly buffer TubeIndexBuffer {
    uvec3 indexBuffer[];
};

layout(std430, binding = 7) readonly buffer TubeTriangleVertexDataBuffer {
    TubeTriangleVertexData tubeTriangleVertexDataBuffer[];
};

layout(std430, binding = 8) readonly buffer TubeLinePointDataBuffer {
    TubeLinePointData tubeLinePointDataBuffer[];
};

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadInEXT RayPayload payload;

#define RAYTRACING
#include "Lighting.glsl"

//#define USE_ORTHOGRAPHIC_TUBE_PROJECTION

#ifndef USE_ORTHOGRAPHIC_TUBE_PROJECTION
mat3 shearSymmetricMatrix(vec3 p) {
    return mat3(vec3(0.0, -p.z, p.y), vec3(p.z, 0.0, -p.x), vec3(-p.y, p.x, 0.0));
}
#endif

void main() {
    uvec3 triangleIndices = indexBuffer[gl_PrimitiveID];
    const vec3 barycentricCoordinates = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    TubeTriangleVertexData vertexData0 = tubeTriangleVertexDataBuffer[triangleIndices.x];
    TubeTriangleVertexData vertexData1 = tubeTriangleVertexDataBuffer[triangleIndices.y];
    TubeTriangleVertexData vertexData2 = tubeTriangleVertexDataBuffer[triangleIndices.z];

#ifdef USE_CAPPED_TUBES
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex & 0x7FFFFFFFu;
    TubeLinePointData linePointData0 =  tubeLinePointDataBuffer[vertexLinePointIndex0];
    TubeLinePointData linePointData1 =  tubeLinePointDataBuffer[vertexLinePointIndex1];
    TubeLinePointData linePointData2 =  tubeLinePointDataBuffer[vertexLinePointIndex2];
    bool isCap =
            bitfieldExtract(vertexData0.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData1.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData2.vertexLinePointIndex, 31, 1) > 0u;
#else
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex;
    TubeLinePointData linePointData0 =  tubeLinePointDataBuffer[vertexLinePointIndex0];
    TubeLinePointData linePointData1 =  tubeLinePointDataBuffer[vertexLinePointIndex1];
    TubeLinePointData linePointData2 =  tubeLinePointDataBuffer[vertexLinePointIndex2];
#endif

    vec3 fragmentPositionWorld = interpolateVec3(
            vertexData0.vertexPosition, vertexData1.vertexPosition, vertexData2.vertexPosition, barycentricCoordinates);
    vec3 fragmentNormal = interpolateVec3(
            vertexData0.vertexNormal, vertexData1.vertexNormal, vertexData2.vertexNormal, barycentricCoordinates);
    fragmentNormal = normalize(fragmentNormal);
    vec3 fragmentTangent = interpolateVec3(
            linePointData0.lineTangent, linePointData1.lineTangent, linePointData2.lineTangent, barycentricCoordinates);
    fragmentTangent = normalize(fragmentTangent);
    float fragmentAttribute = interpolateFloat(
            linePointData0.lineAttribute, linePointData1.lineAttribute, linePointData2.lineAttribute,
            barycentricCoordinates);

    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 t = normalize(fragmentTangent);

#if defined (STRESS_LINE_DATA) || defined(USE_AMBIENT_OCCLUSION)
    float phi = interpolateAngle(
            vertexData0.phi, vertexData1.phi, vertexData2.phi, barycentricCoordinates);
#endif
#ifdef USE_AMBIENT_OCCLUSION
     float fragmentVertexId = interpolateFloat(
            float(vertexLinePointIndex0), float(vertexLinePointIndex1), float(vertexLinePointIndex2),
            barycentricCoordinates);
#endif

#ifdef STRESS_LINE_DATA
    bool useBand = psUseBands[linePointData0.principalStressIndex] > 0;
    const float thickness = useBand ? 0.15 : 1.0; // hard-coded

    vec3 linePosition = interpolateVec3(
    linePointData0.linePosition, linePointData1.linePosition, linePointData2.linePosition, barycentricCoordinates);
    vec3 lineNormal = interpolateVec3(
    linePointData0.lineNormal, linePointData1.lineNormal, linePointData2.lineNormal, barycentricCoordinates);
#endif

    float ribbonPosition;

#ifdef USE_CAPPED_TUBES
    if (isCap) {
        vec3 crossProdVn = cross(v, n);
        ribbonPosition = length(crossProdVn);


        // Project v into plane perpendicular to t to get newV.
        vec3 helperVec = normalize(cross(t, v));
        vec3 newV = normalize(cross(helperVec, t));

        // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
        // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
        vec3 crossProdVn2 = cross(newV, n);
        float ribbonPosition2 = length(crossProdVn2);

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
            ribbonPosition2 = -ribbonPosition2;
        }
        // Normalize the ribbon position: [-1, 1] -> [0, 1].
        //ribbonPosition = ribbonPosition / 2.0 + 0.5;
        ribbonPosition2 = clamp(ribbonPosition2, -1.0, 1.0);

        ribbonPosition = min(ribbonPosition, abs(ribbonPosition2));
    } else {
#endif

#ifdef STRESS_LINE_DATA
    vec3 lineN = normalize(lineNormal);
    vec3 lineB = cross(t, lineN);
    mat3 tangentFrameMatrix = mat3(lineN, lineB, t);

#ifdef USE_ORTHOGRAPHIC_TUBE_PROJECTION
    vec2 localV = normalize((transpose(tangentFrameMatrix) * newV).xy);
    vec2 p = vec2(thickness * cos(phi), sin(phi));
    float d = length(p);
    p = normalize(p);
    float alpha = acos(dot(localV, p));

    float phiMax = atan(localV.x, -thickness * localV.y);

    vec2 pointMax0 = vec2(thickness * cos(phiMax), sin(phiMax));
    vec2 pointMax1 = vec2(thickness * cos(phiMax + M_PI), sin(phiMax + M_PI));
    vec2 planeDir = pointMax1 - pointMax0;
    float totalDist = length(planeDir);
    planeDir = normalize(planeDir);

    float beta = acos(dot(planeDir, localV));

    float x = d / sin(beta) * sin(alpha);
    ribbonPosition = x / totalDist * 2.0;
#else
    // Project onto the tangent plane.
    const vec3 cNorm = cameraPosition - linePosition;
    const float dist = dot(cNorm, fragmentTangent);
    const vec3 cHat = transpose(tangentFrameMatrix) * (cNorm - dist * fragmentTangent);
    const float lineRadius = (useBand ? bandWidth : lineWidth) * 0.5;

    // Homogeneous, normalized coordinates of the camera position in the tangent plane.
    const vec3 c = vec3(cHat.xy / lineRadius, 1.0);

    // Primal conic section matrix.
    //const mat3 A = mat3(
    //        1.0 / (thickness*thickness), 0.0, 0.0,
    //        0.0, 1.0, 0.0,
    //        0.0, 0.0, -1.0
    //);

    // Polar of c.
    //const vec3 l = A * c;

    // Polar of c.
    const float a = 1.0 / (thickness*thickness);
    const vec3 l = vec3(a * c.x, c.y, -1.0);

    const mat3 M_l = shearSymmetricMatrix(l);
    //const mat3 B = transpose(M_l) * A * M_l;

    const mat3 B = mat3(
            l.z*l.z - l.y*l.y, l.x*l.y, -l.x*l.z,
            l.x*l.y, a*l.z*l.z - l.x*l.x, -a*l.y*l.z,
            -l.x*l.z, -a*l.y*l.z, a*l.y*l.y + l.x*l.x
    );

    const float EPSILON = 1e-4;
    float alpha = 0.0;
    float discr = 0.0;
    if (abs(l.z) > EPSILON) {
        discr = -B[0][0] * B[1][1] + B[0][1] * B[1][0];
        alpha = sqrt(discr) / l.z;
    } else if (abs(l.y) > EPSILON) {
        discr = -B[0][0] * B[2][2] + B[0][2] * B[2][0];
        alpha = sqrt(discr) / l.y;
    } else if (abs(l.x) > EPSILON) {
        discr = -B[1][1] * B[2][2] + B[1][2] * B[2][1];
        alpha = sqrt(discr) / l.x;
    }

    mat3 C = B + alpha * M_l;

    vec2 pointMax0 = vec2(0.0);
    vec2 pointMax1 = vec2(0.0);
    for (int i = 0; i < 2; ++i) {
        if (abs(C[i][i]) > EPSILON) {
            pointMax0 = C[i].xy / C[i].z; // column vector
            pointMax1 = vec2(C[0][i], C[1][i]) / C[2][i]; // row vector
        }
    }

    vec2 p = vec2(thickness * cos(phi), sin(phi));

    vec3 pLineHomogeneous = cross(l, cross(c, vec3(p, 1.0)));
    vec2 pLine = pLineHomogeneous.xy / pLineHomogeneous.z;

    ribbonPosition = length(pLine - pointMax0) / length(pointMax1 - pointMax0) * 2.0 - 1.0;
#endif

#else
    // Project v into plane perpendicular to t to get newV.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));

    // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
    // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
    vec3 crossProdVn = cross(newV, n);
    ribbonPosition = length(crossProdVn);

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
    //ribbonPosition = ribbonPosition / 2.0 + 0.5;
    ribbonPosition = clamp(ribbonPosition, -1.0, 1.0);
#endif

#ifdef USE_CAPPED_TUBES
    }
#endif

#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    vec4 fragmentColor = transferFunction(fragmentAttribute, linePointData0.principalStressIndex);
#else
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#endif

#ifdef USE_DEPTH_CUES
    vec3 screenSpacePosition = (camera.viewMatrix * vec4(fragmentPositionWorld, 1.0)).xyz;
#endif

    fragmentColor = blinnPhongShadingTube(
            fragmentColor, fragmentPositionWorld,
#ifdef USE_DEPTH_CUES
            screenSpacePosition,
#endif
#ifdef USE_AMBIENT_OCCLUSION
            fragmentVertexId, phi,
#endif
#ifdef STRESS_LINE_DATA
            useBand ? 1 : 0,
#endif
            n, t);

    float absCoords = abs(ribbonPosition);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
#ifdef STRESS_LINE_DATA
    float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / (useBand ? bandWidth : lineWidth), 0.0, 0.49);
    float EPSILON_WHITE = 1e-5; // TODO
#else
    float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / lineWidth, 0.0, 0.49);
    float EPSILON_WHITE = 1e-5; // TODO
#endif
    float coverage = 1.0 - smoothstep(1.0 - EPSILON_OUTLINE, 1.0, absCoords);
    vec4 colorOut = vec4(mix(fragmentColor.rgb, foregroundColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON_WHITE, WHITE_THRESHOLD + EPSILON_WHITE, absCoords)),
            fragmentColor.a * coverage);

#ifdef USE_AMBIENT_OCCLUSION
    //colorOut = vec4(getAoFactor(fragmentVertexId, phi), 0.0, 0.0, 1.0);
#endif

    payload.hitColor = colorOut;
    payload.hitT = length(fragmentPositionWorld - cameraPosition);
    payload.hasHit = true;
}


-- ClosestHitHull

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

#include "BarycentricInterpolation.glsl"

struct HullVertex {
    vec3 vertexPosition;
    float padding0;
    vec3 vertexNormal;
    float padding1;
};

layout (binding = 3) uniform RayTracerSettingsBuffer {
    vec3 cameraPosition;
    uint maxDepthComplexity;
    vec4 backgroundColor;
    vec4 foregroundColor;

    uvec3 paddingVec;

    // The number of this frame (used for accumulation of samples across frames).
    uint frameNumber;
};

layout (binding = 9) uniform HullRenderSettingsBuffer {
    vec4 color;
    ivec3 padding;
    int useShading;
};

layout(scalar, binding = 10) readonly buffer HullIndexBuffer {
    uvec3 hullIndexBuffer[];
};

layout(std430, binding = 11) readonly buffer HullTriangleVertexDataBuffer {
    HullVertex hullVertices[];
};

struct RayPayload {
    vec4 hitColor;
    float hitT;
    bool hasHit;
};
layout(location = 0) rayPayloadInEXT RayPayload payload;

#define RAYTRACING
#ifdef USE_DEPTH_CUES
#undef USE_DEPTH_CUES
#endif
#ifdef USE_AMBIENT_OCCLUSION
#undef USE_AMBIENT_OCCLUSION
#endif
#include "Lighting.glsl"

void main() {
    uvec3 triangleIndices = hullIndexBuffer[gl_PrimitiveID];
    const vec3 barycentricCoordinates = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    HullVertex vertex0 = hullVertices[triangleIndices.x];
    HullVertex vertex1 = hullVertices[triangleIndices.y];
    HullVertex vertex2 = hullVertices[triangleIndices.z];

    vec3 fragmentPositionWorld = interpolateVec3(
            vertex0.vertexPosition, vertex1.vertexPosition, vertex2.vertexPosition, barycentricCoordinates);
    vec3 fragmentNormal = interpolateVec3(
            vertex0.vertexNormal, vertex1.vertexNormal, vertex2.vertexNormal, barycentricCoordinates);
    fragmentNormal = normalize(fragmentNormal);

    vec4 phongColor;
    if (useShading == 1) {
        phongColor = blinnPhongShading(color, fragmentPositionWorld, fragmentNormal);
    } else {
        phongColor = color;
    }

    vec3 viewDir = normalize(cameraPosition - fragmentPositionWorld);
    float cosAngle = dot(fragmentNormal, viewDir);
    if (cosAngle < 0.0f) {
        cosAngle *= -1.0f;
    }
    phongColor.a *= 1.0f - cosAngle;

    payload.hitColor = phongColor;
    payload.hitT = length(fragmentPositionWorld - cameraPosition);
    payload.hasHit = true;
}
