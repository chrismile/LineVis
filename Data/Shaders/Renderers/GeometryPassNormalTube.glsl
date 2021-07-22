-- VBO.Vertex

#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexAttribute;
layout(location = 2) in vec3 vertexTangent;
layout(location = 3) in vec3 vertexNormal;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
layout(location = 4) in uint vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
layout(location = 5) in float vertexLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
layout(location = 6) in uint vertexLineAppearanceOrder;
#endif
#ifdef USE_THIN_RIBBONS_AT_DEGENERATE_POINTS
layout(location = 7) in float vertexMajorStress;
layout(location = 8) in float vertexMediumStress;
layout(location = 9) in float vertexMinorStress;
#endif

out VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    vec3 lineNormal;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
    uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    float lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    uint lineLineAppearanceOrder;
#endif
#ifdef USE_THIN_RIBBONS_AT_DEGENERATE_POINTS
    float lineMajorStress;
    float lineMediumStress;
    float lineMinorStress;
#endif
};

#include "TransferFunction.glsl"

void main() {
    linePosition = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    lineAttribute = vertexAttribute;
    lineTangent = vertexTangent;
    lineNormal = vertexNormal;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
    linePrincipalStressIndex = vertexPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    lineLineHierarchyLevel = vertexLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    lineLineAppearanceOrder = vertexLineAppearanceOrder;
#endif
#ifdef USE_THIN_RIBBONS_AT_DEGENERATE_POINTS
    lineMajorStress = vertexMajorStress;
    lineMediumStress = vertexMediumStress;
    lineMinorStress = vertexMinorStress;
#endif
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- VBO.Geometry

#version 430 core

layout(lines) in;
layout(triangle_strip, max_vertices = 32) out;

uniform vec3 cameraPosition;
uniform float lineWidth;

out vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
out vec3 screenSpacePosition;
#endif
out float fragmentAttribute;
out vec3 fragmentNormal;
out vec3 fragmentTangent;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
flat out uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
flat out float fragmentLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
flat out uint fragmentLineAppearanceOrder;
#endif

#ifdef USE_BANDS
uniform float bandWidth;
uniform ivec3 psUseBands;
flat out int useBand;
out float phi;
out float thickness;
out vec3 lineNormal;
out vec3 linePosition;
#endif

in VertexData {
    vec3 linePosition;
    float lineAttribute;
    vec3 lineTangent;
    vec3 lineNormal;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
    uint linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
    float lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    uint lineLineAppearanceOrder;
#endif
#ifdef USE_THIN_RIBBONS_AT_DEGENERATE_POINTS
    float lineMajorStress;
    float lineMediumStress;
    float lineMinorStress;
#endif
} v_in[];

# define M_PI 3.14159265358979323846

bool isDegenerate(float sigma1, float sigma2, float sigma3) {
    const float EPSILON = 1e-6;
    if (0.5 * abs((sigma1 - sigma2) / (sigma1 + sigma2)) < EPSILON) {
        return true;
    }
    if (0.5 * abs((sigma3 - sigma2) / (sigma3 + sigma2)) < EPSILON) {
        return true;
    }
    return false;
}

void main() {
    vec3 linePosition0 = (mMatrix * vec4(v_in[0].linePosition, 1.0)).xyz;
    vec3 linePosition1 = (mMatrix * vec4(v_in[1].linePosition, 1.0)).xyz;

#ifdef USE_BANDS
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(IS_PSL_DATA)
    uint principalStressIndex = v_in[0].linePrincipalStressIndex;
    useBand = psUseBands[principalStressIndex];
#else
    useBand = 1;
#endif

#ifdef BAND_RENDERING_THICK
    const float MIN_THICKNESS = 0.15;
#else
    const float MIN_THICKNESS = 1e-2;
#endif
    thickness = useBand != 0 ? MIN_THICKNESS : 1.0f;

    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;
#else
    const float lineRadius = lineWidth * 0.5;
#endif
    const mat4 pvMatrix = pMatrix * vMatrix;

    vec3 circlePointsCurrent[NUM_TUBE_SUBDIVISIONS];
    vec3 circlePointsNext[NUM_TUBE_SUBDIVISIONS];
    vec3 vertexNormalsCurrent[NUM_TUBE_SUBDIVISIONS];
    vec3 vertexNormalsNext[NUM_TUBE_SUBDIVISIONS];

    vec3 normalCurrent = v_in[0].lineNormal;
    vec3 tangentCurrent = v_in[0].lineTangent;
    vec3 binormalCurrent = cross(tangentCurrent, normalCurrent);
    vec3 normalNext = v_in[1].lineNormal;
    vec3 tangentNext = v_in[1].lineTangent;
    vec3 binormalNext = cross(tangentNext, normalNext);

#ifdef USE_BANDS
    mat3 tangentFrameMatrixCurrent = mat3(normalCurrent, binormalCurrent, tangentCurrent);
    mat3 tangentFrameMatrixNext = mat3(normalNext, binormalNext, tangentNext);
#else
    mat3 tangentFrameMatrixCurrent = mat3(normalCurrent, binormalCurrent, tangentCurrent);
    mat3 tangentFrameMatrixNext = mat3(normalNext, binormalNext, tangentNext);
#endif

#ifdef USE_BANDS
    for (int i = 0; i < NUM_TUBE_SUBDIVISIONS; i++) {
        float t = float(i) / float(NUM_TUBE_SUBDIVISIONS) * 2.0 * M_PI;
        float cosAngle = cos(t);
        float sinAngle = sin(t);
#ifdef USE_THIN_RIBBONS_AT_DEGENERATE_POINTS
        float factorCurrent = isDegenerate(v_in[0].lineMinorStress, v_in[0].lineMediumStress, v_in[0].lineMajorStress) ? thickness : 1.0;
        float factorNext = isDegenerate(v_in[1].lineMinorStress, v_in[1].lineMediumStress, v_in[1].lineMajorStress) ? thickness : 1.0;
        vec3 localPositionCurrent = vec3(cosAngle * thickness, sinAngle * factorCurrent, 0.0f);
        vec3 localNormalCurrent = vec3(cosAngle * factorCurrent, sinAngle * thickness, 0.0f);
        vec3 localPositionNext = vec3(cosAngle * thickness, sinAngle * factorNext, 0.0f);
        vec3 localNormalNext = vec3(cosAngle * factorNext, sinAngle * thickness, 0.0f);
        circlePointsCurrent[i] = lineRadius * (tangentFrameMatrixCurrent * localPositionCurrent) + linePosition0;
        circlePointsNext[i] = lineRadius * (tangentFrameMatrixNext * localPositionNext) + linePosition1;
        vertexNormalsCurrent[i] = normalize(tangentFrameMatrixCurrent * localNormalCurrent);
        vertexNormalsNext[i] = normalize(tangentFrameMatrixNext * localNormalNext);
#else
        vec3 localPosition = vec3(thickness * cosAngle, sinAngle, 0.0f);
        vec3 localNormal = vec3(cosAngle, thickness * sinAngle, 0.0f);
        circlePointsCurrent[i] = lineRadius * (tangentFrameMatrixCurrent * localPosition) + linePosition0;
        circlePointsNext[i] = lineRadius * (tangentFrameMatrixNext * localPosition) + linePosition1;
        vertexNormalsCurrent[i] = normalize(tangentFrameMatrixCurrent * localNormal);
        vertexNormalsNext[i] = normalize(tangentFrameMatrixNext * localNormal);
#endif
    }
#else
    const float theta = 2.0 * M_PI / float(NUM_TUBE_SUBDIVISIONS);
    const float tangetialFactor = tan(theta); // opposite / adjacent
    const float radialFactor = cos(theta); // adjacent / hypotenuse

    vec2 position = vec2(lineRadius, 0.0);
    for (int i = 0; i < NUM_TUBE_SUBDIVISIONS; i++) {
        vec3 point2D0 = tangentFrameMatrixCurrent * vec3(position, 0.0);
        vec3 point2D1 = tangentFrameMatrixNext * vec3(position, 0.0);
        circlePointsCurrent[i] = point2D0.xyz + linePosition0;
        circlePointsNext[i] = point2D1.xyz + linePosition1;
        vertexNormalsCurrent[i] = normalize(circlePointsCurrent[i] - linePosition0);
        vertexNormalsNext[i] = normalize(circlePointsNext[i] - linePosition1);

        // Add the tangent vector and correct the position using the radial factor.
        vec2 circleTangent = vec2(-position.y, position.x);
        position += tangetialFactor * circleTangent;
        position *= radialFactor;
    }
#endif


#ifdef USE_BANDS
    const float factor = 2.0 * M_PI / float(NUM_TUBE_SUBDIVISIONS);
#endif

    // Emit the tube triangle vertices
    for (int i = 0; i < NUM_TUBE_SUBDIVISIONS; i++) {
#ifdef USE_BANDS
        phi = float(i) * factor;
        linePosition = linePosition0;
        lineNormal = normalCurrent;
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
        fragmentPrincipalStressIndex = principalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
        fragmentLineHierarchyLevel = v_in[0].lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
        fragmentLineAppearanceOrder = v_in[0].lineLineAppearanceOrder;
#endif
        fragmentAttribute = v_in[0].lineAttribute;
        fragmentTangent = tangentCurrent;

        gl_Position = pvMatrix * vec4(circlePointsCurrent[i], 1.0);
        fragmentNormal = vertexNormalsCurrent[i];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsCurrent[i], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsCurrent[i], 1.0)).xyz;
#endif
        EmitVertex();

#ifdef USE_BANDS
        phi = float(i + 1) * factor;
#endif

        gl_Position = pvMatrix * vec4(circlePointsCurrent[(i+1)%NUM_TUBE_SUBDIVISIONS], 1.0);
        fragmentNormal = vertexNormalsCurrent[(i+1)%NUM_TUBE_SUBDIVISIONS];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsCurrent[(i+1)%NUM_TUBE_SUBDIVISIONS], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsCurrent[(i+1)%NUM_TUBE_SUBDIVISIONS], 1.0)).xyz;
#endif
        EmitVertex();


#ifdef USE_BANDS
        phi = float(i) * factor;
        linePosition = linePosition1;
        lineNormal = normalNext;
#endif

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
        fragmentPrincipalStressIndex = v_in[1].linePrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
        fragmentLineHierarchyLevel = v_in[1].lineLineHierarchyLevel;
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
        fragmentLineAppearanceOrder = v_in[1].lineLineAppearanceOrder;
#endif
        fragmentAttribute = v_in[1].lineAttribute;
        fragmentTangent = tangentNext;

        gl_Position = pvMatrix * vec4(circlePointsNext[i], 1.0);
        fragmentNormal = vertexNormalsNext[i];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsNext[i], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsNext[i], 1.0)).xyz;
#endif
        EmitVertex();

#ifdef USE_BANDS
        phi = float(i + 1) * factor;
#endif

        gl_Position = pvMatrix * vec4(circlePointsNext[(i+1)%NUM_TUBE_SUBDIVISIONS], 1.0);
        fragmentNormal = vertexNormalsNext[(i+1)%NUM_TUBE_SUBDIVISIONS];
        fragmentPositionWorld = (mMatrix * vec4(circlePointsNext[(i+1)%NUM_TUBE_SUBDIVISIONS], 1.0)).xyz;
#ifdef USE_SCREEN_SPACE_POSITION
        screenSpacePosition = (vMatrix * vec4(circlePointsNext[(i+1)%NUM_TUBE_SUBDIVISIONS], 1.0)).xyz;
#endif
        EmitVertex();

        EndPrimitive();
    }
}

-- Fragment

#version 450 core

in vec3 fragmentPositionWorld;
#ifdef USE_SCREEN_SPACE_POSITION
in vec3 screenSpacePosition;
#endif
in float fragmentAttribute;
in vec3 fragmentNormal;
in vec3 fragmentTangent;
#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL)
flat in uint fragmentPrincipalStressIndex;
#endif
#ifdef USE_LINE_HIERARCHY_LEVEL
flat in float fragmentLineHierarchyLevel;
#ifdef USE_TRANSPARENCY
uniform sampler1DArray lineHierarchyImportanceMap;
#else
uniform vec3 lineHierarchySlider;
#endif
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
flat in uint fragmentLineAppearanceOrder;
uniform int currentSeedIdx;
#endif

#if defined(DIRECT_BLIT_GATHER)
out vec4 fragColor;
#endif

uniform vec3 cameraPosition;
uniform float lineWidth;
uniform vec3 backgroundColor;
uniform vec3 foregroundColor;

#ifdef USE_BANDS
uniform float bandWidth;
flat in int useBand;
in float phi;
in float thickness;
//in mat3 tangentFrameMatrix;
in vec3 lineNormal;
in vec3 linePosition;
#endif

#define M_PI 3.14159265358979323846

#include "TransferFunction.glsl"

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"
#include "Lighting.glsl"

//#define USE_ORTHOGRAPHIC_TUBE_PROJECTION

#ifndef USE_ORTHOGRAPHIC_TUBE_PROJECTION
mat3 shearSymmetricMatrix(vec3 p) {
    return mat3(vec3(0.0, -p.z, p.y), vec3(p.z, 0.0, -p.x), vec3(-p.y, p.x, 0.0));
}
#endif


void main() {
#if defined(USE_LINE_HIERARCHY_LEVEL) && !defined(USE_TRANSPARENCY)
    float slider = lineHierarchySlider[fragmentPrincipalStressIndex];
    if (slider > fragmentLineHierarchyLevel) {
        discard;
    }
#endif
#ifdef VISUALIZE_SEEDING_PROCESS
    if (int(fragmentLineAppearanceOrder) > currentSeedIdx) {
        discard;
    }
#endif

    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 t = normalize(fragmentTangent);
    // Project v into plane perpendicular to t to get newV.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));

#ifdef USE_BANDS
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
    float ribbonPosition = x / totalDist * 2.0;
#else
    // Project onto the tangent plane.
    const vec3 cNorm = cameraPosition - linePosition;
    const float dist = dot(cNorm, fragmentTangent);
    const vec3 cHat = transpose(tangentFrameMatrix) * (cNorm - dist * fragmentTangent);
    const float lineRadius = (useBand != 0 ? bandWidth : lineWidth) * 0.5;

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

    float ribbonPosition = length(pLine - pointMax0) / length(pointMax1 - pointMax0) * 2.0 - 1.0;
#endif

#else
    // Get the symmetric ribbon position (ribbon direction is perpendicular to line direction) between 0 and 1.
    // NOTE: len(cross(a, b)) == area of parallelogram spanned by a and b.
    vec3 crossProdVn = cross(newV, n);
    float ribbonPosition = length(crossProdVn);

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


#ifdef USE_PRINCIPAL_STRESS_DIRECTION_INDEX
    vec4 fragmentColor = transferFunction(fragmentAttribute, fragmentPrincipalStressIndex);
#else
    vec4 fragmentColor = transferFunction(fragmentAttribute);
#endif

#if defined(USE_LINE_HIERARCHY_LEVEL) && defined(USE_TRANSPARENCY)
    fragmentColor.a *= texture(
            lineHierarchyImportanceMap, vec2(fragmentLineHierarchyLevel, float(fragmentPrincipalStressIndex))).r;
#endif

    fragmentColor = blinnPhongShading(fragmentColor, fragmentNormal);

    float absCoords = abs(ribbonPosition);
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
#ifdef USE_BANDS
    float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / (useBand != 0 ? bandWidth : lineWidth), 0.0, 0.49);
    float EPSILON_WHITE = fwidth(ribbonPosition);
#else
    float EPSILON_OUTLINE = clamp(fragmentDepth * 0.0005 / lineWidth, 0.0, 0.49);
    float EPSILON_WHITE = fwidth(ribbonPosition);
#endif
    float coverage = 1.0 - smoothstep(1.0 - EPSILON_OUTLINE, 1.0, absCoords);
    //float coverage = 1.0 - smoothstep(1.0, 1.0, abs(ribbonPosition));
    vec4 colorOut = vec4(mix(fragmentColor.rgb, fragmentColor.rgb,
            smoothstep(WHITE_THRESHOLD - EPSILON_WHITE, WHITE_THRESHOLD + EPSILON_WHITE, absCoords)),
            fragmentColor.a * coverage);

    //colorOut = vec4(vec3(absCoords), 1.0);

#if defined(DIRECT_BLIT_GATHER)
    // To counteract depth fighting with overlay wireframe.
    float depthOffset = -0.00001;
    if (absCoords >= WHITE_THRESHOLD - EPSILON_WHITE) {
        depthOffset = 0.002;
    }
    //gl_FragDepth = clamp(gl_FragCoord.z + depthOffset, 0.0, 0.999);
    gl_FragDepth = convertLinearDepthToDepthBufferValue(
            convertDepthBufferValueToLinearDepth(gl_FragCoord.z) + fragmentDepth
            - length(fragmentPositionWorld - cameraPosition) - 0.0001);
#ifdef LOW_OPACITY_DISCARD
    if (colorOut.a < 0.01) {
        discard;
    }
#endif
    colorOut.a = 1.0;
    fragColor = colorOut;
#elif defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK)
    // Area of mutual exclusion for fragments mapping to the same pixel
    beginInvocationInterlockARB();
    gatherFragment(colorOut);
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
                gatherFragment(colorOut);
                memoryBarrier();
                atomicExchange(spinlockViewportBuffer[pixelIndex], 0);
                keepWaiting = false;
            }
        }
    }
#else
    gatherFragment(colorOut);
#endif
}
