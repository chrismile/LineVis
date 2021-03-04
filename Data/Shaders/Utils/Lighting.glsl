#ifdef USE_DEPTH_CUES
uniform float minDepth = 0.0f;
uniform float maxDepth = 1.0f;
#endif

/**
 * Simplified Blinn-Phong shading assuming the ambient and diffuse color are equal and the specular color is white.
 * Assumes the following global variables are given: cameraPosition, fragmentPositionWorld, fragmentNormal.
 * The camera position is assumed to be the source of a point light.
*/
vec4 blinnPhongShading(in vec4 baseColor, in vec3 fragmentNormal) {
    // Blinn-Phong Shading
    const vec3 lightColor = vec3(1,1,1);
    const vec3 ambientColor = baseColor.rgb;
    const vec3 diffuseColor = ambientColor;
    vec3 phongColor = vec3(0);

    const float kA = 0.2;
    const vec3 Ia = kA * ambientColor;
    const float kD = 0.7;
    const float kS = 0.1;
    const float s = 10;

    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 l = v;//normalize(lightDirection);
    const vec3 h = normalize(v + l);

    vec3 Id = kD * clamp(abs(dot(n, l)), 0.0, 1.0) * diffuseColor;
    vec3 Is = kS * pow(clamp(abs(dot(n, h)), 0.0, 1.0), s) * lightColor;

    phongColor = Ia + Id + Is;

#ifdef USE_DEPTH_CUES
    float depthCueFactor = (-screenSpacePosition.z - minDepth) / (maxDepth - minDepth);
    depthCueFactor = depthCueFactor * 0.7;
    depthCueFactor = depthCueFactor*depthCueFactor;
    phongColor = mix(phongColor, vec3(0.5, 0.5, 0.5), depthCueFactor);
#endif

    vec4 color = vec4(phongColor, baseColor.a);
    return color;
}

#ifdef TUBE_HALO_LIGHTING
/**
 * Simplified Blinn-Phong shading for tubes assuming the ambient and diffuse color are equal and the specular color is
 * white. It adds a halo at the outline of the tube. Assumes the following global variables are given:
 * cameraPosition, fragmentPositionWorld, fragmentNormal, fragmentTangent.
 * The camera position is assumed to be the source of a point light.
*/
vec4 blinnPhongShadingTubeHalo(in vec4 baseColor) {
    // Blinn-Phong Shading
    const vec3 lightColor = vec3(1,1,1);
    const vec3 ambientColor = baseColor.rgb;
    const vec3 diffuseColor = ambientColor;
    vec3 phongColor = vec3(0);

    const float kA = 0.2;
    const vec3 Ia = kA * ambientColor;
    const float kD = 0.7;
    const float kS = 0.1;
    const float s = 10;

    const vec3 n = normalize(fragmentNormal);
    const vec3 t = normalize(fragmentTangent);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 l = v;//normalize(lightDirection);
    const vec3 h = normalize(v + l);

    vec3 Id = kD * clamp(abs(dot(n, l)), 0.0, 1.0) * diffuseColor;
    vec3 Is = kS * pow(clamp(abs(dot(n, h)), 0.0, 1.0), s) * lightColor;

    float haloParameter = 1.0;
    float angle1 = abs(dot(v, n));
    float angle2 = abs(dot(v, t)) * 0.7;
    float halo = clamp(mix(1.0, angle1 + angle2, haloParameter), 0.0, 1.0);

    phongColor = Ia + Id + Is;
    phongColor *= halo;

    vec4 color = vec4(phongColor, baseColor.a);
    return color;
}

const float PI = 3.1415926535897932f;
const float TWO_PI = PI * 2.0f;
const float HALF_PI = PI / 2.0f;

/**
 * Flat shading, but adds a constant-sized halo at the outline of the tube. Assumes the following global variables are
 * given: cameraPosition, fragmentPositionWorld, fragmentNormal, fragmentTangent.
*/
vec4 flatShadingTubeHalo(in vec4 baseColor, out float fragmentDepthFrag) {
    const vec3 n = normalize(fragmentNormal);
    const vec3 t = normalize(fragmentTangent);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);

    // n lies in the orthonormal space of t. Thus, project v onto this plane orthogonal to t to compute the angle to n.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));

    float angle = abs(acos(dot(newV, n)));

    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.65;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, angle);
    vec4 color = vec4(mix(baseColor.rgb, vec3(1.0, 1.0, 1.0),
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, angle)), baseColor.a * coverage);

    // To counteract depth fighting with overlay wireframe.
    float depthOffset = -0.00001;
    if (angle >= WHITE_THRESHOLD - EPSILON) {
        fragmentDepth += 0.005;
    }
    fragmentDepthFrag = fragmentDepth;

    return color;
}

/**
 * Flat shading, but adds a constant-sized halo at the outline of the tube. Assumes the following global variables are
 * given: cameraPosition, fragmentPositionWorld, fragmentNormal, fragmentTangent.
*/
vec4 flatShadingTubeTronHalo(in vec4 baseColor, out float fragmentDepthFrag) {
    const vec3 n = normalize(fragmentNormal);
    const vec3 t = normalize(fragmentTangent);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);

    // n lies in the orthonormal space of t. Thus, project v onto this plane orthogonal to t to compute the angle to n.
    vec3 helperVec = normalize(cross(t, v));
    vec3 newV = normalize(cross(helperVec, t));

    float angle = abs(acos(dot(newV, n)));

    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float CORE_THRESHOLD = 0.4;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);

    const float GLOW_STRENGTH = 0.5;
    vec4 colorSolid = vec4(baseColor.rgb, 1.0 - smoothstep(CORE_THRESHOLD - EPSILON, CORE_THRESHOLD + EPSILON, angle));
    vec4 colorGlow = vec4(baseColor.rgb, GLOW_STRENGTH * (1.0 - smoothstep(0.0, 1.2, angle)));

    // Back-to-front blending:
    float a_out = colorGlow.a + colorSolid.a * (1.0 - colorGlow.a);
    vec3 c_out = (colorGlow.rgb * colorGlow.a + colorSolid.rgb * colorSolid.a) / a_out;

    fragmentDepthFrag = fragmentDepth;

    return vec4(c_out, a_out);
}

#endif


#ifdef WIREFRAME_SURFACE_HALO_LIGHTING

/**
 * Flat shading, but adds a constant-sized halo at the outline of the surface. Assumes the following global variables
 * are given: cameraPosition, fragmentPositionWorld.
*/
vec4 flatShadingWireframeSurfaceHalo(in vec4 baseColor, out float fragmentDepthFrag, in float lineCoordinates) {
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth / 2.0, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates);
    vec4 color = vec4(mix(baseColor.rgb, vec3(1.0, 1.0, 1.0),
    smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), baseColor.a * coverage);

    // To counteract depth fighting with overlay wireframe.
    float depthOffset = -0.00001;
    if (lineCoordinates >= WHITE_THRESHOLD - EPSILON) {
        fragmentDepth += 0.005;
    }
    fragmentDepthFrag = fragmentDepth;

    return color;
}

/**
 * Flat shading, but adds a constant-sized halo at the outline of the surface. Assumes the following global variables
 * are given: cameraPosition, fragmentPositionWorld.
*/
vec4 flatShadingWireframeSurfaceTronHalo(in vec4 baseColor, out float fragmentDepthFrag, in float lineCoordinates) {
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    const float CORE_THRESHOLD = 0.4;
    float EPSILON = clamp(fragmentDepth, 0.0, 0.49);

    const float GLOW_STRENGTH = 0.5;
    vec4 colorSolid = vec4(baseColor.rgb,
            1.0 - smoothstep(CORE_THRESHOLD - EPSILON, CORE_THRESHOLD + EPSILON, lineCoordinates));
    vec4 colorGlow = vec4(baseColor.rgb, GLOW_STRENGTH * (1.0 - smoothstep(0.0, 1.2, lineCoordinates)));

    // Back-to-front blending:
    float a_out = colorGlow.a + colorSolid.a * (1.0 - colorGlow.a);
    vec3 c_out = (colorGlow.rgb * colorGlow.a + colorSolid.rgb * colorSolid.a) / a_out;

    fragmentDepthFrag = fragmentDepth;

    return vec4(c_out, a_out);
}

/**
* Normal flat shading.
*/
vec4 flatShadingWireframeSingleColor(in vec4 baseColor, out float fragmentDepthFrag, in float lineCoordinates) {
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);
    fragmentDepthFrag = fragmentDepth;

    float EPSILON = clamp(fragmentDepth / 2.0, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates);
    vec4 color = vec4(baseColor.rgb, baseColor.a * coverage);

    return color;
}


#ifdef USE_CLEAR_VIEW

#define DEPTH_CUE_MAIN_COLOR
#define DEPTH_CUE_OUTLINE_COLOR
#define DEPTH_CUE_LINE_WIDTH

/**
* Flat shading, but adds a constant-sized halo at the outline of the surface. Assumes the following global variables
* are given: cameraPosition, fragmentPositionWorld.
*/
vec4 flatShadingWireframeSurfaceHalo_DepthCue(in vec4 baseColor, out float fragmentDepthFrag, in float lineCoordinates) {
    float fragmentDepth = length(fragmentPositionWorld - cameraPosition);

    float distanceToFocusPointNormalized = length(fragmentPositionWorld - sphereCenter) / sphereRadius;
    float depthCueFactorFocus = clamp(pow(distanceToFocusPointNormalized, 1.9), 0.0, 1.0);
    float depthCueFactorDistance = clamp(fragmentDepth, 0.0, 1.0) * 0.002 / lineWidth; //  * 5.0

    #ifdef DEPTH_CUE_MAIN_COLOR
    baseColor.rgb = mix(baseColor.rgb, vec3(0.5, 0.5, 0.5), depthCueFactorFocus * 0.6);
    #endif

    // Fade out the outline with increasing distance to the viewer and increasing distance to the focus center.
    vec3 outlineColor = vec3(1.0, 1.0, 1.0);
    #ifdef DEPTH_CUE_OUTLINE_COLOR
    outlineColor = mix(outlineColor, baseColor.rgb, clamp(max(depthCueFactorFocus, depthCueFactorDistance), 0.0, 1.0));
    #endif

    //#ifdef DEPTH_CUE_LINE_WIDTH
    //lineCoordinates /= -distanceToFocusPointNormalized * 0.6 + 1.0;
    //#endif

    // Fade out the outline with increasing distance
    const float WHITE_THRESHOLD = 0.7;
    float EPSILON = clamp(fragmentDepth / 2.0, 0.0, 0.49);
    float coverage = 1.0 - smoothstep(1.0 - 2.0*EPSILON, 1.0, lineCoordinates);
    vec4 color = vec4(mix(baseColor.rgb, outlineColor,
            smoothstep(WHITE_THRESHOLD - EPSILON, WHITE_THRESHOLD + EPSILON, lineCoordinates)), baseColor.a * coverage);

    if (lineCoordinates >= WHITE_THRESHOLD - EPSILON) {
        fragmentDepth += 0.005;// * (lineCoordinates - WHITE_THRESHOLD + EPSILON) / (lineCoordinates - WHITE_THRESHOLD + EPSILON);
    }
    fragmentDepthFrag = fragmentDepth;

    return color;
}
#endif

#endif