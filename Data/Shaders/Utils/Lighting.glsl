/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2021, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if defined(USE_DEPTH_CUES) && !defined(MESH_HULL)
layout (std430, binding = DEPTH_MIN_MAX_BUFFER_BINDING) readonly buffer DepthMinMaxBuffer {
    float minDepth;
    float maxDepth;
};
#endif

#if defined(USE_AMBIENT_OCCLUSION) && defined(GEOMETRY_PASS_TUBE) && !defined(MESH_HULL)
#include "AmbientOcclusion.glsl"
#endif

/**
 * Simplified Blinn-Phong shading assuming the ambient and diffuse color are equal and the specular color is white.
 * Assumes the following global variables are given: cameraPosition, fragmentPositionWorld, fragmentNormal.
 * The camera position is assumed to be the source of a point light.
*/
vec4 blinnPhongShading(
        in vec4 baseColor,
#if defined(VULKAN_RAY_TRACER) || defined(VOXEL_RAY_CASTING) || defined(DEFERRED_SHADING)
        in vec3 fragmentPositionWorld,
#ifdef USE_DEPTH_CUES
        in vec3 screenSpacePosition,
#endif
#endif
        in vec3 fragmentNormal) {
    // Blinn-Phong Shading
    const vec3 lightColor = vec3(1.0);
    const vec3 ambientColor = baseColor.rgb;
    const vec3 diffuseColor = ambientColor;
    vec3 phongColor = vec3(0.0);

#ifdef USE_BANDS
    const float kA = 0.1;
    const vec3 Ia = kA * ambientColor;
    const float kD = 0.9;
    const float kS = 0.3;
    const float s = 30;
#else
    const float kA = 0.1;//0.2;
    const vec3 Ia = kA * ambientColor;
    const float kD = 1.0;//0.7;
    const float kS = 0.3;//0.1;
    const float s = 50;//10;
#endif

    const vec3 n = normalize(fragmentNormal);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 l = v;//normalize(lightDirection);
    const vec3 h = normalize(v + l);

    vec3 Id = kD * clamp(abs(dot(n, l)), 0.0, 1.0) * diffuseColor;
    vec3 Is = kS * pow(clamp(abs(dot(n, h)), 0.0, 1.0), s) * lightColor;

    phongColor = Ia + Id + Is;

#if defined(USE_DEPTH_CUES) && !defined(ANALYTIC_HIT_COMPUTATION)
    float depthCueFactor = clamp((-screenSpacePosition.z - minDepth) / (maxDepth - minDepth), 0.0, 1.0);
    depthCueFactor = depthCueFactor * depthCueFactor * depthCueStrength;
    phongColor = mix(phongColor, vec3(0.5, 0.5, 0.5), depthCueFactor);
#endif

    vec4 color = vec4(phongColor, baseColor.a);
    return color;
}

#if !defined(MESH_HULL)
/**
 * Simplified Blinn-Phong shading assuming the ambient and diffuse color are equal and the specular color is white.
 * Assumes the following global variables are given: cameraPosition, fragmentPositionWorld, fragmentNormal.
 * The camera position is assumed to be the source of a point light.
*/
vec4 blinnPhongShadingTube(
        in vec4 baseColor,
#if defined(VULKAN_RAY_TRACER) || defined(VOXEL_RAY_CASTING) || defined(DEFERRED_SHADING)
        in vec3 fragmentPositionWorld,
#if defined(USE_DEPTH_CUES) || (defined(USE_AMBIENT_OCCLUSION) && !defined(STATIC_AMBIENT_OCCLUSION_PREBAKING))
        in vec3 screenSpacePosition,
#endif
#ifdef USE_AMBIENT_OCCLUSION
        in float fragmentVertexId,
        in float phi,
#endif
#if defined(USE_BANDS) && (defined(VULKAN_RAY_TRACER) || defined(DEFERRED_SHADING))
        in int useBand,
#endif
#endif
        in vec3 fragmentNormal,
        in vec3 fragmentTangent) {
    // Blinn-Phong Shading
    const vec3 lightColor = vec3(1.0);
    const vec3 ambientColor = baseColor.rgb;
    const vec3 diffuseColor = ambientColor;
    vec3 phongColor = vec3(0.0);

#if defined(USE_AMBIENT_OCCLUSION) && defined(GEOMETRY_PASS_TUBE)
#if defined(STATIC_AMBIENT_OCCLUSION_PREBAKING)
    float ambientOcclusionFactor = getAoFactor(fragmentVertexId, phi);
#elif defined(VOXEL_BASED_AMBIENT_OCCLUSION)
    float ambientOcclusionFactor = getAoFactor(fragmentPositionWorld);
#else
    float ambientOcclusionFactor = getAoFactor(screenSpacePosition);
#endif
    //float ambientFactor = (1.0 - ambientOcclusionStrength) * 0.75;
    const float kA = 0.2 + (1.0 - ambientOcclusionFactor) * 0.5;
    const vec3 Ia = kA * ambientColor;
    const float kD = 0.9 * ambientOcclusionFactor;
    const float kS = 0.3;
    const float s = 30;
#else
    const float kA = 0.1;
    const vec3 Ia = kA * ambientColor;
    const float kD = 0.9;
    const float kS = 0.3;
    const float s = 30;
#endif

    const vec3 n = normalize(fragmentNormal);
    const vec3 t = normalize(fragmentTangent);
    const vec3 v = normalize(cameraPosition - fragmentPositionWorld);
    const vec3 l = v;//normalize(lightDirection);
    const vec3 h = normalize(v + l);

    vec3 helperVec = normalize(cross(t, l));
    vec3 newL = normalize(cross(helperVec, t));

#if defined(USE_BANDS)
    const float exponent = useBand == 0 ? 1.7 : 1.0;
#else
    const float exponent = 1.7;
#endif
    float cosNormal1 = pow(clamp(abs(dot(n, l)), 0.0, 1.0), exponent);
    float cosNormal2 = pow(clamp(abs(dot(n, newL)), 0.0, 1.0), exponent);
    float cosNormalCombined = 0.3 * cosNormal1 + 0.7 * cosNormal2;

    vec3 Id = kD * cosNormalCombined * diffuseColor;
    vec3 Is = kS * pow(clamp(abs(dot(n, h)), 0.0, 1.0), s) * lightColor;

    phongColor = Ia + Id + Is;

#if defined(USE_AMBIENT_OCCLUSION) && defined(GEOMETRY_PASS_TUBE)
#ifdef STATIC_AMBIENT_OCCLUSION_PREBAKING
    phongColor *= ambientOcclusionFactor;
#else
    phongColor *= ambientOcclusionFactor;
#endif
#endif

#ifdef USE_DEPTH_CUES
    float depthCueFactor = clamp((-screenSpacePosition.z - minDepth) / (maxDepth - minDepth), 0.0, 1.0);
    depthCueFactor = depthCueFactor * depthCueFactor * depthCueStrength;
    phongColor = mix(phongColor, vec3(0.5, 0.5, 0.5), depthCueFactor);
#endif

    vec4 color = vec4(phongColor, baseColor.a);
    return color;
}
#endif
