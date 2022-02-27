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

-- Vertex

#version 450 core

#include "LineUniformData.glsl"

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;

layout(location = 0) out vec3 fragmentPositionWorld;
layout(location = 1) out vec3 fragmentNormal;

void main() {
    fragmentNormal = vertexNormal;
    fragmentPositionWorld = (mMatrix * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Fragment

#version 450 core

#include "LineUniformData.glsl"

layout(location = 0) in vec3 fragmentPositionWorld;
layout(location = 1) in vec3 fragmentNormal;

#if defined(DIRECT_BLIT_GATHER)
layout(location = 0) out vec4 fragColor;
#endif

#if !defined(DIRECT_BLIT_GATHER)
#include OIT_GATHER_HEADER
#endif

#ifdef USE_DEPTH_CUES
#undef USE_DEPTH_CUES
#endif
#define MESH_HULL
#include "Lighting.glsl"

void main() {
    vec4 phongColor;
    if (hullUseShading == 1) {
        phongColor = blinnPhongShading(hullColor, fragmentNormal);
    } else {
        phongColor = hullColor;
    }

    vec3 viewDir = normalize(cameraPosition - fragmentPositionWorld);
    float cosAngle = dot(fragmentNormal, viewDir);
    if (cosAngle < 0.0f) {
        cosAngle *= -1.0;
    }
    phongColor.a *= 1.0 - cosAngle;

#if defined(DIRECT_BLIT_GATHER)
    // Direct rendering, no transparency.
    fragColor = phongColor.rgba;
    gl_FragDepth = gl_FragCoord.z + 1e-6;
#elif defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK)
    // Area of mutual exclusion for fragments mapping to the same pixel
    beginInvocationInterlockARB();
    gatherFragment(phongColor);
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
                gatherFragment(phongColor);
                memoryBarrier();
                atomicExchange(spinlockViewportBuffer[pixelIndex], 0);
                keepWaiting = false;
            }
        }
    }
#else
    gatherFragment(phongColor);
#endif
}
