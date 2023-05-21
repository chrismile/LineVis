/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2023, Christoph Neuhauser
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

layout(location = 0) in vec3 vertexPosition;

void main()
{
    gl_Position = vec4(vertexPosition, 1.0);
}


-- Fragment

#version 450 core

#extension GL_EXT_control_flow_attributes : require

// Use early z-test to cull transparent fragments occluded by opaque fragments.
layout(early_fragment_tests) in;

layout(std430, binding = 0) coherent buffer FragmentNodes {
    uvec2 nodes[];
};

layout(binding = 1) uniform UniformDataBuffer {
    // Size of the viewport in x direction (in pixels).
    int viewportW;
};

#include "TiledAddress.glsl"

layout(location = 0) out vec4 fragColor;

void main() {
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    // Read data from SSBO
    vec3 color = vec3(0.0, 0.0, 0.0);
    float transmittance = 1.0;
    [[unroll]] for (uint i = 0; i < MAX_NUM_LAYERS; i++) {
        // Blend the accumulated color with the color of the fragment node
        uint idx = pixelIndex * MAX_NUM_LAYERS + i;
        uvec2 node = nodes[idx];
        if (node.y != 0xFFFFFFFFu) {
            vec4 colorSrc = unpackUnorm4x8(node.x);
            color.rgb = color.rgb + transmittance * colorSrc.a * colorSrc.rgb;
            transmittance *= 1.0 - colorSrc.a;
            // Clear data for the next rendering pass.
            nodes[idx] = uvec2(0xFFFFFFFFu, 0xFFFFFFFFu);
        } else {
            break;
        }
    }

    float alphaOut = 1.0 - transmittance;
    fragColor = vec4(color.rgb / alphaOut, alphaOut);
}
