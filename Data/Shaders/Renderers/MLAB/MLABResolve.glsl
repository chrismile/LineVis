/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2021, Christoph Neuhauser
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

#version 430 core

layout(location = 0) in vec3 vertexPosition;

void main()
{
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}


-- Fragment

#version 430 core

#include "MLABHeader.glsl"
#include "ColorPack.glsl"
#include "TiledAddress.glsl"

layout(location = 0) out vec4 fragColor;

void main() {
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));

    memoryBarrierBuffer();

    // Read data from SSBO
    MLABFragmentNode nodeArray[MAX_NUM_LAYERS+1];
    loadFragmentNodes(pixelIndex, nodeArray);

    // Read data from SSBO
    vec3 color = vec3(0.0, 0.0, 0.0);
    float transmittance = 1.0;
    [[unroll]] for (uint i = 0; i < MAX_NUM_LAYERS; i++) {
        // Blend the accumulated color with the color of the fragment node
        vec4 colorSrc = unpackUnorm4x8(nodeArray[i].premulColor);
        color.rgb = color.rgb + transmittance * colorSrc.rgb;
        transmittance *= colorSrc.a;
    }

    // Make sure data is cleared for next rendering pass
    clearPixel(pixelIndex);

    float alphaOut = 1.0 - transmittance;
    fragColor = vec4(color.rgb / alphaOut, alphaOut);
    //fragColor = vec4(vec3(alphaOut), 1.0); // Output opacity
}
