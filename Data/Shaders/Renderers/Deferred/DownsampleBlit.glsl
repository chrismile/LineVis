/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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
layout(location = 1) in vec2 vertexTexCoord;
layout(location = 0) out vec2 fragTexCoord;

void main() {
    fragTexCoord = vertexTexCoord;
    gl_Position = vec4(vertexPosition, 1.0);
}

-- Fragment

#version 450 core

layout(location = 0) in vec2 fragTexCoord;
layout(location = 0) out vec4 fragColor;

layout(binding = 0) uniform sampler2D inputTexture;

layout(push_constant) uniform PushConstants {
    int scalingFactor;
};

void main() {
    ivec2 inputSize = textureSize(inputTexture, 0);
    ivec2 outputSize = inputSize / scalingFactor;
    ivec2 readCoords = ivec2(int(fragTexCoord.x * outputSize.x), int(fragTexCoord.y * outputSize.y));
    ivec2 writeCoords = readCoords * scalingFactor;
    vec4 color = vec4(0.0);
    vec4 totalSum = vec4(0.0);
    int numSamples = scalingFactor * scalingFactor;
    for (int sampleIdx = 0; sampleIdx < numSamples; sampleIdx++) {
        int x = sampleIdx % scalingFactor;
        int y = sampleIdx / scalingFactor;
        vec4 fetchedColor = texelFetch(inputTexture, writeCoords + ivec2(x, y), 0);
        totalSum += fetchedColor;
        color.rgb += fetchedColor.rgb * fetchedColor.a;
        color.a += fetchedColor.a;
    }
    color /= float(numSamples);
    //fragColor = vec4(color.rgb, color.a);
    if (color.a > 1.0 / 256.0) {
        fragColor = vec4(color.rgb / color.a, color.a);
    } else {
        fragColor = totalSum / float(numSamples);
    }
}
