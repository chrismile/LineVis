/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2022, Christoph Neuhauser
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

layout (location = 0) in vec3 vertexPosition;
layout (location = 1) in vec2 vertexTexCoord;
layout(location = 0) out vec2 fragTexCoord;

void main() {
    fragTexCoord = vertexTexCoord;
    gl_Position = vec4(vertexPosition, 1.0);
}

-- Fragment

#version 450 core

#define KERNEL_SIZE 64

// Values used are proposed by https://learnopengl.com/Advanced-Lighting/SSAO
layout(binding = 0) uniform SamplesBuffer {
    vec4 samples[KERNEL_SIZE];
};

layout(binding = 1) uniform SettingsBuffer {
    float radius; // = 0.05;
    float bias; // = 0.005;
};

layout(binding = 2) uniform sampler2D positionTexture;
layout(binding = 3) uniform sampler2D normalTexture;
layout(binding = 4) uniform sampler2D rotationVectorTexture;

layout(location = 0) in vec2 fragTexCoord;

layout(location = 0) out float fragColor;

void main() {
    vec3 fragPosition = texture(positionTexture, fragTexCoord).xyz;
    float fragDepth = fragPosition.z;
    vec3 normal = normalize(texture(normalTexture, fragTexCoord).rgb);

    vec2 rotationVecTexScale = textureSize(positionTexture, 0) / textureSize(rotationVectorTexture, 0);
    vec2 rotationVecSamplePos = fragTexCoord * rotationVecTexScale.xy;
    vec3 rotationVec = normalize(texture(rotationVectorTexture, rotationVecSamplePos).xyz);

    // Create basis change matrix converting tangent space to view space.
    vec3 tangent = normalize(rotationVec - normal * dot(rotationVec, normal));
    vec3 bitangent = cross(normal, tangent);
    mat3 frameMatrix = mat3(tangent, bitangent, normal);

    // Compute occlusion factor as occlusion average over all kernel samples.
    float occlusion = 0.0;
    for (int i = 0; i < KERNEL_SIZE; i++) {
        // Convert sample position from tangent space to view space.
        vec3 sampleViewSpace = fragPosition + frameMatrix * samples[i].xyz * radius;

        // Apply projection matrix to view space sample to get position in clip space.
        vec4 screenSpacePosition = vec4(sampleViewSpace, 1.0);
        screenSpacePosition = pMatrix * screenSpacePosition;
        screenSpacePosition.xyz /= screenSpacePosition.w;
        screenSpacePosition.xyz = screenSpacePosition.xyz * 0.5 + 0.5; // From NDC [-1,1] to [0,1].

        // Get depth at sample position (of kernel sample).
        float sampleDepth = texture(positionTexture, screenSpacePosition.xy).z;

        // Range check: Make sure only depth differences in the radius contribute to occlusion.
        float rangeCheck = smoothstep(0.0, 1.0, radius / abs(fragDepth - sampleDepth));

        // Check if the sample contributes to occlusion.
        occlusion += (sampleDepth >= sampleViewSpace.z + bias ? 1.0 : 0.0) * rangeCheck;
    }

    fragColor = 1.0 - (occlusion / KERNEL_SIZE);
}
