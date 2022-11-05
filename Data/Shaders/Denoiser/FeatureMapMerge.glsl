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

-- Compute

#version 460
#extension GL_EXT_scalar_block_layout : enable

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;

#ifdef USE_COLOR
layout(binding = 0, rgba32f) uniform readonly image2D colorMap;
#endif

#ifdef USE_ALBEDO
layout(binding = 1, rgba32f) uniform readonly image2D albedoMap;
#endif

#ifdef USE_NORMAL
layout(binding = 2, rgba32f) uniform readonly image2D normalMap;
#endif

#ifdef USE_DEPTH
layout(binding = 3, r32f) uniform readonly image2D depthMap;
#endif

#ifdef USE_POSITION
layout(binding = 4, rgba32f) uniform readonly image2D positionMap;
#endif

#ifdef USE_FLOW
layout(binding = 5, rg32f) uniform readonly image2D flowMap;
#endif

layout(scalar, binding = 6) writeonly buffer OutputBuffer {
    float outputBuffer[];
};

layout(binding = 7) uniform UniformBuffer {
    uint numChannelsOut;
    uint colorWriteStartOffset;
    uint albedoWriteStartOffset;
    uint normalWriteStartOffset;
    uint depthWriteStartOffset;
    uint positionWriteStartOffset;
    uint flowWriteStartOffset;
    uint padding;
};

// Flattens the vec4 image to a vec3 buffer without padding for use, e.g., with OptiX.
void main() {
#if defined(USE_COLOR)
    ivec2 inputImageSize = imageSize(colorMap);
#elif defined(USE_ALBEDO)
    ivec2 inputImageSize = imageSize(albedoMap);
#elif defined(USE_NORMAL)
    ivec2 inputImageSize = imageSize(normalMap);
#elif defined(USE_DEPTH)
    ivec2 inputImageSize = imageSize(depthMap);
#elif defined(USE_POSITION)
    ivec2 inputImageSize = imageSize(positionMap);
#elif defined(USE_FLOW)
    ivec2 inputImageSize = imageSize(flowMap);
#endif

    uint channelsCounter = 0;
    if (gl_GlobalInvocationID.x >= inputImageSize.x || gl_GlobalInvocationID.y >= inputImageSize.y) {
        return;
    }
    ivec2 readPos = ivec2(gl_GlobalInvocationID.xy);
    uint writePos = (gl_GlobalInvocationID.x + gl_GlobalInvocationID.y * inputImageSize.x) * numChannelsOut;

#ifdef USE_COLOR
    vec4 colorInput = imageLoad(colorMap, readPos);
    uint writePosColor = writePos + colorWriteStartOffset;
    outputBuffer[writePosColor] = colorInput.x;
    outputBuffer[writePosColor + 1] = colorInput.y;
    outputBuffer[writePosColor + 2] = colorInput.z;
    outputBuffer[writePosColor + 3] = colorInput.w;
#endif

#ifdef USE_ALBEDO
    vec4 albedoInput = imageLoad(albedoMap, readPos);
    uint writePosAlbedo = writePos + albedoWriteStartOffset;
    outputBuffer[writePosAlbedo] = albedoInput.x;
    outputBuffer[writePosAlbedo + 1] = albedoInput.y;
    outputBuffer[writePosAlbedo + 2] = albedoInput.z;
    outputBuffer[writePosAlbedo + 3] = albedoInput.w;
#endif

#ifdef USE_NORMAL
    vec3 normalInput = imageLoad(normalMap, readPos).xyz;
    uint writePosNormal = writePos + normalWriteStartOffset;
    outputBuffer[writePosNormal] = normalInput.x;
    outputBuffer[writePosNormal + 1] = normalInput.y;
    outputBuffer[writePosNormal + 2] = normalInput.z;
#endif

#ifdef USE_NORMAL
    float depthInput = imageLoad(depthMap, readPos).x;
    uint writePosDepth = writePos + depthWriteStartOffset;
    outputBuffer[writePosDepth] = depthInput;
#endif

#ifdef USE_POSITION
    vec3 positionInput = imageLoad(positionMap, readPos).xyz;
    uint writePosPosition = writePos + positionWriteStartOffset;
    outputBuffer[writePosPosition] = positionInput.x;
    outputBuffer[writePosPosition + 1] = positionInput.y;
    outputBuffer[writePosPosition + 2] = positionInput.z;
#endif

#ifdef USE_FLOW
    vec2 flowInput = imageLoad(flowMap, readPos).xy;
    uint writePosFlow = writePos + flowWriteStartOffset;
    outputBuffer[writePosFlow] = flowInput.x;
    outputBuffer[writePosFlow + 1] = flowInput.y;
#endif
}
