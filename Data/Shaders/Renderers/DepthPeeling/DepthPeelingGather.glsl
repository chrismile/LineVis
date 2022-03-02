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

#define DEPTH_HELPER_USE_PROJECTION_MATRIX
#include "DepthHelper.glsl"

layout(push_constant) uniform PushConstants {
    int iteration;
};

layout(binding = 0) uniform sampler2D depthReadTexture;

in vec4 gl_FragCoord;

layout(location = 0) out vec4 fragColor;

void gatherFragment(vec4 color) {
    float previousDepthValue = texelFetch(depthReadTexture, ivec2(gl_FragCoord.xy), 0).x;
    if (gl_FragCoord.z <= previousDepthValue && iteration != 0) {
        discard;
    }

    fragColor = vec4(color.rgb * color.a, color.a);
}

void gatherFragmentCustomDepth(vec4 color, float depth) {
    float previousDepthValue = texelFetch(depthReadTexture, ivec2(gl_FragCoord.xy), 0).x;
    if (convertLinearDepthToDepthBufferValue(depth) <= previousDepthValue && iteration != 0) {
        discard;
    }

    fragColor = vec4(color.rgb * color.a, color.a);
}
