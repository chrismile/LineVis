/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2026, Christoph Neuhauser
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

layout(location = 0) in vec4 vertexPosition;

void main() {
    gl_Position = vertexPosition;
}


-- Fragment

#version 450 core

layout(binding = 0) uniform MotionVectorPassUniformData {
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
    mat4 lastFrameViewProjectionMatrix;
    vec2 viewportSize;
};

/*
 * While storing only the primitive index, thus resulting in an actual visibility buffer [Burns and Hunt 2013], might
 * result in lower memory usage, storing also a depth buffer might result in performance benefits due to allowing for
 * early depth testing.
 *
 * Burns, Christopher A. and Hunt, Warren A. (2013). The Visibility Buffer: A Cache-Friendly Approach to Deferred
 * Shading. Journal of Computer Graphics Techniques, 2(2):55-69.
 */
layout(binding = 1) uniform usampler2D primitiveIndexBuffer; //< Clear value: 0xFFFFFFFFu
layout(binding = 2) uniform sampler2D depthBuffer;

#ifdef USE_RESPONSIVE_PIXEL_MASK
layout(binding = 3) uniform sampler2D depthBufferLastFrame;
#endif

in vec4 gl_FragCoord;
layout(location = 0) out vec2 motionVectorsOut;

#ifdef USE_RESPONSIVE_PIXEL_MASK
// 1.0: reject history, 0.0: use history
layout(location = 1) out float responsivePixelMask;
#endif

void main() {
    ivec2 fragCoord = ivec2(gl_FragCoord.xy);
    uint primitiveIndex = texelFetch(primitiveIndexBuffer, fragCoord, 0).x;

    // Clear value of primitive index, i.e., no primitive maps to this pixel?
    if (primitiveIndex == 0xFFFFFFFFu) {
        motionVectorsOut = vec2(0.0);
#ifdef USE_RESPONSIVE_PIXEL_MASK
        responsivePixelMask = 1.0;
#endif
        return;
    }

    float fragmentDepth = texelFetch(depthBuffer, fragCoord, 0).x;
    vec4 fragPosNdc = vec4(2.0 * gl_FragCoord.xy / vec2(viewportSize) - vec2(1.0), fragmentDepth, 1.0);
    vec4 fragPosView = inverseProjectionMatrix * fragPosNdc;
    vec4 fragPosWorld = inverseViewMatrix * fragPosView;

    vec3 fragmentPositionWorld = fragPosWorld.xyz / fragPosWorld.w;

    vec4 lastFramePositionNdc = lastFrameViewProjectionMatrix * vec4(fragmentPositionWorld, 1.0);
    lastFramePositionNdc.xyz /= lastFramePositionNdc.w;
    vec2 pixelPositionLastFrame = (0.5 * lastFramePositionNdc.xy + vec2(0.5)) * vec2(viewportSize);
    vec2 backwardMotionVectors = pixelPositionLastFrame - gl_FragCoord.xy; // new -> old (backward motion vectors)
    motionVectorsOut = vec2(backwardMotionVectors.x, backwardMotionVectors.y); // Upscalers use top left as (0, 0)

#ifdef USE_RESPONSIVE_PIXEL_MASK
    float surfaceDepthLastFrame = lastFramePositionNdc.z;
    float actualPixelDepthLastFrame = texture(depthBufferLastFrame, 0.5 * lastFramePositionNdc.xy + vec2(0.5), 0).x;
    // Check if a disocclusion occurred and write 1 in this case.
    // surfaceDepthLastFrame < actualPixelDepthLastFrame - EPSILON => disocclusion
    // <=> surfaceDepthLastFrame - actualPixelDepthLastFrame + EPSILON < 0 => write 1
    // Apply logic from https://registry.khronos.org/OpenGL-Refpages/gl4/html/step.xhtml: x < edge => 0
    const float EPSILON = 1e-3;
    responsivePixelMask = step(0.0, actualPixelDepthLastFrame - surfaceDepthLastFrame - EPSILON);
#endif
}
