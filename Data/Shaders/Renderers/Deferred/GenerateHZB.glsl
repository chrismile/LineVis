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

layout(binding = 1) uniform sampler2D inputTexture;

in vec4 gl_FragCoord;
layout(location = 0) in vec2 fragTexCoord;

/*
 * For more details see: https://www.rastergrid.com/blog/2010/10/hierarchical-z-map-based-occlusion-culling/
 */
void main() {
    float val0 = texture(inputTexture, fragTexCoord).x;
    float val1 = textureOffset(inputTexture, fragTexCoord, ivec2(-1,  0)).x;
    float val2 = textureOffset(inputTexture, fragTexCoord, ivec2(-1, -1)).x;
    float val3 = textureOffset(inputTexture, fragTexCoord, ivec2( 0, -1)).x;
    float maxDepth = max(max(val0, val1), max(val2, val3));

    ivec2 lastMipSize = textureSize(inputTexture, 0);

    // Odd-width texture?
    if ((lastMipSize.x & 1) != 0 && int(gl_FragCoord.x) == lastMipSize.x - 3) {
        // Odd-height texture?
        if ((lastMipSize.y & 1) != 0 && int(gl_FragCoord.y) == lastMipSize.y - 3) {
            float depthCornerTopLeft = textureOffset(inputTexture, fragTexCoord, ivec2(1, 1)).x;
            maxDepth = max(maxDepth, depthCornerTopLeft);
        }
        float depthEdge0 = textureOffset(inputTexture, fragTexCoord, ivec2(1,  0)).x;
        float depthEdge1 = textureOffset(inputTexture, fragTexCoord, ivec2(1, -1)).x;
        maxDepth = max(maxDepth, max(depthEdge0, depthEdge1));
    } else if ((lastMipSize.y & 1) != 0 && int(gl_FragCoord.y) == lastMipSize.y - 3) {
        // Odd-height texture.
        float depthEdge0 = textureOffset(inputTexture, fragTexCoord, ivec2( 0, 1)).x;
        float depthEdge1 = textureOffset(inputTexture, fragTexCoord, ivec2(-1, 1)).x;
        maxDepth = max(maxDepth, max(depthEdge0, depthEdge1));
    }

    gl_FragDepth = maxDepth;
}
