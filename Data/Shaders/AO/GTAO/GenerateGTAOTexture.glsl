/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2018 - 2022, Christoph Neuhauser, Felix Brendel
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

layout (location = 0) out vec2 fragTexCoord;

void main() {
    fragTexCoord = vertexTexCoord;
    gl_Position = vec4(vertexPosition, 1.0);
}

-- Fragment

#version 450 core

#define PI              3.1415926535897932
#define HALF_PI         1.5707963267948966


layout(binding = 1) uniform sampler2D positionTexture;
layout(binding = 2) uniform sampler2D normalTexture;
layout(binding = 3) uniform sampler2D depthTexture;


layout(push_constant) uniform PushConstants  {
    vec4 projInfo;
    /*  projInfo = {
     *      2.0f / (gbuffer->GetWidth()  * proj._11),
     *      2.0f / (gbuffer->GetHeight() * proj._22),
     *     -1.0f / proj._11,
     *     -1.0f / proj._22
     *  };
     */
    vec4 clipInfo;
    /*  clipInfo = {
     *     camera.GetNearPlane(),
     *     camera.GetFarPlane(),
     *     0.5f * (gbuffer->GetHeight() / (2.0f * tanf(camera.GetFov() * 0.5f)))
     *  };
     */
    vec3 eyePos;
    float _padding;

    uint  sliceCount;
    uint  directionSampleCount;
    float scaling;
};

layout(location = 0) out float output_ao;

vec3 GetViewPosition(vec2 uv) {
    vec3 pos = texelFetch(positionTexture, ivec2(round(uv * vec2(textureSize(positionTexture, 0)))), 0).xyz;
    if (pos.z == 0)
        pos.z = -1000;
    return pos;
}

void main()
{
    vec2  cTexCoord = (vec2(gl_FragCoord.xy)+0.001) / vec2(textureSize(positionTexture, 0));
    vec3  normalV   = texture(normalTexture, cTexCoord, 0).rgb;

    vec3  cPosV     = GetViewPosition(cTexCoord);
    vec3  viewV     = normalize(-cPosV);
    float visibility = 0;

    for (int slice = 0; slice < sliceCount; ++slice) {
        float phi = (PI/sliceCount) * slice;
        vec2 omega = vec2(cos(phi), sin(phi));

        vec3 directionV = vec3(omega, 0);
        vec3 orthoDirectionV = directionV - dot(directionV, viewV) * viewV;
        vec3 axisV = cross(directionV, viewV);
        vec3 projNormalV = normalV - axisV * dot(normalV, axisV);

        float sgnN = sign(dot(orthoDirectionV, projNormalV));
        float cosN = clamp(dot(projNormalV, viewV)/length(projNormalV), 0, 1);
        float n = sgnN * acos(cosN);

        for (int side = 0; side <= 1; ++side) {
            float cHorizonCos = -1;

            for (int smpl = 0; smpl < directionSampleCount; ++smpl) {
                float s = (1.0f*smpl) / directionSampleCount;
                vec2 sTexCoord = cTexCoord + (-1+ 2*side) * s * scaling/abs(cPosV.z) *  vec2(omega.x, -omega.y);
                vec3 sPosV = GetViewPosition(sTexCoord);
                vec3 sHorizonV = normalize(sPosV - cPosV);
                cHorizonCos = max(cHorizonCos, dot(sHorizonV, viewV));
            }

            float h_side = n + clamp((-1 + 2*side)*acos(cHorizonCos) - n, -HALF_PI, HALF_PI);
            visibility = visibility + length(projNormalV) * (cosN + 2*h_side * sin(n) - cos(2*h_side - n)) / 4;

        }
    }

    output_ao = visibility / sliceCount;
}
