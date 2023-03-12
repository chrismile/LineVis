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


/*
uint num_directions  32
#define NUM_STEPS       16
#define RADIUS          0.01         // in world space

 */

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

    uint  num_directions;
    uint  num_steps;
    float radius;
};

layout(location = 0) out float output_ao;

vec4 GetViewPosition(vec2 i_pos) {
    // vec2 basesize = vec2(textureSize(depthTexture, 0));
    // vec2 coord = (i_pos / basesize);

    // float d = texture(depthTexture, coord).r;
    // vec4 ret = vec4(0.0, 0.0, 0.0, d);

    // ret.z = clipInfo.x + d * (clipInfo.y - clipInfo.x);
    // ret.xy = (i_pos * projInfo.xy + projInfo.zw) * ret.z;

    // return ret;

    vec2 basesize = vec2(textureSize(positionTexture, 0));
    vec2 coord = (i_pos / basesize);

    vec4 pos = texture(positionTexture, coord).xyzw;
    pos.z = -pos.z;
    if (pos.z == 0)
        pos.z = clipInfo.y;


    return pos;

    // vec4 p = vec4(texture(positionTexture, uv).xyz - eyePos, 1);
    // p.z = -p.z;
    // return p;
}

void main()
{
    ivec2 loc   = ivec2(gl_FragCoord.xy);
    vec4  vpos  = GetViewPosition(gl_FragCoord.xy);
    vec3  vnorm = texelFetch(normalTexture, loc, 0).rgb;

    if (vnorm == vec3(0)) {
        output_ao = 1;
        return;
    }

    vec4 s;
    vec3 vdir   = normalize(-vpos.xyz);
    vec3 dir, ws;

    // calculation uses left handed system
    vnorm.z = -vnorm.z;
    vnorm.y = -vnorm.y;

    vec2 offset;
    vec2 horizons = vec2(-1.0, -1.0);

    float ss_radius = (radius * clipInfo.z) / vpos.z;
    ss_radius = max(num_steps, ss_radius);

    float stepsize  = ss_radius / num_steps;
    float phi       = 0.0;
    float ao        = 0.0;
    float currstep  = 1.0;
    float dist2, invdist, falloff, cosh;

    for (int k = 0; k < num_directions; ++k) {
        phi = float(k) * (PI / num_directions);
        currstep = 1.0;

        dir = vec3(cos(phi), sin(phi), 0.0);
        horizons = vec2(-1.0);

        // calculate horizon angles
        for (int j = 0; j < num_steps; ++j) {
            offset = round(dir.xy * currstep);

            // h1
            s = GetViewPosition(gl_FragCoord.xy + offset);
            ws = s.xyz - vpos.xyz;

            dist2 = dot(ws, ws);
            invdist = inversesqrt(dist2);
            cosh = invdist * dot(ws, vdir);

            horizons.x = max(horizons.x, cosh);

            // h2
            s = GetViewPosition(gl_FragCoord.xy - offset);
            ws = s.xyz - vpos.xyz;

            dist2 = dot(ws, ws);
            invdist = inversesqrt(dist2);
            cosh = invdist * dot(ws, vdir);

            horizons.y = max(horizons.y, cosh);

            // increment
            currstep += stepsize;
        }

        horizons = acos(horizons);

        // calculate gamma
        vec3 bitangent  = normalize(cross(dir, vdir));
        vec3 tangent    = cross(vdir, bitangent);
        vec3 nx             = vnorm - bitangent * dot(vnorm, bitangent);

        float nnx       = length(nx);
        float invnnx    = 1.0 / (nnx + 1e-6);           // to avoid division with zero
        float cosxi         = dot(nx, tangent) * invnnx;    // xi = gamma + HALF_PI
        float gamma         = acos(cosxi) - HALF_PI;
        float cosgamma  = dot(nx, vdir) * invnnx;
        float singamma2     = -2.0 * cosxi;                     // cos(x + HALF_PI) = -sin(x)
        // float singamma2     = 2.0*sin(gamma);                     // cos(x + HALF_PI) = -sin(x)

        // clamp to normal hemisphere
        horizons.x = gamma + max(-horizons.x - gamma, -HALF_PI);
        horizons.y = gamma + min(horizons.y - gamma, HALF_PI);

        // Riemann integral is additive
        ao += nnx * 0.25 * (
            (horizons.x * singamma2 + cosgamma - cos(2.0 * horizons.x - gamma)) +
            (horizons.y * singamma2 + cosgamma - cos(2.0 * horizons.y - gamma)));
    }

    // PDF = 1 / pi and must normalize with pi because of Lambert
    ao = ao / float(num_directions);

    output_ao = ao;
}
