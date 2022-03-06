/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#if !defined(COMPUTE_SHADER) && !defined(FRAGMENT_SHADER)
hitAttributeEXT vec2 attribs;
#endif

#define M_PI 3.14159265358979323846

float interpolateFloat(float v0, float v1, float v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}

vec3 interpolateVec3(vec3 v0, vec3 v1, vec3 v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}

/// Assumes all angles are 0 <= phi <= 2pi
float interpolateAngle(float v0, float v1, float v2, vec3 barycentricCoordinates) {
    if (v1 - v0 > M_PI || v2 - v0 > M_PI) {
        v0 += 2.0 * M_PI;
    }
    if (v0 - v1 > M_PI || v2 - v1 > M_PI) {
        v1 += 2.0 * M_PI;
    }
    if (v0 - v2 > M_PI || v1 - v2 > M_PI) {
        v2 += 2.0 * M_PI;
    }
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}
