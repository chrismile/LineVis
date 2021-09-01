/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020 - 2021, Christoph Neuhauser
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

// Compresses RGBA color into 32-bit unsigned integer (8 bits per channel)
uint packColorRGBA(vec4 vecColor)
{
    uint packedColor;
    packedColor = uint(round(clamp(vecColor.r, 0.0, 1.0) * 255.0)) & 0xFFu;
    packedColor |= (uint(round(clamp(vecColor.g, 0.0, 1.0) * 255.0)) & 0xFFu) << 8;
    packedColor |= (uint(round(clamp(vecColor.b, 0.0, 1.0) * 255.0)) & 0xFFu) << 16;
    packedColor |= (uint(round(clamp(vecColor.a, 0.0, 1.0) * 255.0)) & 0xFFu) << 24;
    return packedColor;
}

// Decompression equivalent to function above
vec4 unpackColorRGBA(uint packedColor)
{
    vec4 vecColor;
    vecColor.r = float(packedColor & 0xFFu) / 255.0;
    vecColor.g = float((packedColor >> 8)  & 0xFFu) / 255.0;
    vecColor.b = float((packedColor >> 16) & 0xFFu) / 255.0;
    vecColor.a = float((packedColor >> 24) & 0xFFu) / 255.0;
    return vecColor;
}

// Decompression equivalent (only alpha!) to function above
float unpackColorAlpha(uint packedColor)
{
    return float((packedColor >> 24) & 0xFFu) / 255.0;
}

// Replace alpha with new value. Mainly used in Adaptive Transparency
// to update alpha of packed color without decompressing.
void updatePackedColorAlpha(inout uint packedColor, in float newAlpha)
{
    packedColor &= 0xFFFFFFu;
    packedColor |= (uint(round(clamp(newAlpha, 0.0, 1.0) * 255.0)) & 0xFFu) << 24;
}



// --- Functions below used for Hybrid Transparency algorithm ---

// Compresses HDR RGB color with 10 bits per channel (last two bits unused)
uint packColor30bit(vec4 vecColor)
{
    uint packedColor;
    packedColor = uint(round(vecColor.r * 1023.0)) & 0x3FFu;
    packedColor |= (uint(round(vecColor.g * 1023.0)) & 0x3FFu) << 10;
    packedColor |= (uint(round(vecColor.b * 1023.0)) & 0x3FFu) << 20;
    return packedColor;
}

// Decompression equivalent to function above
vec4 unpackColor30bit(uint packedColor)
{
    vec4 vecColor;
    vecColor.r = float(packedColor & 0x3FFu) / 1023.0;
    vecColor.g = float((packedColor >> 10)  & 0x3FFu) / 1023.0;
    vecColor.b = float((packedColor >> 20) & 0x3FFu) / 1023.0;
    vecColor.a = 1.0;
    return vecColor;
}

// Packs accumulated alpha into lower two bytes, fragment count into higher two bytes
uint packAccumAlphaAndFragCount(float accumAlpha, uint fragCount)
{
    uint packedValue = uint(round(accumAlpha * 255.0)) & 0xFFFFu;
    packedValue |= (fragCount & 0xFFFFu) << 16;
    return packedValue;
}

// Decompression equivalent to function above
void unpackAccumAlphaAndFragCount(in uint packedValue, out float accumAlpha, out uint fragCount)
{
    accumAlpha = float(packedValue & 0xFFFFu) / 255.0;
    fragCount = (packedValue >> 16) & 0xFFFFu;
}