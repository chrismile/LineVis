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

#include <cstring>
#include "base64.h"

static const uint8_t asciiToBase64Table[256] = {
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 62, 64, 64, 64, 63,
        52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 64, 64, 64, 64, 64,
        64,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
        15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 64, 64, 64, 64, 64,
        64, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
        41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
};

int base64GetNumBytesDecoded(int numBytesBase64) {
    int numBytesDecoded = ((numBytesBase64 + 3) / 4) * 3;
    return numBytesDecoded;
}

int base64DecodeSized(char* bufferDecoded, const char* bufferBase64, int numBytesBase64) {
    int numBytesDecoded = ((numBytesBase64 + 3) / 4) * 3;
    auto* bufferInput = reinterpret_cast<const uint8_t*>(bufferBase64);
    auto* bufferOutput = reinterpret_cast<uint8_t*>(bufferDecoded);

    while (numBytesBase64 > 4) {
        *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[0]] << 2 | asciiToBase64Table[bufferInput[1]] >> 4);
        *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[1]] << 4 | asciiToBase64Table[bufferInput[2]] >> 2);
        *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[2]] << 6 | asciiToBase64Table[bufferInput[3]]);
        bufferInput += 4;
        numBytesBase64 -= 4;
    }

    if (numBytesBase64 > 1) {
        *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[0]] << 2 | asciiToBase64Table[bufferInput[1]] >> 4);
    }
    if (numBytesBase64 > 2) {
        *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[1]] << 4 | asciiToBase64Table[bufferInput[2]] >> 2);
    }
    if (numBytesBase64 > 3) {
        *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[2]] << 6 | asciiToBase64Table[bufferInput[3]]);
    }

    numBytesDecoded -= (4 - numBytesBase64) & 3;
    return numBytesDecoded;
}

uint32_t base64DecodeUint32(const char* bufferBase64) {
    auto* bufferInput = reinterpret_cast<const uint8_t*>(bufferBase64);
    uint32_t value = 0;
    auto* bufferOutput = reinterpret_cast<uint8_t*>(&value);
    *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[0]] << 2 | asciiToBase64Table[bufferInput[1]] >> 4);
    *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[1]] << 4 | asciiToBase64Table[bufferInput[2]] >> 2);
    *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[2]] << 6 | asciiToBase64Table[bufferInput[3]]);
    *(bufferOutput++) = uint8_t(asciiToBase64Table[bufferInput[4]] << 2 | asciiToBase64Table[bufferInput[5]] >> 4);
    return value;
}
