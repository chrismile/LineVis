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

//#define ADDRESSING_TILED_2x2
//#define ADDRESSING_TILED_2x8
//#define ADDRESSING_TILED_NxM
//#define TILE_N 2u
//#define TILE_M 8u

#if defined(ADRESSING_MORTON_CODE_8x8)
// Space-filling and locality-preserving curve mapping pixel positions in an 8x8 tile to
// a linearized memory offset in the tile.
uint mortonCodeLookupTable[64] = {
    0,  1,  4,  5,  16, 17, 20, 21,
    2,  3,  6,  7,  18, 19, 22, 23,
    8,  9,  12, 13, 24, 25, 28, 29,
    10, 11, 14, 15, 26, 27, 30, 31,
    32, 33, 36, 37, 48, 49, 52, 53,
    34, 35, 38, 39, 50, 51, 54, 55,
    40, 41, 44, 45, 56, 57, 60, 61,
    42, 43, 46, 47, 58, 59, 62, 63
};
#endif

// Address 1D structured buffers as tiled to better data exploit locality
// "OIT to Volumetric Shadow Mapping, 101 Uses for Raster Ordered Views using DirectX 12",
// by Leigh Davies (Intel), March 05, 2015
uint addrGen(uvec2 addr2D) {
#if defined(ADRESSING_MORTON_CODE_8x8)
    uint surfaceWidth = viewportW >> 3U; // / 8U
    uvec2 tileAddr2D = addr2D >> 3U; // / 8U
    uint tileAddr1D = (tileAddr2D.x + surfaceWidth * tileAddr2D.y) << 6U; // * 64U
    uvec2 pixelAddr2D = addr2D & 7U;
    uint pixelAddr1D = pixelAddr2D.x + (pixelAddr2D.y << 3U);
    return tileAddr1D | mortonCodeLookupTable[pixelAddr1D];
#elif defined(ADDRESSING_TILED_2x2)
    uint surfaceWidth = viewportW >> 1U; // / 2U
    uvec2 tileAddr2D = addr2D >> 1U; // / 2U
    uint tileAddr1D = (tileAddr2D.x + surfaceWidth * tileAddr2D.y) << 2U; // * 4U
    uvec2 pixelAddr2D = addr2D & 1U;
    uint pixelAddr1D = (pixelAddr2D.x) + (pixelAddr2D.y << 1U);
    return tileAddr1D | pixelAddr1D;
#elif defined(ADDRESSING_TILED_2x8)
    uint surfaceWidth = viewportW >> 1U; // / 2U;
    uvec2 tileAddr2D = addr2D / uvec2(2U, 8U);
    uint tileAddr1D = (tileAddr2D.x + surfaceWidth * tileAddr2D.y) << 4U; // * 16U;
    uvec2 pixelAddr2D = addr2D & uvec2(1U, 7U);
    uint pixelAddr1D = pixelAddr2D.x + pixelAddr2D.y * 2U;
    return tileAddr1D | pixelAddr1D;
#elif defined(ADDRESSING_TILED_NxM)
    uint surfaceWidth = viewportW / TILE_N;
    uvec2 tileAddr2D = addr2D / uvec2(TILE_N, TILE_M);
    uint tileAddr1D = (tileAddr2D.x + surfaceWidth * tileAddr2D.y) * (TILE_N * TILE_M);
    uvec2 pixelAddr2D = addr2D & uvec2(TILE_N-1, TILE_M-1);
    uint pixelAddr1D = pixelAddr2D.x + pixelAddr2D.y * TILE_N;
    return tileAddr1D | pixelAddr1D;
#else
    return addr2D.x + viewportW * addr2D.y;
#endif
}

// For use with Image Load/Store
ivec2 addrGen2D(ivec2 addr2D) {
    int addr1D = int(addrGen(addr2D));
    return ivec2(addr1D%viewportW, addr1D/viewportW);
}
