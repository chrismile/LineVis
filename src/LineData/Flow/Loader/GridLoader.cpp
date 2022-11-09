/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021-2022, Christoph Neuhauser
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

#include <cmath>

#ifdef USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#endif

#include <Utils/File/Logfile.hpp>

#include "../StreamlineTracingDefines.hpp"
#include "GridLoader.hpp"

void computeVectorMagnitudeField(
        const float* vectorField, float* vectorMagnitudeField, int xs, int ys, int zs) {
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, zs), [&](auto const& r) {
        for (auto z = r.begin(); z != r.end(); z++) {
#else
    #pragma omp parallel for shared(xs, ys, zs, vectorField, vectorMagnitudeField) default(none)
    for (int z = 0; z < zs; z++) {
#endif
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                float vx = vectorField[IDXV(x, y, z, 0)];
                float vy = vectorField[IDXV(x, y, z, 1)];
                float vz = vectorField[IDXV(x, y, z, 2)];
                vectorMagnitudeField[IDXS(x, y, z)] = std::sqrt(vx * vx + vy * vy + vz * vz);
            }
        }
    }
#ifdef USE_TBB
    });
#endif
}

void computeVorticityField(
        const float* velocityField, float* vorticityField, int xs, int ys, int zs, float dx, float dy, float dz) {
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, zs), [&](auto const& r) {
        for (auto z = r.begin(); z != r.end(); z++) {
#else
    #pragma omp parallel for shared(xs, ys, zs, dx, dy, dz, velocityField, vorticityField) default(none)
    for (int z = 0; z < zs; z++) {
#endif
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                int left = x > 0 ? -1 : 0;
                int right = x < xs - 1 ? 1 : 0;
                int down = y > 0 ? -1 : 0;
                int up = y < ys - 1 ? 1 : 0;
                int back = z > 0 ? -1 : 0;
                int front = z < zs - 1 ? 1 : 0;
                float dVzdy =
                        (velocityField[IDXV(x, y + up, z, 2)]
                         - velocityField[IDXV(x, y + down, z, 2)]) / (dy * float(up - down));
                float dVydz =
                        (velocityField[IDXV(x, y, z + front, 1)]
                         - velocityField[IDXV(x, y, z + back, 1)]) / (dy * float(front - back));
                float dVxdz =
                        (velocityField[IDXV(x, y, z + front, 0)]
                         - velocityField[IDXV(x, y, z + back, 0)]) / (dy * float(front - back));
                float dVzdx =
                        (velocityField[IDXV(x + right, y, z, 2)]
                         - velocityField[IDXV(x + left, y, z, 2)]) / (dy * float(right - left));
                float dVydx =
                        (velocityField[IDXV(x + right, y, z, 1)]
                         - velocityField[IDXV(x + left, y, z, 1)]) / (dy * float(right - left));
                float dVxdy =
                        (velocityField[IDXV(x, y + up, z, 0)]
                         - velocityField[IDXV(x, y + down, z, 0)]) / (dy * float(up - down));

                float vorticityX = dVzdy - dVydz;
                float vorticityY = dVxdz - dVzdx;
                float vorticityZ = dVydx - dVxdy;
                vorticityField[IDXV(x, y, z, 0)] = vorticityX;
                vorticityField[IDXV(x, y, z, 1)] = vorticityY;
                vorticityField[IDXV(x, y, z, 2)] = vorticityZ;
            }
        }
    }
#ifdef USE_TBB
    });
#endif
}

void computeHelicityField(
        const float* velocityField, const float* vorticityField, float* helicityField, int xs, int ys, int zs) {
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, zs), [&](auto const& r) {
        for (auto z = r.begin(); z != r.end(); z++) {
#else
    #pragma omp parallel for shared(xs, ys, zs, velocityField, vorticityField, helicityField) default(none)
    for (int z = 0; z < zs; z++) {
#endif
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                float vorticityX = vorticityField[IDXV(x, y, z, 0)];
                float vorticityY = vorticityField[IDXV(x, y, z, 1)];
                float vorticityZ = vorticityField[IDXV(x, y, z, 2)];
                float velocityX = velocityField[IDXV(x, y, z, 0)];
                float velocityY = velocityField[IDXV(x, y, z, 1)];
                float velocityZ = velocityField[IDXV(x, y, z, 2)];
                helicityField[IDXS(x, y, z)] = velocityX * vorticityX + velocityY * vorticityY + velocityZ * vorticityZ;
            }
        }
    }
#ifdef USE_TBB
    });
#endif
}

void computeHelicityFieldNormalized(
        const float* velocityField, const float* vorticityField, float* helicityField, int xs, int ys, int zs,
        bool normalizeVelocity, bool normalizeVorticity) {
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, zs), [&](auto const& r) {
        for (auto z = r.begin(); z != r.end(); z++) {
#else
    #pragma omp parallel for shared(xs, ys, zs, velocityField, vorticityField, helicityField, normalizeVelocity, normalizeVorticity) default(none)
    for (int z = 0; z < zs; z++) {
#endif
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                float vorticityX = vorticityField[IDXV(x, y, z, 0)];
                float vorticityY = vorticityField[IDXV(x, y, z, 1)];
                float vorticityZ = vorticityField[IDXV(x, y, z, 2)];
                float velocityX = velocityField[IDXV(x, y, z, 0)];
                float velocityY = velocityField[IDXV(x, y, z, 1)];
                float velocityZ = velocityField[IDXV(x, y, z, 2)];
                if (normalizeVelocity) {
                    float velocityMagnitude = std::sqrt(
                            velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);
                    if (velocityMagnitude > 1e-6) {
                        velocityX /= velocityMagnitude;
                        velocityY /= velocityMagnitude;
                        velocityZ /= velocityMagnitude;
                    }
                }
                if (normalizeVorticity) {
                    float vorticityMagnitude = std::sqrt(
                            vorticityX * vorticityX + vorticityY * vorticityY + vorticityZ * vorticityZ);
                    if (vorticityMagnitude > 1e-6) {
                        vorticityX /= vorticityMagnitude;
                        vorticityY /= vorticityMagnitude;
                        vorticityZ /= vorticityMagnitude;
                    }
                }
                helicityField[IDXS(x, y, z)] = velocityX * vorticityX + velocityY * vorticityY + velocityZ * vorticityZ;
            }
        }
    }
#ifdef USE_TBB
    });
#endif
}

void swapEndianness(uint8_t* byteArray, size_t sizeInBytes, size_t bytesPerEntry) {
    /*
     * Variable length arrays (VLAs) are a C99-only feature, and supported by GCC and Clang merely as C++ extensions.
     * Visual Studio reports the following error when attempting to use VLAs:
     * "error C3863: array type 'uint8_t [this->8<L_ALIGN>8]' is not assignable".
     * Consequently, we will assume 'sizeInBytes' is less than or equal to 8 (i.e., 64 bit values).
     */
    if (sizeInBytes > 8) {
        sgl::Logfile::get()->throwError("Error in swapEndianness: sizeInBytes is larger than 8.");
        return;
    }
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, sizeInBytes, bytesPerEntry), [&byteArray, bytesPerEntry](auto const& r) {
        uint8_t swappedEntry[8];
        for (auto i = r.begin(); i != r.end(); i += r.grainsize()) {
#else
    uint8_t swappedEntry[8];
    #pragma omp parallel for shared(byteArray, sizeInBytes, bytesPerEntry) private(swappedEntry) default(none)
    for (size_t i = 0; i < sizeInBytes; i += bytesPerEntry) {
#endif
        for (size_t j = 0; j < bytesPerEntry; j++) {
            swappedEntry[j] = byteArray[i + bytesPerEntry - j - 1];
        }
        for (size_t j = 0; j < bytesPerEntry; j++) {
            byteArray[i + j] = swappedEntry[j];
        }
    }
#ifdef USE_TBB
    });
#endif
}
