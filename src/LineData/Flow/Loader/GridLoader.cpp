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

#include <cmath>
#include "../StreamlineTracingDefines.hpp"
#include "GridLoader.hpp"

void computeVectorMagnitudeField(
        const float* vectorField, float* vectorMagnitudeField, int xs, int ys, int zs) {
    #pragma omp parallel for shared(xs, ys, zs, vectorField, vectorMagnitudeField)  default(none)
    for (int z = 0; z < zs; z++) {
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                float vx = vectorField[IDXV(x, y, z, 0)];
                float vy = vectorField[IDXV(x, y, z, 1)];
                float vz = vectorField[IDXV(x, y, z, 2)];
                vectorMagnitudeField[IDXS(x, y, z)] = std::sqrt(vx * vx + vy * vy + vz * vz);
            }
        }
    }
}

void computeVorticityField(
        const float* velocityField, float* vorticityField, int xs, int ys, int zs, float dx, float dy, float dz) {
    #pragma omp parallel for shared(xs, ys, zs, dx, dy, dz, velocityField, vorticityField) default(none)
    for (int z = 0; z < zs; z++) {
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
}

void computeHelicityField(
        const float* velocityField, const float* vorticityField, float* helicityField, int xs, int ys, int zs) {
    #pragma omp parallel for shared(xs, ys, zs, velocityField, vorticityField, helicityField) default(none)
    for (int z = 0; z < zs; z++) {
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
}
