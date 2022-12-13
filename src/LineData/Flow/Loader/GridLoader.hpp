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

#ifndef LINEVIS_GRIDLOADER_HPP
#define LINEVIS_GRIDLOADER_HPP

#include <cstdint>

/**
 * Computes the magnitude field of a given vector field.
 * @param vectorField A float array of size xs * ys * zs * 3 storing the 3D vector field (input).
 * @param vectorMagnitudeField A float array of size xs * ys * zs into which the vector magnitude is written (output).
 * @param xs The grid size in x direction.
 * @param ys The grid size in y direction.
 * @param zs The grid size in z direction.
 */
void computeVectorMagnitudeField(
        const float* vectorField, float* vectorMagnitudeField, int xs, int ys, int zs);

/**
 * Computes the magnitude field of a given vector field.
 * @param velocityField A float array of size xs * ys * zs * 3 storing the 3D velocity field (input).
 * @param vorticityField A float array of size xs * ys * zs * 3 into which the vorticity is written (output).
 * @param xs The grid size in x direction.
 * @param ys The grid size in y direction.
 * @param zs The grid size in z direction.
 */
void computeVorticityField(
        const float* velocityField, float* vorticityField, int xs, int ys, int zs, float dx, float dy, float dz);

/**
 * Computes the helicity field. The helicity is the dot product of the velocity and the vorticity.
 * @param velocityField A float array of size xs * ys * zs * 3 storing the 3D velocity field (input).
 * @param vorticityField A float array of size xs * ys * zs * 3 storing the 3D vorticity field (input).
 * @param helicityField A float array of size xs * ys * zs into which the helicity is written (output).
 * @param xs The grid size in x direction.
 * @param ys The grid size in y direction.
 * @param zs The grid size in z direction.
 */
void computeHelicityField(
        const float* velocityField, const float* vorticityField, float* helicityField, int xs, int ys, int zs);

/**
 * Computes the helicity field. The helicity is the dot product of the velocity and the vorticity.
 * In the following papers, different definitions were used for the helicity.
 * - Helicity: Rees et al. 2017, "A Stream Ribbon Seeding Strategy"
 * - Helicity with normalized velocity: Ueng et al. 1996,
 *   "Efficient Streamline, Streamribbon, and Streamlube Constructions on Unstructured Grids"
 * - Normalized helicity (normalized velocity & normalized vorticity): Karch et al. 2014,
 *   "Streamline-Based Concepts for Space-Time Analysis of 2D Time-Dependent Flow"
 * @param velocityField A float array of size xs * ys * zs * 3 storing the 3D velocity field (input).
 * @param vorticityField A float array of size xs * ys * zs * 3 storing the 3D vorticity field (input).
 * @param helicityField A float array of size xs * ys * zs into which the helicity is written (output).
 * @param xs The grid size in x direction.
 * @param ys The grid size in y direction.
 * @param zs The grid size in z direction.
 */
void computeHelicityFieldNormalized(
        const float* velocityField, const float* vorticityField, float* helicityField, int xs, int ys, int zs,
        bool normalizeVelocity, bool normalizeVorticity);

/**
 * Swaps the endianness of the passed array.
 * @param values The array to swap endianness for.
 * @param sizeInBytes The byte size of the array.
 * @param bytesPerEntry The size in bytes of one array entry. The bytes in each entry are shuffled.
 */
void swapEndianness(uint8_t* byteArray, size_t sizeInBytes, size_t bytesPerEntry);

/**
 * Swaps the endianness of the passed array.
 * @tparam T The data type. The bytes in each range of sizeof(T) are shuffled.
 * @param values The array to swap endianness for.
 * @param n The number of entries of byte size sizeof(T) in the array.
 */
template <typename T>
void swapEndianness(T* values, int n) {
    swapEndianness((uint8_t*)values, size_t(n) * sizeof(T), sizeof(T));
    /*auto* byteArray = (uint8_t*)values;
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, n), [&values, &byteArray](auto const& r) {
        for (auto i = r.begin(); i != r.end(); i++) {
#else
    #pragma omp parallel for shared(byteArray, values, n) default(none)
    for (int i = 0; i < n; i++) {
#endif
        uint8_t swappedEntry[sizeof(T)];
        for (size_t j = 0; j < sizeof(T); j++) {
            swappedEntry[j] = byteArray[i * sizeof(T) + sizeof(T) - j - 1];
        }
        values[i] = *((T*)&swappedEntry);
    }
#ifdef USE_TBB
    });
#endif*/
}

#endif //LINEVIS_GRIDLOADER_HPP
