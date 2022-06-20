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

#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileLoader.hpp>
#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
#include "RbcBinFileLoader.hpp"

void RbcBinFileLoader::load(
        const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
        StreamlineTracingGrid* grid) {
    uint8_t* buffer = nullptr;
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(dataSourceFilename, buffer, length, false);
    if (!loaded) {
        sgl::Logfile::get()->throwError(
                "Error in RbcBinFileLoader::load: Couldn't open file \"" + dataSourceFilename + "\".");
    }
    auto* dataField = reinterpret_cast<float*>(buffer);

    int xs = 1024;
    int ys = 32;
    int zs = 1024;
    float cellStep = 1.0f / 1023.0f;

    if (length != size_t(xs) * size_t(ys) * size_t(zs) * sizeof(float) * 4) {
        sgl::Logfile::get()->throwError(
                "Error in RbcBinFileLoader::load: Inconsistent number of bytes read from file \""
                + dataSourceFilename + "\".");
    }

    int vectorFieldNumEntries = xs * ys * zs * 3;
    int scalarFieldNumEntries = xs * ys * zs;

    auto* velocityField = new float[vectorFieldNumEntries];
    auto* velocityMagnitudeField = new float[scalarFieldNumEntries];
    auto* vorticityField = new float[vectorFieldNumEntries];
    auto* vorticityMagnitudeField = new float[scalarFieldNumEntries];
    auto* helicityField = new float[scalarFieldNumEntries];
    auto* temperatureField = new float[scalarFieldNumEntries];

    for (int z = 0; z < zs; z++) {
        for (int y = 0; y < ys; y++) {
            for (int x = 0; x < xs; x++) {
                velocityField[IDXV(x, y, z, 0)] = dataField[IDXV4(x, y, z, 0)];
                velocityField[IDXV(x, y, z, 1)] = dataField[IDXV4(x, y, z, 1)];
                velocityField[IDXV(x, y, z, 2)] = dataField[IDXV4(x, y, z, 2)];
                temperatureField[IDXS(x, y, z)] = dataField[IDXV4(x, y, z, 3)];
            }
        }
    }

    computeVectorMagnitudeField(velocityField, velocityMagnitudeField, xs, ys, zs);
    computeVorticityField(velocityField, vorticityField, xs, ys, zs, cellStep, cellStep, cellStep);
    computeVectorMagnitudeField(vorticityField, vorticityMagnitudeField, xs, ys, zs);
    computeHelicityField(velocityField, vorticityField, helicityField, xs, ys, zs);

    grid->setGridExtent(xs, ys, zs, cellStep, cellStep, cellStep);
    grid->addVectorField(velocityField, "Velocity");
    grid->addVectorField(vorticityField, "Vorticity");
    grid->addScalarField(helicityField, "Helicity");
    grid->addScalarField(velocityMagnitudeField, "Velocity Magnitude");
    grid->addScalarField(vorticityMagnitudeField, "Vorticity Magnitude");
    grid->addScalarField(temperatureField, "Temperature");

    delete[] buffer;
}
